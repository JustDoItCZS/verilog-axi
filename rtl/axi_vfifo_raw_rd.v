/*

Copyright (c) 2023 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// 语言: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * AXI4 虚拟 FIFO（原始读侧）
 *
 * 模块目录
 * 1) 从 AXI 内存端按突发读取虚拟 FIFO 原始数据。
 * 2) 通过跨时钟异步 FIFO 把读数据交给 output_clk 域的解码侧。
 * 3) 维护读指针推进、起止指针回报与可选控制通道输出。
 */
module axi_vfifo_raw_rd #
(
    // 输入分段位宽
    parameter SEG_WIDTH = 32,
    // 分段数量
    parameter SEG_CNT = 2,
    // AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = SEG_WIDTH*SEG_CNT,
    // AXI 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // AXI WSTRB 位宽（按字节）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 位宽
    parameter AXI_ID_WIDTH = 8,
    // 允许生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // 长度字段位宽
    parameter LEN_WIDTH = AXI_ADDR_WIDTH,
    // AXI 读数据输出 FIFO 深度（全宽字）
    parameter READ_FIFO_DEPTH = 128,
    // AXI 最大读突发长度
    parameter READ_MAX_BURST_LEN = READ_FIFO_DEPTH/4,
    // 是否启用控制输出
    parameter CTRL_OUT_EN = 0
)
(
    input  wire                          clk, // 读侧主时钟(AXI 域)。
    input  wire                          rst, // 读侧同步复位。

    /*
     * 分段数据输出（到解码逻辑）
     */
    input  wire                          output_clk, // 输出侧时钟(解码域)。
    input  wire                          output_rst, // 输出侧复位输入。
    output wire                          output_rst_out, // 同步后的输出侧复位输出。
    output wire [SEG_CNT*SEG_WIDTH-1:0]  output_data, // 分段数据输出总线。
    output wire [SEG_CNT-1:0]            output_valid, // 分段数据有效位。
    input  wire [SEG_CNT-1:0]            output_ready, // 分段数据就绪位。
    output wire [SEG_CNT*SEG_WIDTH-1:0]  output_ctrl_data, // 分段控制输出总线。
    output wire [SEG_CNT-1:0]            output_ctrl_valid, // 分段控制有效位。
    input  wire [SEG_CNT-1:0]            output_ctrl_ready, // 分段控制就绪位。

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]       m_axi_arid, // AXI 读地址 ID。
    output wire [AXI_ADDR_WIDTH-1:0]     m_axi_araddr, // AXI 读地址。
    output wire [7:0]                    m_axi_arlen, // AXI 读突发长度。
    output wire [2:0]                    m_axi_arsize, // AXI 读每拍大小。
    output wire [1:0]                    m_axi_arburst, // AXI 读突发类型。
    output wire                          m_axi_arlock, // AXI 读锁属性。
    output wire [3:0]                    m_axi_arcache, // AXI 读缓存属性。
    output wire [2:0]                    m_axi_arprot, // AXI 读保护属性。
    output wire                          m_axi_arvalid, // AXI 读地址有效。
    input  wire                          m_axi_arready, // AXI 读地址就绪。
    input  wire [AXI_ID_WIDTH-1:0]       m_axi_rid, // AXI 读响应 ID。
    input  wire [AXI_DATA_WIDTH-1:0]     m_axi_rdata, // AXI 读响应数据。
    input  wire [1:0]                    m_axi_rresp, // AXI 读响应状态。
    input  wire                          m_axi_rlast, // AXI 读响应最后一拍。
    input  wire                          m_axi_rvalid, // AXI 读响应有效。
    output wire                          m_axi_rready, // AXI 读响应就绪。

    /*
     * FIFO 控制
     */
    input  wire [LEN_WIDTH+1-1:0]        wr_start_ptr_in, // 写侧起始指针输入。
    input  wire [LEN_WIDTH+1-1:0]        wr_finish_ptr_in, // 写侧完成指针输入。
    output wire [LEN_WIDTH+1-1:0]        rd_start_ptr_out, // 读侧起始指针输出。
    output wire [LEN_WIDTH+1-1:0]        rd_finish_ptr_out, // 读侧完成指针输出。

    /*
     * 配置
     */
    input  wire [AXI_ADDR_WIDTH-1:0]     cfg_fifo_base_addr, // FIFO 映射基地址。
    input  wire [LEN_WIDTH-1:0]          cfg_fifo_size_mask, // FIFO 地址空间掩码。
    input  wire                          cfg_enable, // 功能使能。
    input  wire                          cfg_reset, // 配置触发复位请求。

    /*
     * 状态
     */
    output wire                          sts_read_active // 读侧活跃状态。
);

localparam AXI_BYTE_LANES = AXI_STRB_WIDTH;
localparam AXI_BYTE_SIZE = AXI_DATA_WIDTH/AXI_BYTE_LANES;
localparam AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
localparam AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE;

localparam OFFSET_ADDR_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
localparam OFFSET_ADDR_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_ADDR_WIDTH{1'b1}} : 0;
localparam ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH);
localparam CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

localparam READ_FIFO_ADDR_WIDTH = $clog2(READ_FIFO_DEPTH);

// 用于求最小值对数的辅助公式：mask(x) = (2**$clog2(x))-1
// 用于求最小值对数的辅助公式：log2(min(x, y, z)) = (mask & mask & mask)+1
// 用于求向下取整对数的辅助公式：floor(log2(x)) = $clog2(x+1)-1
// 组合限制条件下的 floor(log2(min(...))) 计算说明
localparam READ_MAX_BURST_LEN_INT = ((2**($clog2(AXI_MAX_BURST_LEN+1)-1)-1) & (2**($clog2(READ_MAX_BURST_LEN+1)-1)-1) & (2**(READ_FIFO_ADDR_WIDTH-1)-1) & ((4096/AXI_BYTE_LANES)-1)) + 1;
localparam READ_MAX_BURST_SIZE_INT = READ_MAX_BURST_LEN_INT << AXI_BURST_SIZE;
localparam READ_BURST_LEN_WIDTH = $clog2(READ_MAX_BURST_LEN_INT);
localparam READ_BURST_ADDR_WIDTH = $clog2(READ_MAX_BURST_SIZE_INT);
localparam READ_BURST_ADDR_MASK = READ_BURST_ADDR_WIDTH > 1 ? {READ_BURST_ADDR_WIDTH{1'b1}} : 0;

// 参数检查
initial begin
    if (AXI_BYTE_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisible (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_BYTE_LANES) != AXI_BYTE_LANES) begin
        $error("Error: AXI byte lane count must be even power of two (instance %m)");
        $finish;
    end

    if (AXI_MAX_BURST_LEN < 1 || AXI_MAX_BURST_LEN > 256) begin
        $error("Error: AXI_MAX_BURST_LEN must be between 1 and 256 (instance %m)");
        $finish;
    end

    if (SEG_CNT * SEG_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: Width mismatch (instance %m)");
        $finish;
    end
end

localparam [1:0]
    AXI_RESP_OKAY = 2'b00,
    AXI_RESP_EXOKAY = 2'b01,
    AXI_RESP_SLVERR = 2'b10,
    AXI_RESP_DECERR = 2'b11;

reg [AXI_ADDR_WIDTH-1:0] m_axi_araddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_araddr_next; // AR 地址寄存器。
reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next; // AR 长度寄存器。
reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next; // ARVALID 寄存器。

assign m_axi_arid = {AXI_ID_WIDTH{1'b0}};
assign m_axi_araddr = m_axi_araddr_reg;
assign m_axi_arlen = m_axi_arlen_reg;
assign m_axi_arsize = AXI_BURST_SIZE;
assign m_axi_arburst = 2'b01;
assign m_axi_arlock = 1'b0;
assign m_axi_arcache = 4'b0011;
assign m_axi_arprot = 3'b010;
assign m_axi_arvalid = m_axi_arvalid_reg;

// 复位同步
wire rst_req_int = cfg_reset; // 输出域复位同步请求。

(* shreg_extract = "no" *)
reg rst_sync_1_reg = 1'b1,  rst_sync_2_reg = 1'b1, rst_sync_3_reg = 1'b1; // 输出域复位同步链寄存器。

assign output_rst_out = rst_sync_3_reg;

always @(posedge output_clk or posedge rst_req_int) begin
    if (rst_req_int) begin
        rst_sync_1_reg <= 1'b1;
    end else begin
        rst_sync_1_reg <= 1'b0;
    end
end

always @(posedge output_clk) begin
    rst_sync_2_reg <= rst_sync_1_reg;
    rst_sync_3_reg <= rst_sync_2_reg;
end

// 输出数据通路逻辑（读数据）
reg [AXI_DATA_WIDTH-1:0] m_axis_tdata_reg  = {AXI_DATA_WIDTH{1'b0}}; // 输出域聚合数据寄存器。
reg                      m_axis_tvalid_reg = 1'b0; // 输出域聚合数据有效位。

reg [READ_FIFO_ADDR_WIDTH-1:0] read_fifo_read_start_cnt = 0; // 本次读启动计数(预留读启动推进量)。
reg read_fifo_read_start_en = 1'b0; // 读启动计数使能。

reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_read_start_ptr_reg = 0; // 读启动指针。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_wr_ptr_reg = 0; // 读数据 FIFO 写指针(AXI 域)。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_wr_ptr_gray_reg = 0; // 读数据 FIFO 写指针格雷码。
wire [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_rd_ptr; // 读数据 FIFO 读指针(输出域)。
wire [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_rd_ptr_gray; // 读数据 FIFO 读指针格雷码。
wire [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_ctrl_rd_ptr; // 控制 FIFO 读指针(输出域)。
wire [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_ctrl_rd_ptr_gray; // 控制 FIFO 读指针格雷码。

reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_wr_ptr_temp; // 写指针临时变量。

(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_wr_ptr_gray_sync_1_reg = 0; // 写指针格雷码同步链第 1 级（输出域）。
(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_wr_ptr_gray_sync_2_reg = 0; // 写指针格雷码同步链第 2 级（输出域）。

(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_rd_ptr_gray_sync_1_reg = 0; // 读指针格雷码同步链第 1 级（AXI 域）。
(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_rd_ptr_gray_sync_2_reg = 0; // 读指针格雷码同步链第 2 级（AXI 域）。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_rd_ptr_sync_reg = 0; // 同步后的二进制读指针(AXI 域)。

(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_ctrl_rd_ptr_gray_sync_1_reg = 0; // 控制读指针格雷码同步链第 1 级（AXI 域）。
(* shreg_extract = "no" *)
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_ctrl_rd_ptr_gray_sync_2_reg = 0; // 控制读指针格雷码同步链第 2 级（AXI 域）。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_ctrl_rd_ptr_sync_reg = 0; // 同步后的控制读指针(AXI 域)。

reg read_fifo_half_full_reg = 1'b0; // 读数据 FIFO 半满标志。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_occupancy_reg = 0; // 读数据 FIFO 占用计数。
reg [READ_FIFO_ADDR_WIDTH+1-1:0] read_fifo_occupancy_lookahead_reg = 0; // 前瞻占用计数(考虑读启动)。

wire read_fifo_full = read_fifo_wr_ptr_gray_reg == (read_fifo_rd_ptr_gray_sync_2_reg ^ {2'b11, {READ_FIFO_ADDR_WIDTH-1{1'b0}}}); // 读数据 FIFO 满标志。
wire read_fifo_empty = read_fifo_rd_ptr_gray == read_fifo_wr_ptr_gray_sync_2_reg; // 读数据 FIFO 空标志。

wire read_fifo_ctrl_full = read_fifo_wr_ptr_gray_reg == (read_fifo_ctrl_rd_ptr_gray_sync_2_reg ^ {2'b11, {READ_FIFO_ADDR_WIDTH-1{1'b0}}}); // 控制 FIFO 满标志。
wire read_fifo_ctrl_empty = read_fifo_ctrl_rd_ptr_gray == read_fifo_wr_ptr_gray_sync_2_reg; // 控制 FIFO 空标志。

assign m_axi_rready = (!read_fifo_full && (!CTRL_OUT_EN || !read_fifo_ctrl_full)) || cfg_reset;

genvar n;
integer k; // 格雷码转二进制的循环索引。

generate

for (n = 0; n < SEG_CNT; n = n + 1) begin : read_fifo_seg

    reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_reg = 0; // 第 n 段读 FIFO 读指针。
    reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_gray_reg = 0; // 第 n 段读 FIFO 读指针格雷码。

    reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_temp; // 第 n 段读指针临时变量。

    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg [SEG_WIDTH-1:0] seg_mem_data[2**READ_FIFO_ADDR_WIDTH-1:0]; // 第 n 段数据 RAM。

    reg [SEG_WIDTH-1:0] seg_rd_data_reg = 0; // 第 n 段输出数据寄存器。
    reg seg_rd_data_valid_reg = 0; // 第 n 段输出有效位。

    wire seg_empty = seg_rd_ptr_gray_reg == read_fifo_wr_ptr_gray_sync_2_reg; // 第 n 段 FIFO 空标志。

    assign output_data[n*SEG_WIDTH +: SEG_WIDTH] = seg_rd_data_reg;
    assign output_valid[n] = seg_rd_data_valid_reg;

    if (n == SEG_CNT-1) begin
        assign read_fifo_rd_ptr = seg_rd_ptr_reg;
        assign read_fifo_rd_ptr_gray = seg_rd_ptr_gray_reg;
    end

    always @(posedge clk) begin
        if (!read_fifo_full && m_axi_rready && m_axi_rvalid) begin
            seg_mem_data[read_fifo_wr_ptr_reg[READ_FIFO_ADDR_WIDTH-1:0]] <= m_axi_rdata[n*SEG_WIDTH +: SEG_WIDTH];
        end
    end

    // 分段读出逻辑
    always @(posedge output_clk) begin
        seg_rd_data_valid_reg <= seg_rd_data_valid_reg && !output_ready[n];

        if (!seg_empty && (!seg_rd_data_valid_reg || output_ready[n])) begin
            seg_rd_data_reg <= seg_mem_data[seg_rd_ptr_reg[READ_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_data_valid_reg <= 1'b1;

            seg_rd_ptr_temp = seg_rd_ptr_reg + 1;
            seg_rd_ptr_reg <= seg_rd_ptr_temp;
            seg_rd_ptr_gray_reg <= seg_rd_ptr_temp ^ (seg_rd_ptr_temp >> 1);
        end

        if (output_rst || output_rst_out) begin
            seg_rd_ptr_reg <= 0;
            seg_rd_ptr_gray_reg <= 0;
            seg_rd_data_valid_reg <= 1'b0;
        end
    end

end

endgenerate

// 写入缓存逻辑
always @(posedge clk) begin
    read_fifo_occupancy_reg <= read_fifo_wr_ptr_reg - read_fifo_rd_ptr_sync_reg;
    read_fifo_half_full_reg <= $unsigned(read_fifo_wr_ptr_reg - read_fifo_rd_ptr_sync_reg) >= 2**(READ_FIFO_ADDR_WIDTH-1);

    if (read_fifo_read_start_en) begin
        read_fifo_read_start_ptr_reg <= read_fifo_read_start_ptr_reg + read_fifo_read_start_cnt;
        read_fifo_occupancy_lookahead_reg <= read_fifo_read_start_ptr_reg + read_fifo_read_start_cnt - read_fifo_rd_ptr_sync_reg;
    end else begin
        read_fifo_occupancy_lookahead_reg <= read_fifo_read_start_ptr_reg - read_fifo_rd_ptr_sync_reg;
    end

    if (!read_fifo_full && m_axi_rready && m_axi_rvalid) begin
        read_fifo_wr_ptr_temp = read_fifo_wr_ptr_reg + 1;
        read_fifo_wr_ptr_reg <= read_fifo_wr_ptr_temp;
        read_fifo_wr_ptr_gray_reg <= read_fifo_wr_ptr_temp ^ (read_fifo_wr_ptr_temp >> 1);

        read_fifo_occupancy_reg <= read_fifo_wr_ptr_temp - read_fifo_rd_ptr_sync_reg;
    end

    if (rst || cfg_reset) begin
        read_fifo_read_start_ptr_reg <= 0;
        read_fifo_wr_ptr_reg <= 0;
        read_fifo_wr_ptr_gray_reg <= 0;
    end
end

// 指针同步逻辑
always @(posedge clk) begin
    read_fifo_rd_ptr_gray_sync_1_reg <= read_fifo_rd_ptr_gray;
    read_fifo_rd_ptr_gray_sync_2_reg <= read_fifo_rd_ptr_gray_sync_1_reg;

    for (k = 0; k < READ_FIFO_ADDR_WIDTH+1; k = k + 1) begin
        read_fifo_rd_ptr_sync_reg[k] <= ^(read_fifo_rd_ptr_gray_sync_2_reg >> k);
    end

    if (rst || cfg_reset) begin
        read_fifo_rd_ptr_gray_sync_1_reg <= 0;
        read_fifo_rd_ptr_gray_sync_2_reg <= 0;
        read_fifo_rd_ptr_sync_reg <= 0;
    end
end

always @(posedge clk) begin
    read_fifo_ctrl_rd_ptr_gray_sync_1_reg <= read_fifo_ctrl_rd_ptr_gray;
    read_fifo_ctrl_rd_ptr_gray_sync_2_reg <= read_fifo_ctrl_rd_ptr_gray_sync_1_reg;

    for (k = 0; k < READ_FIFO_ADDR_WIDTH+1; k = k + 1) begin
        read_fifo_ctrl_rd_ptr_sync_reg[k] <= ^(read_fifo_ctrl_rd_ptr_gray_sync_2_reg >> k);
    end

    if (rst || cfg_reset) begin
        read_fifo_ctrl_rd_ptr_gray_sync_1_reg <= 0;
        read_fifo_ctrl_rd_ptr_gray_sync_2_reg <= 0;
        read_fifo_ctrl_rd_ptr_sync_reg <= 0;
    end
end

always @(posedge output_clk) begin
    read_fifo_wr_ptr_gray_sync_1_reg <= read_fifo_wr_ptr_gray_reg;
    read_fifo_wr_ptr_gray_sync_2_reg <= read_fifo_wr_ptr_gray_sync_1_reg;

    if (output_rst || output_rst_out) begin
        read_fifo_wr_ptr_gray_sync_1_reg <= 0;
        read_fifo_wr_ptr_gray_sync_2_reg <= 0;
    end
end

generate

if (CTRL_OUT_EN) begin
    
    for (n = 0; n < SEG_CNT; n = n + 1) begin : read_fifo_ctrl_seg

        reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_reg = 0; // 本段控制输出FIFO的读指针（二进制），在output_clk域前进
        reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_gray_reg = 0; // 本段控制输出 FIFO 读指针的格雷码，用于与写指针同步值比较空满

        reg [READ_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_temp; // 组合计算的下一拍读指针，统一用于更新二进制和格雷码指针

        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg [SEG_WIDTH-1:0] seg_mem_data[2**READ_FIFO_ADDR_WIDTH-1:0]; // 每段控制输出路径的本地存储 RAM，缓存 AXI 读回的对应分段数据

        reg [SEG_WIDTH-1:0] seg_rd_data_reg = 0; // 从本段 RAM 预取出的数据寄存器，送入下游暂存缓冲
        reg seg_rd_data_valid_reg = 0; // 预取数据有效标志，表示seg_rd_data_reg当前可消费

        reg seg_output_ready_reg = 1'b0; // 暂存缓冲向上游回传的就绪寄存，控制何时继续从 RAM 取数

        wire seg_empty = seg_rd_ptr_gray_reg == read_fifo_wr_ptr_gray_sync_2_reg; // 本段控制 FIFO 空标志：读写格雷指针相等

        if (n == SEG_CNT-1) begin
            assign read_fifo_ctrl_rd_ptr = seg_rd_ptr_reg;
            assign read_fifo_ctrl_rd_ptr_gray = seg_rd_ptr_gray_reg;
        end

        always @(posedge clk) begin
            if (!read_fifo_full && m_axi_rready && m_axi_rvalid) begin
                seg_mem_data[read_fifo_wr_ptr_reg[READ_FIFO_ADDR_WIDTH-1:0]] <= m_axi_rdata[n*SEG_WIDTH +: SEG_WIDTH];
            end
        end

        // 分段读出逻辑
        always @(posedge output_clk) begin
            seg_rd_data_valid_reg <= seg_rd_data_valid_reg && !seg_output_ready_reg;

            if (!seg_empty && (!seg_rd_data_valid_reg || seg_output_ready_reg)) begin
                seg_rd_data_reg <= seg_mem_data[seg_rd_ptr_reg[READ_FIFO_ADDR_WIDTH-1:0]];
                seg_rd_data_valid_reg <= 1'b1;

                seg_rd_ptr_temp = seg_rd_ptr_reg + 1;
                seg_rd_ptr_reg <= seg_rd_ptr_temp;
                seg_rd_ptr_gray_reg <= seg_rd_ptr_temp ^ (seg_rd_ptr_temp >> 1);
            end

            if (output_rst || output_rst_out) begin
                seg_rd_ptr_reg <= 0;
                seg_rd_ptr_gray_reg <= 0;
                seg_rd_data_valid_reg <= 1'b0;
            end
        end

        // 暂存缓冲
        reg [SEG_WIDTH-1:0] seg_output_data_reg = 0; // 暂存缓冲主输出寄存器，对接 output_ctrl_data
        reg seg_output_valid_reg = 1'b0; // 主输出寄存器有效位

        reg [SEG_WIDTH-1:0] temp_seg_output_data_reg = 0; // 暂存缓冲临时寄存器，在下游背压时暂存新到达数据
        reg temp_seg_output_valid_reg = 1'b0; // 临时寄存器有效位

        assign output_ctrl_data[n*SEG_WIDTH +: SEG_WIDTH] = seg_output_data_reg;
        assign output_ctrl_valid[n] = seg_output_valid_reg;

        always @(posedge output_clk) begin
            // 若下游就绪，或下一拍不会写满临时寄存器（主输出为空或无新输入），则下一拍拉高上游就绪
            seg_output_ready_reg <= output_ctrl_ready[n] || (!temp_seg_output_valid_reg && (!seg_output_valid_reg || !seg_rd_data_valid_reg));

            if (seg_output_ready_reg) begin
                // 上游已就绪
                if (output_ctrl_ready[n] || !seg_output_valid_reg) begin
                    // 下游就绪或当前输出无效：直接写入主输出寄存器
                    seg_output_data_reg <= seg_rd_data_reg;
                    seg_output_valid_reg <= seg_rd_data_valid_reg;
                end else begin
                    // 下游未就绪：写入临时寄存器
                    temp_seg_output_data_reg <= seg_rd_data_reg;
                    temp_seg_output_valid_reg <= seg_rd_data_valid_reg;
                end
            end else if (output_ctrl_ready[n]) begin
                // 上游未就绪但下游就绪
                seg_output_data_reg <= temp_seg_output_data_reg;
                seg_output_valid_reg <= temp_seg_output_valid_reg;
                temp_seg_output_valid_reg <= 1'b0;
            end

            if (output_rst || output_rst_out) begin
                seg_output_ready_reg <= 1'b0;
                seg_output_valid_reg <= 1'b0;
                temp_seg_output_valid_reg <= 1'b0;
            end
        end

    end

end

endgenerate

reg [READ_BURST_LEN_WIDTH+1-1:0] rd_burst_len; // 当前计划发起的 AXI 读突发拍数
reg [READ_BURST_LEN_WIDTH+1-1:0] rd_outstanding_inc; // 发起新AR时对在途读拍数的增量
reg rd_outstanding_dec; // 接收1拍R通道数据时对在途读拍数的减量使能
reg [READ_FIFO_ADDR_WIDTH+1-1:0] rd_outstanding_reg = 0, rd_outstanding_next; // 在途读数据拍数计数（已发AR未被R通道完全消费）
reg [LEN_WIDTH+1-1:0] rd_start_ptr; // 本次组合逻辑计算出的下一次AR起始指针候选值
reg [7:0] rd_timeout_count_reg = 0, rd_timeout_count_next; // 部分突发等待计时器，避免尾包长期滞留
reg rd_timeout_reg = 0, rd_timeout_next; // 超时触发标志，允许不足满突发时也发起读取

reg [LEN_WIDTH+1-1:0] rd_start_ptr_reg = 0, rd_start_ptr_next; // 已经申请/计划读取到的位置指针（读侧头指针）
reg [LEN_WIDTH+1-1:0] rd_finish_ptr_reg = 0, rd_finish_ptr_next; // 已经实际从R通道接收完成的位置指针（读侧尾指针）

assign rd_start_ptr_out = rd_start_ptr_reg;
assign rd_finish_ptr_out = rd_finish_ptr_reg;

assign sts_read_active = rd_outstanding_reg != 0;

// 读控制逻辑
always @* begin
    rd_start_ptr_next = rd_start_ptr_reg;
    rd_finish_ptr_next = rd_finish_ptr_reg;

    rd_outstanding_inc = 0;
    rd_outstanding_dec = 0;
    rd_outstanding_next = rd_outstanding_reg;
    rd_timeout_count_next = rd_timeout_count_reg;
    rd_timeout_next = rd_timeout_reg;

    m_axi_araddr_next = m_axi_araddr_reg;
    m_axi_arlen_next = m_axi_arlen_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;

    // 非满突发超时处理
    rd_timeout_next = rd_timeout_count_reg == 0;
    if (wr_finish_ptr_in == rd_start_ptr_reg || m_axi_arvalid) begin
        rd_timeout_count_next = 8'hff;
        rd_timeout_next = 1'b0;
    end else if (rd_timeout_count_reg > 0) begin
        rd_timeout_count_next = rd_timeout_count_reg - 1;
    end

    // 按外部存储中的可读占用量计算本次突发长度
    if ((wr_finish_ptr_in ^ rd_start_ptr_reg) >> READ_BURST_ADDR_WIDTH != 0) begin
        // 跨越突发边界：读到边界为止
        rd_burst_len = READ_MAX_BURST_LEN_INT - ((rd_start_ptr_reg & READ_BURST_ADDR_MASK) >> AXI_BURST_SIZE);
        rd_start_ptr = (rd_start_ptr_reg & ~READ_BURST_ADDR_MASK) + (1 << READ_BURST_ADDR_WIDTH);
    end else begin
        // 未跨越突发边界：读取当前可用数据
        rd_burst_len = (wr_finish_ptr_in - rd_start_ptr_reg) >> AXI_BURST_SIZE;
        rd_start_ptr = wr_finish_ptr_in;
    end

    read_fifo_read_start_cnt = rd_burst_len;
    read_fifo_read_start_en = 1'b0;

    // 生成 AXI 读突发
    if (!m_axi_arvalid_reg) begin
        // 可以启动新突发

        m_axi_araddr_next = cfg_fifo_base_addr + (rd_start_ptr_reg & cfg_fifo_size_mask);
        m_axi_arlen_next = rd_burst_len - 1;

        if (cfg_enable && (wr_finish_ptr_in ^ rd_start_ptr_reg) != 0 && read_fifo_occupancy_lookahead_reg < 2**READ_FIFO_ADDR_WIDTH - READ_MAX_BURST_LEN_INT) begin
            // 已使能，且有可读数据，同时 FIFO 仍有空间缓存返回数据
            if ((wr_finish_ptr_in ^ rd_start_ptr_reg) >> READ_BURST_ADDR_WIDTH != 0 || rd_timeout_reg) begin
                // 满突发可发起，或已超时允许发起非满突发
                read_fifo_read_start_en = 1'b1;
                rd_outstanding_inc = rd_burst_len;
                m_axi_arvalid_next = 1'b1;
                rd_start_ptr_next = rd_start_ptr;
            end
        end
    end

    // 处理 AXI 读完成
    if (m_axi_rready && m_axi_rvalid) begin
        rd_finish_ptr_next = rd_finish_ptr_reg + AXI_BYTE_LANES;
        rd_outstanding_dec = 1;
    end

    rd_outstanding_next = rd_outstanding_reg + rd_outstanding_inc - rd_outstanding_dec;

    if (cfg_reset) begin
        rd_start_ptr_next = 0;
        rd_finish_ptr_next = 0;
    end
end

always @(posedge clk) begin
    rd_start_ptr_reg <= rd_start_ptr_next;
    rd_finish_ptr_reg <= rd_finish_ptr_next;

    rd_outstanding_reg <= rd_outstanding_next;
    rd_timeout_count_reg <= rd_timeout_count_next;
    rd_timeout_reg <= rd_timeout_next;

    m_axi_araddr_reg <= m_axi_araddr_next;
    m_axi_arlen_reg <= m_axi_arlen_next;
    m_axi_arvalid_reg <= m_axi_arvalid_next;

    if (rst) begin
        rd_outstanding_reg <= 0;
        m_axi_arvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
