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
 * AXI4 虚拟 FIFO（原始写侧）
 *
 * 模块目录：
 * 1) 在 input_clk 域接收分段输入数据并写入分段异步 FIFO。
 * 2) 在 clk 域按突发策略从输入 FIFO 取数，经 AXI AW/W/B 写入外部存储。
 * 3) 维护写侧起始/完成指针，并根据读侧完成指针计算可用空间与满空状态。
 */
module axi_vfifo_raw_wr #
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
    // AXI 写数据输入 FIFO 深度（全宽字）
    parameter WRITE_FIFO_DEPTH = 64,
    // AXI 最大写突发长度
    parameter WRITE_MAX_BURST_LEN = WRITE_FIFO_DEPTH/4,
    // 水位阈值
    parameter WATERMARK_LEVEL = WRITE_FIFO_DEPTH/2
)
(
    input  wire                          clk, // AXI写侧主时钟（核心控制与AXI主口时序）
    input  wire                          rst, // AXI写侧主复位（同步清空状态机和寄存器）

    /*
     * 分段数据输入（来自编码逻辑）
     */
    input  wire                          input_clk, // 分段输入域时钟（编码端/写入FIFO上游时钟）
    input  wire                          input_rst, // 分段输入域复位
    output wire                          input_rst_out, // 输出给输入域上游的同步复位请求
    output wire                          input_watermark, // 输入域回压水位信号（FIFO趋近满时拉高）
    input  wire [SEG_CNT*SEG_WIDTH-1:0]  input_data, // 分段输入数据总线
    input  wire [SEG_CNT-1:0]            input_valid, // 各分段输入有效
    output wire [SEG_CNT-1:0]            input_ready, // 各分段输入就绪

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]       m_axi_awid, // AXI写地址通道ID（本模块固定输出0）
    output wire [AXI_ADDR_WIDTH-1:0]     m_axi_awaddr, // AXI写地址
    output wire [7:0]                    m_axi_awlen, // AXI 写突发长度（拍数减 1）
    output wire [2:0]                    m_axi_awsize, // AXI写突发每拍字节数编码
    output wire [1:0]                    m_axi_awburst, // AXI写突发类型（INCR）
    output wire                          m_axi_awlock, // AXI锁访问信号（未使用，固定0）
    output wire [3:0]                    m_axi_awcache, // AXI缓存属性
    output wire [2:0]                    m_axi_awprot, // AXI保护属性
    output wire                          m_axi_awvalid, // AXI写地址有效
    input  wire                          m_axi_awready, // AXI写地址就绪
    output wire [AXI_DATA_WIDTH-1:0]     m_axi_wdata, // AXI写数据
    output wire [AXI_STRB_WIDTH-1:0]     m_axi_wstrb, // AXI写字节使能
    output wire                          m_axi_wlast, // AXI写突发最后一拍标志
    output wire                          m_axi_wvalid, // AXI写数据有效
    input  wire                          m_axi_wready, // AXI写数据就绪
    input  wire [AXI_ID_WIDTH-1:0]       m_axi_bid, // AXI写响应ID（本模块不使用）
    input  wire [1:0]                    m_axi_bresp, // AXI写响应码（可用于错误检查）
    input  wire                          m_axi_bvalid, // AXI写响应有效
    output wire                          m_axi_bready, // AXI写响应就绪

    /*
     * FIFO 控制
     */
    output wire [LEN_WIDTH+1-1:0]        wr_start_ptr_out, // 写侧起始指针（已申请写出的逻辑位置）
    output wire [LEN_WIDTH+1-1:0]        wr_finish_ptr_out, // 写侧完成指针（收到B响应后确认写完的位置）
    input  wire [LEN_WIDTH+1-1:0]        rd_start_ptr_in, // 读侧起始指针输入（预留接口，本模块未直接使用）
    input  wire [LEN_WIDTH+1-1:0]        rd_finish_ptr_in, // 读侧完成指针输入（用于计算可用空间）

    /*
     * 配置
     */
    input  wire [AXI_ADDR_WIDTH-1:0]     cfg_fifo_base_addr, // 环形FIFO在外部存储中的基地址
    input  wire [LEN_WIDTH-1:0]          cfg_fifo_size_mask, // 环形FIFO地址掩码（大小需为2的幂）
    input  wire                          cfg_enable, // 使能写路径
    input  wire                          cfg_reset, // FIFO逻辑复位请求（保持时会清空读写指针）

    /*
     * 状态
     */
    output wire [LEN_WIDTH+1-1:0]        sts_fifo_occupancy, // FIFO占用字节数估计（写起点-读完成点）
    output wire                          sts_fifo_empty, // FIFO空状态
    output wire                          sts_fifo_full, // FIFO满状态（无法再安全启动整块写突发）
    output wire                          sts_write_active // 写侧活动状态（存在在途突发或未完成响应）
);

localparam AXI_BYTE_LANES = AXI_STRB_WIDTH;
localparam AXI_BYTE_SIZE = AXI_DATA_WIDTH/AXI_BYTE_LANES;
localparam AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
localparam AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE;

localparam OFFSET_ADDR_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
localparam OFFSET_ADDR_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_ADDR_WIDTH{1'b1}} : 0;
localparam ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH);
localparam CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

localparam WRITE_FIFO_ADDR_WIDTH = $clog2(WRITE_FIFO_DEPTH);
localparam RESP_FIFO_ADDR_WIDTH = 5;

// 用于求最小值对数的辅助公式：mask(x) = (2**$clog2(x))-1
// 用于求最小值对数的辅助公式：log2(min(x, y, z)) = (mask & mask & mask)+1
// 用于求向下取整对数的辅助公式：floor(log2(x)) = $clog2(x+1)-1
// 组合限制条件下的 floor(log2(min(...))) 计算说明
localparam WRITE_MAX_BURST_LEN_INT = ((2**($clog2(AXI_MAX_BURST_LEN+1)-1)-1) & (2**($clog2(WRITE_MAX_BURST_LEN+1)-1)-1) & (2**(WRITE_FIFO_ADDR_WIDTH-1)-1) & ((4096/AXI_BYTE_LANES)-1)) + 1;
localparam WRITE_MAX_BURST_SIZE_INT = WRITE_MAX_BURST_LEN_INT << AXI_BURST_SIZE;
localparam WRITE_BURST_LEN_WIDTH = $clog2(WRITE_MAX_BURST_LEN_INT);
localparam WRITE_BURST_ADDR_WIDTH = $clog2(WRITE_MAX_BURST_SIZE_INT);
localparam WRITE_BURST_ADDR_MASK = WRITE_BURST_ADDR_WIDTH > 1 ? {WRITE_BURST_ADDR_WIDTH{1'b1}} : 0;

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

reg [AXI_ADDR_WIDTH-1:0] m_axi_awaddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_awaddr_next; // AXI AW地址寄存器及其下一状态
reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next; // AXI AWLEN寄存器及其下一状态
reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next; // AXI AWVALID寄存器及其下一状态
reg [AXI_DATA_WIDTH-1:0] m_axi_wdata_reg = {AXI_DATA_WIDTH{1'b0}}, m_axi_wdata_next; // AXI WDATA寄存器及其下一状态
reg [AXI_STRB_WIDTH-1:0] m_axi_wstrb_reg = {AXI_STRB_WIDTH{1'b0}}, m_axi_wstrb_next; // AXI WSTRB寄存器及其下一状态
reg m_axi_wlast_reg = 1'b0, m_axi_wlast_next; // AXI WLAST寄存器及其下一状态
reg m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // AXI WVALID寄存器及其下一状态
reg m_axi_bready_reg = 1'b0, m_axi_bready_next; // AXI BREADY寄存器及其下一状态

assign m_axi_awid = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awaddr = m_axi_awaddr_reg;
assign m_axi_awlen = m_axi_awlen_reg;
assign m_axi_awsize = AXI_BURST_SIZE;
assign m_axi_awburst = 2'b01;
assign m_axi_awlock = 1'b0;
assign m_axi_awcache = 4'b0011;
assign m_axi_awprot = 3'b010;
assign m_axi_awvalid = m_axi_awvalid_reg;
assign m_axi_wdata = m_axi_wdata_reg;
assign m_axi_wstrb = m_axi_wstrb_reg;
assign m_axi_wvalid = m_axi_wvalid_reg;
assign m_axi_wlast = m_axi_wlast_reg;
assign m_axi_bready = m_axi_bready_reg;

// 复位同步
wire rst_req_int = cfg_reset; // 输入域复位请求来源：配置复位

(* shreg_extract = "no" *)
reg rst_sync_1_reg = 1'b1,  rst_sync_2_reg = 1'b1, rst_sync_3_reg = 1'b1; // input_clk 域三级同步复位链

assign input_rst_out = rst_sync_3_reg;

always @(posedge input_clk or posedge rst_req_int) begin
    if (rst_req_int) begin
        rst_sync_1_reg <= 1'b1;
    end else begin
        rst_sync_1_reg <= 1'b0;
    end
end

always @(posedge input_clk) begin
    rst_sync_2_reg <= rst_sync_1_reg;
    rst_sync_3_reg <= rst_sync_2_reg;
end

// 输入数据通路逻辑（写数据）
wire [AXI_DATA_WIDTH-1:0] input_data_int; // 由各段拼接出的整拍AXI写数据
reg input_valid_int_reg = 1'b0; // input_data_int有效标志（clk域）

reg input_read_en; // 读取输入FIFO一拍数据的使能（clk域）

wire [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_wr_ptr; // 输入侧写指针（二进制，来自最后一个段）
wire [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_wr_ptr_gray; // 输入侧写指针格雷码（用于 CDC 同步）
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_reg = 0; // 核心clk域读指针（二进制）
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_gray_reg = 0; // 核心 clk 域读指针格雷码

reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_temp; // 读指针下一值临时变量

(* shreg_extract = "no" *)
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_wr_ptr_gray_sync_1_reg = 0; // 写指针格雷码同步到 clk 域第 1 级
(* shreg_extract = "no" *)
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_wr_ptr_gray_sync_2_reg = 0; // 写指针格雷码同步到 clk 域第 2 级
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_wr_ptr_sync_reg = 0; // 同步后转换得到的写指针二进制值（clk域）

(* shreg_extract = "no" *)
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_gray_sync_1_reg = 0; // 读指针格雷码同步到 input_clk 域第 1 级
(* shreg_extract = "no" *)
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_gray_sync_2_reg = 0; // 读指针格雷码同步到 input_clk 域第 2 级
reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_rd_ptr_sync_reg = 0; // 同步后转换得到的读指针二进制值（input_clk域）

reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] write_fifo_occupancy_reg = 0; // 输入FIFO占用拍数（clk域估计）

wire [SEG_CNT-1:0] write_fifo_seg_full; // 各段FIFO满标志
wire [SEG_CNT-1:0] write_fifo_seg_empty; // 各段FIFO空标志
wire [SEG_CNT-1:0] write_fifo_seg_watermark; // 各段FIFO水位超限标志

wire write_fifo_full = |write_fifo_seg_full; // 任一段满即整体不可再写
wire write_fifo_empty = |write_fifo_seg_empty; // 任一段空则整体不可组成完整一拍输出

assign input_watermark = |write_fifo_seg_watermark | input_rst_out;

genvar n;
integer k; // 格雷码转二进制循环变量

generate

for (n = 0; n < SEG_CNT; n = n + 1) begin : write_fifo_seg

    reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] seg_wr_ptr_reg = 0; // 当前段写指针（二进制，input_clk域）
    reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] seg_wr_ptr_gray_reg = 0; // 当前段写指针格雷码

    reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] seg_wr_ptr_temp; // 当前段写指针下一值临时变量

    reg [WRITE_FIFO_ADDR_WIDTH+1-1:0] seg_occupancy_reg = 0; // 当前段FIFO占用深度

    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg [SEG_WIDTH-1:0] seg_mem_data[2**WRITE_FIFO_ADDR_WIDTH-1:0]; // 当前段分布式RAM，存储分段输入数据

    reg [SEG_WIDTH-1:0] seg_rd_data_reg = 0; // 当前段从RAM读出的数据，参与拼接input_data_int

    wire seg_full = seg_wr_ptr_gray_reg == (write_fifo_rd_ptr_gray_sync_2_reg ^ {2'b11, {WRITE_FIFO_ADDR_WIDTH-1{1'b0}}}); // 当前段满标志（格雷码环形满判定）
    wire seg_empty = write_fifo_rd_ptr_reg == write_fifo_wr_ptr_sync_reg; // 当前段空标志（读写指针相等）
    wire seg_watermark = seg_occupancy_reg > WATERMARK_LEVEL; // 当前段水位超阈值标志

    assign input_data_int[n*SEG_WIDTH +: SEG_WIDTH] = seg_rd_data_reg;

    assign input_ready[n] = !seg_full && !input_rst_out;

    assign write_fifo_seg_full[n] = seg_full;
    assign write_fifo_seg_empty[n] = seg_empty;
    assign write_fifo_seg_watermark[n] = seg_watermark;

    if (n == SEG_CNT-1) begin
        assign write_fifo_wr_ptr = seg_wr_ptr_reg;
        assign write_fifo_wr_ptr_gray = seg_wr_ptr_gray_reg;
    end

    // 分段写入逻辑
    always @(posedge input_clk) begin
        seg_occupancy_reg <= seg_wr_ptr_reg - write_fifo_rd_ptr_sync_reg;

        if (input_ready[n] && input_valid[n]) begin
            seg_mem_data[seg_wr_ptr_reg[WRITE_FIFO_ADDR_WIDTH-1:0]] <= input_data[n*SEG_WIDTH +: SEG_WIDTH];

            seg_wr_ptr_temp = seg_wr_ptr_reg + 1;
            seg_wr_ptr_reg <= seg_wr_ptr_temp;
            seg_wr_ptr_gray_reg <= seg_wr_ptr_temp ^ (seg_wr_ptr_temp >> 1);
        end

        if (input_rst || input_rst_out) begin
            seg_wr_ptr_reg <= 0;
            seg_wr_ptr_gray_reg <= 0;
        end
    end

    always @(posedge clk) begin
        if (!write_fifo_empty && (!input_valid_int_reg || input_read_en)) begin
            seg_rd_data_reg <= seg_mem_data[write_fifo_rd_ptr_reg[WRITE_FIFO_ADDR_WIDTH-1:0]];
        end
    end

end

endgenerate

// 指针同步逻辑
always @(posedge input_clk) begin
    write_fifo_rd_ptr_gray_sync_1_reg <= write_fifo_rd_ptr_gray_reg;
    write_fifo_rd_ptr_gray_sync_2_reg <= write_fifo_rd_ptr_gray_sync_1_reg;

    for (k = 0; k < WRITE_FIFO_ADDR_WIDTH+1; k = k + 1) begin
        write_fifo_rd_ptr_sync_reg[k] <= ^(write_fifo_rd_ptr_gray_sync_2_reg >> k);
    end

    if (input_rst || input_rst_out) begin
        write_fifo_rd_ptr_gray_sync_1_reg <= 0;
        write_fifo_rd_ptr_gray_sync_2_reg <= 0;
        write_fifo_rd_ptr_sync_reg <= 0;
    end
end

always @(posedge clk) begin
    write_fifo_wr_ptr_gray_sync_1_reg <= write_fifo_wr_ptr_gray;
    write_fifo_wr_ptr_gray_sync_2_reg <= write_fifo_wr_ptr_gray_sync_1_reg;

    for (k = 0; k < WRITE_FIFO_ADDR_WIDTH+1; k = k + 1) begin
        write_fifo_wr_ptr_sync_reg[k] <= ^(write_fifo_wr_ptr_gray_sync_2_reg >> k);
    end

    if (rst || cfg_reset) begin
        write_fifo_wr_ptr_gray_sync_1_reg <= 0;
        write_fifo_wr_ptr_gray_sync_2_reg <= 0;
        write_fifo_wr_ptr_sync_reg <= 0;
    end
end

// 读出拼接逻辑
always @(posedge clk) begin
    write_fifo_occupancy_reg <= write_fifo_wr_ptr_sync_reg - write_fifo_rd_ptr_reg + input_valid_int_reg;

    if (input_read_en) begin
        input_valid_int_reg <= 1'b0;
        write_fifo_occupancy_reg <= write_fifo_wr_ptr_sync_reg - write_fifo_rd_ptr_reg;
    end

    if (!write_fifo_empty && (!input_valid_int_reg || input_read_en)) begin
        input_valid_int_reg <= 1'b1;

        write_fifo_rd_ptr_temp = write_fifo_rd_ptr_reg + 1;
        write_fifo_rd_ptr_reg <= write_fifo_rd_ptr_temp;
        write_fifo_rd_ptr_gray_reg <= write_fifo_rd_ptr_temp ^ (write_fifo_rd_ptr_temp >> 1);

        write_fifo_occupancy_reg <= write_fifo_wr_ptr_sync_reg - write_fifo_rd_ptr_reg;
    end

    if (rst || cfg_reset) begin
        write_fifo_rd_ptr_reg <= 0;
        write_fifo_rd_ptr_gray_reg <= 0;
        input_valid_int_reg <= 1'b0;
    end
end

reg [WRITE_BURST_LEN_WIDTH+1-1:0] wr_burst_len; // 本轮组合逻辑计算的突发长度（拍数减 1）
reg [LEN_WIDTH+1-1:0] wr_start_ptr; // 本轮候选写起始指针（按当前占用推进后）
reg [LEN_WIDTH+1-1:0] wr_start_ptr_blk_adj; // 以突发块边界对齐后的候选写起始指针（用于满判断）
reg wr_burst_reg = 1'b0, wr_burst_next; // 当前是否正在发送一个写突发
reg [WRITE_BURST_LEN_WIDTH-1:0] wr_burst_len_reg = 0, wr_burst_len_next; // 当前写突发剩余长度计数
reg [7:0] wr_timeout_count_reg = 0, wr_timeout_count_next; // 部分突发超时计数器
reg wr_timeout_reg = 0, wr_timeout_next; // 超时触发标志，允许非满突发启动写出
reg fifo_full_wr_blk_adj_reg = 1'b0, fifo_full_wr_blk_adj_next; // 基于块对齐指针的FIFO满状态寄存

reg [LEN_WIDTH+1-1:0] wr_start_ptr_reg = 0, wr_start_ptr_next; // 已申请写出的逻辑起始指针
reg [LEN_WIDTH+1-1:0] wr_start_ptr_blk_adj_reg = 0, wr_start_ptr_blk_adj_next; // 已申请写出的块对齐指针
reg [LEN_WIDTH+1-1:0] wr_finish_ptr_reg = 0, wr_finish_ptr_next; // 已收到B响应确认完成的写指针

reg resp_fifo_we_reg = 1'b0, resp_fifo_we_next; // 写响应跟踪FIFO写使能
reg [RESP_FIFO_ADDR_WIDTH+1-1:0] resp_fifo_wr_ptr_reg = 0; // 写响应跟踪FIFO写指针
reg [RESP_FIFO_ADDR_WIDTH+1-1:0] resp_fifo_rd_ptr_reg = 0, resp_fifo_rd_ptr_next; // 写响应跟踪FIFO读指针
reg [WRITE_BURST_LEN_WIDTH+1-1:0] resp_fifo_burst_len[(2**RESP_FIFO_ADDR_WIDTH)-1:0]; // 写响应跟踪FIFO内容：每个已发AW的突发长度
reg [WRITE_BURST_LEN_WIDTH+1-1:0] resp_fifo_wr_burst_len_reg = 0, resp_fifo_wr_burst_len_next; // 将写入响应FIFO的突发长度寄存

assign wr_start_ptr_out = wr_start_ptr_reg;
assign wr_finish_ptr_out = wr_finish_ptr_reg;

// 使用块对齐写起点计算 FIFO 占用量
wire [LEN_WIDTH+1-1:0] fifo_occupancy_wr_blk_adj = wr_start_ptr_blk_adj_reg - rd_finish_ptr_in; // 以块对齐写指针估计的占用
// FIFO 满判定：是否还有空间启动一个完整写块
wire fifo_full_wr_blk_adj = (fifo_occupancy_wr_blk_adj & ~cfg_fifo_size_mask) || ((~fifo_occupancy_wr_blk_adj & cfg_fifo_size_mask & ~WRITE_BURST_ADDR_MASK) == 0 && (fifo_occupancy_wr_blk_adj & WRITE_BURST_ADDR_MASK)); // 预留整突发空间后是否判满

// FIFO 占用量（包含在途读写）
assign sts_fifo_occupancy = wr_start_ptr_reg - rd_finish_ptr_in;
// FIFO 空判定（包含在途读写）
assign sts_fifo_empty = wr_start_ptr_reg == rd_finish_ptr_in;
// FIFO 满判定
assign sts_fifo_full = fifo_full_wr_blk_adj_reg;

assign sts_write_active = wr_burst_reg || resp_fifo_we_reg || (resp_fifo_wr_ptr_reg != resp_fifo_rd_ptr_reg);

// 写控制逻辑
always @* begin
    wr_start_ptr_next = wr_start_ptr_reg;
    wr_start_ptr_blk_adj_next = wr_start_ptr_blk_adj_reg;
    wr_finish_ptr_next = wr_finish_ptr_reg;

    wr_burst_next = wr_burst_reg;
    wr_burst_len_next = wr_burst_len_reg;
    wr_timeout_count_next = wr_timeout_count_reg;
    wr_timeout_next = wr_timeout_reg;

    fifo_full_wr_blk_adj_next = fifo_full_wr_blk_adj;

    resp_fifo_we_next = 1'b0;
    resp_fifo_rd_ptr_next = resp_fifo_rd_ptr_reg;
    resp_fifo_wr_burst_len_next = wr_burst_len_reg;

    input_read_en = 1'b0;

    m_axi_awaddr_next = m_axi_awaddr_reg;
    m_axi_awlen_next = m_axi_awlen_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;

    m_axi_wdata_next = m_axi_wdata_reg;
    m_axi_wstrb_next = m_axi_wstrb_reg;
    m_axi_wlast_next = m_axi_wlast_reg;
    m_axi_wvalid_next = m_axi_wvalid_reg && !m_axi_wready;

    m_axi_bready_next = 1'b0;

    // 非满突发超时处理
    wr_timeout_next = wr_timeout_count_reg == 0;
    if (!input_valid_int_reg || m_axi_awvalid) begin
        wr_timeout_count_next = 8'hff;
        wr_timeout_next = 1'b0;
    end else if (wr_timeout_count_reg > 0) begin
        wr_timeout_count_next = wr_timeout_count_reg - 1;
    end

    // 按输入 FIFO 占用量计算本次突发长度
    if ((((wr_start_ptr_reg & WRITE_BURST_ADDR_MASK) >> AXI_BURST_SIZE) + write_fifo_occupancy_reg) >> WRITE_BURST_LEN_WIDTH != 0) begin
        // 跨越突发边界：写到边界为止
        wr_burst_len = WRITE_MAX_BURST_LEN_INT-1 - ((wr_start_ptr_reg & WRITE_BURST_ADDR_MASK) >> AXI_BURST_SIZE);
        wr_start_ptr = (wr_start_ptr_reg & ~WRITE_BURST_ADDR_MASK) + (1 << WRITE_BURST_ADDR_WIDTH);
        wr_start_ptr_blk_adj = (wr_start_ptr_reg & ~WRITE_BURST_ADDR_MASK) + (1 << WRITE_BURST_ADDR_WIDTH);
    end else begin
        // 未跨越突发边界：写入当前可用数据
        wr_burst_len = write_fifo_occupancy_reg-1;
        wr_start_ptr = wr_start_ptr_reg + (write_fifo_occupancy_reg << AXI_BURST_SIZE);
        wr_start_ptr_blk_adj = (wr_start_ptr_reg & ~WRITE_BURST_ADDR_MASK) + (1 << WRITE_BURST_ADDR_WIDTH);
    end

    resp_fifo_wr_burst_len_next = wr_burst_len;

    // 生成 AXI 写突发
    if (!m_axi_awvalid_reg && !wr_burst_reg) begin
        // 可以启动新突发

        wr_burst_len_next = wr_burst_len;

        m_axi_awaddr_next = cfg_fifo_base_addr + (wr_start_ptr_reg & cfg_fifo_size_mask);
        m_axi_awlen_next = wr_burst_len;

        if (cfg_enable && input_valid_int_reg && !fifo_full_wr_blk_adj_reg) begin
            // 已使能，且有数据可写，同时有空间可写
            if ((write_fifo_occupancy_reg) >> WRITE_BURST_LEN_WIDTH != 0 || wr_timeout_reg) begin
                // 满突发可发起，或已超时允许发起非满突发
                wr_burst_next = 1'b1;
                m_axi_awvalid_next = 1'b1;
                resp_fifo_we_next = 1'b1;
                wr_start_ptr_next = wr_start_ptr;
                wr_start_ptr_blk_adj_next = wr_start_ptr_blk_adj;
            end
        end
    end

    if (!m_axi_wvalid_reg || m_axi_wready) begin
        // 传输写数据
        m_axi_wdata_next = input_data_int;
        m_axi_wlast_next = wr_burst_len_reg == 0;

        if (wr_burst_reg) begin
            m_axi_wstrb_next = {AXI_STRB_WIDTH{1'b1}};
            if (cfg_reset) begin
                m_axi_wstrb_next = 0;
                m_axi_wvalid_next = 1'b1;
                wr_burst_len_next = wr_burst_len_reg - 1;
                wr_burst_next = wr_burst_len_reg != 0;
            end else if (input_valid_int_reg) begin
                input_read_en = 1'b1;
                m_axi_wvalid_next = 1'b1;
                wr_burst_len_next = wr_burst_len_reg - 1;
                wr_burst_next = wr_burst_len_reg != 0;
            end
        end
    end

    // 处理 AXI 写完成
    m_axi_bready_next = 1'b1;
    if (m_axi_bvalid) begin
        wr_finish_ptr_next = wr_finish_ptr_reg + ((resp_fifo_burst_len[resp_fifo_rd_ptr_reg[RESP_FIFO_ADDR_WIDTH-1:0]]+1) << AXI_BURST_SIZE);
        resp_fifo_rd_ptr_next = resp_fifo_rd_ptr_reg + 1;
    end

    if (cfg_reset) begin
        wr_start_ptr_next = 0;
        wr_start_ptr_blk_adj_next = 0;
        wr_finish_ptr_next = 0;
    end
end

always @(posedge clk) begin
    wr_start_ptr_reg <= wr_start_ptr_next;
    wr_start_ptr_blk_adj_reg <= wr_start_ptr_blk_adj_next;
    wr_finish_ptr_reg <= wr_finish_ptr_next;

    wr_burst_reg <= wr_burst_next;
    wr_burst_len_reg <= wr_burst_len_next;
    wr_timeout_count_reg <= wr_timeout_count_next;
    wr_timeout_reg <= wr_timeout_next;
    fifo_full_wr_blk_adj_reg <= fifo_full_wr_blk_adj_next;

    m_axi_awaddr_reg <= m_axi_awaddr_next;
    m_axi_awlen_reg <= m_axi_awlen_next;
    m_axi_awvalid_reg <= m_axi_awvalid_next;

    m_axi_wdata_reg <= m_axi_wdata_next;
    m_axi_wstrb_reg <= m_axi_wstrb_next;
    m_axi_wlast_reg <= m_axi_wlast_next;
    m_axi_wvalid_reg <= m_axi_wvalid_next;

    m_axi_bready_reg <= m_axi_bready_next;

    resp_fifo_we_reg <= resp_fifo_we_next;
    resp_fifo_wr_burst_len_reg <= resp_fifo_wr_burst_len_next;

    if (resp_fifo_we_reg) begin
        resp_fifo_burst_len[resp_fifo_wr_ptr_reg[RESP_FIFO_ADDR_WIDTH-1:0]] <= resp_fifo_wr_burst_len_reg;
        resp_fifo_wr_ptr_reg <= resp_fifo_wr_ptr_reg + 1;
    end
    resp_fifo_rd_ptr_reg <= resp_fifo_rd_ptr_next;

    if (rst) begin
        wr_burst_reg <= 1'b0;
        m_axi_awvalid_reg <= 1'b0;
        m_axi_wvalid_reg <= 1'b0;
        m_axi_bready_reg <= 1'b0;
        resp_fifo_we_reg <= 1'b0;
        resp_fifo_wr_ptr_reg <= 0;
        resp_fifo_rd_ptr_reg <= 0;
    end
end

endmodule

`resetall
