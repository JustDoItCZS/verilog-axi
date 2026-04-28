/*

Copyright (c) 2018 Alex Forencich

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
 * AXI4 RAM 存储模块
 *
 * 模块目录
 * 1) 写 FSM：接收 AW，按拍写入 W 数据，返回 B 响应。
 * 2) 读 FSM：接收 AR，按拍从存储阵列输出 R 数据。
 * 3) 使用带字节 lane 写掩码的共享存储阵列。
 * 4) R 通道支持可选输出流水寄存器。
 */
module axi_ram #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 输出端额外流水寄存器开关
    parameter PIPELINE_OUTPUT = 0
)
(
    input  wire                   clk, // AXI 状态机与存储访问核心时钟。
    input  wire                   rst, // 读写通道状态同步复位。

    input  wire [ID_WIDTH-1:0]    s_axi_awid, // 生成写响应时锁存的 AW ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr, // AW 起始地址。
    input  wire [7:0]             s_axi_awlen, // AW 突发长度。
    input  wire [2:0]             s_axi_awsize, // AW 突发尺寸。
    input  wire [1:0]             s_axi_awburst, // AW 突发类型。
    input  wire                   s_axi_awlock, // AW 锁属性（接收但 RAM 模型不使用）。
    input  wire [3:0]             s_axi_awcache, // AW cache 属性（接收但内部不使用）。
    input  wire [2:0]             s_axi_awprot, // AW 保护属性（接收但内部不使用）。
    input  wire                   s_axi_awvalid, // 主机侧 AWVALID。
    output wire                   s_axi_awready, // RAM 写 FSM 返回 AWREADY。
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata, // W 数据载荷。
    input  wire [STRB_WIDTH-1:0]  s_axi_wstrb, // W 字节使能（lane 级写掩码）。
    input  wire                   s_axi_wlast, // WLAST 末拍标志。
    input  wire                   s_axi_wvalid, // 主机侧 WVALID。
    output wire                   s_axi_wready, // RAM 写 FSM 返回 WREADY。
    output wire [ID_WIDTH-1:0]    s_axi_bid, // 写突发完成后返回 BID。
    output wire [1:0]             s_axi_bresp, // BRESP（固定 OKAY）。
    output wire                   s_axi_bvalid, // 写响应可用时拉高 BVALID。
    input  wire                   s_axi_bready, // 主机侧 BREADY。
    input  wire [ID_WIDTH-1:0]    s_axi_arid, // 读响应流使用的 AR ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr, // AR 起始地址。
    input  wire [7:0]             s_axi_arlen, // AR 突发长度。
    input  wire [2:0]             s_axi_arsize, // AR 突发尺寸。
    input  wire [1:0]             s_axi_arburst, // AR 突发类型。
    input  wire                   s_axi_arlock, // AR 锁属性（接收但 RAM 模型不使用）。
    input  wire [3:0]             s_axi_arcache, // AR cache 属性（接收但内部不使用）。
    input  wire [2:0]             s_axi_arprot, // AR 保护属性（接收但内部不使用）。
    input  wire                   s_axi_arvalid, // 主机侧 ARVALID。
    output wire                   s_axi_arready, // RAM 读 FSM 返回 ARREADY。
    output wire [ID_WIDTH-1:0]    s_axi_rid, // 当前读数据拍 RID。
    output wire [DATA_WIDTH-1:0]  s_axi_rdata, // 当前读数据拍 RDATA。
    output wire [1:0]             s_axi_rresp, // RRESP（固定 OKAY）。
    output wire                   s_axi_rlast, // 突发最后一拍 RLAST。
    output wire                   s_axi_rvalid, // 读数据通道 RVALID。
    input  wire                   s_axi_rready // 主机侧 RREADY。
);

parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH); // 去掉字节 lane 位后的字地址位宽。
parameter WORD_WIDTH = STRB_WIDTH; // 每个存储字包含的字节 lane 数量。
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH; // 每个字节 lane 段对应位宽。

// 总线位宽断言检查
initial begin
    if (WORD_SIZE * STRB_WIDTH != DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisble (instance %m)");
        $finish;
    end

    if (2**$clog2(WORD_WIDTH) != WORD_WIDTH) begin
        $error("Error: AXI word width must be even power of two (instance %m)");
        $finish;
    end
end

localparam [0:0]
    READ_STATE_IDLE = 1'd0, // 等待 AR 握手。
    READ_STATE_BURST = 1'd1; // 为当前突发连续输出读数据拍。

reg [0:0] read_state_reg = READ_STATE_IDLE, read_state_next; // 读 FSM 状态寄存器。

localparam [1:0]
    WRITE_STATE_IDLE = 2'd0, // 等待 AW 握手。
    WRITE_STATE_BURST = 2'd1, // 接收并写入 W 数据拍。
    WRITE_STATE_RESP = 2'd2; // 发生反压时等待发出 B 响应。

reg [1:0] write_state_reg = WRITE_STATE_IDLE, write_state_next; // 写 FSM 状态寄存器。

reg mem_wr_en; // 存储阵列一拍写使能脉冲。
reg mem_rd_en; // 存储阵列一拍读使能脉冲。

reg [ID_WIDTH-1:0] read_id_reg = {ID_WIDTH{1'b0}}, read_id_next; // 当前读突发 ID。
reg [ADDR_WIDTH-1:0] read_addr_reg = {ADDR_WIDTH{1'b0}}, read_addr_next; // 当前读地址指针。
reg [7:0] read_count_reg = 8'd0, read_count_next; // 剩余待读拍计数。
reg [2:0] read_size_reg = 3'd0, read_size_next; // 有效读地址步进尺寸。
reg [1:0] read_burst_reg = 2'd0, read_burst_next; // 读突发类型。
reg [ID_WIDTH-1:0] write_id_reg = {ID_WIDTH{1'b0}}, write_id_next; // 当前写突发 ID。
reg [ADDR_WIDTH-1:0] write_addr_reg = {ADDR_WIDTH{1'b0}}, write_addr_next; // 当前写地址指针。
reg [7:0] write_count_reg = 8'd0, write_count_next; // 剩余待写拍计数。
reg [2:0] write_size_reg = 3'd0, write_size_next; // 有效写地址步进尺寸。
reg [1:0] write_burst_reg = 2'd0, write_burst_next; // 写突发类型。

reg s_axi_awready_reg = 1'b0, s_axi_awready_next; // AWREADY 寄存器。
reg s_axi_wready_reg = 1'b0, s_axi_wready_next; // WREADY 寄存器。
reg [ID_WIDTH-1:0] s_axi_bid_reg = {ID_WIDTH{1'b0}}, s_axi_bid_next; // BID 寄存器。
reg s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // BVALID 寄存器。
reg s_axi_arready_reg = 1'b0, s_axi_arready_next; // ARREADY 寄存器。
reg [ID_WIDTH-1:0] s_axi_rid_reg = {ID_WIDTH{1'b0}}, s_axi_rid_next; // RID 寄存器。
reg [DATA_WIDTH-1:0] s_axi_rdata_reg = {DATA_WIDTH{1'b0}}, s_axi_rdata_next; // RDATA 寄存器。
reg s_axi_rlast_reg = 1'b0, s_axi_rlast_next; // RLAST 寄存器。
reg s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // RVALID 寄存器。
reg [ID_WIDTH-1:0] s_axi_rid_pipe_reg = {ID_WIDTH{1'b0}}; // 可选流水 RID 寄存器。
reg [DATA_WIDTH-1:0] s_axi_rdata_pipe_reg = {DATA_WIDTH{1'b0}}; // 可选流水 RDATA 寄存器。
reg s_axi_rlast_pipe_reg = 1'b0; // 可选流水 RLAST 寄存器。
reg s_axi_rvalid_pipe_reg = 1'b0; // 可选流水 RVALID 寄存器。

// RAM 风格属性示例：(* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[(2**VALID_ADDR_WIDTH)-1:0]; // 主存储阵列。

wire [VALID_ADDR_WIDTH-1:0] s_axi_awaddr_valid = s_axi_awaddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 输入 AW 对应字地址索引。
wire [VALID_ADDR_WIDTH-1:0] s_axi_araddr_valid = s_axi_araddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 输入 AR 对应字地址索引。
wire [VALID_ADDR_WIDTH-1:0] read_addr_valid = read_addr_reg >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 读 FSM 当前字地址索引。
wire [VALID_ADDR_WIDTH-1:0] write_addr_valid = write_addr_reg >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 写 FSM 当前字地址索引。

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = s_axi_wready_reg;
assign s_axi_bid = s_axi_bid_reg;
assign s_axi_bresp = 2'b00;
assign s_axi_bvalid = s_axi_bvalid_reg;
assign s_axi_arready = s_axi_arready_reg;
assign s_axi_rid = PIPELINE_OUTPUT ? s_axi_rid_pipe_reg : s_axi_rid_reg;
assign s_axi_rdata = PIPELINE_OUTPUT ? s_axi_rdata_pipe_reg : s_axi_rdata_reg;
assign s_axi_rresp = 2'b00;
assign s_axi_rlast = PIPELINE_OUTPUT ? s_axi_rlast_pipe_reg : s_axi_rlast_reg;
assign s_axi_rvalid = PIPELINE_OUTPUT ? s_axi_rvalid_pipe_reg : s_axi_rvalid_reg;

integer i, j; // 初始化与字节 lane 写入循环变量。

initial begin
    // 使用两层嵌套循环，降低单层循环迭代次数
    // 规避综合器对大循环计数的告警
    for (i = 0; i < 2**VALID_ADDR_WIDTH; i = i + 2**(VALID_ADDR_WIDTH/2)) begin
        for (j = i; j < i + 2**(VALID_ADDR_WIDTH/2); j = j + 1) begin
            mem[j] = 0;
        end
    end
end

always @* begin
    write_state_next = WRITE_STATE_IDLE;

    mem_wr_en = 1'b0;

    write_id_next = write_id_reg;
    write_addr_next = write_addr_reg;
    write_count_next = write_count_reg;
    write_size_next = write_size_reg;
    write_burst_next = write_burst_reg;

    s_axi_awready_next = 1'b0;
    s_axi_wready_next = 1'b0;
    s_axi_bid_next = s_axi_bid_reg;
    s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;

    case (write_state_reg)
        WRITE_STATE_IDLE: begin
            s_axi_awready_next = 1'b1;

            if (s_axi_awready && s_axi_awvalid) begin
                write_id_next = s_axi_awid;
                write_addr_next = s_axi_awaddr;
                write_count_next = s_axi_awlen;
                write_size_next = s_axi_awsize < $clog2(STRB_WIDTH) ? s_axi_awsize : $clog2(STRB_WIDTH);
                write_burst_next = s_axi_awburst;

                s_axi_awready_next = 1'b0;
                s_axi_wready_next = 1'b1;
                write_state_next = WRITE_STATE_BURST;
            end else begin
                write_state_next = WRITE_STATE_IDLE;
            end
        end
        WRITE_STATE_BURST: begin
            s_axi_wready_next = 1'b1;

            if (s_axi_wready && s_axi_wvalid) begin
                mem_wr_en = 1'b1;
                if (write_burst_reg != 2'b00) begin
                    write_addr_next = write_addr_reg + (1 << write_size_reg);
                end
                write_count_next = write_count_reg - 1;
                if (write_count_reg > 0) begin
                    write_state_next = WRITE_STATE_BURST;
                end else begin
                    s_axi_wready_next = 1'b0;
                    if (s_axi_bready || !s_axi_bvalid) begin
                        s_axi_bid_next = write_id_reg;
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = 1'b1;
                        write_state_next = WRITE_STATE_IDLE;
                    end else begin
                        write_state_next = WRITE_STATE_RESP;
                    end
                end
            end else begin
                write_state_next = WRITE_STATE_BURST;
            end
        end
        WRITE_STATE_RESP: begin
            if (s_axi_bready || !s_axi_bvalid) begin
                s_axi_bid_next = write_id_reg;
                s_axi_bvalid_next = 1'b1;
                s_axi_awready_next = 1'b1;
                write_state_next = WRITE_STATE_IDLE;
            end else begin
                write_state_next = WRITE_STATE_RESP;
            end
        end
    endcase
end

always @(posedge clk) begin
    write_state_reg <= write_state_next;

    write_id_reg <= write_id_next;
    write_addr_reg <= write_addr_next;
    write_count_reg <= write_count_next;
    write_size_reg <= write_size_next;
    write_burst_reg <= write_burst_next;

    s_axi_awready_reg <= s_axi_awready_next;
    s_axi_wready_reg <= s_axi_wready_next;
    s_axi_bid_reg <= s_axi_bid_next;
    s_axi_bvalid_reg <= s_axi_bvalid_next;

    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
        if (mem_wr_en & s_axi_wstrb[i]) begin
            mem[write_addr_valid][WORD_SIZE*i +: WORD_SIZE] <= s_axi_wdata[WORD_SIZE*i +: WORD_SIZE];
        end
    end

    if (rst) begin
        write_state_reg <= WRITE_STATE_IDLE;

        s_axi_awready_reg <= 1'b0;
        s_axi_wready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;
    end
end

always @* begin
    read_state_next = READ_STATE_IDLE;

    mem_rd_en = 1'b0;

    s_axi_rid_next = s_axi_rid_reg;
    s_axi_rlast_next = s_axi_rlast_reg;
    s_axi_rvalid_next = s_axi_rvalid_reg && !(s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg));

    read_id_next = read_id_reg;
    read_addr_next = read_addr_reg;
    read_count_next = read_count_reg;
    read_size_next = read_size_reg;
    read_burst_next = read_burst_reg;

    s_axi_arready_next = 1'b0;

    case (read_state_reg)
        READ_STATE_IDLE: begin
            s_axi_arready_next = 1'b1;

            if (s_axi_arready && s_axi_arvalid) begin
                read_id_next = s_axi_arid;
                read_addr_next = s_axi_araddr;
                read_count_next = s_axi_arlen;
                read_size_next = s_axi_arsize < $clog2(STRB_WIDTH) ? s_axi_arsize : $clog2(STRB_WIDTH);
                read_burst_next = s_axi_arburst;

                s_axi_arready_next = 1'b0;
                read_state_next = READ_STATE_BURST;
            end else begin
                read_state_next = READ_STATE_IDLE;
            end
        end
        READ_STATE_BURST: begin
            if (s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg) || !s_axi_rvalid_reg) begin
                mem_rd_en = 1'b1;
                s_axi_rvalid_next = 1'b1;
                s_axi_rid_next = read_id_reg;
                s_axi_rlast_next = read_count_reg == 0;
                if (read_burst_reg != 2'b00) begin
                    read_addr_next = read_addr_reg + (1 << read_size_reg);
                end
                read_count_next = read_count_reg - 1;
                if (read_count_reg > 0) begin
                    read_state_next = READ_STATE_BURST;
                end else begin
                    s_axi_arready_next = 1'b1;
                    read_state_next = READ_STATE_IDLE;
                end
            end else begin
                read_state_next = READ_STATE_BURST;
            end
        end
    endcase
end

always @(posedge clk) begin
    read_state_reg <= read_state_next;

    read_id_reg <= read_id_next;
    read_addr_reg <= read_addr_next;
    read_count_reg <= read_count_next;
    read_size_reg <= read_size_next;
    read_burst_reg <= read_burst_next;

    s_axi_arready_reg <= s_axi_arready_next;
    s_axi_rid_reg <= s_axi_rid_next;
    s_axi_rlast_reg <= s_axi_rlast_next;
    s_axi_rvalid_reg <= s_axi_rvalid_next;

    if (mem_rd_en) begin
        s_axi_rdata_reg <= mem[read_addr_valid];
    end

    if (!s_axi_rvalid_pipe_reg || s_axi_rready) begin
        s_axi_rid_pipe_reg <= s_axi_rid_reg;
        s_axi_rdata_pipe_reg <= s_axi_rdata_reg;
        s_axi_rlast_pipe_reg <= s_axi_rlast_reg;
        s_axi_rvalid_pipe_reg <= s_axi_rvalid_reg;
    end

    if (rst) begin
        read_state_reg <= READ_STATE_IDLE;

        s_axi_arready_reg <= 1'b0;
        s_axi_rvalid_reg <= 1'b0;
        s_axi_rvalid_pipe_reg <= 1'b0;
    end
end

endmodule

`resetall
