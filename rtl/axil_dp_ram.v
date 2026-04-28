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
 * AXI4-Lite 双端口 RAM
 *
 * 模块目录
 * 1) A 端口与 B 端口分别在独立时钟域提供完整 AXI-Lite 从接口。
 * 2) 两个端口访问同一片共享存储阵列；各端口本地在读写同时可执行时进行仲裁。
 * 3) `last_read_*` 用于公平性偏好切换，使竞争时读写机会交替。
 */
module axil_dp_ram #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 输出端额外流水寄存器开关
    parameter PIPELINE_OUTPUT = 0
)
(
    input  wire                   a_clk, // AXI-Lite A 端口状态机与存储访问时钟。
    input  wire                   a_rst, // A 端口时钟域同步复位。

    input  wire                   b_clk, // AXI-Lite B 端口状态机与存储访问时钟。
    input  wire                   b_rst, // B 端口时钟域同步复位。

    input  wire [ADDR_WIDTH-1:0]  s_axil_a_awaddr, // A 端口写地址。
    input  wire [2:0]             s_axil_a_awprot, // A 端口 AW 保护属性（接收但不使用）。
    input  wire                   s_axil_a_awvalid, // A 端口 AWVALID。
    output wire                   s_axil_a_awready, // A 端口 AWREADY；写事务接纳时脉冲拉高。
    input  wire [DATA_WIDTH-1:0]  s_axil_a_wdata, // A 端口写数据载荷。
    input  wire [STRB_WIDTH-1:0]  s_axil_a_wstrb, // A 端口写掩码字节使能。
    input  wire                   s_axil_a_wvalid, // A 端口 WVALID。
    output wire                   s_axil_a_wready, // A 端口 WREADY；写接纳时脉冲拉高。
    output wire [1:0]             s_axil_a_bresp, // A 端口 BRESP；固定 OKAY。
    output wire                   s_axil_a_bvalid, // A 端口 BVALID；写接纳后拉高。
    input  wire                   s_axil_a_bready, // A 端口 BREADY。
    input  wire [ADDR_WIDTH-1:0]  s_axil_a_araddr, // A 端口读地址。
    input  wire [2:0]             s_axil_a_arprot, // A 端口 AR 保护属性（接收但不使用）。
    input  wire                   s_axil_a_arvalid, // A 端口 ARVALID。
    output wire                   s_axil_a_arready, // A 端口 ARREADY；读请求接纳时脉冲拉高。
    output wire [DATA_WIDTH-1:0]  s_axil_a_rdata, // A 端口读数据。
    output wire [1:0]             s_axil_a_rresp, // A 端口 RRESP；固定 OKAY。
    output wire                   s_axil_a_rvalid, // A 端口 RVALID；直到被消费前保持。
    input  wire                   s_axil_a_rready, // A 端口 RREADY。

    input  wire [ADDR_WIDTH-1:0]  s_axil_b_awaddr, // B 端口写地址。
    input  wire [2:0]             s_axil_b_awprot, // B 端口 AW 保护属性（接收但不使用）。
    input  wire                   s_axil_b_awvalid, // B 端口 AWVALID。
    output wire                   s_axil_b_awready, // B 端口 AWREADY；写事务接纳时脉冲拉高。
    input  wire [DATA_WIDTH-1:0]  s_axil_b_wdata, // B 端口写数据载荷。
    input  wire [STRB_WIDTH-1:0]  s_axil_b_wstrb, // B 端口写掩码字节使能。
    input  wire                   s_axil_b_wvalid, // B 端口 WVALID。
    output wire                   s_axil_b_wready, // B 端口 WREADY；写接纳时脉冲拉高。
    output wire [1:0]             s_axil_b_bresp, // B 端口 BRESP；固定 OKAY。
    output wire                   s_axil_b_bvalid, // B 端口 BVALID；写接纳后拉高。
    input  wire                   s_axil_b_bready, // B 端口 BREADY。
    input  wire [ADDR_WIDTH-1:0]  s_axil_b_araddr, // B 端口读地址。
    input  wire [2:0]             s_axil_b_arprot, // B 端口 AR 保护属性（接收但不使用）。
    input  wire                   s_axil_b_arvalid, // B 端口 ARVALID。
    output wire                   s_axil_b_arready, // B 端口 ARREADY；读请求接纳时脉冲拉高。
    output wire [DATA_WIDTH-1:0]  s_axil_b_rdata, // B 端口读数据。
    output wire [1:0]             s_axil_b_rresp, // B 端口 RRESP；固定 OKAY。
    output wire                   s_axil_b_rvalid, // B 端口 RVALID；直到被消费前保持。
    input  wire                   s_axil_b_rready // B 端口 RREADY。
);

parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH); // 去掉字节 lane 低位后的字地址位宽。
parameter WORD_WIDTH = STRB_WIDTH; // 每个存储字包含的字节 lane 数量。
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH; // 每个写 strobe lane 对应位宽。

reg read_eligible_a; // A 端口组合标志：本拍可接纳 AR 事务。
reg write_eligible_a; // A 端口组合标志：本拍可接纳 AW+W 配对写事务。

reg read_eligible_b; // B 端口组合标志：本拍可接纳 AR 事务。
reg write_eligible_b; // B 端口组合标志：本拍可接纳 AW+W 配对写事务。

reg mem_wr_en_a; // A 端口对存储阵列的一拍写使能脉冲。
reg mem_rd_en_a; // A 端口对存储阵列的一拍读使能脉冲。

reg mem_wr_en_b; // B 端口对存储阵列的一拍写使能脉冲。
reg mem_rd_en_b; // B 端口对存储阵列的一拍读使能脉冲。

reg last_read_a_reg = 1'b0, last_read_a_next; // A 端口公平性提示：记住上次接纳操作是否为读。
reg last_read_b_reg = 1'b0, last_read_b_next; // B 端口公平性提示：记住上次接纳操作是否为读。

reg s_axil_a_awready_reg = 1'b0, s_axil_a_awready_next; // A 端口 AWREADY 状态寄存器。
reg s_axil_a_wready_reg = 1'b0, s_axil_a_wready_next; // A 端口 WREADY 状态寄存器。
reg s_axil_a_bvalid_reg = 1'b0, s_axil_a_bvalid_next; // A 端口 BVALID 状态寄存器。
reg s_axil_a_arready_reg = 1'b0, s_axil_a_arready_next; // A 端口 ARREADY 状态寄存器。
reg [DATA_WIDTH-1:0] s_axil_a_rdata_reg = {DATA_WIDTH{1'b0}}, s_axil_a_rdata_next; // A 端口读数据寄存器。
reg s_axil_a_rvalid_reg = 1'b0, s_axil_a_rvalid_next; // A 端口 RVALID 状态寄存器。
reg [DATA_WIDTH-1:0] s_axil_a_rdata_pipe_reg = {DATA_WIDTH{1'b0}}; // A 端口可选输出流水数据寄存器。
reg s_axil_a_rvalid_pipe_reg = 1'b0; // A 端口可选输出流水 valid 寄存器。

reg s_axil_b_awready_reg = 1'b0, s_axil_b_awready_next; // B 端口 AWREADY 状态寄存器。
reg s_axil_b_wready_reg = 1'b0, s_axil_b_wready_next; // B 端口 WREADY 状态寄存器。
reg s_axil_b_bvalid_reg = 1'b0, s_axil_b_bvalid_next; // B 端口 BVALID 状态寄存器。
reg s_axil_b_arready_reg = 1'b0, s_axil_b_arready_next; // B 端口 ARREADY 状态寄存器。
reg [DATA_WIDTH-1:0] s_axil_b_rdata_reg = {DATA_WIDTH{1'b0}}, s_axil_b_rdata_next; // B 端口读数据寄存器。
reg s_axil_b_rvalid_reg = 1'b0, s_axil_b_rvalid_next; // B 端口 RVALID 状态寄存器。
reg [DATA_WIDTH-1:0] s_axil_b_rdata_pipe_reg = {DATA_WIDTH{1'b0}}; // B 端口可选输出流水数据寄存器。
reg s_axil_b_rvalid_pipe_reg = 1'b0; // B 端口可选输出流水 valid 寄存器。

// RAM 风格属性示例：(* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[(2**VALID_ADDR_WIDTH)-1:0]; // 共享双端口访问存储阵列。

wire [VALID_ADDR_WIDTH-1:0] s_axil_a_awaddr_valid = s_axil_a_awaddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // A 端口按字对齐写索引。
wire [VALID_ADDR_WIDTH-1:0] s_axil_a_araddr_valid = s_axil_a_araddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // A 端口按字对齐读索引。

wire [VALID_ADDR_WIDTH-1:0] s_axil_b_awaddr_valid = s_axil_b_awaddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // B 端口按字对齐写索引。
wire [VALID_ADDR_WIDTH-1:0] s_axil_b_araddr_valid = s_axil_b_araddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // B 端口按字对齐读索引。

assign s_axil_a_awready = s_axil_a_awready_reg;
assign s_axil_a_wready = s_axil_a_wready_reg;
assign s_axil_a_bresp = 2'b00;
assign s_axil_a_bvalid = s_axil_a_bvalid_reg;
assign s_axil_a_arready = s_axil_a_arready_reg;
assign s_axil_a_rdata = PIPELINE_OUTPUT ? s_axil_a_rdata_pipe_reg : s_axil_a_rdata_reg;
assign s_axil_a_rresp = 2'b00;
assign s_axil_a_rvalid = PIPELINE_OUTPUT ? s_axil_a_rvalid_pipe_reg : s_axil_a_rvalid_reg;

assign s_axil_b_awready = s_axil_b_awready_reg;
assign s_axil_b_wready = s_axil_b_wready_reg;
assign s_axil_b_bresp = 2'b00;
assign s_axil_b_bvalid = s_axil_b_bvalid_reg;
assign s_axil_b_arready = s_axil_b_arready_reg;
assign s_axil_b_rdata = PIPELINE_OUTPUT ? s_axil_b_rdata_pipe_reg : s_axil_b_rdata_reg;
assign s_axil_b_rresp = 2'b00;
assign s_axil_b_rvalid = PIPELINE_OUTPUT ? s_axil_b_rvalid_pipe_reg : s_axil_b_rvalid_reg;

integer i, j; // 存储初始化与字节 lane 更新循环变量。

initial begin
    // 采用两层嵌套循环，降低单层循环迭代次数
    // 规避综合器对超大循环计数的告警
    for (i = 0; i < 2**VALID_ADDR_WIDTH; i = i + 2**(VALID_ADDR_WIDTH/2)) begin
        for (j = i; j < i + 2**(VALID_ADDR_WIDTH/2); j = j + 1) begin
            mem[j] = 0;
        end
    end
end

always @* begin
    mem_wr_en_a = 1'b0;
    mem_rd_en_a = 1'b0;

    last_read_a_next = last_read_a_reg;

    s_axil_a_awready_next = 1'b0;
    s_axil_a_wready_next = 1'b0;
    s_axil_a_bvalid_next = s_axil_a_bvalid_reg && !s_axil_a_bready;

    s_axil_a_arready_next = 1'b0;
    s_axil_a_rvalid_next = s_axil_a_rvalid_reg && !(s_axil_a_rready || (PIPELINE_OUTPUT && !s_axil_a_rvalid_pipe_reg));

    write_eligible_a = s_axil_a_awvalid && s_axil_a_wvalid && (!s_axil_a_bvalid || s_axil_a_bready) && (!s_axil_a_awready && !s_axil_a_wready);
    read_eligible_a = s_axil_a_arvalid && (!s_axil_a_rvalid || s_axil_a_rready || (PIPELINE_OUTPUT && !s_axil_a_rvalid_pipe_reg)) && (!s_axil_a_arready);

    if (write_eligible_a && (!read_eligible_a || last_read_a_reg)) begin
        last_read_a_next = 1'b0;

        s_axil_a_awready_next = 1'b1;
        s_axil_a_wready_next = 1'b1;
        s_axil_a_bvalid_next = 1'b1;

        mem_wr_en_a = 1'b1;
    end else if (read_eligible_a) begin
        last_read_a_next = 1'b1;

        s_axil_a_arready_next = 1'b1;
        s_axil_a_rvalid_next = 1'b1;

        mem_rd_en_a = 1'b1;
    end
end

always @(posedge a_clk) begin
    last_read_a_reg <= last_read_a_next;

    s_axil_a_awready_reg <= s_axil_a_awready_next;
    s_axil_a_wready_reg <= s_axil_a_wready_next;
    s_axil_a_bvalid_reg <= s_axil_a_bvalid_next;

    s_axil_a_arready_reg <= s_axil_a_arready_next;
    s_axil_a_rvalid_reg <= s_axil_a_rvalid_next;

    if (mem_rd_en_a) begin
        s_axil_a_rdata_reg <= mem[s_axil_a_araddr_valid];
    end else begin
        for (i = 0; i < WORD_WIDTH; i = i + 1) begin
            if (mem_wr_en_a && s_axil_a_wstrb[i]) begin
                mem[s_axil_a_awaddr_valid][WORD_SIZE*i +: WORD_SIZE] <= s_axil_a_wdata[WORD_SIZE*i +: WORD_SIZE];
            end
        end
    end

    if (!s_axil_a_rvalid_pipe_reg || s_axil_a_rready) begin
        s_axil_a_rdata_pipe_reg <= s_axil_a_rdata_reg;
        s_axil_a_rvalid_pipe_reg <= s_axil_a_rvalid_reg;
    end

    if (a_rst) begin
        last_read_a_reg <= 1'b0;

        s_axil_a_awready_reg <= 1'b0;
        s_axil_a_wready_reg <= 1'b0;
        s_axil_a_bvalid_reg <= 1'b0;

        s_axil_a_arready_reg <= 1'b0;
        s_axil_a_rvalid_reg <= 1'b0;
        s_axil_a_rvalid_pipe_reg <= 1'b0;
    end
end

always @* begin
    mem_wr_en_b = 1'b0;
    mem_rd_en_b = 1'b0;

    last_read_b_next = last_read_b_reg;

    s_axil_b_awready_next = 1'b0;
    s_axil_b_wready_next = 1'b0;
    s_axil_b_bvalid_next = s_axil_b_bvalid_reg && !s_axil_b_bready;

    s_axil_b_arready_next = 1'b0;
    s_axil_b_rvalid_next = s_axil_b_rvalid_reg && !(s_axil_b_rready || (PIPELINE_OUTPUT && !s_axil_b_rvalid_pipe_reg));

    write_eligible_b = s_axil_b_awvalid && s_axil_b_wvalid && (!s_axil_b_bvalid || s_axil_b_bready) && (!s_axil_b_awready && !s_axil_b_wready);
    read_eligible_b = s_axil_b_arvalid && (!s_axil_b_rvalid || s_axil_b_rready || (PIPELINE_OUTPUT && !s_axil_b_rvalid_pipe_reg)) && (!s_axil_b_arready);

    if (write_eligible_b && (!read_eligible_b || last_read_b_reg)) begin
        last_read_b_next = 1'b0;

        s_axil_b_awready_next = 1'b1;
        s_axil_b_wready_next = 1'b1;
        s_axil_b_bvalid_next = 1'b1;

        mem_wr_en_b = 1'b1;
    end else if (read_eligible_b) begin
        last_read_b_next = 1'b1;

        s_axil_b_arready_next = 1'b1;
        s_axil_b_rvalid_next = 1'b1;

        mem_rd_en_b = 1'b1;
    end
end

always @(posedge b_clk) begin
        last_read_b_reg <= last_read_b_next;

    s_axil_b_awready_reg <= s_axil_b_awready_next;
    s_axil_b_wready_reg <= s_axil_b_wready_next;
    s_axil_b_bvalid_reg <= s_axil_b_bvalid_next;

    s_axil_b_arready_reg <= s_axil_b_arready_next;
    s_axil_b_rvalid_reg <= s_axil_b_rvalid_next;

    if (mem_rd_en_b) begin
        s_axil_b_rdata_reg <= mem[s_axil_b_araddr_valid];
    end else begin
        for (i = 0; i < WORD_WIDTH; i = i + 1) begin
            if (mem_wr_en_b && s_axil_b_wstrb[i]) begin
                mem[s_axil_b_awaddr_valid][WORD_SIZE*i +: WORD_SIZE] <= s_axil_b_wdata[WORD_SIZE*i +: WORD_SIZE];
            end
        end
    end

    if (!s_axil_b_rvalid_pipe_reg || s_axil_b_rready) begin
        s_axil_b_rdata_pipe_reg <= s_axil_b_rdata_reg;
        s_axil_b_rvalid_pipe_reg <= s_axil_b_rvalid_reg;
    end

    if (b_rst) begin
        last_read_b_reg <= 1'b0;

        s_axil_b_awready_reg <= 1'b0;
        s_axil_b_wready_reg <= 1'b0;
        s_axil_b_bvalid_reg <= 1'b0;

        s_axil_b_arready_reg <= 1'b0;
        s_axil_b_rvalid_reg <= 1'b0;
        s_axil_b_rvalid_pipe_reg <= 1'b0;
    end
end

endmodule

`resetall
