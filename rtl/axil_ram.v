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
 * AXI4-Lite RAM 存储模块
 *
 * 模块目录
 * 1) 参数：数据/地址位宽及可选读输出流水线。
 * 2) AXI-Lite 接口：写地址/写数据/写响应与读地址/读数据/读响应通道。
 * 3) 内部声明：存储阵列与握手/状态寄存器。
 * 4) 逻辑：
 *    - 写路径：AW 与 W 同拍接收，按字节写入，返回 B 响应。
 *    - 读路径：响应路径可接收时接收 AR，返回 R 响应。
 */
module axil_ram #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 读输出额外流水级开关
    parameter PIPELINE_OUTPUT = 0
)
(
    input  wire                   clk,            // 时序状态更新时钟。
    input  wire                   rst,            // 同步复位，清空输出 valid/ready 状态。

    input  wire [ADDR_WIDTH-1:0]  s_axil_awaddr,  // 主机写地址，在 AW 握手时采样。
    input  wire [2:0]             s_axil_awprot,  // 写保护属性，接收但内部不使用。
    input  wire                   s_axil_awvalid, // 主机写地址有效。
    output wire                   s_axil_awready, // RAM 写地址就绪，AW 被接收时脉冲拉高。
    input  wire [DATA_WIDTH-1:0]  s_axil_wdata,   // 主机写数据，在 W 握手时采样。
    input  wire [STRB_WIDTH-1:0]  s_axil_wstrb,   // 字节使能掩码，每位控制一个字节 lane 的写入。
    input  wire                   s_axil_wvalid,  // 主机写数据有效。
    output wire                   s_axil_wready,  // RAM 写数据就绪，W 被接收时脉冲拉高。
    output wire [1:0]             s_axil_bresp,   // 写响应码，本 RAM 固定返回 OKAY。
    output wire                   s_axil_bvalid,  // 写响应有效，写事务被接收后置位。
    input  wire                   s_axil_bready,  // 主机可接收写响应。
    input  wire [ADDR_WIDTH-1:0]  s_axil_araddr,  // 主机读地址，在 AR 握手时采样。
    input  wire [2:0]             s_axil_arprot,  // 读保护属性，接收但内部不使用。
    input  wire                   s_axil_arvalid, // 主机读地址有效。
    output wire                   s_axil_arready, // RAM 读地址就绪，AR 被接收时脉冲拉高。
    output wire [DATA_WIDTH-1:0]  s_axil_rdata,   // 从存储阵列返回的读数据。
    output wire [1:0]             s_axil_rresp,   // 读响应码，本 RAM 固定返回 OKAY。
    output wire                   s_axil_rvalid,  // 读响应有效，直到主机接收数据前保持。
    input  wire                   s_axil_rready   // 主机可接收读响应。
);

// 用于索引 mem[] 的有效字地址位宽。
// AXI-Lite 地址以字节为单位，因此需丢弃低位字节 lane 偏移位。
parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH);
parameter WORD_WIDTH = STRB_WIDTH;
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH;

reg mem_wr_en; // 单拍写使能脉冲，AW 与 W 被接收时置位。
reg mem_rd_en; // 单拍读使能脉冲，AR 被接收时置位。

reg s_axil_awready_reg = 1'b0, s_axil_awready_next; // AW ready 寄存器及组合下一状态。
reg s_axil_wready_reg = 1'b0, s_axil_wready_next; // W ready 寄存器及组合下一状态。
reg s_axil_bvalid_reg = 1'b0, s_axil_bvalid_next; // B valid 寄存器，直到 BREADY 前保持。
reg s_axil_arready_reg = 1'b0, s_axil_arready_next; // AR ready 寄存器及组合下一状态。
reg [DATA_WIDTH-1:0] s_axil_rdata_reg = {DATA_WIDTH{1'b0}}, s_axil_rdata_next; // R 数据寄存器及占位的下一状态变量。
reg s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next; // R valid 寄存器，表示 s_axil_rdata_reg 的有效性。
reg [DATA_WIDTH-1:0] s_axil_rdata_pipe_reg = {DATA_WIDTH{1'b0}}; // 可选输出流水级的数据寄存器。
reg s_axil_rvalid_pipe_reg = 1'b0; // 可选输出流水级的 valid 寄存器。

// RAM 风格属性示例（可选）：(* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[(2**VALID_ADDR_WIDTH)-1:0]; // 主存储阵列，每个地址存一拍 DATA_WIDTH 数据。

// AXI 字节地址转换为内部字地址索引。
wire [VALID_ADDR_WIDTH-1:0] s_axil_awaddr_valid = s_axil_awaddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 写事务字地址索引。
wire [VALID_ADDR_WIDTH-1:0] s_axil_araddr_valid = s_axil_araddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // 读事务字地址索引。

assign s_axil_awready = s_axil_awready_reg;
assign s_axil_wready = s_axil_wready_reg;
assign s_axil_bresp = 2'b00;
assign s_axil_bvalid = s_axil_bvalid_reg;
assign s_axil_arready = s_axil_arready_reg;
assign s_axil_rdata = PIPELINE_OUTPUT ? s_axil_rdata_pipe_reg : s_axil_rdata_reg;
assign s_axil_rresp = 2'b00;
assign s_axil_rvalid = PIPELINE_OUTPUT ? s_axil_rvalid_pipe_reg : s_axil_rvalid_reg;

integer i, j; // 仅用于存储初始化与字节 lane 遍历的循环变量。

initial begin
    // 使用两层循环可降低单层循环迭代次数，规避部分综合告警。
    for (i = 0; i < 2**VALID_ADDR_WIDTH; i = i + 2**(VALID_ADDR_WIDTH/2)) begin
        for (j = i; j < i + 2**(VALID_ADDR_WIDTH/2); j = j + 1) begin
            mem[j] = 0;
        end
    end
end

always @* begin
    mem_wr_en = 1'b0;

    s_axil_awready_next = 1'b0;
    s_axil_wready_next = 1'b0;
    // 在 B 通道握手完成前保持写响应 pending。
    s_axil_bvalid_next = s_axil_bvalid_reg && !s_axil_bready;

    // 本 RAM 在同一拍同时接收写地址与写数据（无解耦 skid buffer）。
    // 仅当 AW/W 同时有效且响应通道可接收新响应时，产生 mem_wr_en。
    if (s_axil_awvalid && s_axil_wvalid && (!s_axil_bvalid || s_axil_bready) && (!s_axil_awready && !s_axil_wready)) begin
        s_axil_awready_next = 1'b1;
        s_axil_wready_next = 1'b1;
        s_axil_bvalid_next = 1'b1;

        mem_wr_en = 1'b1;
    end
end

always @(posedge clk) begin
    s_axil_awready_reg <= s_axil_awready_next;
    s_axil_wready_reg <= s_axil_wready_next;
    s_axil_bvalid_reg <= s_axil_bvalid_next;

    // 按字节写行为由 WSTRB 控制。
    // 仅在 mem_wr_en 置位且对应 strobe 位为 1 时更新对应字节 lane。
    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
        if (mem_wr_en && s_axil_wstrb[i]) begin
            mem[s_axil_awaddr_valid][WORD_SIZE*i +: WORD_SIZE] <= s_axil_wdata[WORD_SIZE*i +: WORD_SIZE];
        end
    end

    if (rst) begin
        s_axil_awready_reg <= 1'b0;
        s_axil_wready_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
    end
end

always @* begin
    mem_rd_en = 1'b0;

    s_axil_arready_next = 1'b0;
    // RVALID 在读数据被接收前保持置位。
    // 启用输出流水后，新事务接收还需受流水级可用性约束。
    s_axil_rvalid_next = s_axil_rvalid_reg && !(s_axil_rready || (PIPELINE_OUTPUT && !s_axil_rvalid_pipe_reg));

    // 仅在响应路径有空间时接收 AR。
    // AR 被接收的同一拍产生 mem_rd_en 脉冲。
    if (s_axil_arvalid && (!s_axil_rvalid || s_axil_rready || (PIPELINE_OUTPUT && !s_axil_rvalid_pipe_reg)) && (!s_axil_arready)) begin
        s_axil_arready_next = 1'b1;
        s_axil_rvalid_next = 1'b1;

        mem_rd_en = 1'b1;
    end
end

always @(posedge clk) begin
    s_axil_arready_reg <= s_axil_arready_next;
    s_axil_rvalid_reg <= s_axil_rvalid_next;

    // 在 AR 被接收时抓取存储阵列读数据。
    if (mem_rd_en) begin
        s_axil_rdata_reg <= mem[s_axil_araddr_valid];
    end

    // 可选输出流水级更新条件：
    // 当流水级为空，或下游已消费当前输出时转移新数据。
    if (!s_axil_rvalid_pipe_reg || s_axil_rready) begin
        s_axil_rdata_pipe_reg <= s_axil_rdata_reg;
        s_axil_rvalid_pipe_reg <= s_axil_rvalid_reg;
    end

    if (rst) begin
        s_axil_arready_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
        s_axil_rvalid_pipe_reg <= 1'b0;
    end
end

endmodule

`resetall
