/*

Copyright (c) 2021 Alex Forencich

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
 * AXI-Lite 寄存器接口模块（写通道）
 *
 * 模块目录
 * 1) 接收 AXI-Lite 的 AW 与 W 通道。
 * 2) 将其转换为简单的寄存器写握手（reg_wr_*）。
 * 3) 寄存器侧应答或超时时返回 B 响应。
 */
module axil_reg_if_wr #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 超时延迟（周期）
    parameter TIMEOUT = 4
)
(
    input  wire                   clk, // AXI 到寄存器写桥接时钟。
    input  wire                   rst, // 采样请求/响应状态的同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]  s_axil_awaddr, // AXI-Lite 写地址；AW 通道空闲时锁存。
    input  wire [2:0]             s_axil_awprot, // AXI-Lite 写保护属性（接收但内部不使用）。
    input  wire                   s_axil_awvalid, // 来自主机的 AXI-Lite AWVALID。
    output wire                   s_axil_awready, // AXI-Lite AWREADY；无 pending AW 时拉高。
    input  wire [DATA_WIDTH-1:0]  s_axil_wdata, // AXI-Lite 写数据；W 通道空闲时锁存。
    input  wire [STRB_WIDTH-1:0]  s_axil_wstrb, // 与写数据对应的 AXI-Lite 字节使能。
    input  wire                   s_axil_wvalid, // 来自主机的 AXI-Lite WVALID。
    output wire                   s_axil_wready, // AXI-Lite WREADY；无 pending W 时拉高。
    output wire [1:0]             s_axil_bresp, // AXI-Lite BRESP；本桥接固定 OKAY。
    output wire                   s_axil_bvalid, // AXI-Lite BVALID；写完成/超时后置位。
    input  wire                   s_axil_bready, // 来自主机的 AXI-Lite BREADY。

    /*
     * 寄存器接口
     */
    output wire [ADDR_WIDTH-1:0]  reg_wr_addr, // 由采样 AXI AW 导出的寄存器总线写地址。
    output wire [DATA_WIDTH-1:0]  reg_wr_data, // 由采样 AXI W 导出的寄存器总线写数据。
    output wire [STRB_WIDTH-1:0]  reg_wr_strb, // 由采样 AXI WSTRB 导出的寄存器总线写字节使能。
    output wire                   reg_wr_en, // 寄存器总线写请求有效，等待应答/超时期间保持。
    input  wire                   reg_wr_wait, // 寄存器总线背压；为高时超时计数停止递减。
    input  wire                   reg_wr_ack // 寄存器总线完成脉冲。
);

parameter TIMEOUT_WIDTH = $clog2(TIMEOUT); // 表示超时周期所需计数器位宽。

reg [TIMEOUT_WIDTH-1:0] timeout_count_reg = 0, timeout_count_next; // 等待 reg_wr_ack 期间的倒计时。

reg [ADDR_WIDTH-1:0] s_axil_awaddr_reg = {ADDR_WIDTH{1'b0}}, s_axil_awaddr_next; // 已采样 AW 地址，等待发往寄存器总线。
reg s_axil_awvalid_reg = 1'b0, s_axil_awvalid_next; // 表示采样 AW 地址有效。
reg [DATA_WIDTH-1:0] s_axil_wdata_reg = {DATA_WIDTH{1'b0}}, s_axil_wdata_next; // 已采样 W 数据，等待发往寄存器总线。
reg [STRB_WIDTH-1:0] s_axil_wstrb_reg = {STRB_WIDTH{1'b0}}, s_axil_wstrb_next; // 已采样寄存器写字节使能。
reg s_axil_wvalid_reg = 1'b0, s_axil_wvalid_next; // 表示采样 W 数据有效。
reg s_axil_bvalid_reg = 1'b0, s_axil_bvalid_next; // 面向 AXI 主机的 pending B 响应标志。

reg reg_wr_en_reg = 1'b0, reg_wr_en_next; // 当前寄存器写请求有效标志。

assign s_axil_awready = !s_axil_awvalid_reg;
assign s_axil_wready = !s_axil_wvalid_reg;
assign s_axil_bresp = 2'b00;
assign s_axil_bvalid = s_axil_bvalid_reg;

assign reg_wr_addr = s_axil_awaddr_reg;
assign reg_wr_data = s_axil_wdata_reg;
assign reg_wr_strb = s_axil_wstrb_reg;
assign reg_wr_en = reg_wr_en_reg;

always @* begin
    timeout_count_next = timeout_count_reg;

    s_axil_awaddr_next = s_axil_awaddr_reg;
    s_axil_awvalid_next = s_axil_awvalid_reg;
    s_axil_wdata_next = s_axil_wdata_reg;
    s_axil_wstrb_next = s_axil_wstrb_reg;
    s_axil_wvalid_next = s_axil_wvalid_reg;
    s_axil_bvalid_next = s_axil_bvalid_reg && !s_axil_bready;

    if (reg_wr_en_reg && (reg_wr_ack || timeout_count_reg == 0)) begin
        s_axil_awvalid_next = 1'b0;
        s_axil_wvalid_next = 1'b0;
        s_axil_bvalid_next = 1'b1;
    end

    if (!s_axil_awvalid_reg) begin
        s_axil_awaddr_next = s_axil_awaddr;
        s_axil_awvalid_next = s_axil_awvalid;
        timeout_count_next = TIMEOUT-1;
    end

    if (!s_axil_wvalid_reg) begin
        s_axil_wdata_next = s_axil_wdata;
        s_axil_wstrb_next = s_axil_wstrb;
        s_axil_wvalid_next = s_axil_wvalid;
    end

    if (reg_wr_en && !reg_wr_wait && timeout_count_reg != 0)begin
        timeout_count_next = timeout_count_reg - 1;
    end

    reg_wr_en_next = s_axil_awvalid_next && s_axil_wvalid_next && !s_axil_bvalid_next;
end

always @(posedge clk) begin
    timeout_count_reg <= timeout_count_next;

    s_axil_awaddr_reg <= s_axil_awaddr_next;
    s_axil_awvalid_reg <= s_axil_awvalid_next;
    s_axil_wdata_reg <= s_axil_wdata_next;
    s_axil_wstrb_reg <= s_axil_wstrb_next;
    s_axil_wvalid_reg <= s_axil_wvalid_next;
    s_axil_bvalid_reg <= s_axil_bvalid_next;

    reg_wr_en_reg <= reg_wr_en_next;

    if (rst) begin
        s_axil_awvalid_reg <= 1'b0;
        s_axil_wvalid_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
        reg_wr_en_reg <= 1'b0;
    end
end

endmodule

`resetall
