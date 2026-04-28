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
 * AXI-Lite 寄存器接口模块
 *
 * 模块目录
 * 1) 轻量封装：组合写桥接（axil_reg_if_wr）和读桥接（axil_reg_if_rd）。
 * 2) 将 AXI-Lite 通道拆分为独立的读/写寄存器侧握手接口。
 */
module axil_reg_if #
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
    input  wire                   clk, // 读写桥接子模块共享时钟。
    input  wire                   rst, // 读写桥接子模块共享同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]  s_axil_awaddr, // 送入写桥接的 AXI-Lite AW 地址。
    input  wire [2:0]             s_axil_awprot, // AXI-Lite AW 保护属性。
    input  wire                   s_axil_awvalid, // 送入写桥接的 AXI-Lite AWVALID。
    output wire                   s_axil_awready, // 来自写桥接的 AXI-Lite AWREADY。
    input  wire [DATA_WIDTH-1:0]  s_axil_wdata, // 送入写桥接的 AXI-Lite W 数据。
    input  wire [STRB_WIDTH-1:0]  s_axil_wstrb, // 送入写桥接的 AXI-Lite W 字节使能。
    input  wire                   s_axil_wvalid, // 送入写桥接的 AXI-Lite WVALID。
    output wire                   s_axil_wready, // 来自写桥接的 AXI-Lite WREADY。
    output wire [1:0]             s_axil_bresp, // 来自写桥接的 AXI-Lite BRESP。
    output wire                   s_axil_bvalid, // 来自写桥接的 AXI-Lite BVALID。
    input  wire                   s_axil_bready, // 送入写桥接的 AXI-Lite BREADY。
    input  wire [ADDR_WIDTH-1:0]  s_axil_araddr, // 送入读桥接的 AXI-Lite AR 地址。
    input  wire [2:0]             s_axil_arprot, // AXI-Lite AR 保护属性。
    input  wire                   s_axil_arvalid, // 送入读桥接的 AXI-Lite ARVALID。
    output wire                   s_axil_arready, // 来自读桥接的 AXI-Lite ARREADY。
    output wire [DATA_WIDTH-1:0]  s_axil_rdata, // 来自读桥接的 AXI-Lite RDATA。
    output wire [1:0]             s_axil_rresp, // 来自读桥接的 AXI-Lite RRESP。
    output wire                   s_axil_rvalid, // 来自读桥接的 AXI-Lite RVALID。
    input  wire                   s_axil_rready, // 送入读桥接的 AXI-Lite RREADY。

    /*
     * 寄存器接口
     */
    output wire [ADDR_WIDTH-1:0]  reg_wr_addr, // 寄存器侧写地址输出。
    output wire [DATA_WIDTH-1:0]  reg_wr_data, // 寄存器侧写数据输出。
    output wire [STRB_WIDTH-1:0]  reg_wr_strb, // 寄存器侧写字节使能输出。
    output wire                   reg_wr_en, // 寄存器侧写请求有效。
    input  wire                   reg_wr_wait, // 寄存器侧写等待/背压。
    input  wire                   reg_wr_ack, // 寄存器侧写完成应答。
    output wire [ADDR_WIDTH-1:0]  reg_rd_addr, // 寄存器侧读地址输出。
    output wire                   reg_rd_en, // 寄存器侧读请求有效。
    input  wire [DATA_WIDTH-1:0]  reg_rd_data, // 寄存器侧读数据输入。
    input  wire                   reg_rd_wait, // 寄存器侧读等待/背压。
    input  wire                   reg_rd_ack // 寄存器侧读完成应答。
);

axil_reg_if_wr #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .TIMEOUT(TIMEOUT)
)
axil_reg_if_wr_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI-Lite 从接口
     */
    .s_axil_awaddr(s_axil_awaddr),
    .s_axil_awprot(s_axil_awprot),
    .s_axil_awvalid(s_axil_awvalid),
    .s_axil_awready(s_axil_awready),
    .s_axil_wdata(s_axil_wdata),
    .s_axil_wstrb(s_axil_wstrb),
    .s_axil_wvalid(s_axil_wvalid),
    .s_axil_wready(s_axil_wready),
    .s_axil_bresp(s_axil_bresp),
    .s_axil_bvalid(s_axil_bvalid),
    .s_axil_bready(s_axil_bready),

    /*
     * 寄存器接口
     */
    .reg_wr_addr(reg_wr_addr),
    .reg_wr_data(reg_wr_data),
    .reg_wr_strb(reg_wr_strb),
    .reg_wr_en(reg_wr_en),
    .reg_wr_wait(reg_wr_wait),
    .reg_wr_ack(reg_wr_ack)
);

axil_reg_if_rd #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .TIMEOUT(TIMEOUT)
)
axil_reg_if_rd_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI-Lite 从接口
     */
    .s_axil_araddr(s_axil_araddr),
    .s_axil_arprot(s_axil_arprot),
    .s_axil_arvalid(s_axil_arvalid),
    .s_axil_arready(s_axil_arready),
    .s_axil_rdata(s_axil_rdata),
    .s_axil_rresp(s_axil_rresp),
    .s_axil_rvalid(s_axil_rvalid),
    .s_axil_rready(s_axil_rready),

    /*
     * 寄存器接口
     */
    .reg_rd_addr(reg_rd_addr),
    .reg_rd_en(reg_rd_en),
    .reg_rd_data(reg_rd_data),
    .reg_rd_wait(reg_rd_wait),
    .reg_rd_ack(reg_rd_ack)
);

endmodule

`resetall
