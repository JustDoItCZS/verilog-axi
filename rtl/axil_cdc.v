/*

Copyright (c) 2019 Alex Forencich

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
 * AXI4-Lite 跨时钟域模块
 *
 * 模块目录
 * 1) 源时钟域（s_clk）承载从侧 AXI-Lite 接口。
 * 2) 目标时钟域（m_clk）承载主侧 AXI-Lite 接口。
 * 3) 写通道跨域逻辑由 axil_cdc_wr 实现。
 * 4) 读通道跨域逻辑由 axil_cdc_rd 实现。
 */
module axil_cdc #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8)
)
(
    /*
     * AXI-Lite 从接口
     */
    input  wire                   s_clk,           // 源时钟域时钟（从侧）。
    input  wire                   s_rst,           // 源时钟域同步复位。
    input  wire [ADDR_WIDTH-1:0]  s_axil_awaddr,   // 源时钟域 AW 地址。
    input  wire [2:0]             s_axil_awprot,   // 源时钟域 AW 保护属性。
    input  wire                   s_axil_awvalid,  // 源时钟域 AW 有效。
    output wire                   s_axil_awready,  // 源时钟域 AW 就绪（来自写通道跨域模块）。
    input  wire [DATA_WIDTH-1:0]  s_axil_wdata,    // 源时钟域 W 数据。
    input  wire [STRB_WIDTH-1:0]  s_axil_wstrb,    // 源时钟域 W 字节使能。
    input  wire                   s_axil_wvalid,   // 源时钟域 W 有效。
    output wire                   s_axil_wready,   // 源时钟域 W 就绪（来自写通道跨域模块）。
    output wire [1:0]             s_axil_bresp,    // 源时钟域 B 响应（跨域返回）。
    output wire                   s_axil_bvalid,   // 源时钟域 B 有效（跨域返回）。
    input  wire                   s_axil_bready,   // 源时钟域 B 就绪。
    input  wire [ADDR_WIDTH-1:0]  s_axil_araddr,   // 源时钟域 AR 地址。
    input  wire [2:0]             s_axil_arprot,   // 源时钟域 AR 保护属性。
    input  wire                   s_axil_arvalid,  // 源时钟域 AR 有效。
    output wire                   s_axil_arready,  // 源时钟域 AR 就绪（来自读通道跨域模块）。
    output wire [DATA_WIDTH-1:0]  s_axil_rdata,    // 源时钟域 R 数据（跨域返回）。
    output wire [1:0]             s_axil_rresp,    // 源时钟域 R 响应（跨域返回）。
    output wire                   s_axil_rvalid,   // 源时钟域 R 有效（跨域返回）。
    input  wire                   s_axil_rready,   // 源时钟域 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    input  wire                   m_clk,           // 目标时钟域时钟（主侧）。
    input  wire                   m_rst,           // 目标时钟域同步复位。
    output wire [ADDR_WIDTH-1:0]  m_axil_awaddr,   // 目标时钟域 AW 地址。
    output wire [2:0]             m_axil_awprot,   // 目标时钟域 AW 保护属性。
    output wire                   m_axil_awvalid,  // 目标时钟域 AW 有效（来自写通道跨域模块）。
    input  wire                   m_axil_awready,  // 目标时钟域 AW 就绪（来自下游目标）。
    output wire [DATA_WIDTH-1:0]  m_axil_wdata,    // 目标时钟域 W 数据（来自写通道跨域模块）。
    output wire [STRB_WIDTH-1:0]  m_axil_wstrb,    // 目标时钟域 W 字节使能（来自写通道跨域模块）。
    output wire                   m_axil_wvalid,   // 目标时钟域 W 有效（来自写通道跨域模块）。
    input  wire                   m_axil_wready,   // 目标时钟域 W 就绪（来自下游目标）。
    input  wire [1:0]             m_axil_bresp,    // 目标时钟域 B 响应（来自下游目标）。
    input  wire                   m_axil_bvalid,   // 目标时钟域 B 有效（来自下游目标）。
    output wire                   m_axil_bready,   // 目标时钟域 B 就绪（来自写通道跨域模块）。
    output wire [ADDR_WIDTH-1:0]  m_axil_araddr,   // 目标时钟域 AR 地址（来自读通道跨域模块）。
    output wire [2:0]             m_axil_arprot,   // 目标时钟域 AR 保护属性。
    output wire                   m_axil_arvalid,  // 目标时钟域 AR 有效（来自读通道跨域模块）。
    input  wire                   m_axil_arready,  // 目标时钟域 AR 就绪（来自下游目标）。
    input  wire [DATA_WIDTH-1:0]  m_axil_rdata,    // 目标时钟域 R 数据（来自下游目标）。
    input  wire [1:0]             m_axil_rresp,    // 目标时钟域 R 响应（来自下游目标）。
    input  wire                   m_axil_rvalid,   // 目标时钟域 R 有效（来自下游目标）。
    output wire                   m_axil_rready    // 目标时钟域 R 就绪（来自读通道跨域模块）。
);

axil_cdc_wr #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH)
)
axil_cdc_wr_inst (
    /*
     * AXI-Lite 从接口
     */
    .s_clk(s_clk),
    .s_rst(s_rst),
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
     * AXI-Lite 主接口
     */
    .m_clk(m_clk),
    .m_rst(m_rst),
    .m_axil_awaddr(m_axil_awaddr),
    .m_axil_awprot(m_axil_awprot),
    .m_axil_awvalid(m_axil_awvalid),
    .m_axil_awready(m_axil_awready),
    .m_axil_wdata(m_axil_wdata),
    .m_axil_wstrb(m_axil_wstrb),
    .m_axil_wvalid(m_axil_wvalid),
    .m_axil_wready(m_axil_wready),
    .m_axil_bresp(m_axil_bresp),
    .m_axil_bvalid(m_axil_bvalid),
    .m_axil_bready(m_axil_bready)
);

axil_cdc_rd #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH)
)
axil_cdc_rd_inst (
    /*
     * AXI-Lite 从接口
     */
    .s_clk(s_clk),
    .s_rst(s_rst),
    .s_axil_araddr(s_axil_araddr),
    .s_axil_arprot(s_axil_arprot),
    .s_axil_arvalid(s_axil_arvalid),
    .s_axil_arready(s_axil_arready),
    .s_axil_rdata(s_axil_rdata),
    .s_axil_rresp(s_axil_rresp),
    .s_axil_rvalid(s_axil_rvalid),
    .s_axil_rready(s_axil_rready),

    /*
     * AXI-Lite 主接口
     */
    .m_clk(m_clk),
    .m_rst(m_rst),
    .m_axil_araddr(m_axil_araddr),
    .m_axil_arprot(m_axil_arprot),
    .m_axil_arvalid(m_axil_arvalid),
    .m_axil_arready(m_axil_arready),
    .m_axil_rdata(m_axil_rdata),
    .m_axil_rresp(m_axil_rresp),
    .m_axil_rvalid(m_axil_rvalid),
    .m_axil_rready(m_axil_rready)
);

endmodule

`resetall
