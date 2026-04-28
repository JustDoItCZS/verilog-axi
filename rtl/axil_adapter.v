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
 * AXI4-Lite 位宽适配器
 *
 * 模块目录
 * 1) 对外提供一组从侧 AXI-Lite 接口和一组主侧 AXI-Lite 接口。
 * 2) 写通道位宽转换由 `axil_adapter_wr` 子模块完成。
 * 3) 读通道位宽转换由 `axil_adapter_rd` 子模块完成。
 * 4) 目标：在保持 AXI-Lite 语义不变的前提下完成主从数据位宽适配。
 */
module axil_adapter #
(
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 输入（从侧）接口数据位宽
    parameter S_DATA_WIDTH = 32,
    // 输入（从侧）接口 WSTRB 位宽（按字节）
    parameter S_STRB_WIDTH = (S_DATA_WIDTH/8),
    // 输出（主侧）接口数据位宽
    parameter M_DATA_WIDTH = 32,
    // 输出（主侧）接口 WSTRB 位宽（按字节）
    parameter M_STRB_WIDTH = (M_DATA_WIDTH/8)
)
(
    input  wire                     clk,            // 读写适配子模块共用核心时钟。
    input  wire                     rst,            // 同步复位，传递给读写适配子模块。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]    s_axil_awaddr,  // 从侧 AW 地址（位宽转换前）。
    input  wire [2:0]               s_axil_awprot,  // 从侧 AW 保护属性。
    input  wire                     s_axil_awvalid, // 从侧 AW 有效。
    output wire                     s_axil_awready, // 从侧 AW 就绪（来自写适配器）。
    input  wire [S_DATA_WIDTH-1:0]  s_axil_wdata,   // 从侧 W 数据（从侧位宽）。
    input  wire [S_STRB_WIDTH-1:0]  s_axil_wstrb,   // 从侧 W 字节使能（从侧位宽）。
    input  wire                     s_axil_wvalid,  // 从侧 W 有效。
    output wire                     s_axil_wready,  // 从侧 W 就绪（来自写适配器）。
    output wire [1:0]               s_axil_bresp,   // 从侧 B 响应（写转换后返回）。
    output wire                     s_axil_bvalid,  // 从侧 B 有效（写转换后返回）。
    input  wire                     s_axil_bready,  // 从侧 B 就绪。
    input  wire [ADDR_WIDTH-1:0]    s_axil_araddr,  // 从侧 AR 地址（位宽转换前）。
    input  wire [2:0]               s_axil_arprot,  // 从侧 AR 保护属性。
    input  wire                     s_axil_arvalid, // 从侧 AR 有效。
    output wire                     s_axil_arready, // 从侧 AR 就绪（来自读适配器）。
    output wire [S_DATA_WIDTH-1:0]  s_axil_rdata,   // 从侧 R 数据（从侧位宽）。
    output wire [1:0]               s_axil_rresp,   // 从侧 R 响应（来自读适配器）。
    output wire                     s_axil_rvalid,  // 从侧 R 有效（来自读适配器）。
    input  wire                     s_axil_rready,  // 从侧 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]    m_axil_awaddr,  // 主侧 AW 地址（指向下游目标）。
    output wire [2:0]               m_axil_awprot,  // 主侧 AW 保护属性。
    output wire                     m_axil_awvalid, // 主侧 AW 有效（来自写适配器）。
    input  wire                     m_axil_awready, // 主侧 AW 就绪（来自下游目标）。
    output wire [M_DATA_WIDTH-1:0]  m_axil_wdata,   // 主侧 W 数据（主侧位宽）。
    output wire [M_STRB_WIDTH-1:0]  m_axil_wstrb,   // 主侧 W 字节使能（主侧位宽）。
    output wire                     m_axil_wvalid,  // 主侧 W 有效（来自写适配器）。
    input  wire                     m_axil_wready,  // 主侧 W 就绪（来自下游目标）。
    input  wire [1:0]               m_axil_bresp,   // 主侧 B 响应（来自下游目标）。
    input  wire                     m_axil_bvalid,  // 主侧 B 有效（来自下游目标）。
    output wire                     m_axil_bready,  // 主侧 B 就绪（来自写适配器）。
    output wire [ADDR_WIDTH-1:0]    m_axil_araddr,  // 主侧 AR 地址（指向下游目标）。
    output wire [2:0]               m_axil_arprot,  // 主侧 AR 保护属性。
    output wire                     m_axil_arvalid, // 主侧 AR 有效（来自读适配器）。
    input  wire                     m_axil_arready, // 主侧 AR 就绪（来自下游目标）。
    input  wire [M_DATA_WIDTH-1:0]  m_axil_rdata,   // 主侧 R 数据（来自下游目标）。
    input  wire [1:0]               m_axil_rresp,   // 主侧 R 响应（来自下游目标）。
    input  wire                     m_axil_rvalid,  // 主侧 R 有效（来自下游目标）。
    output wire                     m_axil_rready   // 主侧 R 就绪（来自读适配器）。
);

axil_adapter_wr #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .S_DATA_WIDTH(S_DATA_WIDTH),
    .S_STRB_WIDTH(S_STRB_WIDTH),
    .M_DATA_WIDTH(M_DATA_WIDTH),
    .M_STRB_WIDTH(M_STRB_WIDTH)
)
axil_adapter_wr_inst (
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
     * AXI-Lite 主接口
     */
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

axil_adapter_rd #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .S_DATA_WIDTH(S_DATA_WIDTH),
    .S_STRB_WIDTH(S_STRB_WIDTH),
    .M_DATA_WIDTH(M_DATA_WIDTH),
    .M_STRB_WIDTH(M_STRB_WIDTH)
)
axil_adapter_rd_inst (
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
     * AXI-Lite 主接口
     */
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
