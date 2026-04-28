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
 * AXI4-Lite 寄存器切片
 *
 * 模块目录
 * 1) 对外提供一组 AXI-Lite 从接口和一组 AXI-Lite 主接口。
 * 2) 写通道流水化由 axil_register_wr 实现。
 * 3) 读通道流水化由 axil_register_rd 实现。
 * 4) 每个通道可按参数选择旁路或一级寄存缓冲。
 */
module axil_register #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // AW 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter AW_REG_TYPE = 1,
    // W 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter W_REG_TYPE = 1,
    // B 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter B_REG_TYPE = 1,
    // AR 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter AR_REG_TYPE = 1,
    // R 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter R_REG_TYPE = 1
)
(
    input  wire                     clk,            // 读写寄存切片共享时钟。
    input  wire                     rst,            // 所有通道缓冲状态同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]    s_axil_awaddr,  // 进入寄存切片的从侧 AW 地址。
    input  wire [2:0]               s_axil_awprot,  // 从侧 AW 保护属性。
    input  wire                     s_axil_awvalid, // 从侧 AW 有效。
    output wire                     s_axil_awready, // 从侧 AW 就绪（经可选 AW 缓冲后）。
    input  wire [DATA_WIDTH-1:0]    s_axil_wdata,   // 从侧 W 数据。
    input  wire [STRB_WIDTH-1:0]    s_axil_wstrb,   // 从侧 W 字节使能。
    input  wire                     s_axil_wvalid,  // 从侧 W 有效。
    output wire                     s_axil_wready,  // 从侧 W 就绪（经可选 W 缓冲后）。
    output wire [1:0]               s_axil_bresp,   // 从侧 B 响应（来自下游主侧路径）。
    output wire                     s_axil_bvalid,  // 从侧 B 有效（经可选 B 缓冲后）。
    input  wire                     s_axil_bready,  // 从侧 B 就绪。
    input  wire [ADDR_WIDTH-1:0]    s_axil_araddr,  // 进入寄存切片的从侧 AR 地址。
    input  wire [2:0]               s_axil_arprot,  // 从侧 AR 保护属性。
    input  wire                     s_axil_arvalid, // 从侧 AR 有效。
    output wire                     s_axil_arready, // 从侧 AR 就绪（经可选 AR 缓冲后）。
    output wire [DATA_WIDTH-1:0]    s_axil_rdata,   // 从侧 R 数据（来自下游主侧路径）。
    output wire [1:0]               s_axil_rresp,   // 从侧 R 响应（来自下游主侧路径）。
    output wire                     s_axil_rvalid,  // 从侧 R 有效（经可选 R 缓冲后）。
    input  wire                     s_axil_rready,  // 从侧 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]    m_axil_awaddr,  // 主侧 AW 地址（发往下游目标）。
    output wire [2:0]               m_axil_awprot,  // 主侧 AW 保护属性。
    output wire                     m_axil_awvalid, // 主侧 AW 有效（经可选 AW 缓冲后）。
    input  wire                     m_axil_awready, // 主侧 AW 就绪（来自下游目标）。
    output wire [DATA_WIDTH-1:0]    m_axil_wdata,   // 主侧 W 数据。
    output wire [STRB_WIDTH-1:0]    m_axil_wstrb,   // 主侧 W 字节使能。
    output wire                     m_axil_wvalid,  // 主侧 W 有效（经可选 W 缓冲后）。
    input  wire                     m_axil_wready,  // 主侧 W 就绪（来自下游目标）。
    input  wire [1:0]               m_axil_bresp,   // 主侧 B 响应（来自下游目标）。
    input  wire                     m_axil_bvalid,  // 主侧 B 有效（来自下游目标）。
    output wire                     m_axil_bready,  // 主侧 B 就绪（经可选 B 缓冲后）。
    output wire [ADDR_WIDTH-1:0]    m_axil_araddr,  // 主侧 AR 地址（发往下游目标）。
    output wire [2:0]               m_axil_arprot,  // 主侧 AR 保护属性。
    output wire                     m_axil_arvalid, // 主侧 AR 有效（经可选 AR 缓冲后）。
    input  wire                     m_axil_arready, // 主侧 AR 就绪（来自下游目标）。
    input  wire [DATA_WIDTH-1:0]    m_axil_rdata,   // 主侧 R 数据（来自下游目标）。
    input  wire [1:0]               m_axil_rresp,   // 主侧 R 响应（来自下游目标）。
    input  wire                     m_axil_rvalid,  // 主侧 R 有效（来自下游目标）。
    output wire                     m_axil_rready   // 主侧 R 就绪（经可选 R 缓冲后）。
);

axil_register_wr #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .AW_REG_TYPE(AW_REG_TYPE),
    .W_REG_TYPE(W_REG_TYPE),
    .B_REG_TYPE(B_REG_TYPE)
)
axil_register_wr_inst (
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

axil_register_rd #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .AR_REG_TYPE(AR_REG_TYPE),
    .R_REG_TYPE(R_REG_TYPE)
)
axil_register_rd_inst (
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
