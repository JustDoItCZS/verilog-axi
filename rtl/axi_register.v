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
 * AXI4 寄存器切片
 *
 * 模块目录
 * 1) 封装写通道寄存切片（`axi_register_wr`）。
 * 2) 封装读通道寄存切片（`axi_register_rd`）。
 * 3) 本模块无本地数据通路状态，全部缓冲行为由子模块实现。
 */
module axi_register #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 AWUSER 信号
    parameter AWUSER_ENABLE = 0,
    // AWUSER 位宽
    parameter AWUSER_WIDTH = 1,
    // 是否透传 WUSER 信号
    parameter WUSER_ENABLE = 0,
    // WUSER 位宽
    parameter WUSER_WIDTH = 1,
    // 是否透传 BUSER 信号
    parameter BUSER_ENABLE = 0,
    // BUSER 位宽
    parameter BUSER_WIDTH = 1,
    // 是否透传 ARUSER 信号
    parameter ARUSER_ENABLE = 0,
    // ARUSER 位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 RUSER 信号
    parameter RUSER_ENABLE = 0,
    // RUSER 位宽
    parameter RUSER_WIDTH = 1,
    // AW 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter AW_REG_TYPE = 1,
    // W 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter W_REG_TYPE = 2,
    // B 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter B_REG_TYPE = 1,
    // AR 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter AR_REG_TYPE = 1,
    // R 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter R_REG_TYPE = 2
)
(
    input  wire                     clk, // 读写寄存切片共享时钟。
    input  wire                     rst, // 读写寄存切片共享同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // 从侧 AW ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // 从侧 AW 地址。
    input  wire [7:0]               s_axi_awlen, // 从侧 AW 突发长度。
    input  wire [2:0]               s_axi_awsize, // 从侧 AW 突发粒度。
    input  wire [1:0]               s_axi_awburst, // 从侧 AW 突发类型。
    input  wire                     s_axi_awlock, // 从侧 AW 锁属性。
    input  wire [3:0]               s_axi_awcache, // 从侧 AW cache 属性。
    input  wire [2:0]               s_axi_awprot, // 从侧 AW 保护属性。
    input  wire [3:0]               s_axi_awqos, // 从侧 AW QoS。
    input  wire [3:0]               s_axi_awregion, // 从侧 AW region。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // 从侧 AW 用户旁带。
    input  wire                     s_axi_awvalid, // 从侧 AWVALID。
    output wire                     s_axi_awready, // 从侧 AWREADY。
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata, // 从侧 W 数据。
    input  wire [STRB_WIDTH-1:0]    s_axi_wstrb, // 从侧 W 字节使能。
    input  wire                     s_axi_wlast, // 从侧 WLAST。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // 从侧 W 用户旁带。
    input  wire                     s_axi_wvalid, // 从侧 WVALID。
    output wire                     s_axi_wready, // 从侧 WREADY。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 从侧 B ID（来自下游）。
    output wire [1:0]               s_axi_bresp, // 从侧 B 响应码。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 从侧 B 用户旁带。
    output wire                     s_axi_bvalid, // 从侧 BVALID。
    input  wire                     s_axi_bready, // 从侧 BREADY。
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // 从侧 AR ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // 从侧 AR 地址。
    input  wire [7:0]               s_axi_arlen, // 从侧 AR 突发长度。
    input  wire [2:0]               s_axi_arsize, // 从侧 AR 突发粒度。
    input  wire [1:0]               s_axi_arburst, // 从侧 AR 突发类型。
    input  wire                     s_axi_arlock, // 从侧 AR 锁属性。
    input  wire [3:0]               s_axi_arcache, // 从侧 AR cache 属性。
    input  wire [2:0]               s_axi_arprot, // 从侧 AR 保护属性。
    input  wire [3:0]               s_axi_arqos, // 从侧 AR QoS。
    input  wire [3:0]               s_axi_arregion, // 从侧 AR region。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // 从侧 AR 用户旁带。
    input  wire                     s_axi_arvalid, // 从侧 ARVALID。
    output wire                     s_axi_arready, // 从侧 ARREADY。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 从侧 R ID。
    output wire [DATA_WIDTH-1:0]    s_axi_rdata, // 从侧 R 数据。
    output wire [1:0]               s_axi_rresp, // 从侧 R 响应。
    output wire                     s_axi_rlast, // 从侧 RLAST。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 从侧 R 用户旁带。
    output wire                     s_axi_rvalid, // 从侧 RVALID。
    input  wire                     s_axi_rready, // 从侧 RREADY。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_awid, // 主侧 AW ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr, // 主侧 AW 地址。
    output wire [7:0]               m_axi_awlen, // 主侧 AW 突发长度。
    output wire [2:0]               m_axi_awsize, // 主侧 AW 突发粒度。
    output wire [1:0]               m_axi_awburst, // 主侧 AW 突发类型。
    output wire                     m_axi_awlock, // 主侧 AW 锁属性。
    output wire [3:0]               m_axi_awcache, // 主侧 AW cache 属性。
    output wire [2:0]               m_axi_awprot, // 主侧 AW 保护属性。
    output wire [3:0]               m_axi_awqos, // 主侧 AW QoS。
    output wire [3:0]               m_axi_awregion, // 主侧 AW region。
    output wire [AWUSER_WIDTH-1:0]  m_axi_awuser, // 主侧 AW 用户旁带。
    output wire                     m_axi_awvalid, // 主侧 AWVALID。
    input  wire                     m_axi_awready, // 主侧 AWREADY。
    output wire [DATA_WIDTH-1:0]    m_axi_wdata, // 主侧 W 数据。
    output wire [STRB_WIDTH-1:0]    m_axi_wstrb, // 主侧 W 字节使能。
    output wire                     m_axi_wlast, // 主侧 WLAST。
    output wire [WUSER_WIDTH-1:0]   m_axi_wuser, // 主侧 W 用户旁带。
    output wire                     m_axi_wvalid, // 主侧 WVALID。
    input  wire                     m_axi_wready, // 主侧 WREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 主侧 B ID（来自下游）。
    input  wire [1:0]               m_axi_bresp, // 主侧 B 响应。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 主侧 B 用户旁带。
    input  wire                     m_axi_bvalid, // 主侧 BVALID。
    output wire                     m_axi_bready, // 主侧 BREADY。
    output wire [ID_WIDTH-1:0]      m_axi_arid, // 主侧 AR ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr, // 主侧 AR 地址。
    output wire [7:0]               m_axi_arlen, // 主侧 AR 突发长度。
    output wire [2:0]               m_axi_arsize, // 主侧 AR 突发粒度。
    output wire [1:0]               m_axi_arburst, // 主侧 AR 突发类型。
    output wire                     m_axi_arlock, // 主侧 AR 锁属性。
    output wire [3:0]               m_axi_arcache, // 主侧 AR cache 属性。
    output wire [2:0]               m_axi_arprot, // 主侧 AR 保护属性。
    output wire [3:0]               m_axi_arqos, // 主侧 AR QoS。
    output wire [3:0]               m_axi_arregion, // 主侧 AR region。
    output wire [ARUSER_WIDTH-1:0]  m_axi_aruser, // 主侧 AR 用户旁带。
    output wire                     m_axi_arvalid, // 主侧 ARVALID。
    input  wire                     m_axi_arready, // 主侧 ARREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 主侧 R ID（来自下游）。
    input  wire [DATA_WIDTH-1:0]    m_axi_rdata, // 主侧 R 数据。
    input  wire [1:0]               m_axi_rresp, // 主侧 R 响应。
    input  wire                     m_axi_rlast, // 主侧 RLAST。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 主侧 R 用户旁带。
    input  wire                     m_axi_rvalid, // 主侧 RVALID。
    output wire                     m_axi_rready // 主侧 RREADY。
);

axi_register_wr #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .AWUSER_ENABLE(AWUSER_ENABLE),
    .AWUSER_WIDTH(AWUSER_WIDTH),
    .WUSER_ENABLE(WUSER_ENABLE),
    .WUSER_WIDTH(WUSER_WIDTH),
    .BUSER_ENABLE(BUSER_ENABLE),
    .BUSER_WIDTH(BUSER_WIDTH),
    .AW_REG_TYPE(AW_REG_TYPE),
    .W_REG_TYPE(W_REG_TYPE),
    .B_REG_TYPE(B_REG_TYPE)
)
axi_register_wr_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI 从接口
     */
    .s_axi_awid(s_axi_awid),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awlen(s_axi_awlen),
    .s_axi_awsize(s_axi_awsize),
    .s_axi_awburst(s_axi_awburst),
    .s_axi_awlock(s_axi_awlock),
    .s_axi_awcache(s_axi_awcache),
    .s_axi_awprot(s_axi_awprot),
    .s_axi_awqos(s_axi_awqos),
    .s_axi_awregion(s_axi_awregion),
    .s_axi_awuser(s_axi_awuser),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wlast(s_axi_wlast),
    .s_axi_wuser(s_axi_wuser),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bid(s_axi_bid),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_buser(s_axi_buser),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),

    /*
     * AXI 主接口
     */
    .m_axi_awid(m_axi_awid),
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awlen(m_axi_awlen),
    .m_axi_awsize(m_axi_awsize),
    .m_axi_awburst(m_axi_awburst),
    .m_axi_awlock(m_axi_awlock),
    .m_axi_awcache(m_axi_awcache),
    .m_axi_awprot(m_axi_awprot),
    .m_axi_awqos(m_axi_awqos),
    .m_axi_awregion(m_axi_awregion),
    .m_axi_awuser(m_axi_awuser),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),
    .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wlast(m_axi_wlast),
    .m_axi_wuser(m_axi_wuser),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_wready(m_axi_wready),
    .m_axi_bid(m_axi_bid),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_buser(m_axi_buser),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready)
);

axi_register_rd #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .ARUSER_ENABLE(ARUSER_ENABLE),
    .ARUSER_WIDTH(ARUSER_WIDTH),
    .RUSER_ENABLE(RUSER_ENABLE),
    .RUSER_WIDTH(RUSER_WIDTH),
    .AR_REG_TYPE(AR_REG_TYPE),
    .R_REG_TYPE(R_REG_TYPE)
)
axi_register_rd_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI 从接口
     */
    .s_axi_arid(s_axi_arid),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arlen(s_axi_arlen),
    .s_axi_arsize(s_axi_arsize),
    .s_axi_arburst(s_axi_arburst),
    .s_axi_arlock(s_axi_arlock),
    .s_axi_arcache(s_axi_arcache),
    .s_axi_arprot(s_axi_arprot),
    .s_axi_arqos(s_axi_arqos),
    .s_axi_arregion(s_axi_arregion),
    .s_axi_aruser(s_axi_aruser),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rid(s_axi_rid),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rlast(s_axi_rlast),
    .s_axi_ruser(s_axi_ruser),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),

    /*
     * AXI 主接口
     */
    .m_axi_arid(m_axi_arid),
    .m_axi_araddr(m_axi_araddr),
    .m_axi_arlen(m_axi_arlen),
    .m_axi_arsize(m_axi_arsize),
    .m_axi_arburst(m_axi_arburst),
    .m_axi_arlock(m_axi_arlock),
    .m_axi_arcache(m_axi_arcache),
    .m_axi_arprot(m_axi_arprot),
    .m_axi_arqos(m_axi_arqos),
    .m_axi_arregion(m_axi_arregion),
    .m_axi_aruser(m_axi_aruser),
    .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rid(m_axi_rid),
    .m_axi_rdata(m_axi_rdata),
    .m_axi_rresp(m_axi_rresp),
    .m_axi_rlast(m_axi_rlast),
    .m_axi_ruser(m_axi_ruser),
    .m_axi_rvalid(m_axi_rvalid),
    .m_axi_rready(m_axi_rready)
);

endmodule

`resetall
