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
 * AXI4 FIFO 模块
 *
 * 模块目录
 * 1) 写通道由 axi_fifo_wr 负责：AW/W 可选缓存后再发往下游，B 返回直通/对齐。
 * 2) 读通道由 axi_fifo_rd 负责：AR 可选延迟，R 通道进入 FIFO 后回给上游。
 * 3) 该顶层主要做读写子模块拼接，不额外引入状态机。
 */
module axi_fifo #
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
    // AWUSER 信号位宽
    parameter AWUSER_WIDTH = 1,
    // 是否透传 WUSER 信号
    parameter WUSER_ENABLE = 0,
    // WUSER 信号位宽
    parameter WUSER_WIDTH = 1,
    // 是否透传 BUSER 信号
    parameter BUSER_ENABLE = 0,
    // BUSER 信号位宽
    parameter BUSER_WIDTH = 1,
    // 是否透传 ARUSER 信号
    parameter ARUSER_ENABLE = 0,
    // ARUSER 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 RUSER 信号
    parameter RUSER_ENABLE = 0,
    // RUSER 信号位宽
    parameter RUSER_WIDTH = 1,
    // 写数据 FIFO 深度（拍）
    parameter WRITE_FIFO_DEPTH = 32,
    // 读数据 FIFO 深度（拍）
    parameter READ_FIFO_DEPTH = 32,
    // 尽可能等待写数据进入 FIFO 后再放行写地址
    parameter WRITE_FIFO_DELAY = 0,
    // 尽可能等待 FIFO 有足够读返回空间后再放行读地址
    parameter READ_FIFO_DELAY = 0
)
(
    input  wire                     clk, // 全局时钟。
    input  wire                     rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // 上游写地址 ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // 上游写地址。
    input  wire [7:0]               s_axi_awlen, // 上游写突发长度。
    input  wire [2:0]               s_axi_awsize, // 上游写突发 beat 大小。
    input  wire [1:0]               s_axi_awburst, // 上游写突发类型。
    input  wire                     s_axi_awlock, // 上游写锁属性。
    input  wire [3:0]               s_axi_awcache, // 上游写 cache 属性。
    input  wire [2:0]               s_axi_awprot, // 上游写保护属性。
    input  wire [3:0]               s_axi_awqos, // 上游写 QoS。
    input  wire [3:0]               s_axi_awregion, // 上游写 region。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // 上游写地址 user sideband。
    input  wire                     s_axi_awvalid, // 上游写地址有效。
    output wire                     s_axi_awready, // FIFO 可接收写地址。
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata, // 上游写数据。
    input  wire [STRB_WIDTH-1:0]    s_axi_wstrb, // 上游写字节使能。
    input  wire                     s_axi_wlast, // 上游写突发最后一拍。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // 上游写数据 user sideband。
    input  wire                     s_axi_wvalid, // 上游写数据有效。
    output wire                     s_axi_wready, // FIFO 可接收写数据。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 返回给上游的写响应 ID。
    output wire [1:0]               s_axi_bresp, // 返回给上游的写响应状态。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 返回给上游的写响应 user。
    output wire                     s_axi_bvalid, // 返回给上游的写响应有效。
    input  wire                     s_axi_bready, // 上游接受写响应。
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // 上游读地址 ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // 上游读地址。
    input  wire [7:0]               s_axi_arlen, // 上游读突发长度。
    input  wire [2:0]               s_axi_arsize, // 上游读突发 beat 大小。
    input  wire [1:0]               s_axi_arburst, // 上游读突发类型。
    input  wire                     s_axi_arlock, // 上游读锁属性。
    input  wire [3:0]               s_axi_arcache, // 上游读 cache 属性。
    input  wire [2:0]               s_axi_arprot, // 上游读保护属性。
    input  wire [3:0]               s_axi_arqos, // 上游读 QoS。
    input  wire [3:0]               s_axi_arregion, // 上游读 region。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // 上游读地址 user sideband。
    input  wire                     s_axi_arvalid, // 上游读地址有效。
    output wire                     s_axi_arready, // FIFO 可接收读地址。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 返回给上游的读数据 ID。
    output wire [DATA_WIDTH-1:0]    s_axi_rdata, // 返回给上游的读数据。
    output wire [1:0]               s_axi_rresp, // 返回给上游的读响应状态。
    output wire                     s_axi_rlast, // 返回给上游的读突发最后一拍。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 返回给上游的读数据 user。
    output wire                     s_axi_rvalid, // 返回给上游的读数据有效。
    input  wire                     s_axi_rready, // 上游接受读数据。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_awid, // 下游写地址 ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr, // 下游写地址。
    output wire [7:0]               m_axi_awlen, // 下游写突发长度。
    output wire [2:0]               m_axi_awsize, // 下游写突发 beat 大小。
    output wire [1:0]               m_axi_awburst, // 下游写突发类型。
    output wire                     m_axi_awlock, // 下游写锁属性。
    output wire [3:0]               m_axi_awcache, // 下游写 cache 属性。
    output wire [2:0]               m_axi_awprot, // 下游写保护属性。
    output wire [3:0]               m_axi_awqos, // 下游写 QoS。
    output wire [3:0]               m_axi_awregion, // 下游写 region。
    output wire [AWUSER_WIDTH-1:0]  m_axi_awuser, // 下游写地址 user sideband。
    output wire                     m_axi_awvalid, // 下游写地址有效。
    input  wire                     m_axi_awready, // 下游可接收写地址。
    output wire [DATA_WIDTH-1:0]    m_axi_wdata, // 下游写数据。
    output wire [STRB_WIDTH-1:0]    m_axi_wstrb, // 下游写字节使能。
    output wire                     m_axi_wlast, // 下游写突发最后一拍。
    output wire [WUSER_WIDTH-1:0]   m_axi_wuser, // 下游写数据 user sideband。
    output wire                     m_axi_wvalid, // 下游写数据有效。
    input  wire                     m_axi_wready, // 下游可接收写数据。
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 下游写响应 ID。
    input  wire [1:0]               m_axi_bresp, // 下游写响应状态。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 下游写响应 user。
    input  wire                     m_axi_bvalid, // 下游写响应有效。
    output wire                     m_axi_bready, // FIFO 接受下游写响应。
    output wire [ID_WIDTH-1:0]      m_axi_arid, // 下游读地址 ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr, // 下游读地址。
    output wire [7:0]               m_axi_arlen, // 下游读突发长度。
    output wire [2:0]               m_axi_arsize, // 下游读突发 beat 大小。
    output wire [1:0]               m_axi_arburst, // 下游读突发类型。
    output wire                     m_axi_arlock, // 下游读锁属性。
    output wire [3:0]               m_axi_arcache, // 下游读 cache 属性。
    output wire [2:0]               m_axi_arprot, // 下游读保护属性。
    output wire [3:0]               m_axi_arqos, // 下游读 QoS。
    output wire [3:0]               m_axi_arregion, // 下游读 region。
    output wire [ARUSER_WIDTH-1:0]  m_axi_aruser, // 下游读地址 user sideband。
    output wire                     m_axi_arvalid, // 下游读地址有效。
    input  wire                     m_axi_arready, // 下游可接收读地址。
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 下游读数据 ID。
    input  wire [DATA_WIDTH-1:0]    m_axi_rdata, // 下游读数据。
    input  wire [1:0]               m_axi_rresp, // 下游读响应状态。
    input  wire                     m_axi_rlast, // 下游读突发最后一拍。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 下游读数据 user。
    input  wire                     m_axi_rvalid, // 下游读数据有效。
    output wire                     m_axi_rready // FIFO 接受下游读数据。
);

axi_fifo_wr #(
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
    .FIFO_DEPTH(WRITE_FIFO_DEPTH),
    .FIFO_DELAY(WRITE_FIFO_DELAY)
)
axi_fifo_wr_inst (
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

axi_fifo_rd #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .ARUSER_ENABLE(ARUSER_ENABLE),
    .ARUSER_WIDTH(ARUSER_WIDTH),
    .RUSER_ENABLE(RUSER_ENABLE),
    .RUSER_WIDTH(RUSER_WIDTH),
    .FIFO_DEPTH(READ_FIFO_DEPTH),
    .FIFO_DELAY(READ_FIFO_DELAY)
)
axi_fifo_rd_inst (
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
