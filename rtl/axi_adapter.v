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
 * AXI4 位宽适配器
 *
 * 模块目录
 * 1) 封装并实例化写通路位宽适配器（`axi_adapter_wr`）。
 * 2) 封装并实例化读通路位宽适配器（`axi_adapter_rd`）。
 * 3) 在保持 AXI 协议语义不变前提下，完成从端与主端数据位宽转换。
 */
module axi_adapter #
(
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 输入侧（从接口）数据总线位宽
    parameter S_DATA_WIDTH = 32,
    // 输入侧（从接口）WSTRB 位宽（按字节 lane）
    parameter S_STRB_WIDTH = (S_DATA_WIDTH/8),
    // 输出侧（主接口）数据总线位宽
    parameter M_DATA_WIDTH = 32,
    // 输出侧（主接口）WSTRB 位宽（按字节 lane）
    parameter M_STRB_WIDTH = (M_DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 awuser 信号
    parameter AWUSER_ENABLE = 0,
    // awuser 信号位宽
    parameter AWUSER_WIDTH = 1,
    // 是否透传 wuser 信号
    parameter WUSER_ENABLE = 0,
    // wuser 信号位宽
    parameter WUSER_WIDTH = 1,
    // 是否透传 buser 信号
    parameter BUSER_ENABLE = 0,
    // buser 信号位宽
    parameter BUSER_WIDTH = 1,
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
    // 向更宽总线适配时，尽可能重打包为满宽突发，而不是透传窄突发
    parameter CONVERT_BURST = 1,
    // 向更宽总线适配时，对所有突发执行重打包，而不是透传窄突发
    parameter CONVERT_NARROW_BURST = 0,
    // 是否在适配器中透传 ID
    parameter FORWARD_ID = 0
)
(
    input  wire                     clk, // 读写位宽适配子模块共用时钟。
    input  wire                     rst, // 两个子模块共用同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // 从端 AW ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // 从端 AW 地址。
    input  wire [7:0]               s_axi_awlen, // 从端 AW 突发长度。
    input  wire [2:0]               s_axi_awsize, // 从端 AW 突发尺寸。
    input  wire [1:0]               s_axi_awburst, // 从端 AW 突发类型。
    input  wire                     s_axi_awlock, // 从端 AW 锁属性。
    input  wire [3:0]               s_axi_awcache, // 从端 AW cache 属性。
    input  wire [2:0]               s_axi_awprot, // 从端 AW 保护属性。
    input  wire [3:0]               s_axi_awqos, // 从端 AW QoS。
    input  wire [3:0]               s_axi_awregion, // 从端 AW region。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // 从端 AW 用户旁带信号。
    input  wire                     s_axi_awvalid, // 从端 AWVALID。
    output wire                     s_axi_awready, // 从端 AWREADY。
    input  wire [S_DATA_WIDTH-1:0]  s_axi_wdata, // 从端 W 数据（源位宽）。
    input  wire [S_STRB_WIDTH-1:0]  s_axi_wstrb, // 从端 W 字节使能（源位宽）。
    input  wire                     s_axi_wlast, // 从端 WLAST。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // 从端 W 用户旁带信号。
    input  wire                     s_axi_wvalid, // 从端 WVALID。
    output wire                     s_axi_wready, // 从端 WREADY。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 从端 B ID。
    output wire [1:0]               s_axi_bresp, // 从端 B 响应码。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 从端 B 用户旁带信号。
    output wire                     s_axi_bvalid, // 从端 BVALID。
    input  wire                     s_axi_bready, // 从端 BREADY。
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // 从端 AR ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // 从端 AR 地址。
    input  wire [7:0]               s_axi_arlen, // 从端 AR 突发长度。
    input  wire [2:0]               s_axi_arsize, // 从端 AR 突发尺寸。
    input  wire [1:0]               s_axi_arburst, // 从端 AR 突发类型。
    input  wire                     s_axi_arlock, // 从端 AR 锁属性。
    input  wire [3:0]               s_axi_arcache, // 从端 AR cache 属性。
    input  wire [2:0]               s_axi_arprot, // 从端 AR 保护属性。
    input  wire [3:0]               s_axi_arqos, // 从端 AR QoS。
    input  wire [3:0]               s_axi_arregion, // 从端 AR region。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // 从端 AR 用户旁带信号。
    input  wire                     s_axi_arvalid, // 从端 ARVALID。
    output wire                     s_axi_arready, // 从端 ARREADY。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 从端 R ID。
    output wire [S_DATA_WIDTH-1:0]  s_axi_rdata, // 从端 R 数据（源位宽）。
    output wire [1:0]               s_axi_rresp, // 从端 R 响应码。
    output wire                     s_axi_rlast, // 从端 RLAST。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 从端 R 用户旁带信号。
    output wire                     s_axi_rvalid, // 从端 RVALID。
    input  wire                     s_axi_rready, // 从端 RREADY。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_awid, // 主端 AW ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr, // 主端 AW 地址。
    output wire [7:0]               m_axi_awlen, // 主端 AW 突发长度。
    output wire [2:0]               m_axi_awsize, // 主端 AW 突发尺寸。
    output wire [1:0]               m_axi_awburst, // 主端 AW 突发类型。
    output wire                     m_axi_awlock, // 主端 AW 锁属性。
    output wire [3:0]               m_axi_awcache, // 主端 AW cache 属性。
    output wire [2:0]               m_axi_awprot, // 主端 AW 保护属性。
    output wire [3:0]               m_axi_awqos, // 主端 AW QoS。
    output wire [3:0]               m_axi_awregion, // 主端 AW region。
    output wire [AWUSER_WIDTH-1:0]  m_axi_awuser, // 主端 AW 用户旁带信号。
    output wire                     m_axi_awvalid, // 主端 AWVALID。
    input  wire                     m_axi_awready, // 主端 AWREADY。
    output wire [M_DATA_WIDTH-1:0]  m_axi_wdata, // 主端 W 数据（目标位宽）。
    output wire [M_STRB_WIDTH-1:0]  m_axi_wstrb, // 主端 W 字节使能（目标位宽）。
    output wire                     m_axi_wlast, // 主端 WLAST。
    output wire [WUSER_WIDTH-1:0]   m_axi_wuser, // 主端 W 用户旁带信号。
    output wire                     m_axi_wvalid, // 主端 WVALID。
    input  wire                     m_axi_wready, // 主端 WREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 主端 B ID。
    input  wire [1:0]               m_axi_bresp, // 主端 B 响应码。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 主端 B 用户旁带信号。
    input  wire                     m_axi_bvalid, // 主端 BVALID。
    output wire                     m_axi_bready, // 主端 BREADY。
    output wire [ID_WIDTH-1:0]      m_axi_arid, // 主端 AR ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr, // 主端 AR 地址。
    output wire [7:0]               m_axi_arlen, // 主端 AR 突发长度。
    output wire [2:0]               m_axi_arsize, // 主端 AR 突发尺寸。
    output wire [1:0]               m_axi_arburst, // 主端 AR 突发类型。
    output wire                     m_axi_arlock, // 主端 AR 锁属性。
    output wire [3:0]               m_axi_arcache, // 主端 AR cache 属性。
    output wire [2:0]               m_axi_arprot, // 主端 AR 保护属性。
    output wire [3:0]               m_axi_arqos, // 主端 AR QoS。
    output wire [3:0]               m_axi_arregion, // 主端 AR region。
    output wire [ARUSER_WIDTH-1:0]  m_axi_aruser, // 主端 AR 用户旁带信号。
    output wire                     m_axi_arvalid, // 主端 ARVALID。
    input  wire                     m_axi_arready, // 主端 ARREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 主端 R ID。
    input  wire [M_DATA_WIDTH-1:0]  m_axi_rdata, // 主端 R 数据（目标位宽）。
    input  wire [1:0]               m_axi_rresp, // 主端 R 响应码。
    input  wire                     m_axi_rlast, // 主端 RLAST。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 主端 R 用户旁带信号。
    input  wire                     m_axi_rvalid, // 主端 RVALID。
    output wire                     m_axi_rready // 主端 RREADY。
);

axi_adapter_wr #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .S_DATA_WIDTH(S_DATA_WIDTH),
    .S_STRB_WIDTH(S_STRB_WIDTH),
    .M_DATA_WIDTH(M_DATA_WIDTH),
    .M_STRB_WIDTH(M_STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .AWUSER_ENABLE(AWUSER_ENABLE),
    .AWUSER_WIDTH(AWUSER_WIDTH),
    .WUSER_ENABLE(WUSER_ENABLE),
    .WUSER_WIDTH(WUSER_WIDTH),
    .BUSER_ENABLE(BUSER_ENABLE),
    .BUSER_WIDTH(BUSER_WIDTH),
    .CONVERT_BURST(CONVERT_BURST),
    .CONVERT_NARROW_BURST(CONVERT_NARROW_BURST),
    .FORWARD_ID(FORWARD_ID)
)
axi_adapter_wr_inst (
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

axi_adapter_rd #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .S_DATA_WIDTH(S_DATA_WIDTH),
    .S_STRB_WIDTH(S_STRB_WIDTH),
    .M_DATA_WIDTH(M_DATA_WIDTH),
    .M_STRB_WIDTH(M_STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .ARUSER_ENABLE(ARUSER_ENABLE),
    .ARUSER_WIDTH(ARUSER_WIDTH),
    .RUSER_ENABLE(RUSER_ENABLE),
    .RUSER_WIDTH(RUSER_WIDTH),
    .CONVERT_BURST(CONVERT_BURST),
    .CONVERT_NARROW_BURST(CONVERT_NARROW_BURST),
    .FORWARD_ID(FORWARD_ID)
)
axi_adapter_rd_inst (
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
