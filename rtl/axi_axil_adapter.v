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
 * AXI4 到 AXI4-Lite 适配器
 *
 * 模块目录
 * 1) 封装并实例化 AXI->AXI-Lite 写适配器（`axi_axil_adapter_wr`）。
 * 2) 封装并实例化 AXI->AXI-Lite 读适配器（`axi_axil_adapter_rd`）。
 * 3) 将支持突发的 AXI 事务转换为 AXI-Lite 单拍事务。
 */
module axi_axil_adapter #
(
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 输入侧（从接口）AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = 32,
    // 输入侧（从接口）AXI WSTRB 位宽（按字节 lane）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 信号位宽
    parameter AXI_ID_WIDTH = 8,
    // 输出侧（主接口）AXI-Lite 数据总线位宽
    parameter AXIL_DATA_WIDTH = 32,
    // 输出侧（主接口）AXI-Lite WSTRB 位宽（按字节 lane）
    parameter AXIL_STRB_WIDTH = (AXIL_DATA_WIDTH/8),
    // 向更宽总线适配时，尽可能重打包为满宽突发，而不是透传窄突发
    parameter CONVERT_BURST = 1,
    // 向更宽总线适配时，对所有突发执行重打包，而不是透传窄突发
    parameter CONVERT_NARROW_BURST = 0
)
(
    input  wire                        clk, // 读写 AXI->AXI-Lite 适配路径共用时钟。
    input  wire                        rst, // 两条适配路径共用同步复位。

    /*
     * AXI 从接口
     */
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_awid, // AXI 从端 AW ID。
    input  wire [ADDR_WIDTH-1:0]       s_axi_awaddr, // AXI 从端 AW 地址。
    input  wire [7:0]                  s_axi_awlen, // AXI 从端 AW 突发长度。
    input  wire [2:0]                  s_axi_awsize, // AXI 从端 AW 突发尺寸。
    input  wire [1:0]                  s_axi_awburst, // AXI 从端 AW 突发类型。
    input  wire                        s_axi_awlock, // AXI 从端 AW 锁属性。
    input  wire [3:0]                  s_axi_awcache, // AXI 从端 AW cache 属性。
    input  wire [2:0]                  s_axi_awprot, // AXI 从端 AW 保护属性。
    input  wire                        s_axi_awvalid, // AXI 从端 AWVALID。
    output wire                        s_axi_awready, // AXI 从端 AWREADY。
    input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata, // AXI 从端 W 数据。
    input  wire [AXI_STRB_WIDTH-1:0]   s_axi_wstrb, // AXI 从端 W 字节使能。
    input  wire                        s_axi_wlast, // AXI 从端 WLAST。
    input  wire                        s_axi_wvalid, // AXI 从端 WVALID。
    output wire                        s_axi_wready, // AXI 从端 WREADY。
    output wire [AXI_ID_WIDTH-1:0]     s_axi_bid, // AXI 从端 B ID。
    output wire [1:0]                  s_axi_bresp, // AXI 从端 B 响应码。
    output wire                        s_axi_bvalid, // AXI 从端 BVALID。
    input  wire                        s_axi_bready, // AXI 从端 BREADY。
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_arid, // AXI 从端 AR ID。
    input  wire [ADDR_WIDTH-1:0]       s_axi_araddr, // AXI 从端 AR 地址。
    input  wire [7:0]                  s_axi_arlen, // AXI 从端 AR 突发长度。
    input  wire [2:0]                  s_axi_arsize, // AXI 从端 AR 突发尺寸。
    input  wire [1:0]                  s_axi_arburst, // AXI 从端 AR 突发类型。
    input  wire                        s_axi_arlock, // AXI 从端 AR 锁属性。
    input  wire [3:0]                  s_axi_arcache, // AXI 从端 AR cache 属性。
    input  wire [2:0]                  s_axi_arprot, // AXI 从端 AR 保护属性。
    input  wire                        s_axi_arvalid, // AXI 从端 ARVALID。
    output wire                        s_axi_arready, // AXI 从端 ARREADY。
    output wire [AXI_ID_WIDTH-1:0]     s_axi_rid, // AXI 从端 R ID。
    output wire [AXI_DATA_WIDTH-1:0]   s_axi_rdata, // AXI 从端 R 数据。
    output wire [1:0]                  s_axi_rresp, // AXI 从端 R 响应码。
    output wire                        s_axi_rlast, // AXI 从端 RLAST。
    output wire                        s_axi_rvalid, // AXI 从端 RVALID。
    input  wire                        s_axi_rready, // AXI 从端 RREADY。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]       m_axil_awaddr, // AXI-Lite 主端 AW 地址。
    output wire [2:0]                  m_axil_awprot, // AXI-Lite 主端 AW 保护属性。
    output wire                        m_axil_awvalid, // AXI-Lite 主端 AWVALID。
    input  wire                        m_axil_awready, // AXI-Lite 主端 AWREADY。
    output wire [AXIL_DATA_WIDTH-1:0]  m_axil_wdata, // AXI-Lite 主端 W 数据。
    output wire [AXIL_STRB_WIDTH-1:0]  m_axil_wstrb, // AXI-Lite 主端 W 字节使能。
    output wire                        m_axil_wvalid, // AXI-Lite 主端 WVALID。
    input  wire                        m_axil_wready, // AXI-Lite 主端 WREADY。
    input  wire [1:0]                  m_axil_bresp, // AXI-Lite 主端 B 响应码。
    input  wire                        m_axil_bvalid, // AXI-Lite 主端 BVALID。
    output wire                        m_axil_bready, // AXI-Lite 主端 BREADY。
    output wire [ADDR_WIDTH-1:0]       m_axil_araddr, // AXI-Lite 主端 AR 地址。
    output wire [2:0]                  m_axil_arprot, // AXI-Lite 主端 AR 保护属性。
    output wire                        m_axil_arvalid, // AXI-Lite 主端 ARVALID。
    input  wire                        m_axil_arready, // AXI-Lite 主端 ARREADY。
    input  wire [AXIL_DATA_WIDTH-1:0]  m_axil_rdata, // AXI-Lite 主端 R 数据。
    input  wire [1:0]                  m_axil_rresp, // AXI-Lite 主端 R 响应码。
    input  wire                        m_axil_rvalid, // AXI-Lite 主端 RVALID。
    output wire                        m_axil_rready // AXI-Lite 主端 RREADY。
);


axi_axil_adapter_wr #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXIL_DATA_WIDTH(AXIL_DATA_WIDTH),
    .AXIL_STRB_WIDTH(AXIL_STRB_WIDTH),
    .CONVERT_BURST(CONVERT_BURST),
    .CONVERT_NARROW_BURST(CONVERT_NARROW_BURST)
)
axi_axil_adapter_wr_inst (
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
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wlast(s_axi_wlast),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bid(s_axi_bid),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),

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

axi_axil_adapter_rd #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXIL_DATA_WIDTH(AXIL_DATA_WIDTH),
    .AXIL_STRB_WIDTH(AXIL_STRB_WIDTH),
    .CONVERT_BURST(CONVERT_BURST),
    .CONVERT_NARROW_BURST(CONVERT_NARROW_BURST)
)
axi_axil_adapter_rd_inst (
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
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rid(s_axi_rid),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rlast(s_axi_rlast),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),

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
