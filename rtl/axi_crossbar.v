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
 * AXI4 交叉开关
 *
 * 模块目录
 * 1) 读写通道完全拆分，分别交给 `axi_crossbar_wr` 和 `axi_crossbar_rd`。
 * 2) 支持并发多主多从路由，每个目标口独立仲裁，不像 interconnect 只串行服务一个事务。
 * 3) 顶层主要负责参数汇总和端口拼接，不承载核心状态机。
 */
module axi_crossbar #
(
    // AXI 输入端口数量（从接口数量）
    parameter S_COUNT = 4,
    // AXI 输出端口数量（主接口数量）
    parameter M_COUNT = 4,
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 输入 ID 位宽（来自 AXI 主设备）
    parameter S_ID_WIDTH = 8,
    // 输出 ID 位宽（发往 AXI 从设备）
    // 包含响应路由所需附加位
    parameter M_ID_WIDTH = S_ID_WIDTH+$clog2(S_COUNT),
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
    // 每个从接口可并发唯一 ID 数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_THREADS = {S_COUNT{32'd2}},
    // 每个从接口可并发事务数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_ACCEPT = {S_COUNT{32'd16}},
    // 每个主接口地址区域数量
    parameter M_REGIONS = 1,
    // 主接口基地址表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 ADDR_WIDTH 位字段
    // 置 0 时按 M_ADDR_WIDTH 自动生成默认地址映射
    parameter M_BASE_ADDR = 0,
    // 主接口地址宽度表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 32 位字段
    parameter M_ADDR_WIDTH = {M_COUNT{{M_REGIONS{32'd24}}}},
    // 接口间读通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT_READ = {M_COUNT{{S_COUNT{1'b1}}}},
    // 接口间写通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT_WRITE = {M_COUNT{{S_COUNT{1'b1}}}},
    // 每个主接口可并发事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd4}},
    // 安全主端口配置（基于 awprot/arprot 拒绝访问）
    // M_COUNT 位
    parameter M_SECURE = {M_COUNT{1'b0}},
    // 从接口 AW 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_AW_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 W 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_W_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 B 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_B_REG_TYPE = {S_COUNT{2'd1}},
    // 从接口 AR 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_AR_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 R 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_R_REG_TYPE = {S_COUNT{2'd2}},
    // 主接口 AW 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_AW_REG_TYPE = {M_COUNT{2'd1}},
    // 主接口 W 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_W_REG_TYPE = {M_COUNT{2'd2}},
    // 主接口 B 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_B_REG_TYPE = {M_COUNT{2'd0}},
    // 主接口 AR 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_AR_REG_TYPE = {M_COUNT{2'd1}},
    // 主接口 R 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_R_REG_TYPE = {M_COUNT{2'd0}}
)
(
    input  wire                             clk, // Crossbar 主时钟。
    input  wire                             rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_awid, // 所有 S 口拼接的 AWID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axi_awaddr, // 所有 S 口拼接的 AWADDR。
    input  wire [S_COUNT*8-1:0]             s_axi_awlen, // 所有 S 口拼接的 AWLEN。
    input  wire [S_COUNT*3-1:0]             s_axi_awsize, // 所有 S 口拼接的 AWSIZE。
    input  wire [S_COUNT*2-1:0]             s_axi_awburst, // 所有 S 口拼接的 AWBURST。
    input  wire [S_COUNT-1:0]               s_axi_awlock, // 所有 S 口 AWLOCK。
    input  wire [S_COUNT*4-1:0]             s_axi_awcache, // 所有 S 口拼接的 AWCACHE。
    input  wire [S_COUNT*3-1:0]             s_axi_awprot, // 所有 S 口拼接的 AWPROT。
    input  wire [S_COUNT*4-1:0]             s_axi_awqos, // 所有 S 口拼接的 AWQOS。
    input  wire [S_COUNT*AWUSER_WIDTH-1:0]  s_axi_awuser, // 所有 S 口拼接的 AWUSER。
    input  wire [S_COUNT-1:0]               s_axi_awvalid, // 所有 S 口 AWVALID。
    output wire [S_COUNT-1:0]               s_axi_awready, // 所有 S 口 AWREADY。
    input  wire [S_COUNT*DATA_WIDTH-1:0]    s_axi_wdata, // 所有 S 口拼接的 WDATA。
    input  wire [S_COUNT*STRB_WIDTH-1:0]    s_axi_wstrb, // 所有 S 口拼接的 WSTRB。
    input  wire [S_COUNT-1:0]               s_axi_wlast, // 所有 S 口 WLAST。
    input  wire [S_COUNT*WUSER_WIDTH-1:0]   s_axi_wuser, // 所有 S 口拼接的 WUSER。
    input  wire [S_COUNT-1:0]               s_axi_wvalid, // 所有 S 口 WVALID。
    output wire [S_COUNT-1:0]               s_axi_wready, // 所有 S 口 WREADY。
    output wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_bid, // 所有 S 口拼接的 BID。
    output wire [S_COUNT*2-1:0]             s_axi_bresp, // 所有 S 口拼接的 BRESP。
    output wire [S_COUNT*BUSER_WIDTH-1:0]   s_axi_buser, // 所有 S 口拼接的 BUSER。
    output wire [S_COUNT-1:0]               s_axi_bvalid, // 所有 S 口 BVALID。
    input  wire [S_COUNT-1:0]               s_axi_bready, // 所有 S 口 BREADY。
    input  wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_arid, // 所有 S 口拼接的 ARID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axi_araddr, // 所有 S 口拼接的 ARADDR。
    input  wire [S_COUNT*8-1:0]             s_axi_arlen, // 所有 S 口拼接的 ARLEN。
    input  wire [S_COUNT*3-1:0]             s_axi_arsize, // 所有 S 口拼接的 ARSIZE。
    input  wire [S_COUNT*2-1:0]             s_axi_arburst, // 所有 S 口拼接的 ARBURST。
    input  wire [S_COUNT-1:0]               s_axi_arlock, // 所有 S 口 ARLOCK。
    input  wire [S_COUNT*4-1:0]             s_axi_arcache, // 所有 S 口拼接的 ARCACHE。
    input  wire [S_COUNT*3-1:0]             s_axi_arprot, // 所有 S 口拼接的 ARPROT。
    input  wire [S_COUNT*4-1:0]             s_axi_arqos, // 所有 S 口拼接的 ARQOS。
    input  wire [S_COUNT*ARUSER_WIDTH-1:0]  s_axi_aruser, // 所有 S 口拼接的 ARUSER。
    input  wire [S_COUNT-1:0]               s_axi_arvalid, // 所有 S 口 ARVALID。
    output wire [S_COUNT-1:0]               s_axi_arready, // 所有 S 口 ARREADY。
    output wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_rid, // 所有 S 口拼接的 RID。
    output wire [S_COUNT*DATA_WIDTH-1:0]    s_axi_rdata, // 所有 S 口拼接的 RDATA。
    output wire [S_COUNT*2-1:0]             s_axi_rresp, // 所有 S 口拼接的 RRESP。
    output wire [S_COUNT-1:0]               s_axi_rlast, // 所有 S 口 RLAST。
    output wire [S_COUNT*RUSER_WIDTH-1:0]   s_axi_ruser, // 所有 S 口拼接的 RUSER。
    output wire [S_COUNT-1:0]               s_axi_rvalid, // 所有 S 口 RVALID。
    input  wire [S_COUNT-1:0]               s_axi_rready, // 所有 S 口 RREADY。

    /*
     * AXI 主接口
     */
    output wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_awid, // 所有 M 口拼接的 AWID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axi_awaddr, // 所有 M 口拼接的 AWADDR 输出。
    output wire [M_COUNT*8-1:0]             m_axi_awlen, // 所有 M 口拼接的 AWLEN 输出。
    output wire [M_COUNT*3-1:0]             m_axi_awsize, // 所有 M 口拼接的 AWSIZE 输出。
    output wire [M_COUNT*2-1:0]             m_axi_awburst, // 所有 M 口拼接的 AWBURST 输出。
    output wire [M_COUNT-1:0]               m_axi_awlock, // 所有 M 口 AWLOCK 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awcache, // 所有 M 口拼接的 AWCACHE 输出。
    output wire [M_COUNT*3-1:0]             m_axi_awprot, // 所有 M 口拼接的 AWPROT 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awqos, // 所有 M 口拼接的 AWQOS 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awregion, // 所有 M 口拼接的 AWREGION 输出。
    output wire [M_COUNT*AWUSER_WIDTH-1:0]  m_axi_awuser, // 所有 M 口拼接的 AWUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_awvalid, // 所有 M 口 AWVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_awready, // 所有 M 口 AWREADY 输入。
    output wire [M_COUNT*DATA_WIDTH-1:0]    m_axi_wdata, // 所有 M 口拼接的 WDATA 输出。
    output wire [M_COUNT*STRB_WIDTH-1:0]    m_axi_wstrb, // 所有 M 口拼接的 WSTRB 输出。
    output wire [M_COUNT-1:0]               m_axi_wlast, // 所有 M 口 WLAST 输出。
    output wire [M_COUNT*WUSER_WIDTH-1:0]   m_axi_wuser, // 所有 M 口拼接的 WUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_wvalid, // 所有 M 口 WVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_wready, // 所有 M 口 WREADY 输入。
    input  wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_bid, // 所有 M 口拼接的 BID 输入。
    input  wire [M_COUNT*2-1:0]             m_axi_bresp, // 所有 M 口拼接的 BRESP 输入。
    input  wire [M_COUNT*BUSER_WIDTH-1:0]   m_axi_buser, // 所有 M 口拼接的 BUSER 输入。
    input  wire [M_COUNT-1:0]               m_axi_bvalid, // 所有 M 口 BVALID 输入。
    output wire [M_COUNT-1:0]               m_axi_bready, // 所有 M 口 BREADY 输出。
    output wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_arid, // 所有 M 口拼接的 ARID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axi_araddr, // 所有 M 口拼接的 ARADDR 输出。
    output wire [M_COUNT*8-1:0]             m_axi_arlen, // 所有 M 口拼接的 ARLEN 输出。
    output wire [M_COUNT*3-1:0]             m_axi_arsize, // 所有 M 口拼接的 ARSIZE 输出。
    output wire [M_COUNT*2-1:0]             m_axi_arburst, // 所有 M 口拼接的 ARBURST 输出。
    output wire [M_COUNT-1:0]               m_axi_arlock, // 所有 M 口 ARLOCK 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arcache, // 所有 M 口拼接的 ARCACHE 输出。
    output wire [M_COUNT*3-1:0]             m_axi_arprot, // 所有 M 口拼接的 ARPROT 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arqos, // 所有 M 口拼接的 ARQOS 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arregion, // 所有 M 口拼接的 ARREGION 输出。
    output wire [M_COUNT*ARUSER_WIDTH-1:0]  m_axi_aruser, // 所有 M 口拼接的 ARUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_arvalid, // 所有 M 口 ARVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_arready, // 所有 M 口 ARREADY 输入。
    input  wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_rid, // 所有 M 口拼接的 RID 输入。
    input  wire [M_COUNT*DATA_WIDTH-1:0]    m_axi_rdata, // 所有 M 口拼接的 RDATA 输入。
    input  wire [M_COUNT*2-1:0]             m_axi_rresp, // 所有 M 口拼接的 RRESP 输入。
    input  wire [M_COUNT-1:0]               m_axi_rlast, // 所有 M 口 RLAST 输入。
    input  wire [M_COUNT*RUSER_WIDTH-1:0]   m_axi_ruser, // 所有 M 口拼接的 RUSER 输入。
    input  wire [M_COUNT-1:0]               m_axi_rvalid, // 所有 M 口 RVALID 输入。
    output wire [M_COUNT-1:0]               m_axi_rready // 所有 M 口 RREADY 输出。
);

axi_crossbar_wr #(
    .S_COUNT(S_COUNT),
    .M_COUNT(M_COUNT),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .S_ID_WIDTH(S_ID_WIDTH),
    .M_ID_WIDTH(M_ID_WIDTH),
    .AWUSER_ENABLE(AWUSER_ENABLE),
    .AWUSER_WIDTH(AWUSER_WIDTH),
    .WUSER_ENABLE(WUSER_ENABLE),
    .WUSER_WIDTH(WUSER_WIDTH),
    .BUSER_ENABLE(BUSER_ENABLE),
    .BUSER_WIDTH(BUSER_WIDTH),
    .S_THREADS(S_THREADS),
    .S_ACCEPT(S_ACCEPT),
    .M_REGIONS(M_REGIONS),
    .M_BASE_ADDR(M_BASE_ADDR),
    .M_ADDR_WIDTH(M_ADDR_WIDTH),
    .M_CONNECT(M_CONNECT_WRITE),
    .M_ISSUE(M_ISSUE),
    .M_SECURE(M_SECURE),
    .S_AW_REG_TYPE(S_AW_REG_TYPE),
    .S_W_REG_TYPE (S_W_REG_TYPE),
    .S_B_REG_TYPE (S_B_REG_TYPE)
)
axi_crossbar_wr_inst (
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

axi_crossbar_rd #(
    .S_COUNT(S_COUNT),
    .M_COUNT(M_COUNT),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .S_ID_WIDTH(S_ID_WIDTH),
    .M_ID_WIDTH(M_ID_WIDTH),
    .ARUSER_ENABLE(ARUSER_ENABLE),
    .ARUSER_WIDTH(ARUSER_WIDTH),
    .RUSER_ENABLE(RUSER_ENABLE),
    .RUSER_WIDTH(RUSER_WIDTH),
    .S_THREADS(S_THREADS),
    .S_ACCEPT(S_ACCEPT),
    .M_REGIONS(M_REGIONS),
    .M_BASE_ADDR(M_BASE_ADDR),
    .M_ADDR_WIDTH(M_ADDR_WIDTH),
    .M_CONNECT(M_CONNECT_READ),
    .M_ISSUE(M_ISSUE),
    .M_SECURE(M_SECURE),
    .S_AR_REG_TYPE(S_AR_REG_TYPE),
    .S_R_REG_TYPE (S_R_REG_TYPE)
)
axi_crossbar_rd_inst (
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
