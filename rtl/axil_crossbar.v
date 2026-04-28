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
 * AXI4-Lite 交叉开关
 *
 * 模块目录
 * 1) 顶层封装：将写通路交叉开关和读通路交叉开关组合在一起。
 * 2) 本模块不保存数据通路状态；仲裁、解码与顺序控制都在 axil_crossbar_wr/rd 内实现。
 * 3) 端口全部向量化：每个切片对应一个从/主接口。
 */
module axil_crossbar #
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
    // 每个从接口可并发处理事务数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_ACCEPT = {S_COUNT{32'd16}},
    // 每个主接口的地址区域数量
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
    // 每个主接口可并发处理事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd16}},
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
    input  wire                             clk, // 交叉开关核心时钟（读/写子模块共用）。
    input  wire                             rst, // 同步复位，转发到读/写子模块。

    /*
     * AXI-Lite 从接口
     */
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axil_awaddr, // 从端 AW 地址向量；每个从端占 ADDR_WIDTH 切片。
    input  wire [S_COUNT*3-1:0]             s_axil_awprot, // 每个端口的从端 AW 保护属性。
    input  wire [S_COUNT-1:0]               s_axil_awvalid, // 从端 AWVALID 向量；各位随端口写请求变化。
    output wire [S_COUNT-1:0]               s_axil_awready, // 从端 AWREADY 向量，由写交叉开关接纳逻辑驱动。
    input  wire [S_COUNT*DATA_WIDTH-1:0]    s_axil_wdata, // 从端写数据向量。
    input  wire [S_COUNT*STRB_WIDTH-1:0]    s_axil_wstrb, // 从端字节写使能向量。
    input  wire [S_COUNT-1:0]               s_axil_wvalid, // 从端 WVALID 向量；各从端有写数据时置位。
    output wire [S_COUNT-1:0]               s_axil_wready, // 从端 WREADY 向量；随下游目标主端就绪状态变化。
    output wire [S_COUNT*2-1:0]             s_axil_bresp, // 从端 BRESP 码向量。
    output wire [S_COUNT-1:0]               s_axil_bvalid, // 从端 BVALID 向量；写响应返回时拉高。
    input  wire [S_COUNT-1:0]               s_axil_bready, // 从端 BREADY 向量；各从端消费自身写响应。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axil_araddr, // 从端 AR 地址向量。
    input  wire [S_COUNT*3-1:0]             s_axil_arprot, // 每个端口的从端 AR 保护属性。
    input  wire [S_COUNT-1:0]               s_axil_arvalid, // 从端 ARVALID 向量；各位表示活跃读请求。
    output wire [S_COUNT-1:0]               s_axil_arready, // 从端 ARREADY 向量，由读交叉开关接纳逻辑驱动。
    output wire [S_COUNT*DATA_WIDTH-1:0]    s_axil_rdata, // 从端读数据向量。
    output wire [S_COUNT*2-1:0]             s_axil_rresp, // 从端 RRESP 码向量。
    output wire [S_COUNT-1:0]               s_axil_rvalid, // 从端 RVALID 向量；返回读数据或 DECERR 时变化。
    input  wire [S_COUNT-1:0]               s_axil_rready, // 从端 RREADY 向量；各从端反压输入。

    /*
     * AXI-Lite 主接口
     */
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axil_awaddr, // 主端 AW 地址向量（写通路解码/仲裁后输出）。
    output wire [M_COUNT*3-1:0]             m_axil_awprot, // 主端 AW 保护属性向量。
    output wire [M_COUNT-1:0]               m_axil_awvalid, // 主端 AWVALID 向量；每位对应一个目标主端。
    input  wire [M_COUNT-1:0]               m_axil_awready, // 下游外设返回的主端 AWREADY 向量。
    output wire [M_COUNT*DATA_WIDTH-1:0]    m_axil_wdata, // 主端写数据向量（由获胜从端路由）。
    output wire [M_COUNT*STRB_WIDTH-1:0]    m_axil_wstrb, // 主端写字节使能向量。
    output wire [M_COUNT-1:0]               m_axil_wvalid, // 主端 WVALID 向量。
    input  wire [M_COUNT-1:0]               m_axil_wready, // 主端 WREADY 向量，反馈给选中的源从端。
    input  wire [M_COUNT*2-1:0]             m_axil_bresp, // 目标返回的主端 BRESP 向量。
    input  wire [M_COUNT-1:0]               m_axil_bvalid, // 目标返回的主端 BVALID 向量。
    output wire [M_COUNT-1:0]               m_axil_bready, // 主端 BREADY 向量，由响应路由可用性驱动。
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axil_araddr, // 主端 AR 地址向量（读通路解码/仲裁后输出）。
    output wire [M_COUNT*3-1:0]             m_axil_arprot, // 主端 AR 保护属性向量。
    output wire [M_COUNT-1:0]               m_axil_arvalid, // 主端 ARVALID 向量。
    input  wire [M_COUNT-1:0]               m_axil_arready, // 目标返回的主端 ARREADY 向量。
    input  wire [M_COUNT*DATA_WIDTH-1:0]    m_axil_rdata, // 目标返回的主端读数据向量。
    input  wire [M_COUNT*2-1:0]             m_axil_rresp, // 目标返回的主端读响应码向量。
    input  wire [M_COUNT-1:0]               m_axil_rvalid, // 目标返回的主端 RVALID 向量。
    output wire [M_COUNT-1:0]               m_axil_rready // 主端 RREADY 向量，指向选中回程源。
);

axil_crossbar_wr #(
    .S_COUNT(S_COUNT),
    .M_COUNT(M_COUNT),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
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
axil_crossbar_wr_inst (
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

axil_crossbar_rd #(
    .S_COUNT(S_COUNT),
    .M_COUNT(M_COUNT),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
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
axil_crossbar_rd_inst (
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
