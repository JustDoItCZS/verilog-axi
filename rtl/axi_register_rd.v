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
 * AXI4 寄存器切片（读通道）
 *
 * 模块目录
 * 1) AR 通道可选缓冲（旁路/简单寄存/skid）。
 * 2) R 通道可选缓冲（旁路/简单寄存/skid）。
 * 3) 在保持 AXI 读语义不变的前提下切断关键时序路径。
 */
module axi_register_rd #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 ARUSER 信号
    parameter ARUSER_ENABLE = 0,
    // ARUSER 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 RUSER 信号
    parameter RUSER_ENABLE = 0,
    // RUSER 信号位宽
    parameter RUSER_WIDTH = 1,
    // AR 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter AR_REG_TYPE = 1,
    // R 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter R_REG_TYPE = 2
)
(
    input  wire                     clk, // 读路径寄存切片时钟。
    input  wire                     rst, // AR/R 通道缓冲状态同步复位。

    /*
     * AXI 从接口
     */
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
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 从侧 R ID（来自下游）。
    output wire [DATA_WIDTH-1:0]    s_axi_rdata, // 从侧 R 数据。
    output wire [1:0]               s_axi_rresp, // 从侧 R 响应。
    output wire                     s_axi_rlast, // 从侧 RLAST。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 从侧 R 用户旁带。
    output wire                     s_axi_rvalid, // 从侧 RVALID。
    input  wire                     s_axi_rready, // 从侧 RREADY。

    /*
     * AXI 主接口
     */
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
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 主侧 R ID。
    input  wire [DATA_WIDTH-1:0]    m_axi_rdata, // 主侧 R 数据。
    input  wire [1:0]               m_axi_rresp, // 主侧 R 响应。
    input  wire                     m_axi_rlast, // 主侧 RLAST。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 主侧 R 用户旁带。
    input  wire                     m_axi_rvalid, // 主侧 RVALID。
    output wire                     m_axi_rready // 主侧 RREADY。
);

generate

// AR 通道

if (AR_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                    s_axi_arready_reg = 1'b0; // 从侧 ARREADY 寄存器，由前瞻 ready 逻辑驱动。

reg [ID_WIDTH-1:0]     m_axi_arid_reg     = {ID_WIDTH{1'b0}}; // 主输出 AR ID 寄存器。
reg [ADDR_WIDTH-1:0]   m_axi_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 主输出 AR 地址寄存器。
reg [7:0]              m_axi_arlen_reg    = 8'd0; // 主输出 ARLEN 寄存器。
reg [2:0]              m_axi_arsize_reg   = 3'd0; // 主输出 ARSIZE 寄存器。
reg [1:0]              m_axi_arburst_reg  = 2'd0; // 主输出 ARBURST 寄存器。
reg                    m_axi_arlock_reg   = 1'b0; // 主输出 ARLOCK 寄存器。
reg [3:0]              m_axi_arcache_reg  = 4'd0; // 主输出 ARCACHE 寄存器。
reg [2:0]              m_axi_arprot_reg   = 3'd0; // 主输出 ARPROT 寄存器。
reg [3:0]              m_axi_arqos_reg    = 4'd0; // 主输出 ARQOS 寄存器。
reg [3:0]              m_axi_arregion_reg = 4'd0; // 主输出 ARREGION 寄存器。
reg [ARUSER_WIDTH-1:0] m_axi_aruser_reg   = {ARUSER_WIDTH{1'b0}}; // 主输出 ARUSER 寄存器。
reg                    m_axi_arvalid_reg  = 1'b0, m_axi_arvalid_next; // 主输出 ARVALID 当前状态与下一状态。

reg [ID_WIDTH-1:0]     temp_m_axi_arid_reg     = {ID_WIDTH{1'b0}}; // 输出级阻塞时临时缓存 AR ID。
reg [ADDR_WIDTH-1:0]   temp_m_axi_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 临时缓存 AR 地址。
reg [7:0]              temp_m_axi_arlen_reg    = 8'd0; // 临时缓存 ARLEN。
reg [2:0]              temp_m_axi_arsize_reg   = 3'd0; // 临时缓存 ARSIZE。
reg [1:0]              temp_m_axi_arburst_reg  = 2'd0; // 临时缓存 ARBURST。
reg                    temp_m_axi_arlock_reg   = 1'b0; // 临时缓存 ARLOCK。
reg [3:0]              temp_m_axi_arcache_reg  = 4'd0; // 临时缓存 ARCACHE。
reg [2:0]              temp_m_axi_arprot_reg   = 3'd0; // 临时缓存 ARPROT。
reg [3:0]              temp_m_axi_arqos_reg    = 4'd0; // 临时缓存 ARQOS。
reg [3:0]              temp_m_axi_arregion_reg = 4'd0; // 临时缓存 ARREGION。
reg [ARUSER_WIDTH-1:0] temp_m_axi_aruser_reg   = {ARUSER_WIDTH{1'b0}}; // 临时缓存 ARUSER。
reg                    temp_m_axi_arvalid_reg  = 1'b0, temp_m_axi_arvalid_next; // 临时缓存 ARVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_ar_input_to_output; // 置位时把输入 AR 写入主输出寄存器。
reg store_axi_ar_input_to_temp; // 置位时把输入 AR 写入临时寄存器。
reg store_axi_ar_temp_to_output; // 置位时把临时 AR 回放到主输出寄存器。

assign s_axi_arready  = s_axi_arready_reg;

assign m_axi_arid     = m_axi_arid_reg;
assign m_axi_araddr   = m_axi_araddr_reg;
assign m_axi_arlen    = m_axi_arlen_reg;
assign m_axi_arsize   = m_axi_arsize_reg;
assign m_axi_arburst  = m_axi_arburst_reg;
assign m_axi_arlock   = m_axi_arlock_reg;
assign m_axi_arcache  = m_axi_arcache_reg;
assign m_axi_arprot   = m_axi_arprot_reg;
assign m_axi_arqos    = m_axi_arqos_reg;
assign m_axi_arregion = m_axi_arregion_reg;
assign m_axi_aruser   = ARUSER_ENABLE ? m_axi_aruser_reg : {ARUSER_WIDTH{1'b0}};
assign m_axi_arvalid  = m_axi_arvalid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axi_arready_early = m_axi_arready | (~temp_m_axi_arvalid_reg & (~m_axi_arvalid_reg | ~s_axi_arvalid)); // 前瞻 ready，避免 skid 模式气泡。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_arvalid_next = m_axi_arvalid_reg;
    temp_m_axi_arvalid_next = temp_m_axi_arvalid_reg;

    store_axi_ar_input_to_output = 1'b0;
    store_axi_ar_input_to_temp = 1'b0;
    store_axi_ar_temp_to_output = 1'b0;

    if (s_axi_arready_reg) begin
        // 当前允许接收输入
        if (m_axi_arready | ~m_axi_arvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axi_arvalid_next = s_axi_arvalid;
            store_axi_ar_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axi_arvalid_next = s_axi_arvalid;
            store_axi_ar_input_to_temp = 1'b1;
        end
    end else if (m_axi_arready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        m_axi_arvalid_next = temp_m_axi_arvalid_reg;
        temp_m_axi_arvalid_next = 1'b0;
        store_axi_ar_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_arready_reg <= 1'b0;
        m_axi_arvalid_reg <= 1'b0;
        temp_m_axi_arvalid_reg <= 1'b0;
    end else begin
        s_axi_arready_reg <= s_axi_arready_early;
        m_axi_arvalid_reg <= m_axi_arvalid_next;
        temp_m_axi_arvalid_reg <= temp_m_axi_arvalid_next;
    end

    // 数据通路寄存
    if (store_axi_ar_input_to_output) begin
        m_axi_arid_reg <= s_axi_arid;
        m_axi_araddr_reg <= s_axi_araddr;
        m_axi_arlen_reg <= s_axi_arlen;
        m_axi_arsize_reg <= s_axi_arsize;
        m_axi_arburst_reg <= s_axi_arburst;
        m_axi_arlock_reg <= s_axi_arlock;
        m_axi_arcache_reg <= s_axi_arcache;
        m_axi_arprot_reg <= s_axi_arprot;
        m_axi_arqos_reg <= s_axi_arqos;
        m_axi_arregion_reg <= s_axi_arregion;
        m_axi_aruser_reg <= s_axi_aruser;
    end else if (store_axi_ar_temp_to_output) begin
        m_axi_arid_reg <= temp_m_axi_arid_reg;
        m_axi_araddr_reg <= temp_m_axi_araddr_reg;
        m_axi_arlen_reg <= temp_m_axi_arlen_reg;
        m_axi_arsize_reg <= temp_m_axi_arsize_reg;
        m_axi_arburst_reg <= temp_m_axi_arburst_reg;
        m_axi_arlock_reg <= temp_m_axi_arlock_reg;
        m_axi_arcache_reg <= temp_m_axi_arcache_reg;
        m_axi_arprot_reg <= temp_m_axi_arprot_reg;
        m_axi_arqos_reg <= temp_m_axi_arqos_reg;
        m_axi_arregion_reg <= temp_m_axi_arregion_reg;
        m_axi_aruser_reg <= temp_m_axi_aruser_reg;
    end

    if (store_axi_ar_input_to_temp) begin
        temp_m_axi_arid_reg <= s_axi_arid;
        temp_m_axi_araddr_reg <= s_axi_araddr;
        temp_m_axi_arlen_reg <= s_axi_arlen;
        temp_m_axi_arsize_reg <= s_axi_arsize;
        temp_m_axi_arburst_reg <= s_axi_arburst;
        temp_m_axi_arlock_reg <= s_axi_arlock;
        temp_m_axi_arcache_reg <= s_axi_arcache;
        temp_m_axi_arprot_reg <= s_axi_arprot;
        temp_m_axi_arqos_reg <= s_axi_arqos;
        temp_m_axi_arregion_reg <= s_axi_arregion;
        temp_m_axi_aruser_reg <= s_axi_aruser;
    end
end

end else if (AR_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                    s_axi_arready_reg = 1'b0; // 简单缓冲模式下从侧 ARREADY 寄存器。

reg [ID_WIDTH-1:0]     m_axi_arid_reg     = {ID_WIDTH{1'b0}}; // 缓存的 AR ID。
reg [ADDR_WIDTH-1:0]   m_axi_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 缓存的 AR 地址。
reg [7:0]              m_axi_arlen_reg    = 8'd0; // 缓存的 ARLEN。
reg [2:0]              m_axi_arsize_reg   = 3'd0; // 缓存的 ARSIZE。
reg [1:0]              m_axi_arburst_reg  = 2'd0; // 缓存的 ARBURST。
reg                    m_axi_arlock_reg   = 1'b0; // 缓存的 ARLOCK。
reg [3:0]              m_axi_arcache_reg  = 4'd0; // 缓存的 ARCACHE。
reg [2:0]              m_axi_arprot_reg   = 3'd0; // 缓存的 ARPROT。
reg [3:0]              m_axi_arqos_reg    = 4'd0; // 缓存的 ARQOS。
reg [3:0]              m_axi_arregion_reg = 4'd0; // 缓存的 ARREGION。
reg [ARUSER_WIDTH-1:0] m_axi_aruser_reg   = {ARUSER_WIDTH{1'b0}}; // 缓存的 ARUSER。
reg                    m_axi_arvalid_reg  = 1'b0, m_axi_arvalid_next; // 缓存 ARVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_ar_input_to_output; // 置位时把输入 AR 写入输出缓冲。

assign s_axi_arready  = s_axi_arready_reg;

assign m_axi_arid     = m_axi_arid_reg;
assign m_axi_araddr   = m_axi_araddr_reg;
assign m_axi_arlen    = m_axi_arlen_reg;
assign m_axi_arsize   = m_axi_arsize_reg;
assign m_axi_arburst  = m_axi_arburst_reg;
assign m_axi_arlock   = m_axi_arlock_reg;
assign m_axi_arcache  = m_axi_arcache_reg;
assign m_axi_arprot   = m_axi_arprot_reg;
assign m_axi_arqos    = m_axi_arqos_reg;
assign m_axi_arregion = m_axi_arregion_reg;
assign m_axi_aruser   = ARUSER_ENABLE ? m_axi_aruser_reg : {ARUSER_WIDTH{1'b0}};
assign m_axi_arvalid  = m_axi_arvalid_reg;

// 下拍 ready 预判：输出缓冲为空或将为空时拉高
wire s_axi_arready_early = !m_axi_arvalid_next; // 下拍输出缓冲为空时允许接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_arvalid_next = m_axi_arvalid_reg;

    store_axi_ar_input_to_output = 1'b0;

    if (s_axi_arready_reg) begin
        m_axi_arvalid_next = s_axi_arvalid;
        store_axi_ar_input_to_output = 1'b1;
    end else if (m_axi_arready) begin
        m_axi_arvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_arready_reg <= 1'b0;
        m_axi_arvalid_reg <= 1'b0;
    end else begin
        s_axi_arready_reg <= s_axi_arready_early;
        m_axi_arvalid_reg <= m_axi_arvalid_next;
    end

    // 数据通路寄存
    if (store_axi_ar_input_to_output) begin
        m_axi_arid_reg <= s_axi_arid;
        m_axi_araddr_reg <= s_axi_araddr;
        m_axi_arlen_reg <= s_axi_arlen;
        m_axi_arsize_reg <= s_axi_arsize;
        m_axi_arburst_reg <= s_axi_arburst;
        m_axi_arlock_reg <= s_axi_arlock;
        m_axi_arcache_reg <= s_axi_arcache;
        m_axi_arprot_reg <= s_axi_arprot;
        m_axi_arqos_reg <= s_axi_arqos;
        m_axi_arregion_reg <= s_axi_arregion;
        m_axi_aruser_reg <= s_axi_aruser;
    end
end

end else begin

    // AR 通道旁路
    assign m_axi_arid = s_axi_arid;
    assign m_axi_araddr = s_axi_araddr;
    assign m_axi_arlen = s_axi_arlen;
    assign m_axi_arsize = s_axi_arsize;
    assign m_axi_arburst = s_axi_arburst;
    assign m_axi_arlock = s_axi_arlock;
    assign m_axi_arcache = s_axi_arcache;
    assign m_axi_arprot = s_axi_arprot;
    assign m_axi_arqos = s_axi_arqos;
    assign m_axi_arregion = s_axi_arregion;
    assign m_axi_aruser = ARUSER_ENABLE ? s_axi_aruser : {ARUSER_WIDTH{1'b0}};
    assign m_axi_arvalid = s_axi_arvalid;
    assign s_axi_arready = m_axi_arready;

end

// R 通道

if (R_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   m_axi_rready_reg = 1'b0; // skid 模式下主侧 RREADY 寄存器。

reg [ID_WIDTH-1:0]    s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // 主输出 R ID 寄存器。
reg [DATA_WIDTH-1:0]  s_axi_rdata_reg  = {DATA_WIDTH{1'b0}}; // 主输出 R 数据寄存器。
reg [1:0]             s_axi_rresp_reg  = 2'b0; // 主输出 RRESP 寄存器。
reg                   s_axi_rlast_reg  = 1'b0; // 主输出 RLAST 寄存器。
reg [RUSER_WIDTH-1:0] s_axi_ruser_reg  = {RUSER_WIDTH{1'b0}}; // 主输出 RUSER 寄存器。
reg                   s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // 主输出 RVALID 当前状态与下一状态。

reg [ID_WIDTH-1:0]    temp_s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // 输出阻塞时临时缓存 R ID。
reg [DATA_WIDTH-1:0]  temp_s_axi_rdata_reg  = {DATA_WIDTH{1'b0}}; // 临时缓存 R 数据。
reg [1:0]             temp_s_axi_rresp_reg  = 2'b0; // 临时缓存 RRESP。
reg                   temp_s_axi_rlast_reg  = 1'b0; // 临时缓存 RLAST。
reg [RUSER_WIDTH-1:0] temp_s_axi_ruser_reg  = {RUSER_WIDTH{1'b0}}; // 临时缓存 RUSER。
reg                   temp_s_axi_rvalid_reg = 1'b0, temp_s_axi_rvalid_next; // 临时缓存 RVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_r_input_to_output; // 置位时把输入 R 写入主输出寄存器。
reg store_axi_r_input_to_temp; // 置位时把输入 R 写入临时寄存器。
reg store_axi_r_temp_to_output; // 置位时把临时 R 回放到主输出寄存器。

assign m_axi_rready = m_axi_rready_reg;

assign s_axi_rid    = s_axi_rid_reg;
assign s_axi_rdata  = s_axi_rdata_reg;
assign s_axi_rresp  = s_axi_rresp_reg;
assign s_axi_rlast  = s_axi_rlast_reg;
assign s_axi_ruser  = RUSER_ENABLE ? s_axi_ruser_reg : {RUSER_WIDTH{1'b0}};
assign s_axi_rvalid = s_axi_rvalid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
wire m_axi_rready_early = s_axi_rready | (~temp_s_axi_rvalid_reg & (~s_axi_rvalid_reg | ~m_axi_rvalid)); // 前瞻 ready，避免 skid 气泡。

always @* begin
    // 将下游就绪关系映射到上游
    s_axi_rvalid_next = s_axi_rvalid_reg;
    temp_s_axi_rvalid_next = temp_s_axi_rvalid_reg;

    store_axi_r_input_to_output = 1'b0;
    store_axi_r_input_to_temp = 1'b0;
    store_axi_r_temp_to_output = 1'b0;

    if (m_axi_rready_reg) begin
        // 当前允许接收输入
        if (s_axi_rready | ~s_axi_rvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            s_axi_rvalid_next = m_axi_rvalid;
            store_axi_r_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_s_axi_rvalid_next = m_axi_rvalid;
            store_axi_r_input_to_temp = 1'b1;
        end
    end else if (s_axi_rready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        s_axi_rvalid_next = temp_s_axi_rvalid_reg;
        temp_s_axi_rvalid_next = 1'b0;
        store_axi_r_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axi_rready_reg <= 1'b0;
        s_axi_rvalid_reg <= 1'b0;
        temp_s_axi_rvalid_reg <= 1'b0;
    end else begin
        m_axi_rready_reg <= m_axi_rready_early;
        s_axi_rvalid_reg <= s_axi_rvalid_next;
        temp_s_axi_rvalid_reg <= temp_s_axi_rvalid_next;
    end

    // 数据通路寄存
    if (store_axi_r_input_to_output) begin
        s_axi_rid_reg   <= m_axi_rid;
        s_axi_rdata_reg <= m_axi_rdata;
        s_axi_rresp_reg <= m_axi_rresp;
        s_axi_rlast_reg <= m_axi_rlast;
        s_axi_ruser_reg <= m_axi_ruser;
    end else if (store_axi_r_temp_to_output) begin
        s_axi_rid_reg   <= temp_s_axi_rid_reg;
        s_axi_rdata_reg <= temp_s_axi_rdata_reg;
        s_axi_rresp_reg <= temp_s_axi_rresp_reg;
        s_axi_rlast_reg <= temp_s_axi_rlast_reg;
        s_axi_ruser_reg <= temp_s_axi_ruser_reg;
    end

    if (store_axi_r_input_to_temp) begin
        temp_s_axi_rid_reg   <= m_axi_rid;
        temp_s_axi_rdata_reg <= m_axi_rdata;
        temp_s_axi_rresp_reg <= m_axi_rresp;
        temp_s_axi_rlast_reg <= m_axi_rlast;
        temp_s_axi_ruser_reg <= m_axi_ruser;
    end
end

end else if (R_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   m_axi_rready_reg = 1'b0; // 简单缓冲模式下主侧 RREADY 寄存器。

reg [ID_WIDTH-1:0]    s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // 缓存的 R ID。
reg [DATA_WIDTH-1:0]  s_axi_rdata_reg  = {DATA_WIDTH{1'b0}}; // 缓存的 R 数据。
reg [1:0]             s_axi_rresp_reg  = 2'b0; // 缓存的 RRESP。
reg                   s_axi_rlast_reg  = 1'b0; // 缓存的 RLAST。
reg [RUSER_WIDTH-1:0] s_axi_ruser_reg  = {RUSER_WIDTH{1'b0}}; // 缓存的 RUSER。
reg                   s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // 缓存 RVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_r_input_to_output; // 置位时把输入 R 写入输出缓冲。

assign m_axi_rready = m_axi_rready_reg;

assign s_axi_rid    = s_axi_rid_reg;
assign s_axi_rdata  = s_axi_rdata_reg;
assign s_axi_rresp  = s_axi_rresp_reg;
assign s_axi_rlast  = s_axi_rlast_reg;
assign s_axi_ruser  = RUSER_ENABLE ? s_axi_ruser_reg : {RUSER_WIDTH{1'b0}};
assign s_axi_rvalid = s_axi_rvalid_reg;

// 下拍 ready 预判：输出缓冲为空或将为空时拉高
wire m_axi_rready_early = !s_axi_rvalid_next; // 下拍 R 输出缓冲为空时允许接收。

always @* begin
    // 将下游就绪关系映射到上游
    s_axi_rvalid_next = s_axi_rvalid_reg;

    store_axi_r_input_to_output = 1'b0;

    if (m_axi_rready_reg) begin
        s_axi_rvalid_next = m_axi_rvalid;
        store_axi_r_input_to_output = 1'b1;
    end else if (s_axi_rready) begin
        s_axi_rvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axi_rready_reg <= 1'b0;
        s_axi_rvalid_reg <= 1'b0;
    end else begin
        m_axi_rready_reg <= m_axi_rready_early;
        s_axi_rvalid_reg <= s_axi_rvalid_next;
    end

    // 数据通路寄存
    if (store_axi_r_input_to_output) begin
        s_axi_rid_reg   <= m_axi_rid;
        s_axi_rdata_reg <= m_axi_rdata;
        s_axi_rresp_reg <= m_axi_rresp;
        s_axi_rlast_reg <= m_axi_rlast;
        s_axi_ruser_reg <= m_axi_ruser;
    end
end

end else begin

    // R 通道旁路
    assign s_axi_rid = m_axi_rid;
    assign s_axi_rdata = m_axi_rdata;
    assign s_axi_rresp = m_axi_rresp;
    assign s_axi_rlast = m_axi_rlast;
    assign s_axi_ruser = RUSER_ENABLE ? m_axi_ruser : {RUSER_WIDTH{1'b0}};
    assign s_axi_rvalid = m_axi_rvalid;
    assign m_axi_rready = s_axi_rready;

end

endgenerate

endmodule

`resetall
