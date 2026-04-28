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
 * AXI4 寄存器切片（写通道）
 *
 * 模块目录
 * 1) AW 通道可选缓冲（旁路/简单寄存/skid）。
 * 2) W 通道可选缓冲（旁路/简单寄存/skid）。
 * 3) B 通道可选缓冲（旁路/简单寄存/skid）。
 * 4) 在保持 AXI 写语义不变的前提下切断关键时序路径。
 */
module axi_register_wr #
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
    // AW 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter AW_REG_TYPE = 1,
    // W 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter W_REG_TYPE = 2,
    // B 通道寄存类型
    // 0 表示旁路，1 表示简单缓冲，2 表示 skid buffer
    parameter B_REG_TYPE = 1
)
(
    input  wire                     clk, // 写路径寄存切片时钟。
    input  wire                     rst, // AW/W/B 通道缓冲状态同步复位。

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
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 从侧 B ID。
    output wire [1:0]               s_axi_bresp, // 从侧 B 响应。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 从侧 B 用户旁带。
    output wire                     s_axi_bvalid, // 从侧 BVALID。
    input  wire                     s_axi_bready, // 从侧 BREADY。

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
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 主侧 B ID。
    input  wire [1:0]               m_axi_bresp, // 主侧 B 响应。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 主侧 B 用户旁带。
    input  wire                     m_axi_bvalid, // 主侧 BVALID。
    output wire                     m_axi_bready // 主侧 BREADY。
);

generate

// AW 通道

if (AW_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                    s_axi_awready_reg = 1'b0; // skid 模式下从侧 AWREADY 寄存器。

reg [ID_WIDTH-1:0]     m_axi_awid_reg     = {ID_WIDTH{1'b0}}; // 主输出 AW ID 寄存器。
reg [ADDR_WIDTH-1:0]   m_axi_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 主输出 AW 地址寄存器。
reg [7:0]              m_axi_awlen_reg    = 8'd0; // 主输出 AWLEN 寄存器。
reg [2:0]              m_axi_awsize_reg   = 3'd0; // 主输出 AWSIZE 寄存器。
reg [1:0]              m_axi_awburst_reg  = 2'd0; // 主输出 AWBURST 寄存器。
reg                    m_axi_awlock_reg   = 1'b0; // 主输出 AWLOCK 寄存器。
reg [3:0]              m_axi_awcache_reg  = 4'd0; // 主输出 AWCACHE 寄存器。
reg [2:0]              m_axi_awprot_reg   = 3'd0; // 主输出 AWPROT 寄存器。
reg [3:0]              m_axi_awqos_reg    = 4'd0; // 主输出 AWQOS 寄存器。
reg [3:0]              m_axi_awregion_reg = 4'd0; // 主输出 AWREGION 寄存器。
reg [AWUSER_WIDTH-1:0] m_axi_awuser_reg   = {AWUSER_WIDTH{1'b0}}; // 主输出 AWUSER 寄存器。
reg                    m_axi_awvalid_reg  = 1'b0, m_axi_awvalid_next; // 主输出 AWVALID 当前状态与下一状态。

reg [ID_WIDTH-1:0]     temp_m_axi_awid_reg     = {ID_WIDTH{1'b0}}; // 输出阻塞时临时缓存 AW ID。
reg [ADDR_WIDTH-1:0]   temp_m_axi_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 临时缓存 AW 地址。
reg [7:0]              temp_m_axi_awlen_reg    = 8'd0; // 临时缓存 AWLEN。
reg [2:0]              temp_m_axi_awsize_reg   = 3'd0; // 临时缓存 AWSIZE。
reg [1:0]              temp_m_axi_awburst_reg  = 2'd0; // 临时缓存 AWBURST。
reg                    temp_m_axi_awlock_reg   = 1'b0; // 临时缓存 AWLOCK。
reg [3:0]              temp_m_axi_awcache_reg  = 4'd0; // 临时缓存 AWCACHE。
reg [2:0]              temp_m_axi_awprot_reg   = 3'd0; // 临时缓存 AWPROT。
reg [3:0]              temp_m_axi_awqos_reg    = 4'd0; // 临时缓存 AWQOS。
reg [3:0]              temp_m_axi_awregion_reg = 4'd0; // 临时缓存 AWREGION。
reg [AWUSER_WIDTH-1:0] temp_m_axi_awuser_reg   = {AWUSER_WIDTH{1'b0}}; // 临时缓存 AWUSER。
reg                    temp_m_axi_awvalid_reg  = 1'b0, temp_m_axi_awvalid_next; // 临时缓存 AWVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_aw_input_to_output; // 置位时把输入 AW 写入主输出寄存器。
reg store_axi_aw_input_to_temp; // 置位时把输入 AW 写入临时寄存器。
reg store_axi_aw_temp_to_output; // 置位时把临时 AW 回放到主输出寄存器。

assign s_axi_awready  = s_axi_awready_reg;

assign m_axi_awid     = m_axi_awid_reg;
assign m_axi_awaddr   = m_axi_awaddr_reg;
assign m_axi_awlen    = m_axi_awlen_reg;
assign m_axi_awsize   = m_axi_awsize_reg;
assign m_axi_awburst  = m_axi_awburst_reg;
assign m_axi_awlock   = m_axi_awlock_reg;
assign m_axi_awcache  = m_axi_awcache_reg;
assign m_axi_awprot   = m_axi_awprot_reg;
assign m_axi_awqos    = m_axi_awqos_reg;
assign m_axi_awregion = m_axi_awregion_reg;
assign m_axi_awuser   = AWUSER_ENABLE ? m_axi_awuser_reg : {AWUSER_WIDTH{1'b0}};
assign m_axi_awvalid  = m_axi_awvalid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axi_awready_early = m_axi_awready | (~temp_m_axi_awvalid_reg & (~m_axi_awvalid_reg | ~s_axi_awvalid)); // 前瞻 ready，避免 skid 气泡。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_awvalid_next = m_axi_awvalid_reg;
    temp_m_axi_awvalid_next = temp_m_axi_awvalid_reg;

    store_axi_aw_input_to_output = 1'b0;
    store_axi_aw_input_to_temp = 1'b0;
    store_axi_aw_temp_to_output = 1'b0;

    if (s_axi_awready_reg) begin
        // 当前允许接收输入
        if (m_axi_awready | ~m_axi_awvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axi_awvalid_next = s_axi_awvalid;
            store_axi_aw_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axi_awvalid_next = s_axi_awvalid;
            store_axi_aw_input_to_temp = 1'b1;
        end
    end else if (m_axi_awready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        m_axi_awvalid_next = temp_m_axi_awvalid_reg;
        temp_m_axi_awvalid_next = 1'b0;
        store_axi_aw_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_awready_reg <= 1'b0;
        m_axi_awvalid_reg <= 1'b0;
        temp_m_axi_awvalid_reg <= 1'b0;
    end else begin
        s_axi_awready_reg <= s_axi_awready_early;
        m_axi_awvalid_reg <= m_axi_awvalid_next;
        temp_m_axi_awvalid_reg <= temp_m_axi_awvalid_next;
    end

    // 数据通路寄存
    if (store_axi_aw_input_to_output) begin
        m_axi_awid_reg <= s_axi_awid;
        m_axi_awaddr_reg <= s_axi_awaddr;
        m_axi_awlen_reg <= s_axi_awlen;
        m_axi_awsize_reg <= s_axi_awsize;
        m_axi_awburst_reg <= s_axi_awburst;
        m_axi_awlock_reg <= s_axi_awlock;
        m_axi_awcache_reg <= s_axi_awcache;
        m_axi_awprot_reg <= s_axi_awprot;
        m_axi_awqos_reg <= s_axi_awqos;
        m_axi_awregion_reg <= s_axi_awregion;
        m_axi_awuser_reg <= s_axi_awuser;
    end else if (store_axi_aw_temp_to_output) begin
        m_axi_awid_reg <= temp_m_axi_awid_reg;
        m_axi_awaddr_reg <= temp_m_axi_awaddr_reg;
        m_axi_awlen_reg <= temp_m_axi_awlen_reg;
        m_axi_awsize_reg <= temp_m_axi_awsize_reg;
        m_axi_awburst_reg <= temp_m_axi_awburst_reg;
        m_axi_awlock_reg <= temp_m_axi_awlock_reg;
        m_axi_awcache_reg <= temp_m_axi_awcache_reg;
        m_axi_awprot_reg <= temp_m_axi_awprot_reg;
        m_axi_awqos_reg <= temp_m_axi_awqos_reg;
        m_axi_awregion_reg <= temp_m_axi_awregion_reg;
        m_axi_awuser_reg <= temp_m_axi_awuser_reg;
    end

    if (store_axi_aw_input_to_temp) begin
        temp_m_axi_awid_reg <= s_axi_awid;
        temp_m_axi_awaddr_reg <= s_axi_awaddr;
        temp_m_axi_awlen_reg <= s_axi_awlen;
        temp_m_axi_awsize_reg <= s_axi_awsize;
        temp_m_axi_awburst_reg <= s_axi_awburst;
        temp_m_axi_awlock_reg <= s_axi_awlock;
        temp_m_axi_awcache_reg <= s_axi_awcache;
        temp_m_axi_awprot_reg <= s_axi_awprot;
        temp_m_axi_awqos_reg <= s_axi_awqos;
        temp_m_axi_awregion_reg <= s_axi_awregion;
        temp_m_axi_awuser_reg <= s_axi_awuser;
    end
end

end else if (AW_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                    s_axi_awready_reg = 1'b0; // 简单缓冲模式下从侧 AWREADY 寄存器。

reg [ID_WIDTH-1:0]     m_axi_awid_reg     = {ID_WIDTH{1'b0}}; // 缓存的 AW ID。
reg [ADDR_WIDTH-1:0]   m_axi_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 缓存的 AW 地址。
reg [7:0]              m_axi_awlen_reg    = 8'd0; // 缓存的 AWLEN。
reg [2:0]              m_axi_awsize_reg   = 3'd0; // 缓存的 AWSIZE。
reg [1:0]              m_axi_awburst_reg  = 2'd0; // 缓存的 AWBURST。
reg                    m_axi_awlock_reg   = 1'b0; // 缓存的 AWLOCK。
reg [3:0]              m_axi_awcache_reg  = 4'd0; // 缓存的 AWCACHE。
reg [2:0]              m_axi_awprot_reg   = 3'd0; // 缓存的 AWPROT。
reg [3:0]              m_axi_awqos_reg    = 4'd0; // 缓存的 AWQOS。
reg [3:0]              m_axi_awregion_reg = 4'd0; // 缓存的 AWREGION。
reg [AWUSER_WIDTH-1:0] m_axi_awuser_reg   = {AWUSER_WIDTH{1'b0}}; // 缓存的 AWUSER。
reg                    m_axi_awvalid_reg  = 1'b0, m_axi_awvalid_next; // 缓存 AWVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_aw_input_to_output; // 置位时把输入 AW 写入输出缓冲。

assign s_axi_awready  = s_axi_awready_reg;

assign m_axi_awid     = m_axi_awid_reg;
assign m_axi_awaddr   = m_axi_awaddr_reg;
assign m_axi_awlen    = m_axi_awlen_reg;
assign m_axi_awsize   = m_axi_awsize_reg;
assign m_axi_awburst  = m_axi_awburst_reg;
assign m_axi_awlock   = m_axi_awlock_reg;
assign m_axi_awcache  = m_axi_awcache_reg;
assign m_axi_awprot   = m_axi_awprot_reg;
assign m_axi_awqos    = m_axi_awqos_reg;
assign m_axi_awregion = m_axi_awregion_reg;
assign m_axi_awuser   = AWUSER_ENABLE ? m_axi_awuser_reg : {AWUSER_WIDTH{1'b0}};
assign m_axi_awvalid  = m_axi_awvalid_reg;

// 下拍 ready 预判：输出缓冲为空或将为空时拉高
wire s_axi_awready_eawly = !m_axi_awvalid_next; // 下拍 AW 输出缓冲为空时允许接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_awvalid_next = m_axi_awvalid_reg;

    store_axi_aw_input_to_output = 1'b0;

    if (s_axi_awready_reg) begin
        m_axi_awvalid_next = s_axi_awvalid;
        store_axi_aw_input_to_output = 1'b1;
    end else if (m_axi_awready) begin
        m_axi_awvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_awready_reg <= 1'b0;
        m_axi_awvalid_reg <= 1'b0;
    end else begin
        s_axi_awready_reg <= s_axi_awready_eawly;
        m_axi_awvalid_reg <= m_axi_awvalid_next;
    end

    // 数据通路寄存
    if (store_axi_aw_input_to_output) begin
        m_axi_awid_reg <= s_axi_awid;
        m_axi_awaddr_reg <= s_axi_awaddr;
        m_axi_awlen_reg <= s_axi_awlen;
        m_axi_awsize_reg <= s_axi_awsize;
        m_axi_awburst_reg <= s_axi_awburst;
        m_axi_awlock_reg <= s_axi_awlock;
        m_axi_awcache_reg <= s_axi_awcache;
        m_axi_awprot_reg <= s_axi_awprot;
        m_axi_awqos_reg <= s_axi_awqos;
        m_axi_awregion_reg <= s_axi_awregion;
        m_axi_awuser_reg <= s_axi_awuser;
    end
end

end else begin

    // AW 通道旁路
    assign m_axi_awid = s_axi_awid;
    assign m_axi_awaddr = s_axi_awaddr;
    assign m_axi_awlen = s_axi_awlen;
    assign m_axi_awsize = s_axi_awsize;
    assign m_axi_awburst = s_axi_awburst;
    assign m_axi_awlock = s_axi_awlock;
    assign m_axi_awcache = s_axi_awcache;
    assign m_axi_awprot = s_axi_awprot;
    assign m_axi_awqos = s_axi_awqos;
    assign m_axi_awregion = s_axi_awregion;
    assign m_axi_awuser = AWUSER_ENABLE ? s_axi_awuser : {AWUSER_WIDTH{1'b0}};
    assign m_axi_awvalid = s_axi_awvalid;
    assign s_axi_awready = m_axi_awready;

end

// W 通道

if (W_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   s_axi_wready_reg = 1'b0; // skid 模式下从侧 WREADY 寄存器。

reg [DATA_WIDTH-1:0]  m_axi_wdata_reg  = {DATA_WIDTH{1'b0}}; // 主输出 WDATA 寄存器。
reg [STRB_WIDTH-1:0]  m_axi_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 主输出 WSTRB 寄存器。
reg                   m_axi_wlast_reg  = 1'b0; // 主输出 WLAST 寄存器。
reg [WUSER_WIDTH-1:0] m_axi_wuser_reg  = {WUSER_WIDTH{1'b0}}; // 主输出 WUSER 寄存器。
reg                   m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // 主输出 WVALID 当前状态与下一状态。

reg [DATA_WIDTH-1:0]  temp_m_axi_wdata_reg  = {DATA_WIDTH{1'b0}}; // 输出阻塞时临时缓存 WDATA。
reg [STRB_WIDTH-1:0]  temp_m_axi_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 临时缓存 WSTRB。
reg                   temp_m_axi_wlast_reg  = 1'b0; // 临时缓存 WLAST。
reg [WUSER_WIDTH-1:0] temp_m_axi_wuser_reg  = {WUSER_WIDTH{1'b0}}; // 临时缓存 WUSER。
reg                   temp_m_axi_wvalid_reg = 1'b0, temp_m_axi_wvalid_next; // 临时缓存 WVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_w_input_to_output; // 置位时把输入 W 写入主输出寄存器。
reg store_axi_w_input_to_temp; // 置位时把输入 W 写入临时寄存器。
reg store_axi_w_temp_to_output; // 置位时把临时 W 回放到主输出寄存器。

assign s_axi_wready = s_axi_wready_reg;

assign m_axi_wdata  = m_axi_wdata_reg;
assign m_axi_wstrb  = m_axi_wstrb_reg;
assign m_axi_wlast  = m_axi_wlast_reg;
assign m_axi_wuser  = WUSER_ENABLE ? m_axi_wuser_reg : {WUSER_WIDTH{1'b0}};
assign m_axi_wvalid = m_axi_wvalid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axi_wready_early = m_axi_wready | (~temp_m_axi_wvalid_reg & (~m_axi_wvalid_reg | ~s_axi_wvalid)); // 前瞻 ready，避免 skid 气泡。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_wvalid_next = m_axi_wvalid_reg;
    temp_m_axi_wvalid_next = temp_m_axi_wvalid_reg;

    store_axi_w_input_to_output = 1'b0;
    store_axi_w_input_to_temp = 1'b0;
    store_axi_w_temp_to_output = 1'b0;

    if (s_axi_wready_reg) begin
        // 当前允许接收输入
        if (m_axi_wready | ~m_axi_wvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axi_wvalid_next = s_axi_wvalid;
            store_axi_w_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axi_wvalid_next = s_axi_wvalid;
            store_axi_w_input_to_temp = 1'b1;
        end
    end else if (m_axi_wready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        m_axi_wvalid_next = temp_m_axi_wvalid_reg;
        temp_m_axi_wvalid_next = 1'b0;
        store_axi_w_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_wready_reg <= 1'b0;
        m_axi_wvalid_reg <= 1'b0;
        temp_m_axi_wvalid_reg <= 1'b0;
    end else begin
        s_axi_wready_reg <= s_axi_wready_early;
        m_axi_wvalid_reg <= m_axi_wvalid_next;
        temp_m_axi_wvalid_reg <= temp_m_axi_wvalid_next;
    end

    // 数据通路寄存
    if (store_axi_w_input_to_output) begin
        m_axi_wdata_reg <= s_axi_wdata;
        m_axi_wstrb_reg <= s_axi_wstrb;
        m_axi_wlast_reg <= s_axi_wlast;
        m_axi_wuser_reg <= s_axi_wuser;
    end else if (store_axi_w_temp_to_output) begin
        m_axi_wdata_reg <= temp_m_axi_wdata_reg;
        m_axi_wstrb_reg <= temp_m_axi_wstrb_reg;
        m_axi_wlast_reg <= temp_m_axi_wlast_reg;
        m_axi_wuser_reg <= temp_m_axi_wuser_reg;
    end

    if (store_axi_w_input_to_temp) begin
        temp_m_axi_wdata_reg <= s_axi_wdata;
        temp_m_axi_wstrb_reg <= s_axi_wstrb;
        temp_m_axi_wlast_reg <= s_axi_wlast;
        temp_m_axi_wuser_reg <= s_axi_wuser;
    end
end

end else if (W_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   s_axi_wready_reg = 1'b0; // 简单缓冲模式下从侧 WREADY 寄存器。

reg [DATA_WIDTH-1:0]  m_axi_wdata_reg  = {DATA_WIDTH{1'b0}}; // 缓存的 WDATA。
reg [STRB_WIDTH-1:0]  m_axi_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 缓存的 WSTRB。
reg                   m_axi_wlast_reg  = 1'b0; // 缓存的 WLAST。
reg [WUSER_WIDTH-1:0] m_axi_wuser_reg  = {WUSER_WIDTH{1'b0}}; // 缓存的 WUSER。
reg                   m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // 缓存 WVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_w_input_to_output; // 置位时把输入 W 写入输出缓冲。

assign s_axi_wready = s_axi_wready_reg;

assign m_axi_wdata  = m_axi_wdata_reg;
assign m_axi_wstrb  = m_axi_wstrb_reg;
assign m_axi_wlast  = m_axi_wlast_reg;
assign m_axi_wuser  = WUSER_ENABLE ? m_axi_wuser_reg : {WUSER_WIDTH{1'b0}};
assign m_axi_wvalid = m_axi_wvalid_reg;

// 下拍 ready 预判：输出缓冲为空或将为空时拉高
wire s_axi_wready_ewly = !m_axi_wvalid_next; // 下拍 W 输出缓冲为空时允许接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axi_wvalid_next = m_axi_wvalid_reg;

    store_axi_w_input_to_output = 1'b0;

    if (s_axi_wready_reg) begin
        m_axi_wvalid_next = s_axi_wvalid;
        store_axi_w_input_to_output = 1'b1;
    end else if (m_axi_wready) begin
        m_axi_wvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_wready_reg <= 1'b0;
        m_axi_wvalid_reg <= 1'b0;
    end else begin
        s_axi_wready_reg <= s_axi_wready_ewly;
        m_axi_wvalid_reg <= m_axi_wvalid_next;
    end

    // 数据通路寄存
    if (store_axi_w_input_to_output) begin
        m_axi_wdata_reg <= s_axi_wdata;
        m_axi_wstrb_reg <= s_axi_wstrb;
        m_axi_wlast_reg <= s_axi_wlast;
        m_axi_wuser_reg <= s_axi_wuser;
    end
end

end else begin

    // W 通道旁路
    assign m_axi_wdata = s_axi_wdata;
    assign m_axi_wstrb = s_axi_wstrb;
    assign m_axi_wlast = s_axi_wlast;
    assign m_axi_wuser = WUSER_ENABLE ? s_axi_wuser : {WUSER_WIDTH{1'b0}};
    assign m_axi_wvalid = s_axi_wvalid;
    assign s_axi_wready = m_axi_wready;

end

// B 通道

if (B_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   m_axi_bready_reg = 1'b0; // skid 模式下主侧 BREADY 寄存器。

reg [ID_WIDTH-1:0]    s_axi_bid_reg    = {ID_WIDTH{1'b0}}; // 主输出 B ID 寄存器。
reg [1:0]             s_axi_bresp_reg  = 2'b0; // 主输出 BRESP 寄存器。
reg [BUSER_WIDTH-1:0] s_axi_buser_reg  = {BUSER_WIDTH{1'b0}}; // 主输出 BUSER 寄存器。
reg                   s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // 主输出 BVALID 当前状态与下一状态。

reg [ID_WIDTH-1:0]    temp_s_axi_bid_reg    = {ID_WIDTH{1'b0}}; // 输出阻塞时临时缓存 B ID。
reg [1:0]             temp_s_axi_bresp_reg  = 2'b0; // 临时缓存 BRESP。
reg [BUSER_WIDTH-1:0] temp_s_axi_buser_reg  = {BUSER_WIDTH{1'b0}}; // 临时缓存 BUSER。
reg                   temp_s_axi_bvalid_reg = 1'b0, temp_s_axi_bvalid_next; // 临时缓存 BVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_b_input_to_output; // 置位时把输入 B 写入主输出寄存器。
reg store_axi_b_input_to_temp; // 置位时把输入 B 写入临时寄存器。
reg store_axi_b_temp_to_output; // 置位时把临时 B 回放到主输出寄存器。

assign m_axi_bready = m_axi_bready_reg;

assign s_axi_bid    = s_axi_bid_reg;
assign s_axi_bresp  = s_axi_bresp_reg;
assign s_axi_buser  = BUSER_ENABLE ? s_axi_buser_reg : {BUSER_WIDTH{1'b0}};
assign s_axi_bvalid = s_axi_bvalid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
wire m_axi_bready_early = s_axi_bready | (~temp_s_axi_bvalid_reg & (~s_axi_bvalid_reg | ~m_axi_bvalid)); // 前瞻 ready，避免 skid 气泡。

always @* begin
    // 将下游就绪关系映射到上游
    s_axi_bvalid_next = s_axi_bvalid_reg;
    temp_s_axi_bvalid_next = temp_s_axi_bvalid_reg;

    store_axi_b_input_to_output = 1'b0;
    store_axi_b_input_to_temp = 1'b0;
    store_axi_b_temp_to_output = 1'b0;

    if (m_axi_bready_reg) begin
        // 当前允许接收输入
        if (s_axi_bready | ~s_axi_bvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            s_axi_bvalid_next = m_axi_bvalid;
            store_axi_b_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_s_axi_bvalid_next = m_axi_bvalid;
            store_axi_b_input_to_temp = 1'b1;
        end
    end else if (s_axi_bready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        s_axi_bvalid_next = temp_s_axi_bvalid_reg;
        temp_s_axi_bvalid_next = 1'b0;
        store_axi_b_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axi_bready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;
        temp_s_axi_bvalid_reg <= 1'b0;
    end else begin
        m_axi_bready_reg <= m_axi_bready_early;
        s_axi_bvalid_reg <= s_axi_bvalid_next;
        temp_s_axi_bvalid_reg <= temp_s_axi_bvalid_next;
    end

    // 数据通路寄存
    if (store_axi_b_input_to_output) begin
        s_axi_bid_reg   <= m_axi_bid;
        s_axi_bresp_reg <= m_axi_bresp;
        s_axi_buser_reg <= m_axi_buser;
    end else if (store_axi_b_temp_to_output) begin
        s_axi_bid_reg   <= temp_s_axi_bid_reg;
        s_axi_bresp_reg <= temp_s_axi_bresp_reg;
        s_axi_buser_reg <= temp_s_axi_buser_reg;
    end

    if (store_axi_b_input_to_temp) begin
        temp_s_axi_bid_reg   <= m_axi_bid;
        temp_s_axi_bresp_reg <= m_axi_bresp;
        temp_s_axi_buser_reg <= m_axi_buser;
    end
end

end else if (B_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   m_axi_bready_reg = 1'b0; // 简单缓冲模式下主侧 BREADY 寄存器。

reg [ID_WIDTH-1:0]    s_axi_bid_reg    = {ID_WIDTH{1'b0}}; // 缓存的 B ID。
reg [1:0]             s_axi_bresp_reg  = 2'b0; // 缓存的 BRESP。
reg [BUSER_WIDTH-1:0] s_axi_buser_reg  = {BUSER_WIDTH{1'b0}}; // 缓存的 BUSER。
reg                   s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // 缓存 BVALID 当前状态与下一状态。

// 数据通路控制
reg store_axi_b_input_to_output; // 置位时把输入 B 写入输出缓冲。

assign m_axi_bready = m_axi_bready_reg;

assign s_axi_bid    = s_axi_bid_reg;
assign s_axi_bresp  = s_axi_bresp_reg;
assign s_axi_buser  = BUSER_ENABLE ? s_axi_buser_reg : {BUSER_WIDTH{1'b0}};
assign s_axi_bvalid = s_axi_bvalid_reg;

// 下拍 ready 预判：输出缓冲为空或将为空时拉高
wire m_axi_bready_early = !s_axi_bvalid_next; // 下拍 B 输出缓冲为空时允许接收。

always @* begin
    // 将下游就绪关系映射到上游
    s_axi_bvalid_next = s_axi_bvalid_reg;

    store_axi_b_input_to_output = 1'b0;

    if (m_axi_bready_reg) begin
        s_axi_bvalid_next = m_axi_bvalid;
        store_axi_b_input_to_output = 1'b1;
    end else if (s_axi_bready) begin
        s_axi_bvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axi_bready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;
    end else begin
        m_axi_bready_reg <= m_axi_bready_early;
        s_axi_bvalid_reg <= s_axi_bvalid_next;
    end

    // 数据通路寄存
    if (store_axi_b_input_to_output) begin
        s_axi_bid_reg   <= m_axi_bid;
        s_axi_bresp_reg <= m_axi_bresp;
        s_axi_buser_reg <= m_axi_buser;
    end
end

end else begin

    // B 通道旁路
    assign s_axi_bid = m_axi_bid;
    assign s_axi_bresp = m_axi_bresp;
    assign s_axi_buser = BUSER_ENABLE ? m_axi_buser : {BUSER_WIDTH{1'b0}};
    assign s_axi_bvalid = m_axi_bvalid;
    assign m_axi_bready = s_axi_bready;

end

endgenerate

endmodule

`resetall
