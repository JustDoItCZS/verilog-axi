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
 * AXI4-Lite 寄存器切片（读通道）
 *
 * 模块目录
 * 1) 输入从侧读通道（AR/R），输出主侧读通道。
 * 2) AR 与 R 通道均支持旁路/简单寄存/skid buffer 三种模式。
 * 3) 控制信号决定数据直通、暂存到临时寄存器或从临时寄存器回放。
 */
module axil_register_rd #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // AR 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter AR_REG_TYPE = 1,
    // R 通道寄存类型
    // 0 表示旁路，1 表示简单一级缓冲
    parameter R_REG_TYPE = 1
)
(
    input  wire                     clk,            // 读通道寄存状态时钟。
    input  wire                     rst,            // AR/R 缓冲状态同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]    s_axil_araddr,  // 从侧输入 AR 地址。
    input  wire [2:0]               s_axil_arprot,  // 从侧输入 AR 保护属性。
    input  wire                     s_axil_arvalid, // 从侧输入 AR 有效。
    output wire                     s_axil_arready, // 从侧输出 AR 就绪（经可选缓冲后）。
    output wire [DATA_WIDTH-1:0]    s_axil_rdata,   // 从侧输出 R 数据（来自下游）。
    output wire [1:0]               s_axil_rresp,   // 从侧输出 R 响应（来自下游）。
    output wire                     s_axil_rvalid,  // 从侧输出 R 有效（经可选缓冲后）。
    input  wire                     s_axil_rready,  // 从侧输入 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]    m_axil_araddr,  // 主侧输出 AR 地址（发往下游）。
    output wire [2:0]               m_axil_arprot,  // 主侧输出 AR 保护属性。
    output wire                     m_axil_arvalid, // 主侧输出 AR 有效（经所选缓冲后）。
    input  wire                     m_axil_arready, // 主侧输入 AR 就绪（来自下游目标）。
    input  wire [DATA_WIDTH-1:0]    m_axil_rdata,   // 主侧输入 R 数据（来自下游目标）。
    input  wire [1:0]               m_axil_rresp,   // 主侧输入 R 响应（来自下游目标）。
    input  wire                     m_axil_rvalid,  // 主侧输入 R 有效（来自下游目标）。
    output wire                     m_axil_rready   // 主侧输出 R 就绪（经所选缓冲后）。
);

generate

// AR 通道

if (AR_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                    s_axil_arready_reg = 1'b0; // 指向从侧 AR 通道的本地 ready。

reg [ADDR_WIDTH-1:0]   m_axil_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 发往主侧的主 AR 数据寄存器。
reg [2:0]              m_axil_arprot_reg   = 3'd0; // 主 AR 保护属性寄存器。
reg                    m_axil_arvalid_reg  = 1'b0, m_axil_arvalid_next; // 主 AR valid 当前状态与下一状态。

reg [ADDR_WIDTH-1:0]   temp_m_axil_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 主输出阻塞时临时缓存的 AR 数据。
reg [2:0]              temp_m_axil_arprot_reg   = 3'd0; // 临时缓存 AR 保护属性。
reg                    temp_m_axil_arvalid_reg  = 1'b0, temp_m_axil_arvalid_next; // 临时缓存 AR valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_ar_input_to_output; // 置位时把输入 AR 写入主输出寄存器。
reg store_axil_ar_input_to_temp; // 置位时把输入 AR 写入临时寄存器。
reg store_axil_ar_temp_to_output; // 置位时把临时 AR 提升到主输出寄存器。

assign s_axil_arready  = s_axil_arready_reg;

assign m_axil_araddr   = m_axil_araddr_reg;
assign m_axil_arprot   = m_axil_arprot_reg;
assign m_axil_arvalid  = m_axil_arvalid_reg;

// 下拍 ready 预判：当输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axil_arready_early = m_axil_arready | (~temp_m_axil_arvalid_reg & (~m_axil_arvalid_reg | ~s_axil_arvalid)); // 前瞻 AR ready，避免 skid buffer 产生气泡。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_arvalid_next = m_axil_arvalid_reg;
    temp_m_axil_arvalid_next = temp_m_axil_arvalid_reg;

    store_axil_ar_input_to_output = 1'b0;
    store_axil_ar_input_to_temp = 1'b0;
    store_axil_ar_temp_to_output = 1'b0;

    if (s_axil_arready_reg) begin
        // 当前允许接收输入
        if (m_axil_arready | ~m_axil_arvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axil_arvalid_next = s_axil_arvalid;
            store_axil_ar_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axil_arvalid_next = s_axil_arvalid;
            store_axil_ar_input_to_temp = 1'b1;
        end
    end else if (m_axil_arready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放到主输出
        m_axil_arvalid_next = temp_m_axil_arvalid_reg;
        temp_m_axil_arvalid_next = 1'b0;
        store_axil_ar_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_arready_reg <= 1'b0;
        m_axil_arvalid_reg <= 1'b0;
        temp_m_axil_arvalid_reg <= 1'b0;
    end else begin
        s_axil_arready_reg <= s_axil_arready_early;
        m_axil_arvalid_reg <= m_axil_arvalid_next;
        temp_m_axil_arvalid_reg <= temp_m_axil_arvalid_next;
    end

    // 数据通路寄存
    if (store_axil_ar_input_to_output) begin
        m_axil_araddr_reg <= s_axil_araddr;
        m_axil_arprot_reg <= s_axil_arprot;
    end else if (store_axil_ar_temp_to_output) begin
        m_axil_araddr_reg <= temp_m_axil_araddr_reg;
        m_axil_arprot_reg <= temp_m_axil_arprot_reg;
    end

    if (store_axil_ar_input_to_temp) begin
        temp_m_axil_araddr_reg <= s_axil_araddr;
        temp_m_axil_arprot_reg <= s_axil_arprot;
    end
end

end else if (AR_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                    s_axil_arready_reg = 1'b0; // 指向从侧的本地 ready，输出槽空闲时拉高。

reg [ADDR_WIDTH-1:0]   m_axil_araddr_reg   = {ADDR_WIDTH{1'b0}}; // 单级 AR 地址寄存器。
reg [2:0]              m_axil_arprot_reg   = 3'd0; // 单级 AR 保护属性寄存器。
reg                    m_axil_arvalid_reg  = 1'b0, m_axil_arvalid_next; // 单级 AR valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_ar_input_to_output; // 置位时把接收的从侧 AR 装入输出寄存器。

assign s_axil_arready  = s_axil_arready_reg;

assign m_axil_araddr   = m_axil_araddr_reg;
assign m_axil_arprot   = m_axil_arprot_reg;
assign m_axil_arvalid  = m_axil_arvalid_reg;

// 下拍 ready 预判：输出寄存器为空或将为空时拉高
wire s_axil_arready_early = !m_axil_arvalid_next; // 输出寄存器为空（或将为空）时，下拍可接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_arvalid_next = m_axil_arvalid_reg;

    store_axil_ar_input_to_output = 1'b0;

    if (s_axil_arready_reg) begin
        m_axil_arvalid_next = s_axil_arvalid;
        store_axil_ar_input_to_output = 1'b1;
    end else if (m_axil_arready) begin
        m_axil_arvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_arready_reg <= 1'b0;
        m_axil_arvalid_reg <= 1'b0;
    end else begin
        s_axil_arready_reg <= s_axil_arready_early;
        m_axil_arvalid_reg <= m_axil_arvalid_next;
    end

    // 数据通路寄存
    if (store_axil_ar_input_to_output) begin
        m_axil_araddr_reg <= s_axil_araddr;
        m_axil_arprot_reg <= s_axil_arprot;
    end
end

end else begin

    // AR 通道旁路
    assign m_axil_araddr = s_axil_araddr;
    assign m_axil_arprot = s_axil_arprot;
    assign m_axil_arvalid = s_axil_arvalid;
    assign s_axil_arready = m_axil_arready;

end

// R 通道

if (R_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   m_axil_rready_reg = 1'b0; // 指向主侧 R 通道的本地 ready。

reg [DATA_WIDTH-1:0]  s_axil_rdata_reg  = {DATA_WIDTH{1'b0}}; // 发往从侧的主 R 数据寄存器。
reg [1:0]             s_axil_rresp_reg  = 2'b0; // 主 R 响应寄存器。
reg                   s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next; // 主 R valid 当前状态与下一状态。

reg [DATA_WIDTH-1:0]  temp_s_axil_rdata_reg  = {DATA_WIDTH{1'b0}}; // 从侧阻塞时临时缓存的 R 数据。
reg [1:0]             temp_s_axil_rresp_reg  = 2'b0; // 临时缓存 R 响应。
reg                   temp_s_axil_rvalid_reg = 1'b0, temp_s_axil_rvalid_next; // 临时缓存 R valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_r_input_to_output; // 置位时把主侧输入 R 写入主输出寄存器。
reg store_axil_r_input_to_temp; // 置位时把主侧输入 R 写入临时寄存器。
reg store_axil_r_temp_to_output; // 置位时把临时 R 提升到主输出寄存器。

assign m_axil_rready = m_axil_rready_reg;

assign s_axil_rdata  = s_axil_rdata_reg;
assign s_axil_rresp  = s_axil_rresp_reg;
assign s_axil_rvalid = s_axil_rvalid_reg;

// 下拍 ready 预判：当输出可接收，或下拍临时寄存器不会被占用时拉高
wire m_axil_rready_early = s_axil_rready | (~temp_s_axil_rvalid_reg & (~s_axil_rvalid_reg | ~m_axil_rvalid)); // 前瞻 R ready，避免 skid buffer 产生气泡。

always @* begin
    // 将下游就绪关系映射到上游
    s_axil_rvalid_next = s_axil_rvalid_reg;
    temp_s_axil_rvalid_next = temp_s_axil_rvalid_reg;

    store_axil_r_input_to_output = 1'b0;
    store_axil_r_input_to_temp = 1'b0;
    store_axil_r_temp_to_output = 1'b0;

    if (m_axil_rready_reg) begin
        // 当前允许接收输入
        if (s_axil_rready | ~s_axil_rvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            s_axil_rvalid_next = m_axil_rvalid;
            store_axil_r_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_s_axil_rvalid_next = m_axil_rvalid;
            store_axil_r_input_to_temp = 1'b1;
        end
    end else if (s_axil_rready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放到主输出
        s_axil_rvalid_next = temp_s_axil_rvalid_reg;
        temp_s_axil_rvalid_next = 1'b0;
        store_axil_r_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axil_rready_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
        temp_s_axil_rvalid_reg <= 1'b0;
    end else begin
        m_axil_rready_reg <= m_axil_rready_early;
        s_axil_rvalid_reg <= s_axil_rvalid_next;
        temp_s_axil_rvalid_reg <= temp_s_axil_rvalid_next;
    end

    // 数据通路寄存
    if (store_axil_r_input_to_output) begin
        s_axil_rdata_reg <= m_axil_rdata;
        s_axil_rresp_reg <= m_axil_rresp;
    end else if (store_axil_r_temp_to_output) begin
        s_axil_rdata_reg <= temp_s_axil_rdata_reg;
        s_axil_rresp_reg <= temp_s_axil_rresp_reg;
    end

    if (store_axil_r_input_to_temp) begin
        temp_s_axil_rdata_reg <= m_axil_rdata;
        temp_s_axil_rresp_reg <= m_axil_rresp;
    end
end

end else if (R_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   m_axil_rready_reg = 1'b0; // 简单模式下指向主侧 R 通道的本地 ready。

reg [DATA_WIDTH-1:0]  s_axil_rdata_reg  = {DATA_WIDTH{1'b0}}; // 单级 R 数据寄存器。
reg [1:0]             s_axil_rresp_reg  = 2'b0; // 单级 R 响应寄存器。
reg                   s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next; // 单级 R valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_r_input_to_output; // 置位时把接收的主侧 R 装入输出寄存器。

assign m_axil_rready = m_axil_rready_reg;

assign s_axil_rdata  = s_axil_rdata_reg;
assign s_axil_rresp  = s_axil_rresp_reg;
assign s_axil_rvalid = s_axil_rvalid_reg;

// 下拍 ready 预判：输出寄存器为空或将为空时拉高
wire m_axil_rready_early = !s_axil_rvalid_next; // 输出寄存器为空（或将为空）时，下拍可接收。

always @* begin
    // 将下游就绪关系映射到上游
    s_axil_rvalid_next = s_axil_rvalid_reg;

    store_axil_r_input_to_output = 1'b0;

    if (m_axil_rready_reg) begin
        s_axil_rvalid_next = m_axil_rvalid;
        store_axil_r_input_to_output = 1'b1;
    end else if (s_axil_rready) begin
        s_axil_rvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axil_rready_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
    end else begin
        m_axil_rready_reg <= m_axil_rready_early;
        s_axil_rvalid_reg <= s_axil_rvalid_next;
    end

    // 数据通路寄存
    if (store_axil_r_input_to_output) begin
        s_axil_rdata_reg <= m_axil_rdata;
        s_axil_rresp_reg <= m_axil_rresp;
    end
end

end else begin

    // R 通道旁路
    assign s_axil_rdata = m_axil_rdata;
    assign s_axil_rresp = m_axil_rresp;
    assign s_axil_rvalid = m_axil_rvalid;
    assign m_axil_rready = s_axil_rready;

end

endgenerate

endmodule

`resetall
