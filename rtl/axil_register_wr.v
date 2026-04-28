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
 * AXI4-Lite 寄存器切片（写通道）
 *
 * 模块目录
 * 1) 输入从侧写通道（AW/W/B），输出主侧写通道。
 * 2) 每个通道可独立选择缓冲模式：
 *    - 旁路
 *    - 简单一级寄存（可能产生气泡）
 *    - skid buffer（背压切换时无气泡）
 * 3) 每个通道都包含本地握手控制和载荷寄存器。
 */
module axil_register_wr #
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
    parameter B_REG_TYPE = 1
)
(
    input  wire                     clk,            // 写通道缓冲状态时钟。
    input  wire                     rst,            // 写通道寄存状态同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]    s_axil_awaddr,  // 从侧输入 AW 地址。
    input  wire [2:0]               s_axil_awprot,  // 从侧输入 AW 保护属性。
    input  wire                     s_axil_awvalid, // 从侧输入 AW 有效。
    output wire                     s_axil_awready, // 从侧输出 AW 就绪（经可选缓冲后）。
    input  wire [DATA_WIDTH-1:0]    s_axil_wdata,   // 从侧输入 W 数据。
    input  wire [STRB_WIDTH-1:0]    s_axil_wstrb,   // 从侧输入 W 字节使能。
    input  wire                     s_axil_wvalid,  // 从侧输入 W 有效。
    output wire                     s_axil_wready,  // 从侧输出 W 就绪（经可选缓冲后）。
    output wire [1:0]               s_axil_bresp,   // 从侧输出 B 响应（来自下游）。
    output wire                     s_axil_bvalid,  // 从侧输出 B 有效（来自下游）。
    input  wire                     s_axil_bready,  // 从侧输入 B 就绪。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]    m_axil_awaddr,  // 主侧输出 AW 地址（发往下游目标）。
    output wire [2:0]               m_axil_awprot,  // 主侧输出 AW 保护属性。
    output wire                     m_axil_awvalid, // 主侧输出 AW 有效（经所选缓冲后）。
    input  wire                     m_axil_awready, // 主侧输入 AW 就绪（来自下游目标）。
    output wire [DATA_WIDTH-1:0]    m_axil_wdata,   // 主侧输出 W 数据。
    output wire [STRB_WIDTH-1:0]    m_axil_wstrb,   // 主侧输出 W 字节使能。
    output wire                     m_axil_wvalid,  // 主侧输出 W 有效（经所选缓冲后）。
    input  wire                     m_axil_wready,  // 主侧输入 W 就绪（来自下游目标）。
    input  wire [1:0]               m_axil_bresp,   // 主侧输入 B 响应（来自下游目标）。
    input  wire                     m_axil_bvalid,  // 主侧输入 B 有效（来自下游目标）。
    output wire                     m_axil_bready   // 主侧输出 B 就绪（经所选缓冲后）。
);

generate

// AW 通道

if (AW_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                    s_axil_awready_reg = 1'b0; // 指向从侧的本地 ready，由 s_axil_awready_early 驱动。

reg [ADDR_WIDTH-1:0]   m_axil_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 发往主侧的主 AW 数据寄存器。
reg [2:0]              m_axil_awprot_reg   = 3'd0; // 与主 AW 数据对齐的保护属性寄存器。
reg                    m_axil_awvalid_reg  = 1'b0, m_axil_awvalid_next; // 主 AW valid 当前状态与下一状态。

reg [ADDR_WIDTH-1:0]   temp_m_axil_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 输出级阻塞时临时缓存的 AW 数据。
reg [2:0]              temp_m_axil_awprot_reg   = 3'd0; // 临时缓存 AW 保护属性。
reg                    temp_m_axil_awvalid_reg  = 1'b0, temp_m_axil_awvalid_next; // 临时缓存 AW valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_aw_input_to_output; // 置位时把输入 AW 写入主输出寄存器。
reg store_axil_aw_input_to_temp; // 置位时在下游阻塞时把输入 AW 写入临时寄存器。
reg store_axil_aw_temp_to_output; // 置位时把临时 AW 提升到主输出寄存器。

assign s_axil_awready  = s_axil_awready_reg;

assign m_axil_awaddr   = m_axil_awaddr_reg;
assign m_axil_awprot   = m_axil_awprot_reg;
assign m_axil_awvalid  = m_axil_awvalid_reg;

// 下拍 ready 预判：当输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axil_awready_early = m_axil_awready | (~temp_m_axil_awvalid_reg & (~m_axil_awvalid_reg | ~s_axil_awvalid)); // 组合前瞻 ready，避免气泡周期。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_awvalid_next = m_axil_awvalid_reg;
    temp_m_axil_awvalid_next = temp_m_axil_awvalid_reg;

    store_axil_aw_input_to_output = 1'b0;
    store_axil_aw_input_to_temp = 1'b0;
    store_axil_aw_temp_to_output = 1'b0;

    if (s_axil_awready_reg) begin
        // 当前允许接收输入
        if (m_axil_awready | ~m_axil_awvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axil_awvalid_next = s_axil_awvalid;
            store_axil_aw_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axil_awvalid_next = s_axil_awvalid;
            store_axil_aw_input_to_temp = 1'b1;
        end
    end else if (m_axil_awready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放到主输出
        m_axil_awvalid_next = temp_m_axil_awvalid_reg;
        temp_m_axil_awvalid_next = 1'b0;
        store_axil_aw_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_awready_reg <= 1'b0;
        m_axil_awvalid_reg <= 1'b0;
        temp_m_axil_awvalid_reg <= 1'b0;
    end else begin
        s_axil_awready_reg <= s_axil_awready_early;
        m_axil_awvalid_reg <= m_axil_awvalid_next;
        temp_m_axil_awvalid_reg <= temp_m_axil_awvalid_next;
    end

    // 数据通路寄存
    if (store_axil_aw_input_to_output) begin
        m_axil_awaddr_reg <= s_axil_awaddr;
        m_axil_awprot_reg <= s_axil_awprot;
    end else if (store_axil_aw_temp_to_output) begin
        m_axil_awaddr_reg <= temp_m_axil_awaddr_reg;
        m_axil_awprot_reg <= temp_m_axil_awprot_reg;
    end

    if (store_axil_aw_input_to_temp) begin
        temp_m_axil_awaddr_reg <= s_axil_awaddr;
        temp_m_axil_awprot_reg <= s_axil_awprot;
    end
end

end else if (AW_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                    s_axil_awready_reg = 1'b0; // 指向从侧的本地 ready，简单输出槽空闲时拉高。

reg [ADDR_WIDTH-1:0]   m_axil_awaddr_reg   = {ADDR_WIDTH{1'b0}}; // 单级 AW 数据寄存器。
reg [2:0]              m_axil_awprot_reg   = 3'd0; // 单级 AW 保护属性寄存器。
reg                    m_axil_awvalid_reg  = 1'b0, m_axil_awvalid_next; // 单级 AW valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_aw_input_to_output; // 置位时把接收的从侧 AW 装入输出寄存器。

assign s_axil_awready  = s_axil_awready_reg;

assign m_axil_awaddr   = m_axil_awaddr_reg;
assign m_axil_awprot   = m_axil_awprot_reg;
assign m_axil_awvalid  = m_axil_awvalid_reg;

// 下拍 ready 预判：输出寄存器为空或将为空时拉高
wire s_axil_awready_early = !m_axil_awvalid_next; // 输出寄存器为空（或将为空）时，下拍可接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_awvalid_next = m_axil_awvalid_reg;

    store_axil_aw_input_to_output = 1'b0;

    if (s_axil_awready_reg) begin
        m_axil_awvalid_next = s_axil_awvalid;
        store_axil_aw_input_to_output = 1'b1;
    end else if (m_axil_awready) begin
        m_axil_awvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_awready_reg <= 1'b0;
        m_axil_awvalid_reg <= 1'b0;
    end else begin
        s_axil_awready_reg <= s_axil_awready_early;
        m_axil_awvalid_reg <= m_axil_awvalid_next;
    end

    // 数据通路寄存
    if (store_axil_aw_input_to_output) begin
        m_axil_awaddr_reg <= s_axil_awaddr;
        m_axil_awprot_reg <= s_axil_awprot;
    end
end

end else begin

    // AW 通道旁路
    assign m_axil_awaddr = s_axil_awaddr;
    assign m_axil_awprot = s_axil_awprot;
    assign m_axil_awvalid = s_axil_awvalid;
    assign s_axil_awready = m_axil_awready;

end

// W 通道

if (W_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   s_axil_wready_reg = 1'b0; // 指向从侧 W 通道的本地 ready。

reg [DATA_WIDTH-1:0]  m_axil_wdata_reg  = {DATA_WIDTH{1'b0}}; // 发往主侧的主 W 数据寄存器。
reg [STRB_WIDTH-1:0]  m_axil_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 主 W 字节使能寄存器。
reg                   m_axil_wvalid_reg = 1'b0, m_axil_wvalid_next; // 主 W valid 当前状态与下一状态。

reg [DATA_WIDTH-1:0]  temp_m_axil_wdata_reg  = {DATA_WIDTH{1'b0}}; // 主输出阻塞时临时缓存的 W 数据。
reg [STRB_WIDTH-1:0]  temp_m_axil_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 主输出阻塞时临时缓存的 W 字节使能。
reg                   temp_m_axil_wvalid_reg = 1'b0, temp_m_axil_wvalid_next; // 临时缓存 W valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_w_input_to_output; // 置位时把输入 W 写入主输出寄存器。
reg store_axil_w_input_to_temp; // 置位时把输入 W 写入临时寄存器。
reg store_axil_w_temp_to_output; // 置位时把临时 W 回放到主输出寄存器。

assign s_axil_wready = s_axil_wready_reg;

assign m_axil_wdata  = m_axil_wdata_reg;
assign m_axil_wstrb  = m_axil_wstrb_reg;
assign m_axil_wvalid = m_axil_wvalid_reg;

// 下拍 ready 预判：当输出可接收，或下拍临时寄存器不会被占用时拉高
wire s_axil_wready_early = m_axil_wready | (~temp_m_axil_wvalid_reg & (~m_axil_wvalid_reg | ~s_axil_wvalid)); // 前瞻 ready，保持无气泡运行。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_wvalid_next = m_axil_wvalid_reg;
    temp_m_axil_wvalid_next = temp_m_axil_wvalid_reg;

    store_axil_w_input_to_output = 1'b0;
    store_axil_w_input_to_temp = 1'b0;
    store_axil_w_temp_to_output = 1'b0;

    if (s_axil_wready_reg) begin
        // 当前允许接收输入
        if (m_axil_wready | ~m_axil_wvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axil_wvalid_next = s_axil_wvalid;
            store_axil_w_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axil_wvalid_next = s_axil_wvalid;
            store_axil_w_input_to_temp = 1'b1;
        end
    end else if (m_axil_wready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放到主输出
        m_axil_wvalid_next = temp_m_axil_wvalid_reg;
        temp_m_axil_wvalid_next = 1'b0;
        store_axil_w_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_wready_reg <= 1'b0;
        m_axil_wvalid_reg <= 1'b0;
        temp_m_axil_wvalid_reg <= 1'b0;
    end else begin
        s_axil_wready_reg <= s_axil_wready_early;
        m_axil_wvalid_reg <= m_axil_wvalid_next;
        temp_m_axil_wvalid_reg <= temp_m_axil_wvalid_next;
    end

    // 数据通路寄存
    if (store_axil_w_input_to_output) begin
        m_axil_wdata_reg <= s_axil_wdata;
        m_axil_wstrb_reg <= s_axil_wstrb;
    end else if (store_axil_w_temp_to_output) begin
        m_axil_wdata_reg <= temp_m_axil_wdata_reg;
        m_axil_wstrb_reg <= temp_m_axil_wstrb_reg;
    end

    if (store_axil_w_input_to_temp) begin
        temp_m_axil_wdata_reg <= s_axil_wdata;
        temp_m_axil_wstrb_reg <= s_axil_wstrb;
    end
end

end else if (W_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   s_axil_wready_reg = 1'b0; // 指向从侧的本地 ready，简单输出槽可用时拉高。

reg [DATA_WIDTH-1:0]  m_axil_wdata_reg  = {DATA_WIDTH{1'b0}}; // 单级 W 数据寄存器。
reg [STRB_WIDTH-1:0]  m_axil_wstrb_reg  = {STRB_WIDTH{1'b0}}; // 单级 W 字节使能寄存器。
reg                   m_axil_wvalid_reg = 1'b0, m_axil_wvalid_next; // 单级 W valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_w_input_to_output; // 置位时把接收的从侧 W 装入输出寄存器。

assign s_axil_wready = s_axil_wready_reg;

assign m_axil_wdata  = m_axil_wdata_reg;
assign m_axil_wstrb  = m_axil_wstrb_reg;
assign m_axil_wvalid = m_axil_wvalid_reg;

// 下拍 ready 预判：输出寄存器为空或将为空时拉高
wire s_axil_wready_early = !m_axil_wvalid_next; // 输出寄存器为空（或将为空）时，下拍可接收。

always @* begin
    // 将下游就绪关系映射到上游
    m_axil_wvalid_next = m_axil_wvalid_reg;

    store_axil_w_input_to_output = 1'b0;

    if (s_axil_wready_reg) begin
        m_axil_wvalid_next = s_axil_wvalid;
        store_axil_w_input_to_output = 1'b1;
    end else if (m_axil_wready) begin
        m_axil_wvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axil_wready_reg <= 1'b0;
        m_axil_wvalid_reg <= 1'b0;
    end else begin
        s_axil_wready_reg <= s_axil_wready_early;
        m_axil_wvalid_reg <= m_axil_wvalid_next;
    end

    // 数据通路寄存
    if (store_axil_w_input_to_output) begin
        m_axil_wdata_reg <= s_axil_wdata;
        m_axil_wstrb_reg <= s_axil_wstrb;
    end
end

end else begin

    // W 通道旁路
    assign m_axil_wdata = s_axil_wdata;
    assign m_axil_wstrb = s_axil_wstrb;
    assign m_axil_wvalid = s_axil_wvalid;
    assign s_axil_wready = m_axil_wready;

end

// B 通道

if (B_REG_TYPE > 1) begin
// skid buffer，无气泡周期

// 数据通路寄存器
reg                   m_axil_bready_reg = 1'b0; // 指向主侧 B 通道的本地 ready。

reg [1:0]             s_axil_bresp_reg  = 2'b0; // 发往从侧的主 B 响应寄存器。
reg                   s_axil_bvalid_reg = 1'b0, s_axil_bvalid_next; // 主 B valid 当前状态与下一状态。

reg [1:0]             temp_s_axil_bresp_reg  = 2'b0; // 从侧阻塞时临时缓存的 B 响应。
reg                   temp_s_axil_bvalid_reg = 1'b0, temp_s_axil_bvalid_next; // 临时缓存 B valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_b_input_to_output; // 置位时把主侧输入 B 写入主输出寄存器。
reg store_axil_b_input_to_temp; // 置位时把主侧输入 B 写入临时寄存器。
reg store_axil_b_temp_to_output; // 置位时把临时 B 回放到主输出寄存器。

assign m_axil_bready = m_axil_bready_reg;

assign s_axil_bresp  = s_axil_bresp_reg;
assign s_axil_bvalid = s_axil_bvalid_reg;

// 下拍 ready 预判：当输出可接收，或下拍临时寄存器不会被占用时拉高
wire m_axil_bready_early = s_axil_bready | (~temp_s_axil_bvalid_reg & (~s_axil_bvalid_reg | ~m_axil_bvalid)); // 前瞻 ready，保持 B 通道无气泡。

always @* begin
    // 将下游就绪关系映射到上游
    s_axil_bvalid_next = s_axil_bvalid_reg;
    temp_s_axil_bvalid_next = temp_s_axil_bvalid_reg;

    store_axil_b_input_to_output = 1'b0;
    store_axil_b_input_to_temp = 1'b0;
    store_axil_b_temp_to_output = 1'b0;

    if (m_axil_bready_reg) begin
        // 当前允许接收输入
        if (s_axil_bready | ~s_axil_bvalid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            s_axil_bvalid_next = m_axil_bvalid;
            store_axil_b_input_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_s_axil_bvalid_next = m_axil_bvalid;
            store_axil_b_input_to_temp = 1'b1;
        end
    end else if (s_axil_bready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放到主输出
        s_axil_bvalid_next = temp_s_axil_bvalid_reg;
        temp_s_axil_bvalid_next = 1'b0;
        store_axil_b_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axil_bready_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
        temp_s_axil_bvalid_reg <= 1'b0;
    end else begin
        m_axil_bready_reg <= m_axil_bready_early;
        s_axil_bvalid_reg <= s_axil_bvalid_next;
        temp_s_axil_bvalid_reg <= temp_s_axil_bvalid_next;
    end

    // 数据通路寄存
    if (store_axil_b_input_to_output) begin
        s_axil_bresp_reg <= m_axil_bresp;
    end else if (store_axil_b_temp_to_output) begin
        s_axil_bresp_reg <= temp_s_axil_bresp_reg;
    end

    if (store_axil_b_input_to_temp) begin
        temp_s_axil_bresp_reg <= m_axil_bresp;
    end
end

end else if (B_REG_TYPE == 1) begin
// 简单寄存模式，会引入气泡周期

// 数据通路寄存器
reg                   m_axil_bready_reg = 1'b0; // 简单模式下指向主侧 B 通道的本地 ready。

reg [1:0]             s_axil_bresp_reg  = 2'b0; // 单级 B 响应寄存器。
reg                   s_axil_bvalid_reg = 1'b0, s_axil_bvalid_next; // 单级 B valid 当前状态与下一状态。

// 数据通路控制
reg store_axil_b_input_to_output; // 置位时把接收的主侧 B 装入输出寄存器。

assign m_axil_bready = m_axil_bready_reg;

assign s_axil_bresp  = s_axil_bresp_reg;
assign s_axil_bvalid = s_axil_bvalid_reg;

// 下拍 ready 预判：输出寄存器为空或将为空时拉高
wire m_axil_bready_early = !s_axil_bvalid_next; // 输出寄存器为空（或将为空）时，下拍可接收。

always @* begin
    // 将下游就绪关系映射到上游
    s_axil_bvalid_next = s_axil_bvalid_reg;

    store_axil_b_input_to_output = 1'b0;

    if (m_axil_bready_reg) begin
        s_axil_bvalid_next = m_axil_bvalid;
        store_axil_b_input_to_output = 1'b1;
    end else if (s_axil_bready) begin
        s_axil_bvalid_next = 1'b0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axil_bready_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
    end else begin
        m_axil_bready_reg <= m_axil_bready_early;
        s_axil_bvalid_reg <= s_axil_bvalid_next;
    end

    // 数据通路寄存
    if (store_axil_b_input_to_output) begin
        s_axil_bresp_reg <= m_axil_bresp;
    end
end

end else begin

    // B 通道旁路
    assign s_axil_bresp = m_axil_bresp;
    assign s_axil_bvalid = m_axil_bvalid;
    assign m_axil_bready = s_axil_bready;

end

endgenerate

endmodule

`resetall
