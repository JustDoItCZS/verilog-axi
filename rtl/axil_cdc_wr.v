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
 * AXI4-Lite 跨时钟域模块（写通道）
 *
 * 模块目录
 * 1) 源时钟域（s_clk）：接收从侧 AW/W，并向从侧返回 B。
 * 2) 目标时钟域（m_clk）：向主侧发起 AW/W，并接收主侧返回 B。
 * 3) 跨域握手：使用 s_flag_reg/m_flag_reg 两个单比特标志双向同步。
 * 4) 数据跨域方式：在源时钟域寄存器保持载荷，直到目标时钟域消费并应答。
 */
module axil_cdc_wr #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8)
)
(
    /*
     * AXI-Lite 从接口
     */
    input  wire                   s_clk,           // 源时钟域时钟（从侧）。
    input  wire                   s_rst,           // 源时钟域同步复位。
    input  wire [ADDR_WIDTH-1:0]  s_axil_awaddr,   // 源时钟域 AW 地址。
    input  wire [2:0]             s_axil_awprot,   // 源时钟域 AW 保护属性。
    input  wire                   s_axil_awvalid,  // 源时钟域 AW 有效。
    output wire                   s_axil_awready,  // 源时钟域 AW 就绪（本地缓存空闲时拉高）。
    input  wire [DATA_WIDTH-1:0]  s_axil_wdata,    // 源时钟域 W 数据。
    input  wire [STRB_WIDTH-1:0]  s_axil_wstrb,    // 源时钟域 W 字节使能。
    input  wire                   s_axil_wvalid,   // 源时钟域 W 有效。
    output wire                   s_axil_wready,   // 源时钟域 W 就绪（本地缓存空闲时拉高）。
    output wire [1:0]             s_axil_bresp,    // 源时钟域 B 响应（由目标时钟域返回）。
    output wire                   s_axil_bvalid,   // 源时钟域 B 有效。
    input  wire                   s_axil_bready,   // 源时钟域 B 就绪。

    /*
     * AXI-Lite 主接口
     */
    input  wire                   m_clk,           // 目标时钟域时钟（主侧）。
    input  wire                   m_rst,           // 目标时钟域同步复位。
    output wire [ADDR_WIDTH-1:0]  m_axil_awaddr,   // 目标时钟域 AW 地址。
    output wire [2:0]             m_axil_awprot,   // 目标时钟域 AW 保护属性。
    output wire                   m_axil_awvalid,  // 目标时钟域 AW 有效。
    input  wire                   m_axil_awready,  // 目标时钟域 AW 就绪（来自下游目标）。
    output wire [DATA_WIDTH-1:0]  m_axil_wdata,    // 目标时钟域 W 数据。
    output wire [STRB_WIDTH-1:0]  m_axil_wstrb,    // 目标时钟域 W 字节使能。
    output wire                   m_axil_wvalid,   // 目标时钟域 W 有效。
    input  wire                   m_axil_wready,   // 目标时钟域 W 就绪（来自下游目标）。
    input  wire [1:0]             m_axil_bresp,    // 目标时钟域 B 响应（来自下游目标）。
    input  wire                   m_axil_bvalid,   // 目标时钟域 B 有效（来自下游目标）。
    output wire                   m_axil_bready    // 目标时钟域 B 就绪（等待捕获响应时拉高）。
);

reg [1:0] s_state_reg = 2'd0; // 源时钟域状态机：空闲 -> 等待应答 -> 等待应答标志清零。
reg s_flag_reg = 1'b0; // 源时钟域请求标志，写请求挂起期间置位。
(* srl_style = "register" *)
reg s_flag_sync_reg_1 = 1'b0; // s_flag_reg 进入目标时钟域的一级同步寄存器。
(* srl_style = "register" *)
reg s_flag_sync_reg_2 = 1'b0; // s_flag_reg 的二级同步寄存器（目标时钟域稳定采样）。

reg [1:0] m_state_reg = 2'd0; // 目标时钟域状态机：空闲 -> 等待 B -> 等待请求标志清零。
reg m_flag_reg = 1'b0; // 目标时钟域应答标志，写响应可返回时置位。
(* srl_style = "register" *)
reg m_flag_sync_reg_1 = 1'b0; // m_flag_reg 返回源时钟域的一级同步寄存器。
(* srl_style = "register" *)
reg m_flag_sync_reg_2 = 1'b0; // m_flag_reg 的二级同步寄存器（源时钟域稳定采样）。

reg [ADDR_WIDTH-1:0]  s_axil_awaddr_reg = {ADDR_WIDTH{1'b0}}; // 源时钟域缓存的 AW 地址，跨域事务期间保持不变。
reg [2:0]             s_axil_awprot_reg = 3'd0; // 源时钟域缓存的 AW 保护属性。
reg                   s_axil_awvalid_reg = 1'b0; // 源时钟域锁存的 AW 有效（表示请求待处理）。
reg [DATA_WIDTH-1:0]  s_axil_wdata_reg = {DATA_WIDTH{1'b0}}; // 源时钟域缓存的 W 数据，跨域事务期间保持不变。
reg [STRB_WIDTH-1:0]  s_axil_wstrb_reg = {STRB_WIDTH{1'b0}}; // 源时钟域缓存的 W 字节使能。
reg                   s_axil_wvalid_reg = 1'b0; // 源时钟域锁存的 W 有效（表示请求待处理）。
reg [1:0]             s_axil_bresp_reg = 2'b00; // 源时钟域返回给从侧的 B 响应寄存器。
reg                   s_axil_bvalid_reg = 1'b0; // 源时钟域返回给从侧的 B 有效状态。

reg [ADDR_WIDTH-1:0]  m_axil_awaddr_reg = {ADDR_WIDTH{1'b0}}; // 目标时钟域驱动到主接口的 AW 地址。
reg [2:0]             m_axil_awprot_reg = 3'd0; // 目标时钟域驱动到主接口的 AW 保护属性。
reg                   m_axil_awvalid_reg = 1'b0; // 目标时钟域 AW 有效状态。
reg [DATA_WIDTH-1:0]  m_axil_wdata_reg = {DATA_WIDTH{1'b0}}; // 目标时钟域驱动到主接口的 W 数据。
reg [STRB_WIDTH-1:0]  m_axil_wstrb_reg = {STRB_WIDTH{1'b0}}; // 目标时钟域驱动到主接口的 W 字节使能。
reg                   m_axil_wvalid_reg = 1'b0; // 目标时钟域 W 有效状态。
reg [1:0]             m_axil_bresp_reg = 2'b00; // 目标时钟域捕获的 B 响应，用于返回源时钟域。
reg                   m_axil_bvalid_reg = 1'b1; // 目标时钟域本地“响应已捕获”指示（配合低有效就绪风格）。

assign s_axil_awready = !s_axil_awvalid_reg && !s_axil_bvalid_reg;
assign s_axil_wready = !s_axil_wvalid_reg && !s_axil_bvalid_reg;
assign s_axil_bresp = s_axil_bresp_reg;
assign s_axil_bvalid = s_axil_bvalid_reg;

assign m_axil_awaddr = m_axil_awaddr_reg;
assign m_axil_awprot = m_axil_awprot_reg;
assign m_axil_awvalid = m_axil_awvalid_reg;
assign m_axil_wdata = m_axil_wdata_reg;
assign m_axil_wstrb = m_axil_wstrb_reg;
assign m_axil_wvalid = m_axil_wvalid_reg;
assign m_axil_bready = !m_axil_bvalid_reg;

// 源时钟域（从侧）
always @(posedge s_clk) begin
    s_axil_bvalid_reg <= s_axil_bvalid_reg && !s_axil_bready;

    if (!s_axil_awvalid_reg && !s_axil_bvalid_reg) begin
        s_axil_awaddr_reg <= s_axil_awaddr;
        s_axil_awprot_reg <= s_axil_awprot;
        s_axil_awvalid_reg <= s_axil_awvalid;
    end

    if (!s_axil_wvalid_reg && !s_axil_bvalid_reg) begin
        s_axil_wdata_reg <= s_axil_wdata;
        s_axil_wstrb_reg <= s_axil_wstrb;
        s_axil_wvalid_reg <= s_axil_wvalid;
    end

    case (s_state_reg)
        2'd0: begin
            if (s_axil_awvalid_reg && s_axil_wvalid_reg) begin
                s_state_reg <= 2'd1;
                s_flag_reg <= 1'b1;
            end
        end
        2'd1: begin
            if (m_flag_sync_reg_2) begin
                s_state_reg <= 2'd2;
                s_flag_reg <= 1'b0;
                s_axil_bresp_reg <= m_axil_bresp_reg;
                s_axil_bvalid_reg <= 1'b1;
            end
        end
        2'd2: begin
            if (!m_flag_sync_reg_2) begin
                s_state_reg <= 2'd0;
                s_axil_awvalid_reg <= 1'b0;
                s_axil_wvalid_reg <= 1'b0;
            end
        end
    endcase

    if (s_rst) begin
        s_state_reg <= 2'd0;
        s_flag_reg <= 1'b0;
        s_axil_awvalid_reg <= 1'b0;
        s_axil_wvalid_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
    end
end

// 跨域同步链
always @(posedge s_clk) begin
    m_flag_sync_reg_1 <= m_flag_reg;
    m_flag_sync_reg_2 <= m_flag_sync_reg_1;
end

always @(posedge m_clk) begin
    s_flag_sync_reg_1 <= s_flag_reg;
    s_flag_sync_reg_2 <= s_flag_sync_reg_1;
end

// 目标时钟域（主侧）
always @(posedge m_clk) begin
    m_axil_awvalid_reg <= m_axil_awvalid_reg && !m_axil_awready;
    m_axil_wvalid_reg <= m_axil_wvalid_reg && !m_axil_wready;

    if (!m_axil_bvalid_reg) begin
        m_axil_bresp_reg <= m_axil_bresp;
        m_axil_bvalid_reg <= m_axil_bvalid;
    end

    case (m_state_reg)
        2'd0: begin
            if (s_flag_sync_reg_2) begin
                m_state_reg <= 2'd1;
                m_axil_awaddr_reg <= s_axil_awaddr_reg;
                m_axil_awprot_reg <= s_axil_awprot_reg;
                m_axil_awvalid_reg <= 1'b1;
                m_axil_wdata_reg <= s_axil_wdata_reg;
                m_axil_wstrb_reg <= s_axil_wstrb_reg;
                m_axil_wvalid_reg <= 1'b1;
                m_axil_bvalid_reg <= 1'b0;
            end
        end
        2'd1: begin
            if (m_axil_bvalid_reg) begin
                m_flag_reg <= 1'b1;
                m_state_reg <= 2'd2;
            end
        end
        2'd2: begin
            if (!s_flag_sync_reg_2) begin
                m_state_reg <= 2'd0;
                m_flag_reg <= 1'b0;
            end
        end
    endcase

    if (m_rst) begin
        m_state_reg <= 2'd0;
        m_flag_reg <= 1'b0;
        m_axil_awvalid_reg <= 1'b0;
        m_axil_wvalid_reg <= 1'b0;
        m_axil_bvalid_reg <= 1'b1;
    end
end

endmodule

`resetall
