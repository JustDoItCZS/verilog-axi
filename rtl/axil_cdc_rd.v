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
 * AXI4-Lite 跨时钟域模块（读通道）
 *
 * 模块目录
 * 1) 源时钟域（s_clk）：接收从侧 AR，并向从侧返回 R。
 * 2) 目标时钟域（m_clk）：向主侧发起 AR，并接收主侧返回 R。
 * 3) 跨域握手：s_flag_reg 发起请求，m_flag_reg 表示完成应答。
 * 4) 握手期间通过两侧影子寄存器保持载荷，实现安全跨域。
 */
module axil_cdc_rd #
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
    input  wire [ADDR_WIDTH-1:0]  s_axil_araddr,   // 源时钟域 AR 地址。
    input  wire [2:0]             s_axil_arprot,   // 源时钟域 AR 保护属性。
    input  wire                   s_axil_arvalid,  // 源时钟域 AR 有效。
    output wire                   s_axil_arready,  // 源时钟域 AR 就绪（本地请求/响应缓存空闲时拉高）。
    output wire [DATA_WIDTH-1:0]  s_axil_rdata,    // 源时钟域 R 数据（由目标时钟域返回）。
    output wire [1:0]             s_axil_rresp,    // 源时钟域 R 响应（由目标时钟域返回）。
    output wire                   s_axil_rvalid,   // 源时钟域 R 有效。
    input  wire                   s_axil_rready,   // 源时钟域 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    input  wire                   m_clk,           // 目标时钟域时钟（主侧）。
    input  wire                   m_rst,           // 目标时钟域同步复位。
    output wire [ADDR_WIDTH-1:0]  m_axil_araddr,   // 目标时钟域 AR 地址。
    output wire [2:0]             m_axil_arprot,   // 目标时钟域 AR 保护属性。
    output wire                   m_axil_arvalid,  // 目标时钟域 AR 有效。
    input  wire                   m_axil_arready,  // 目标时钟域 AR 就绪（来自下游目标）。
    input  wire [DATA_WIDTH-1:0]  m_axil_rdata,    // 目标时钟域 R 数据（来自下游目标）。
    input  wire [1:0]             m_axil_rresp,    // 目标时钟域 R 响应（来自下游目标）。
    input  wire                   m_axil_rvalid,   // 目标时钟域 R 有效（来自下游目标）。
    output wire                   m_axil_rready    // 目标时钟域 R 就绪（等待捕获响应时拉高）。
);

reg [1:0] s_state_reg = 2'd0; // 源时钟域状态机：空闲 -> 等待应答 -> 等待应答标志清零。
reg s_flag_reg = 1'b0; // 源时钟域请求标志，读事务挂起期间置位。
(* srl_style = "register" *)
reg s_flag_sync_reg_1 = 1'b0; // s_flag_reg 进入目标时钟域的一级同步寄存器。
(* srl_style = "register" *)
reg s_flag_sync_reg_2 = 1'b0; // s_flag_reg 的二级同步寄存器（目标时钟域稳定采样）。

reg [1:0] m_state_reg = 2'd0; // 目标时钟域状态机：空闲 -> 等待 R 捕获 -> 等待请求标志清零。
reg m_flag_reg = 1'b0; // 目标时钟域应答标志，读响应捕获后置位。
(* srl_style = "register" *)
reg m_flag_sync_reg_1 = 1'b0; // m_flag_reg 返回源时钟域的一级同步寄存器。
(* srl_style = "register" *)
reg m_flag_sync_reg_2 = 1'b0; // m_flag_reg 的二级同步寄存器（源时钟域稳定采样）。

reg [ADDR_WIDTH-1:0]  s_axil_araddr_reg = {ADDR_WIDTH{1'b0}}; // 源时钟域缓存的 AR 地址，请求跨域期间保持不变。
reg [2:0]             s_axil_arprot_reg = 3'd0; // 源时钟域缓存的 AR 保护属性。
reg                   s_axil_arvalid_reg = 1'b0; // 源时钟域锁存的 AR 有效（表示请求待处理）。
reg [DATA_WIDTH-1:0]  s_axil_rdata_reg = {DATA_WIDTH{1'b0}}; // 源时钟域返回给从侧的 R 数据寄存器。
reg [1:0]             s_axil_rresp_reg = 2'b00; // 源时钟域返回给从侧的 R 响应寄存器。
reg                   s_axil_rvalid_reg = 1'b0; // 源时钟域返回给从侧的 R 有效状态。

reg [ADDR_WIDTH-1:0]  m_axil_araddr_reg = {ADDR_WIDTH{1'b0}}; // 目标时钟域驱动到主接口的 AR 地址。
reg [2:0]             m_axil_arprot_reg = 3'd0; // 目标时钟域驱动到主接口的 AR 保护属性。
reg                   m_axil_arvalid_reg = 1'b0; // 目标时钟域 AR 有效状态。
reg [DATA_WIDTH-1:0]  m_axil_rdata_reg = {DATA_WIDTH{1'b0}}; // 目标时钟域捕获的 R 数据，用于返回源时钟域。
reg [1:0]             m_axil_rresp_reg = 2'b00; // 目标时钟域捕获的 R 响应，用于返回源时钟域。
reg                   m_axil_rvalid_reg = 1'b1; // 目标时钟域本地“响应已捕获”指示（配合低有效就绪风格）。

assign s_axil_arready = !s_axil_arvalid_reg && !s_axil_rvalid_reg;
assign s_axil_rdata = s_axil_rdata_reg;
assign s_axil_rresp = s_axil_rresp_reg;
assign s_axil_rvalid = s_axil_rvalid_reg;

assign m_axil_araddr = m_axil_araddr_reg;
assign m_axil_arprot = m_axil_arprot_reg;
assign m_axil_arvalid = m_axil_arvalid_reg;
assign m_axil_rready = !m_axil_rvalid_reg;

// 源时钟域（从侧）
always @(posedge s_clk) begin
    s_axil_rvalid_reg <= s_axil_rvalid_reg && !s_axil_rready;

    if (!s_axil_arvalid_reg && !s_axil_rvalid_reg) begin
        s_axil_araddr_reg <= s_axil_araddr;
        s_axil_arprot_reg <= s_axil_arprot;
        s_axil_arvalid_reg <= s_axil_arvalid;
    end

    case (s_state_reg)
        2'd0: begin
            if (s_axil_arvalid_reg) begin
                s_state_reg <= 2'd1;
                s_flag_reg <= 1'b1;
            end
        end
        2'd1: begin
            if (m_flag_sync_reg_2) begin
                s_state_reg <= 2'd2;
                s_flag_reg <= 1'b0;
                s_axil_rdata_reg <= m_axil_rdata_reg;
                s_axil_rresp_reg <= m_axil_rresp_reg;
                s_axil_rvalid_reg <= 1'b1;
            end
        end
        2'd2: begin
            if (!m_flag_sync_reg_2) begin
                s_state_reg <= 2'd0;
                s_axil_arvalid_reg <= 1'b0;
            end
        end
    endcase

    if (s_rst) begin
        s_state_reg <= 2'd0;
        s_flag_reg <= 1'b0;
        s_axil_arvalid_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
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
    m_axil_arvalid_reg <= m_axil_arvalid_reg && !m_axil_arready;

    if (!m_axil_rvalid_reg) begin
        m_axil_rdata_reg <= m_axil_rdata;
        m_axil_rresp_reg <= m_axil_rresp;
        m_axil_rvalid_reg <= m_axil_rvalid;
    end

    case (m_state_reg)
        2'd0: begin
            if (s_flag_sync_reg_2) begin
                m_state_reg <= 2'd1;
                m_axil_araddr_reg <= s_axil_araddr_reg;
                m_axil_arprot_reg <= s_axil_arprot_reg;
                m_axil_arvalid_reg <= 1'b1;
                m_axil_rvalid_reg <= 1'b0;
            end
        end
        2'd1: begin
            if (m_axil_rvalid_reg) begin
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
        m_axil_arvalid_reg <= 1'b0;
        m_axil_rvalid_reg <= 1'b1;
    end
end

endmodule

`resetall
