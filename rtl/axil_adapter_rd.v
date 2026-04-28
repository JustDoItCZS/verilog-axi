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
 * AXI4-Lite 位宽适配器（读通道）
 *
 * 模块目录
 * 1) 接收从侧 AXI-Lite 读地址并返回读数据/响应。
 * 2) 在主从位宽不一致时对读数据进行重组。
 * 3) 主侧位宽较窄时：发起多次主侧读并拼装成一次从侧响应。
 * 4) 主侧位宽相同或更宽时：执行单次直接转发。
 */
module axil_adapter_rd #
(
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 输入（从侧）接口数据位宽
    parameter S_DATA_WIDTH = 32,
    // 输入（从侧）接口 WSTRB 位宽（按字节）
    parameter S_STRB_WIDTH = (S_DATA_WIDTH/8),
    // 输出（主侧）接口数据位宽
    parameter M_DATA_WIDTH = 32,
    // 输出（主侧）接口 WSTRB 位宽（按字节）
    parameter M_STRB_WIDTH = (M_DATA_WIDTH/8)
)
(
    input  wire                     clk,            // 适配状态机与读通道寄存器时钟。
    input  wire                     rst,            // 适配器状态同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]    s_axil_araddr,  // 从侧 AR 地址（位宽转换前）。
    input  wire [2:0]               s_axil_arprot,  // 从侧 AR 保护属性。
    input  wire                     s_axil_arvalid, // 从侧 AR 有效。
    output wire                     s_axil_arready, // 从侧 AR 就绪（由适配器状态机生成）。
    output wire [S_DATA_WIDTH-1:0]  s_axil_rdata,   // 从侧 R 数据（按从侧位宽组装）。
    output wire [1:0]               s_axil_rresp,   // 从侧 R 响应（合并后）。
    output wire                     s_axil_rvalid,  // 从侧 R 有效。
    input  wire                     s_axil_rready,  // 从侧 R 就绪。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]    m_axil_araddr,  // 主侧 AR 地址（适配/分段后）。
    output wire [2:0]               m_axil_arprot,  // 主侧 AR 保护属性。
    output wire                     m_axil_arvalid, // 主侧 AR 有效。
    input  wire                     m_axil_arready, // 主侧 AR 就绪（来自下游目标）。
    input  wire [M_DATA_WIDTH-1:0]  m_axil_rdata,   // 主侧 R 数据（来自下游目标）。
    input  wire [1:0]               m_axil_rresp,   // 主侧 R 响应（来自下游目标）。
    input  wire                     m_axil_rvalid,  // 主侧 R 有效（来自下游目标）。
    output wire                     m_axil_rready   // 主侧 R 就绪（由适配器状态机驱动）。
);

parameter S_ADDR_BIT_OFFSET = $clog2(S_STRB_WIDTH);
parameter M_ADDR_BIT_OFFSET = $clog2(M_STRB_WIDTH);
parameter S_WORD_WIDTH = S_STRB_WIDTH;
parameter M_WORD_WIDTH = M_STRB_WIDTH;
parameter S_WORD_SIZE = S_DATA_WIDTH/S_WORD_WIDTH;
parameter M_WORD_SIZE = M_DATA_WIDTH/M_WORD_WIDTH;
parameter S_ADDR_MASK = {ADDR_WIDTH{1'b1}} << S_ADDR_BIT_OFFSET;
parameter M_ADDR_MASK = {ADDR_WIDTH{1'b1}} << M_ADDR_BIT_OFFSET;

// 主侧数据总线更宽
parameter EXPAND = M_STRB_WIDTH > S_STRB_WIDTH;
parameter DATA_WIDTH = EXPAND ? M_DATA_WIDTH : S_DATA_WIDTH;
parameter STRB_WIDTH = EXPAND ? M_STRB_WIDTH : S_STRB_WIDTH;
// 宽总线下所需分段数
parameter SEGMENT_COUNT = EXPAND ? (M_STRB_WIDTH / S_STRB_WIDTH) : (S_STRB_WIDTH / M_STRB_WIDTH);
parameter SEGMENT_COUNT_WIDTH = SEGMENT_COUNT == 1 ? 1 : $clog2(SEGMENT_COUNT);
// 每段数据位宽与字节使能位宽
parameter SEGMENT_DATA_WIDTH = DATA_WIDTH / SEGMENT_COUNT;
parameter SEGMENT_STRB_WIDTH = STRB_WIDTH / SEGMENT_COUNT;

// 总线位宽约束检查
initial begin
    if (S_WORD_SIZE * S_STRB_WIDTH != S_DATA_WIDTH) begin
        $error("Error: AXI slave interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (M_WORD_SIZE * M_STRB_WIDTH != M_DATA_WIDTH) begin
        $error("Error: AXI master interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (S_WORD_SIZE != M_WORD_SIZE) begin
        $error("Error: word size mismatch (instance %m)");
        $finish;
    end

    if (2**$clog2(S_WORD_WIDTH) != S_WORD_WIDTH) begin
        $error("Error: AXI slave interface word width must be even power of two (instance %m)");
        $finish;
    end

    if (2**$clog2(M_WORD_WIDTH) != M_WORD_WIDTH) begin
        $error("Error: AXI master interface word width must be even power of two (instance %m)");
        $finish;
    end
end

localparam [0:0]
    STATE_IDLE = 1'd0,
    STATE_DATA = 1'd1;

reg [0:0] state_reg = STATE_IDLE, state_next; // 读适配状态机当前状态与下一状态。

reg [SEGMENT_COUNT_WIDTH-1:0] current_segment_reg = 0, current_segment_next; // 主侧位宽较窄时的当前分段索引。

reg s_axil_arready_reg = 1'b0, s_axil_arready_next; // 从侧 AR 就绪当前状态与下一状态。
reg [S_DATA_WIDTH-1:0] s_axil_rdata_reg = {S_DATA_WIDTH{1'b0}}, s_axil_rdata_next; // 从侧 R 数据寄存器当前值与下一值。
reg [1:0] s_axil_rresp_reg = 2'd0, s_axil_rresp_next; // 从侧 R 响应寄存器当前值与下一值。
reg s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next; // 从侧 R 有效当前状态与下一状态。

reg [ADDR_WIDTH-1:0] m_axil_araddr_reg = {ADDR_WIDTH{1'b0}}, m_axil_araddr_next; // 主侧 AR 地址寄存器当前值与下一值。
reg [2:0] m_axil_arprot_reg = 3'd0, m_axil_arprot_next; // 主侧 AR 保护属性寄存器当前值与下一值。
reg m_axil_arvalid_reg = 1'b0, m_axil_arvalid_next; // 主侧 AR 有效当前状态与下一状态。
reg m_axil_rready_reg = 1'b0, m_axil_rready_next; // 主侧 R 就绪当前状态与下一状态。

assign s_axil_arready = s_axil_arready_reg;
assign s_axil_rdata = s_axil_rdata_reg;
assign s_axil_rresp = s_axil_rresp_reg;
assign s_axil_rvalid = s_axil_rvalid_reg;

assign m_axil_araddr = m_axil_araddr_reg;
assign m_axil_arprot = m_axil_arprot_reg;
assign m_axil_arvalid = m_axil_arvalid_reg;
assign m_axil_rready = m_axil_rready_reg;

always @* begin
    state_next = STATE_IDLE;

    current_segment_next = current_segment_reg;

    s_axil_arready_next = 1'b0;
    s_axil_rdata_next = s_axil_rdata_reg;
    s_axil_rresp_next = s_axil_rresp_reg;
    s_axil_rvalid_next = s_axil_rvalid_reg && !s_axil_rready;
    m_axil_araddr_next = m_axil_araddr_reg;
    m_axil_arprot_next = m_axil_arprot_reg;
    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
    m_axil_rready_next = 1'b0;

    if (SEGMENT_COUNT == 1 || EXPAND) begin
        // 主侧位宽相同或更宽：单次直接传输
        case (state_reg)
            STATE_IDLE: begin
                s_axil_arready_next = !m_axil_arvalid;

                if (s_axil_arready && s_axil_arvalid) begin
                    s_axil_arready_next = 1'b0;
                    m_axil_araddr_next = s_axil_araddr;
                    m_axil_arprot_next = s_axil_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = !m_axil_rvalid;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axil_rready_next = !s_axil_rvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    m_axil_rready_next = 1'b0;
                    if (M_WORD_WIDTH == S_WORD_WIDTH) begin
                        s_axil_rdata_next = m_axil_rdata;
                    end else begin
                        s_axil_rdata_next = m_axil_rdata >> (m_axil_araddr_reg[M_ADDR_BIT_OFFSET - 1:S_ADDR_BIT_OFFSET] * S_DATA_WIDTH);
                    end
                    s_axil_rresp_next = m_axil_rresp;
                    s_axil_rvalid_next = 1'b1;
                    s_axil_arready_next = !m_axil_arvalid;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_DATA;
                end
            end
        endcase
    end else begin
        // 主侧位宽更窄：可能需要多拍拼装
        case (state_reg)
            STATE_IDLE: begin
                s_axil_arready_next = !m_axil_arvalid;

                current_segment_next = s_axil_araddr >> M_ADDR_BIT_OFFSET;
                s_axil_rresp_next = 2'd0;

                if (s_axil_arready && s_axil_arvalid) begin
                    s_axil_arready_next = 1'b0;
                    m_axil_araddr_next = s_axil_araddr;
                    m_axil_arprot_next = s_axil_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = !m_axil_rvalid;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axil_rready_next = !s_axil_rvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    m_axil_rready_next = 1'b0;
                    m_axil_araddr_next = (m_axil_araddr_reg & M_ADDR_MASK) + SEGMENT_STRB_WIDTH;
                    s_axil_rdata_next[current_segment_reg*SEGMENT_DATA_WIDTH +: SEGMENT_DATA_WIDTH] = m_axil_rdata;
                    current_segment_next = current_segment_reg + 1;
                    if (m_axil_rresp) begin
                        s_axil_rresp_next = m_axil_rresp;
                    end
                    if (current_segment_reg == SEGMENT_COUNT-1) begin
                        s_axil_rvalid_next = 1'b1;
                        s_axil_arready_next = !m_axil_arvalid;
                        state_next = STATE_IDLE;
                    end else begin
                        m_axil_arvalid_next = 1'b1;
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
        endcase
    end
end

always @(posedge clk) begin
    state_reg <= state_next;

    current_segment_reg <= current_segment_next;

    s_axil_arready_reg <= s_axil_arready_next;
    s_axil_rdata_reg <= s_axil_rdata_next;
    s_axil_rresp_reg <= s_axil_rresp_next;
    s_axil_rvalid_reg <= s_axil_rvalid_next;

    m_axil_araddr_reg <= m_axil_araddr_next;
    m_axil_arprot_reg <= m_axil_arprot_next;
    m_axil_arvalid_reg <= m_axil_arvalid_next;
    m_axil_rready_reg <= m_axil_rready_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axil_arready_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;

        m_axil_arvalid_reg <= 1'b0;
        m_axil_rready_reg <= 1'b0;
    end
end

endmodule

`resetall
