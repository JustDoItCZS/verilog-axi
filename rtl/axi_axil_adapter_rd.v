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
 * AXI4 到 AXI4-Lite 适配器（读通道）
 *
 * 模块目录
 * 1) 接收 AXI 读突发请求。
 * 2) 根据位宽转换模式发起一条或多条 AXI-Lite 读事务。
 * 3) 对数据进行重组/拆分，并返回带正确 ID/last/resp 的 AXI R 数据拍。
 */
module axi_axil_adapter_rd #
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
    input  wire                        clk, // 读适配器时钟。
    input  wire                        rst, // 读转换 FSM 与通道状态同步复位。

    /*
     * AXI 从接口
     */
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_arid, // AXI 从端 AR ID（读地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]       s_axi_araddr, // AXI 从端 AR 地址。
    input  wire [7:0]                  s_axi_arlen, // AXI 从端 AR 突发长度。
    input  wire [2:0]                  s_axi_arsize, // AXI 从端 AR 突发尺寸。
    input  wire [1:0]                  s_axi_arburst, // AXI 从端 AR 突发类型。
    input  wire                        s_axi_arlock, // AXI 从端 AR 锁属性。
    input  wire [3:0]                  s_axi_arcache, // AXI 从端 AR cache 属性。
    input  wire [2:0]                  s_axi_arprot, // AXI 从端 AR 保护属性。
    input  wire                        s_axi_arvalid, // AXI 从端 ARVALID（读地址有效）。
    output wire                        s_axi_arready, // AXI 从端 ARREADY（读地址就绪）。
    output wire [AXI_ID_WIDTH-1:0]     s_axi_rid, // AXI 从端 R ID（读数据通道标识）。
    output wire [AXI_DATA_WIDTH-1:0]   s_axi_rdata, // AXI 从端 R 数据。
    output wire [1:0]                  s_axi_rresp, // AXI 从端 R 响应码。
    output wire                        s_axi_rlast, // AXI 从端 RLAST。
    output wire                        s_axi_rvalid, // AXI 从端 RVALID（读数据有效）。
    input  wire                        s_axi_rready, // AXI 从端 RREADY（读数据就绪）。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]       m_axil_araddr, // AXI-Lite 主端 AR 地址。
    output wire [2:0]                  m_axil_arprot, // AXI-Lite 主端 AR 保护属性。
    output wire                        m_axil_arvalid, // AXI-Lite 主端 ARVALID。
    input  wire                        m_axil_arready, // AXI-Lite 主端 ARREADY。
    input  wire [AXIL_DATA_WIDTH-1:0]  m_axil_rdata, // AXI-Lite 主端 R 数据。
    input  wire [1:0]                  m_axil_rresp, // AXI-Lite 主端 R 响应码。
    input  wire                        m_axil_rvalid, // AXI-Lite 主端 RVALID。
    output wire                        m_axil_rready // AXI-Lite 主端 RREADY。
);

parameter AXI_ADDR_BIT_OFFSET = $clog2(AXI_STRB_WIDTH);
parameter AXIL_ADDR_BIT_OFFSET = $clog2(AXIL_STRB_WIDTH);
parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXIL_WORD_WIDTH = AXIL_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH;
parameter AXIL_WORD_SIZE = AXIL_DATA_WIDTH/AXIL_WORD_WIDTH;
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
parameter AXIL_BURST_SIZE = $clog2(AXIL_STRB_WIDTH);

// 输出总线更宽
parameter EXPAND = AXIL_STRB_WIDTH > AXI_STRB_WIDTH;
parameter DATA_WIDTH = EXPAND ? AXIL_DATA_WIDTH : AXI_DATA_WIDTH;
parameter STRB_WIDTH = EXPAND ? AXIL_STRB_WIDTH : AXI_STRB_WIDTH;
// 宽总线中所需分段数量
parameter SEGMENT_COUNT = EXPAND ? (AXIL_STRB_WIDTH / AXI_STRB_WIDTH) : (AXI_STRB_WIDTH / AXIL_STRB_WIDTH);
// 每个分段的数据位宽与 keep 位宽
parameter SEGMENT_DATA_WIDTH = DATA_WIDTH / SEGMENT_COUNT;
parameter SEGMENT_STRB_WIDTH = STRB_WIDTH / SEGMENT_COUNT;

// 总线位宽断言检查
initial begin
    if (AXI_WORD_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI slave interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXIL_WORD_SIZE * AXIL_STRB_WIDTH != AXIL_DATA_WIDTH) begin
        $error("Error: AXI lite master interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXI_WORD_SIZE != AXIL_WORD_SIZE) begin
        $error("Error: word size mismatch (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_WORD_WIDTH) != AXI_WORD_WIDTH) begin
        $error("Error: AXI slave interface word width must be even power of two (instance %m)");
        $finish;
    end

    if (2**$clog2(AXIL_WORD_WIDTH) != AXIL_WORD_WIDTH) begin
        $error("Error: AXI lite master interface word width must be even power of two (instance %m)");
        $finish;
    end
end

localparam [1:0]
    STATE_IDLE = 2'd0, // 等待 AXI AR 请求。
    STATE_DATA = 2'd1, // 将 AXI-Lite 读结果流式输出为 AXI R 数据拍（直接/简单场景）。
    STATE_DATA_READ = 2'd2, // 先读取更宽分段，再进行拆分输出。
    STATE_DATA_SPLIT = 2'd3; // 向 AXI 侧输出已缓存的拆分分段。

reg [1:0] state_reg = STATE_IDLE, state_next; // 读转换 FSM 状态。

reg [AXI_ID_WIDTH-1:0] id_reg = {AXI_ID_WIDTH{1'b0}}, id_next; // 当前突发锁存的 AXI 读 ID。
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next; // 突发遍历过程中的当前地址指针。
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next; // 缓存的合并/拆分数据字。
reg [1:0] resp_reg = 2'd0, resp_next; // 组成 AXI 数据拍时累计的响应状态。
reg [7:0] burst_reg = 8'd0, burst_next; // 剩余 AXI 侧数据拍计数。
reg [2:0] burst_size_reg = 3'd0, burst_size_next; // AXI 侧突发尺寸（log2 bytes/beat）。
reg [7:0] master_burst_reg = 8'd0, master_burst_next; // 当前分块剩余 AXI-Lite 子事务计数。
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next; // AXI-Lite 侧地址步进所用有效突发尺寸。

reg s_axi_arready_reg = 1'b0, s_axi_arready_next; // AXI ARREADY 状态。
reg [AXI_ID_WIDTH-1:0] s_axi_rid_reg = {AXI_ID_WIDTH{1'b0}}, s_axi_rid_next; // AXI R ID 输出寄存器。
reg [AXI_DATA_WIDTH-1:0] s_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}}, s_axi_rdata_next; // AXI R 数据输出寄存器。
reg [1:0] s_axi_rresp_reg = 2'd0, s_axi_rresp_next; // AXI R 响应输出寄存器。
reg s_axi_rlast_reg = 1'b0, s_axi_rlast_next; // AXI RLAST 输出寄存器。
reg s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // AXI RVALID 状态。

reg [ADDR_WIDTH-1:0] m_axil_araddr_reg = {ADDR_WIDTH{1'b0}}, m_axil_araddr_next; // AXI-Lite ARADDR 输出寄存器。
reg [2:0] m_axil_arprot_reg = 3'd0, m_axil_arprot_next; // AXI-Lite ARPROT 输出寄存器。
reg m_axil_arvalid_reg = 1'b0, m_axil_arvalid_next; // AXI-Lite ARVALID 状态。
reg m_axil_rready_reg = 1'b0, m_axil_rready_next; // AXI-Lite RREADY 状态。

assign s_axi_arready = s_axi_arready_reg;
assign s_axi_rid = s_axi_rid_reg;
assign s_axi_rdata = s_axi_rdata_reg;
assign s_axi_rresp = s_axi_rresp_reg;
assign s_axi_rlast = s_axi_rlast_reg;
assign s_axi_rvalid = s_axi_rvalid_reg;

assign m_axil_araddr = m_axil_araddr_reg;
assign m_axil_arprot = m_axil_arprot_reg;
assign m_axil_arvalid = m_axil_arvalid_reg;
assign m_axil_rready = m_axil_rready_reg;

always @* begin
    state_next = STATE_IDLE;

    id_next = id_reg;
    addr_next = addr_reg;
    data_next = data_reg;
    resp_next = resp_reg;
    burst_next = burst_reg;
    burst_size_next = burst_size_reg;
    master_burst_next = master_burst_reg;
    master_burst_size_next = master_burst_size_reg;

    s_axi_arready_next = 1'b0;
    s_axi_rid_next = s_axi_rid_reg;
    s_axi_rdata_next = s_axi_rdata_reg;
    s_axi_rresp_next = s_axi_rresp_reg;
    s_axi_rlast_next = s_axi_rlast_reg;
    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
    m_axil_araddr_next = m_axil_araddr_reg;
    m_axil_arprot_next = m_axil_arprot_reg;
    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
    m_axil_rready_next = 1'b0;

    if (SEGMENT_COUNT == 1) begin
        // 主端与从端位宽相同：直接传输，不做拆分/合并
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axil_arvalid;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axil_araddr_next = s_axi_araddr;
                    addr_next = s_axi_araddr;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输读数据
                m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    s_axi_rid_next = id_reg;
                    s_axi_rdata_next = m_axil_rdata;
                    s_axi_rresp_next = m_axil_rresp;
                    s_axi_rlast_next = 1'b0;
                    s_axi_rvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        // 最后一拍数据，返回空闲态
                        m_axil_rready_next = 1'b0;
                        s_axi_rlast_next = 1'b1;
                        s_axi_arready_next = !m_axil_arvalid;
                        state_next = STATE_IDLE;
                    end else begin
                        // 发起新的 AXI-Lite 读事务
                        m_axil_araddr_next = addr_next;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
        endcase
    end else if (EXPAND) begin
        // 主端输出更宽：执行读数据拆分
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axil_arvalid;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axil_araddr_next = s_axi_araddr;
                    addr_next = s_axi_araddr;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    if (CONVERT_BURST && s_axi_arcache[1] && (CONVERT_NARROW_BURST || s_axi_arsize == AXI_BURST_SIZE)) begin
                        // 拆分读取
                        // 需开启 CONVERT_BURST 且 arcache[1] 置位
                        master_burst_size_next = AXIL_BURST_SIZE;
                        state_next = STATE_DATA_READ;
                    end else begin
                        // 输出窄突发
                        master_burst_size_next = s_axi_arsize;
                        state_next = STATE_DATA;
                    end
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    s_axi_rid_next = id_reg;
                    s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET] * AXI_DATA_WIDTH);
                    s_axi_rresp_next = m_axil_rresp;
                    s_axi_rlast_next = 1'b0;
                    s_axi_rvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        // 最后一拍数据，返回空闲态
                        m_axil_rready_next = 1'b0;
                        s_axi_rlast_next = 1'b1;
                        s_axi_arready_next = !m_axil_arvalid;
                        state_next = STATE_IDLE;
                    end else begin
                        // 发起新的 AXI-Lite 读事务
                        m_axil_araddr_next = addr_next;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_READ: begin
                m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    s_axi_rid_next = id_reg;
                    data_next = m_axil_rdata;
                    resp_next = m_axil_rresp;
                    s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET] * AXI_DATA_WIDTH);
                    s_axi_rresp_next = m_axil_rresp;
                    s_axi_rlast_next = 1'b0;
                    s_axi_rvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        m_axil_rready_next = 1'b0;
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rlast_next = 1'b1;
                        state_next = STATE_IDLE;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        // 发起新的 AXI-Lite 读事务
                        m_axil_araddr_next = addr_next;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                        state_next = STATE_DATA_READ;
                    end else begin
                        m_axil_rready_next = 1'b0;
                        state_next = STATE_DATA_SPLIT;
                    end
                end else begin
                    state_next = STATE_DATA_READ;
                end
            end
            STATE_DATA_SPLIT: begin
                m_axil_rready_next = 1'b0;

                if (s_axi_rready || !s_axi_rvalid) begin
                    s_axi_rid_next = id_reg;
                    s_axi_rdata_next = data_reg >> (addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET] * AXI_DATA_WIDTH);
                    s_axi_rresp_next = resp_reg;
                    s_axi_rlast_next = 1'b0;
                    s_axi_rvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rlast_next = 1'b1;
                        state_next = STATE_IDLE;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        // 发起新的 AXI-Lite 读事务
                        m_axil_araddr_next = addr_next;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                        state_next = STATE_DATA_READ;
                    end else begin
                        state_next = STATE_DATA_SPLIT;
                    end
                end else begin
                    state_next = STATE_DATA_SPLIT;
                end
            end
        endcase
    end else begin
        // 主端输出更窄：执行读数据合并，并可能拆分突发
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axil_arvalid;

                resp_next = 2'd0;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axil_araddr_next = s_axi_araddr;
                    addr_next = s_axi_araddr;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    if (s_axi_arsize > AXIL_BURST_SIZE) begin
                        // 需要调整突发尺寸
                        if (s_axi_arlen >> (8+AXIL_BURST_SIZE-s_axi_arsize) != 0) begin
                            // 将突发长度限制到最大值
                            master_burst_next = (8'd255 << (s_axi_arsize-AXIL_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> AXIL_BURST_SIZE);
                        end else begin
                            master_burst_next = (s_axi_arlen << (s_axi_arsize-AXIL_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> AXIL_BURST_SIZE);
                        end
                        master_burst_size_next = AXIL_BURST_SIZE;
                    end else begin
                        // 直接透传足够窄的突发
                        master_burst_next = s_axi_arlen;
                        master_burst_size_next = s_axi_arsize;
                    end
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;

                if (m_axil_rready && m_axil_rvalid) begin
                    data_next[addr_reg[AXI_ADDR_BIT_OFFSET-1:AXIL_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH +: SEGMENT_DATA_WIDTH] = m_axil_rdata;
                    if (m_axil_rresp) begin
                        resp_next = m_axil_rresp;
                    end
                    s_axi_rid_next = id_reg;
                    s_axi_rdata_next = data_next;
                    s_axi_rresp_next = resp_next;
                    s_axi_rlast_next = 1'b0;
                    s_axi_rvalid_next = 1'b0;
                    master_burst_next = master_burst_reg - 1;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    m_axil_araddr_next = addr_next;
                    if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                        data_next = {DATA_WIDTH{1'b0}};
                        burst_next = burst_reg - 1;
                        s_axi_rvalid_next = 1'b1;
                    end
                    if (master_burst_reg == 0) begin
                        if (burst_next >> (8+AXIL_BURST_SIZE-burst_size_reg) != 0) begin
                            // 将突发长度限制到最大值
                            master_burst_next = 8'd255;
                        end else begin
                            master_burst_next = (burst_next << (burst_size_reg-AXIL_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> AXIL_BURST_SIZE);
                        end

                        if (burst_reg == 0) begin
                            m_axil_rready_next = 1'b0;
                            s_axi_rlast_next = 1'b1;
                            s_axi_rvalid_next = 1'b1;
                            s_axi_arready_next = !m_axil_arvalid;
                            state_next = STATE_IDLE;
                        end else begin
                            m_axil_arvalid_next = 1'b1;
                            m_axil_rready_next = 1'b0;
                            state_next = STATE_DATA;
                        end
                    end else begin
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
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

    id_reg <= id_next;
    addr_reg <= addr_next;
    data_reg <= data_next;
    resp_reg <= resp_next;
    burst_reg <= burst_next;
    burst_size_reg <= burst_size_next;
    master_burst_reg <= master_burst_next;
    master_burst_size_reg <= master_burst_size_next;

    s_axi_arready_reg <= s_axi_arready_next;
    s_axi_rid_reg <= s_axi_rid_next;
    s_axi_rdata_reg <= s_axi_rdata_next;
    s_axi_rresp_reg <= s_axi_rresp_next;
    s_axi_rlast_reg <= s_axi_rlast_next;
    s_axi_rvalid_reg <= s_axi_rvalid_next;

    m_axil_araddr_reg <= m_axil_araddr_next;
    m_axil_arprot_reg <= m_axil_arprot_next;
    m_axil_arvalid_reg <= m_axil_arvalid_next;
    m_axil_rready_reg <= m_axil_rready_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axi_arready_reg <= 1'b0;
        s_axi_rvalid_reg <= 1'b0;

        m_axil_arvalid_reg <= 1'b0;
        m_axil_rready_reg <= 1'b0;
    end
end

endmodule

`resetall
