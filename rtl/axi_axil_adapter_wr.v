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
 * AXI4 到 AXI4-Lite 适配器（写通道）
 *
 * 模块目录
 * 1) 接收 AXI 写突发（AW/W/B）。
 * 2) 将突发写事务转换为一条或多条 AXI-Lite 写事务。
 * 3) 在位宽转换过程中重组/拆分写掩码与写数据，并返回 AXI B 响应。
 */
module axi_axil_adapter_wr #
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
    input  wire                        clk, // 写适配器时钟。
    input  wire                        rst, // 写转换 FSM 与通道状态同步复位。

    /*
     * AXI 从接口
     */
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_awid, // AXI 从端 AW ID（写地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]       s_axi_awaddr, // AXI 从端 AW 地址。
    input  wire [7:0]                  s_axi_awlen, // AXI 从端 AW 突发长度。
    input  wire [2:0]                  s_axi_awsize, // AXI 从端 AW 突发尺寸。
    input  wire [1:0]                  s_axi_awburst, // AXI 从端 AW 突发类型。
    input  wire                        s_axi_awlock, // AXI 从端 AW 锁属性。
    input  wire [3:0]                  s_axi_awcache, // AXI 从端 AW cache 属性。
    input  wire [2:0]                  s_axi_awprot, // AXI 从端 AW 保护属性。
    input  wire                        s_axi_awvalid, // AXI 从端 AWVALID（写地址有效）。
    output wire                        s_axi_awready, // AXI 从端 AWREADY（写地址就绪）。
    input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata, // AXI 从端 W 数据。
    input  wire [AXI_STRB_WIDTH-1:0]   s_axi_wstrb, // AXI 从端 W 字节使能。
    input  wire                        s_axi_wlast, // AXI 从端 WLAST（写突发最后一拍）。
    input  wire                        s_axi_wvalid, // AXI 从端 WVALID（写数据有效）。
    output wire                        s_axi_wready, // AXI 从端 WREADY（写数据就绪）。
    output wire [AXI_ID_WIDTH-1:0]     s_axi_bid, // AXI 从端 B ID（写响应标识）。
    output wire [1:0]                  s_axi_bresp, // AXI 从端 B 响应码。
    output wire                        s_axi_bvalid, // AXI 从端 BVALID（写响应有效）。
    input  wire                        s_axi_bready, // AXI 从端 BREADY（写响应就绪）。

    /*
     * AXI-Lite 主接口
     */
    output wire [ADDR_WIDTH-1:0]       m_axil_awaddr, // AXI-Lite 主端 AW 地址。
    output wire [2:0]                  m_axil_awprot, // AXI-Lite 主端 AW 保护属性。
    output wire                        m_axil_awvalid, // AXI-Lite 主端 AWVALID。
    input  wire                        m_axil_awready, // AXI-Lite 主端 AWREADY。
    output wire [AXIL_DATA_WIDTH-1:0]  m_axil_wdata, // AXI-Lite 主端 W 数据。
    output wire [AXIL_STRB_WIDTH-1:0]  m_axil_wstrb, // AXI-Lite 主端 W 字节使能。
    output wire                        m_axil_wvalid, // AXI-Lite 主端 WVALID。
    input  wire                        m_axil_wready, // AXI-Lite 主端 WREADY。
    input  wire [1:0]                  m_axil_bresp, // AXI-Lite 主端 B 响应码。
    input  wire                        m_axil_bvalid, // AXI-Lite 主端 BVALID。
    output wire                        m_axil_bready // AXI-Lite 主端 BREADY。
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
    STATE_IDLE = 2'd0, // 等待 AXI AW 请求。
    STATE_DATA = 2'd1, // 消费 AXI W 数据拍并发出 AXI-Lite 写事务。
    STATE_DATA_2 = 2'd2, // 为窄到宽转换输出额外拆分段。
    STATE_RESP = 2'd3; // 等待 AXI-Lite B 响应并映射回 AXI B。

reg [1:0] state_reg = STATE_IDLE, state_next; // 写转换 FSM 状态。

reg [AXI_ID_WIDTH-1:0] id_reg = {AXI_ID_WIDTH{1'b0}}, id_next; // 当前突发锁存的 AXI 写 ID。
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next; // 突发遍历过程中的当前地址指针。
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next; // 缓存的合并写数据字。
reg [STRB_WIDTH-1:0] strb_reg = {STRB_WIDTH{1'b0}}, strb_next; // 缓存的合并写掩码字。
reg [7:0] burst_reg = 8'd0, burst_next; // 剩余 AXI 侧数据拍计数。
reg [2:0] burst_size_reg = 3'd0, burst_size_next; // AXI 侧突发尺寸（log2 bytes/beat）。
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next; // AXI-Lite 侧地址步进所用有效突发尺寸。
reg burst_active_reg = 1'b0, burst_active_next; // 指示 AXI 突发是否仍有未完成数据拍。
reg convert_burst_reg = 1'b0, convert_burst_next; // 条件满足时使能突发重打包路径。
reg first_transfer_reg = 1'b0, first_transfer_next; // 标记响应累计时的首个 AXI-Lite 写响应。
reg last_segment_reg = 1'b0, last_segment_next; // 窄化场景下当前 AXI 数据拍的最后拆分段标记。

reg s_axi_awready_reg = 1'b0, s_axi_awready_next; // AXI AWREADY 状态。
reg s_axi_wready_reg = 1'b0, s_axi_wready_next; // AXI WREADY 状态。
reg [AXI_ID_WIDTH-1:0] s_axi_bid_reg = {AXI_ID_WIDTH{1'b0}}, s_axi_bid_next; // AXI BID 输出寄存器。
reg [1:0] s_axi_bresp_reg = 2'd0, s_axi_bresp_next; // AXI BRESP 输出寄存器。
reg s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // AXI BVALID 状态。

reg [ADDR_WIDTH-1:0] m_axil_awaddr_reg = {ADDR_WIDTH{1'b0}}, m_axil_awaddr_next; // AXI-Lite AWADDR 输出寄存器。
reg [2:0] m_axil_awprot_reg = 3'd0, m_axil_awprot_next; // AXI-Lite AWPROT 输出寄存器。
reg m_axil_awvalid_reg = 1'b0, m_axil_awvalid_next; // AXI-Lite AWVALID 状态。
reg [AXIL_DATA_WIDTH-1:0] m_axil_wdata_reg = {AXIL_DATA_WIDTH{1'b0}}, m_axil_wdata_next; // AXI-Lite WDATA 输出寄存器。
reg [AXIL_STRB_WIDTH-1:0] m_axil_wstrb_reg = {AXIL_STRB_WIDTH{1'b0}}, m_axil_wstrb_next; // AXI-Lite WSTRB 输出寄存器。
reg m_axil_wvalid_reg = 1'b0, m_axil_wvalid_next; // AXI-Lite WVALID 状态。
reg m_axil_bready_reg = 1'b0, m_axil_bready_next; // AXI-Lite BREADY 状态。

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = s_axi_wready_reg;
assign s_axi_bid = s_axi_bid_reg;
assign s_axi_bresp = s_axi_bresp_reg;
assign s_axi_bvalid = s_axi_bvalid_reg;

assign m_axil_awaddr = m_axil_awaddr_reg;
//assign m_axil_awlen = m_axil_awlen_reg; // 调试保留：若需要可导出 awlen
//assign m_axil_awsize = m_axil_awsize_reg; // 调试保留：若需要可导出 awsize
//assign m_axil_awburst = m_axil_awburst_reg; // 调试保留：若需要可导出 awburst
assign m_axil_awprot = m_axil_awprot_reg;
assign m_axil_awvalid = m_axil_awvalid_reg;
assign m_axil_wdata = m_axil_wdata_reg;
assign m_axil_wstrb = m_axil_wstrb_reg;
assign m_axil_wvalid = m_axil_wvalid_reg;
assign m_axil_bready = m_axil_bready_reg;

integer i; // 窄突发重打包逻辑中的字节 lane 循环变量。

always @* begin
    state_next = STATE_IDLE;

    id_next = id_reg;
    addr_next = addr_reg;
    data_next = data_reg;
    strb_next = strb_reg;
    burst_next = burst_reg;
    burst_size_next = burst_size_reg;
    master_burst_size_next = master_burst_size_reg;
    burst_active_next = burst_active_reg;
    convert_burst_next = convert_burst_reg;
    first_transfer_next = first_transfer_reg;
    last_segment_next = last_segment_reg;

    s_axi_awready_next = 1'b0;
    s_axi_wready_next = 1'b0;
    s_axi_bid_next = s_axi_bid_reg;
    s_axi_bresp_next = s_axi_bresp_reg;
    s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
    m_axil_awaddr_next = m_axil_awaddr_reg;
    m_axil_awprot_next = m_axil_awprot_reg;
    m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
    m_axil_wdata_next = m_axil_wdata_reg;
    m_axil_wstrb_next = m_axil_wstrb_reg;
    m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
    m_axil_bready_next = 1'b0;

    if (SEGMENT_COUNT == 1) begin
        // 主端与从端位宽相同：直接传输，不做拆分/合并
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axil_awvalid;
                first_transfer_next = 1'b1;

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axil_awaddr_next = s_axi_awaddr;
                    addr_next = s_axi_awaddr;
                    burst_next = s_axi_awlen;
                    burst_size_next = s_axi_awsize;
                    burst_active_next = 1'b1;
                    m_axil_awprot_next = s_axi_awprot;
                    m_axil_awvalid_next = 1'b1;
                    s_axi_wready_next = !m_axil_wvalid;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输写数据
                s_axi_wready_next = !m_axil_wvalid;

                if (s_axi_wready && s_axi_wvalid) begin
                    m_axil_wdata_next = s_axi_wdata;
                    m_axil_wstrb_next = s_axi_wstrb;
                    m_axil_wvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    burst_active_next = burst_reg != 0;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    s_axi_wready_next = 1'b0;
                    m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
                    state_next = STATE_RESP;
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_RESP: begin
                // 响应态：传输写响应
                m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;

                if (m_axil_bready && m_axil_bvalid) begin
                    m_axil_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    first_transfer_next = 1'b0;
                    if (first_transfer_reg || m_axil_bresp != 0) begin
                        s_axi_bresp_next = m_axil_bresp;
                    end
                    if (burst_active_reg) begin
                        // 从端突发仍在进行，发起新的 AXI-Lite 写事务
                        m_axil_awaddr_next = addr_reg;
                        m_axil_awvalid_next = 1'b1;
                        s_axi_wready_next = !m_axil_wvalid;
                        state_next = STATE_DATA;
                    end else begin
                        // 从端突发结束，返回空闲态
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = !m_axil_awvalid;
                        state_next = STATE_IDLE;
                    end
                end else begin
                    state_next = STATE_RESP;
                end
            end
        endcase
    end else if (EXPAND) begin
        // 主端输出更宽：执行写数据合并
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axil_awvalid;

                first_transfer_next = 1'b1;

                data_next = {DATA_WIDTH{1'b0}};
                strb_next = {STRB_WIDTH{1'b0}};

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axil_awaddr_next = s_axi_awaddr;
                    addr_next = s_axi_awaddr;
                    burst_next = s_axi_awlen;
                    burst_size_next = s_axi_awsize;
                    if (CONVERT_BURST && s_axi_awcache[1] && (CONVERT_NARROW_BURST || s_axi_awsize == AXI_BURST_SIZE)) begin
                        // 合并写数据
                        // 需开启 CONVERT_BURST 且 awcache[1] 置位
                        convert_burst_next = 1'b1;
                        master_burst_size_next = AXIL_BURST_SIZE;
                        state_next = STATE_DATA_2;
                    end else begin
                        // 输出窄突发
                        convert_burst_next = 1'b0;
                        master_burst_size_next = s_axi_awsize;
                        state_next = STATE_DATA;
                    end
                    m_axil_awprot_next = s_axi_awprot;
                    m_axil_awvalid_next = 1'b1;
                    s_axi_wready_next = !m_axil_wvalid;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输写数据
                s_axi_wready_next = !m_axil_wvalid || m_axil_wready;

                if (s_axi_wready && s_axi_wvalid) begin
                    m_axil_wdata_next = {(AXIL_WORD_WIDTH/AXI_WORD_WIDTH){s_axi_wdata}};
                    m_axil_wstrb_next = s_axi_wstrb << (addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET] * AXI_STRB_WIDTH);
                    m_axil_wvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    burst_active_next = burst_reg != 0;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    s_axi_wready_next = 1'b0;
                    m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
                    state_next = STATE_RESP;
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_2: begin
                s_axi_wready_next = !m_axil_wvalid;

                if (s_axi_wready && s_axi_wvalid) begin
                    if (CONVERT_NARROW_BURST) begin
                        for (i = 0; i < AXI_WORD_WIDTH; i = i + 1) begin
                            if (s_axi_wstrb[i]) begin
                                data_next[addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH+i*AXIL_WORD_SIZE +: AXIL_WORD_SIZE] = s_axi_wdata[i*AXIL_WORD_SIZE +: AXIL_WORD_SIZE];
                                strb_next[addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET]*SEGMENT_STRB_WIDTH+i] = 1'b1;
                            end
                        end
                    end else begin
                        data_next[addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH +: SEGMENT_DATA_WIDTH] = s_axi_wdata;
                        strb_next[addr_reg[AXIL_ADDR_BIT_OFFSET-1:AXI_ADDR_BIT_OFFSET]*SEGMENT_STRB_WIDTH +: SEGMENT_STRB_WIDTH] = s_axi_wstrb;
                    end
                    m_axil_wdata_next = data_next;
                    m_axil_wstrb_next = strb_next;
                    burst_next = burst_reg - 1;
                    burst_active_next = burst_reg != 0;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0 || addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        data_next = {DATA_WIDTH{1'b0}};
                        strb_next = {STRB_WIDTH{1'b0}};
                        m_axil_wvalid_next = 1'b1;
                        s_axi_wready_next = 1'b0;
                        m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
                        state_next = STATE_RESP;
                    end else begin
                        state_next = STATE_DATA_2;
                    end
                end else begin
                    state_next = STATE_DATA_2;
                end
            end
            STATE_RESP: begin
                // 响应态：传输写响应
                m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;

                if (m_axil_bready && m_axil_bvalid) begin
                    m_axil_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    first_transfer_next = 1'b0;
                    if (first_transfer_reg || m_axil_bresp != 0) begin
                        s_axi_bresp_next = m_axil_bresp;
                    end
                    if (burst_active_reg) begin
                        // 从端突发仍在进行，发起新的 AXI-Lite 写事务
                        m_axil_awaddr_next = addr_reg;
                        m_axil_awvalid_next = 1'b1;
                        s_axi_wready_next = !m_axil_wvalid || m_axil_wready;
                        if (convert_burst_reg) begin
                            state_next = STATE_DATA_2;
                        end else begin
                            state_next = STATE_DATA;
                        end
                    end else begin
                        // 从端突发结束，返回空闲态
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = !m_axil_awvalid;
                        state_next = STATE_IDLE;
                    end
                end else begin
                    state_next = STATE_RESP;
                end
            end
        endcase
    end else begin
        // 主端输出更窄：执行写数据拆分，并可能拆分突发
        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axil_awvalid;

                first_transfer_next = 1'b1;

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axil_awaddr_next = s_axi_awaddr;
                    addr_next = s_axi_awaddr;
                    burst_next = s_axi_awlen;
                    burst_size_next = s_axi_awsize;
                    burst_active_next = 1'b1;
                    if (s_axi_awsize > AXIL_BURST_SIZE) begin
                        // 需要调整突发尺寸
                        master_burst_size_next = AXIL_BURST_SIZE;
                    end else begin
                        // 直接透传足够窄的突发
                        master_burst_size_next = s_axi_awsize;
                    end
                    m_axil_awprot_next = s_axi_awprot;
                    m_axil_awvalid_next = 1'b1;
                    s_axi_wready_next = !m_axil_wvalid;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                s_axi_wready_next = !m_axil_wvalid;

                if (s_axi_wready && s_axi_wvalid) begin
                    data_next = s_axi_wdata;
                    strb_next = s_axi_wstrb;
                    m_axil_wdata_next = s_axi_wdata >> (addr_reg[AXI_ADDR_BIT_OFFSET-1:AXIL_ADDR_BIT_OFFSET] * AXIL_DATA_WIDTH);
                    m_axil_wstrb_next = s_axi_wstrb >> (addr_reg[AXI_ADDR_BIT_OFFSET-1:AXIL_ADDR_BIT_OFFSET] * AXIL_STRB_WIDTH);
                    m_axil_wvalid_next = 1'b1;
                    burst_next = burst_reg - 1;
                    burst_active_next = burst_reg != 0;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    last_segment_next = addr_next[burst_size_reg] != addr_reg[burst_size_reg];
                    s_axi_wready_next = 1'b0;
                    m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
                    state_next = STATE_RESP;
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_2: begin
                s_axi_wready_next = 1'b0;

                if (!m_axil_wvalid || m_axil_wready) begin
                    m_axil_wdata_next = data_reg >> (addr_reg[AXI_ADDR_BIT_OFFSET-1:AXIL_ADDR_BIT_OFFSET] * AXIL_DATA_WIDTH);
                    m_axil_wstrb_next = strb_reg >> (addr_reg[AXI_ADDR_BIT_OFFSET-1:AXIL_ADDR_BIT_OFFSET] * AXIL_STRB_WIDTH);
                    m_axil_wvalid_next = 1'b1;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    last_segment_next = addr_next[burst_size_reg] != addr_reg[burst_size_reg];
                    s_axi_wready_next = 1'b0;
                    m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
                    state_next = STATE_RESP;
                end else begin
                    state_next = STATE_DATA_2;
                end
            end
            STATE_RESP: begin
                // 响应态：传输写响应
                m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;

                if (m_axil_bready && m_axil_bvalid) begin
                    first_transfer_next = 1'b0;
                    m_axil_awaddr_next = addr_reg;
                    m_axil_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    if (first_transfer_reg || m_axil_bresp != 0) begin
                        s_axi_bresp_next = m_axil_bresp;
                    end
                    if (burst_active_reg || !last_segment_reg) begin
                        // 从端突发仍在进行，启动新子突发
                        m_axil_awvalid_next = 1'b1;
                        if (last_segment_reg) begin
                            s_axi_wready_next = !m_axil_wvalid;
                            state_next = STATE_DATA;
                        end else begin
                            s_axi_wready_next = 1'b0;
                            state_next = STATE_DATA_2;
                        end
                    end else begin
                        // 从端突发结束，返回空闲态
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = !m_axil_awvalid;
                        state_next = STATE_IDLE;
                    end
                end else begin
                    state_next = STATE_RESP;
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
    strb_reg <= strb_next;
    burst_reg <= burst_next;
    burst_size_reg <= burst_size_next;
    master_burst_size_reg <= master_burst_size_next;
    burst_active_reg <= burst_active_next;
    convert_burst_reg <= convert_burst_next;
    first_transfer_reg <= first_transfer_next;
    last_segment_reg <= last_segment_next;

    s_axi_awready_reg <= s_axi_awready_next;
    s_axi_wready_reg <= s_axi_wready_next;
    s_axi_bid_reg <= s_axi_bid_next;
    s_axi_bresp_reg <= s_axi_bresp_next;
    s_axi_bvalid_reg <= s_axi_bvalid_next;

    m_axil_awaddr_reg <= m_axil_awaddr_next;
    m_axil_awprot_reg <= m_axil_awprot_next;
    m_axil_awvalid_reg <= m_axil_awvalid_next;
    m_axil_wdata_reg <= m_axil_wdata_next;
    m_axil_wstrb_reg <= m_axil_wstrb_next;
    m_axil_wvalid_reg <= m_axil_wvalid_next;
    m_axil_bready_reg <= m_axil_bready_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axi_awready_reg <= 1'b0;
        s_axi_wready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;

        m_axil_awvalid_reg <= 1'b0;
        m_axil_wvalid_reg <= 1'b0;
        m_axil_bready_reg <= 1'b0;
    end
end

endmodule

`resetall
