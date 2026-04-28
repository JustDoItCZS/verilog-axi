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
 * AXI4 位宽适配器（写通道）
 *
 * 模块目录
 * 1) 在从端接收 AXI 写突发（AW/W/B）。
 * 2) 将突发形态/位宽转换为主端 AXI 写事务。
 * 3) 在位宽转换中对写数据/写掩码进行合并或拆分。
 * 4) 最终 W 通道输出经过内部 skid buffer 级。
 */
module axi_adapter_wr #
(
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 输入侧（从接口）数据总线位宽
    parameter S_DATA_WIDTH = 32,
    // 输入侧（从接口）WSTRB 位宽（按字节 lane）
    parameter S_STRB_WIDTH = (S_DATA_WIDTH/8),
    // 输出侧（主接口）数据总线位宽
    parameter M_DATA_WIDTH = 32,
    // 输出侧（主接口）WSTRB 位宽（按字节 lane）
    parameter M_STRB_WIDTH = (M_DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 awuser 信号
    parameter AWUSER_ENABLE = 0,
    // awuser 信号位宽
    parameter AWUSER_WIDTH = 1,
    // 是否透传 wuser 信号
    parameter WUSER_ENABLE = 0,
    // wuser 信号位宽
    parameter WUSER_WIDTH = 1,
    // 是否透传 buser 信号
    parameter BUSER_ENABLE = 0,
    // buser 信号位宽
    parameter BUSER_WIDTH = 1,
    // 向更宽总线适配时，尽可能重打包为满宽突发，而不是透传窄突发
    parameter CONVERT_BURST = 1,
    // 向更宽总线适配时，对所有突发执行重打包，而不是透传窄突发
    parameter CONVERT_NARROW_BURST = 0,
    // 是否在适配器中透传 ID
    parameter FORWARD_ID = 0
)
(
    input  wire                     clk, // 写位宽适配器时钟。
    input  wire                     rst, // 转换 FSM 与输出数据通路寄存器同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // 从端 AW ID（写地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // 从端 AW 地址。
    input  wire [7:0]               s_axi_awlen, // 从端 AW 突发长度。
    input  wire [2:0]               s_axi_awsize, // 从端 AW 突发尺寸。
    input  wire [1:0]               s_axi_awburst, // 从端 AW 突发类型。
    input  wire                     s_axi_awlock, // 从端 AW 锁属性。
    input  wire [3:0]               s_axi_awcache, // 从端 AW cache 属性。
    input  wire [2:0]               s_axi_awprot, // 从端 AW 保护属性。
    input  wire [3:0]               s_axi_awqos, // 从端 AW QoS（服务质量字段）。
    input  wire [3:0]               s_axi_awregion, // 从端 AW region（区域属性字段）。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // 从端 AW 用户旁带。
    input  wire                     s_axi_awvalid, // 从端 AWVALID（写地址有效）。
    output wire                     s_axi_awready, // 从端 AWREADY（写地址就绪）。
    input  wire [S_DATA_WIDTH-1:0]  s_axi_wdata, // 从端 W 数据（源位宽）。
    input  wire [S_STRB_WIDTH-1:0]  s_axi_wstrb, // 从端 W 字节使能（源位宽）。
    input  wire                     s_axi_wlast, // 从端 WLAST（写突发最后一拍）。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // 从端 W 用户旁带。
    input  wire                     s_axi_wvalid, // 从端 WVALID（写数据有效）。
    output wire                     s_axi_wready, // 从端 WREADY（写数据就绪）。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 从端 B ID（写响应标识）。
    output wire [1:0]               s_axi_bresp, // 从端 B 响应码。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 从端 B 用户旁带。
    output wire                     s_axi_bvalid, // 从端 BVALID（写响应有效）。
    input  wire                     s_axi_bready, // 从端 BREADY（写响应就绪）。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_awid, // 主端 AW ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr, // 主端 AW 地址。
    output wire [7:0]               m_axi_awlen, // 主端 AW 突发长度。
    output wire [2:0]               m_axi_awsize, // 主端 AW 突发尺寸。
    output wire [1:0]               m_axi_awburst, // 主端 AW 突发类型。
    output wire                     m_axi_awlock, // 主端 AW 锁属性。
    output wire [3:0]               m_axi_awcache, // 主端 AW cache 属性。
    output wire [2:0]               m_axi_awprot, // 主端 AW 保护属性。
    output wire [3:0]               m_axi_awqos, // 主端 AW QoS。
    output wire [3:0]               m_axi_awregion, // 主端 AW region。
    output wire [AWUSER_WIDTH-1:0]  m_axi_awuser, // 主端 AW 用户旁带。
    output wire                     m_axi_awvalid, // 主端 AWVALID。
    input  wire                     m_axi_awready, // 主端 AWREADY。
    output wire [M_DATA_WIDTH-1:0]  m_axi_wdata, // 主端 W 数据（目标位宽）。
    output wire [M_STRB_WIDTH-1:0]  m_axi_wstrb, // 主端 W 字节使能（目标位宽）。
    output wire                     m_axi_wlast, // 主端 WLAST。
    output wire [WUSER_WIDTH-1:0]   m_axi_wuser, // 主端 W 用户旁带。
    output wire                     m_axi_wvalid, // 主端 WVALID。
    input  wire                     m_axi_wready, // 主端 WREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 主端 B ID。
    input  wire [1:0]               m_axi_bresp, // 主端 B 响应码。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 主端 B 用户旁带。
    input  wire                     m_axi_bvalid, // 主端 BVALID。
    output wire                     m_axi_bready // 主端 BREADY。
);

parameter S_ADDR_BIT_OFFSET = $clog2(S_STRB_WIDTH);
parameter M_ADDR_BIT_OFFSET = $clog2(M_STRB_WIDTH);
parameter S_WORD_WIDTH = S_STRB_WIDTH;
parameter M_WORD_WIDTH = M_STRB_WIDTH;
parameter S_WORD_SIZE = S_DATA_WIDTH/S_WORD_WIDTH;
parameter M_WORD_SIZE = M_DATA_WIDTH/M_WORD_WIDTH;
parameter S_BURST_SIZE = $clog2(S_STRB_WIDTH);
parameter M_BURST_SIZE = $clog2(M_STRB_WIDTH);

// 输出总线更宽
parameter EXPAND = M_STRB_WIDTH > S_STRB_WIDTH;
parameter DATA_WIDTH = EXPAND ? M_DATA_WIDTH : S_DATA_WIDTH;
parameter STRB_WIDTH = EXPAND ? M_STRB_WIDTH : S_STRB_WIDTH;
// 宽总线中所需分段数量
parameter SEGMENT_COUNT = EXPAND ? (M_STRB_WIDTH / S_STRB_WIDTH) : (S_STRB_WIDTH / M_STRB_WIDTH);
// 每个分段的数据位宽与 keep 位宽
parameter SEGMENT_DATA_WIDTH = DATA_WIDTH / SEGMENT_COUNT;
parameter SEGMENT_STRB_WIDTH = STRB_WIDTH / SEGMENT_COUNT;

// 总线位宽断言检查
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

localparam [1:0]
    STATE_IDLE = 2'd0, // 等待从端 AW 请求。
    STATE_DATA = 2'd1, // 消费从端 W 数据拍并输出第一段主端写数据。
    STATE_DATA_2 = 2'd2, // 为窄到宽转换输出剩余拆分段。
    STATE_RESP = 2'd3; // 等待主端 B 响应并映射回从端。

reg [1:0] state_reg = STATE_IDLE, state_next; // 主写转换 FSM 状态。

reg [ID_WIDTH-1:0] id_reg = {ID_WIDTH{1'b0}}, id_next; // 锁存的写事务 ID。
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next; // 当前地址指针。
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next; // 缓存的合并/拆分写数据。
reg [STRB_WIDTH-1:0] strb_reg = {STRB_WIDTH{1'b0}}, strb_next; // 缓存的合并/拆分写掩码。
reg [WUSER_WIDTH-1:0] wuser_reg = {WUSER_WIDTH{1'b0}}, wuser_next; // 转换后数据拍对应的缓存 WUSER。
reg [7:0] burst_reg = 8'd0, burst_next; // 剩余从端数据拍计数。
reg [2:0] burst_size_reg = 3'd0, burst_size_next; // 从端突发尺寸（log2 bytes/beat）。
reg [7:0] master_burst_reg = 8'd0, master_burst_next; // 当前子突发剩余主端数据拍计数。
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next; // 主端有效突发尺寸。
reg burst_active_reg = 1'b0, burst_active_next; // 指示从端突发是否仍有未完成数据拍。
reg first_transfer_reg = 1'b0, first_transfer_next; // 标记 BRESP 累计时的首个主端响应。

reg s_axi_awready_reg = 1'b0, s_axi_awready_next; // 从端 AWREADY 状态。
reg s_axi_wready_reg = 1'b0, s_axi_wready_next; // 从端 WREADY 状态。
reg [ID_WIDTH-1:0] s_axi_bid_reg = {ID_WIDTH{1'b0}}, s_axi_bid_next; // 从端 BID 输出寄存器。
reg [1:0] s_axi_bresp_reg = 2'd0, s_axi_bresp_next; // 从端 BRESP 输出寄存器。
reg [BUSER_WIDTH-1:0] s_axi_buser_reg = {BUSER_WIDTH{1'b0}}, s_axi_buser_next; // 从端 BUSER 输出寄存器。
reg s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // 从端 BVALID 状态。

reg [ID_WIDTH-1:0] m_axi_awid_reg = {ID_WIDTH{1'b0}}, m_axi_awid_next; // 主端 AWID 输出寄存器。
reg [ADDR_WIDTH-1:0] m_axi_awaddr_reg = {ADDR_WIDTH{1'b0}}, m_axi_awaddr_next; // 主端 AWADDR 输出寄存器。
reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next; // 主端 AWLEN 输出寄存器。
reg [2:0] m_axi_awsize_reg = 3'd0, m_axi_awsize_next; // 主端 AWSIZE 输出寄存器。
reg [1:0] m_axi_awburst_reg = 2'd0, m_axi_awburst_next; // 主端 AWBURST 输出寄存器。
reg m_axi_awlock_reg = 1'b0, m_axi_awlock_next; // 主端 AWLOCK 输出寄存器。
reg [3:0] m_axi_awcache_reg = 4'd0, m_axi_awcache_next; // 主端 AWCACHE 输出寄存器。
reg [2:0] m_axi_awprot_reg = 3'd0, m_axi_awprot_next; // 主端 AWPROT 输出寄存器。
reg [3:0] m_axi_awqos_reg = 4'd0, m_axi_awqos_next; // 主端 AWQOS 输出寄存器。
reg [3:0] m_axi_awregion_reg = 4'd0, m_axi_awregion_next; // 主端 AWREGION 输出寄存器。
reg [AWUSER_WIDTH-1:0] m_axi_awuser_reg = {AWUSER_WIDTH{1'b0}}, m_axi_awuser_next; // 主端 AWUSER 输出寄存器。
reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next; // 主端 AWVALID 状态。
reg m_axi_bready_reg = 1'b0, m_axi_bready_next; // 主端 BREADY 状态。

// 内部数据通路
reg  [M_DATA_WIDTH-1:0] m_axi_wdata_int; // 来自转换 FSM 的内部预缓冲 WDATA。
reg  [M_STRB_WIDTH-1:0] m_axi_wstrb_int; // 来自转换 FSM 的内部预缓冲 WSTRB。
reg                     m_axi_wlast_int; // 来自转换 FSM 的内部预缓冲 WLAST。
reg  [WUSER_WIDTH-1:0]  m_axi_wuser_int; // 来自转换 FSM 的内部预缓冲 WUSER。
reg                     m_axi_wvalid_int; // 来自转换 FSM 的内部预缓冲 WVALID。
reg                     m_axi_wready_int_reg = 1'b0; // 内部到输出 W skid 级的 ready 寄存器。
wire                    m_axi_wready_int_early; // 内部到输出 W skid 级的前瞻 ready。

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = s_axi_wready_reg;
assign s_axi_bid = s_axi_bid_reg;
assign s_axi_bresp = s_axi_bresp_reg;
assign s_axi_buser = BUSER_ENABLE ? s_axi_buser_reg : {BUSER_WIDTH{1'b0}};
assign s_axi_bvalid = s_axi_bvalid_reg;

assign m_axi_awid = FORWARD_ID ? m_axi_awid_reg : {ID_WIDTH{1'b0}};
assign m_axi_awaddr = m_axi_awaddr_reg;
assign m_axi_awlen = m_axi_awlen_reg;
assign m_axi_awsize = m_axi_awsize_reg;
assign m_axi_awburst = m_axi_awburst_reg;
assign m_axi_awlock = m_axi_awlock_reg;
assign m_axi_awcache = m_axi_awcache_reg;
assign m_axi_awprot = m_axi_awprot_reg;
assign m_axi_awqos = m_axi_awqos_reg;
assign m_axi_awregion = m_axi_awregion_reg;
assign m_axi_awuser = AWUSER_ENABLE ? m_axi_awuser_reg : {AWUSER_WIDTH{1'b0}};
assign m_axi_awvalid = m_axi_awvalid_reg;
assign m_axi_bready = m_axi_bready_reg;

integer i; // 窄突发拆分/合并逻辑中的字节 lane 循环变量。

always @* begin
    state_next = STATE_IDLE;

    id_next = id_reg;
    addr_next = addr_reg;
    data_next = data_reg;
    strb_next = strb_reg;
    wuser_next = wuser_reg;
    burst_next = burst_reg;
    burst_size_next = burst_size_reg;
    master_burst_next = master_burst_reg;
    master_burst_size_next = master_burst_size_reg;
    burst_active_next = burst_active_reg;
    first_transfer_next = first_transfer_reg;

    s_axi_awready_next = 1'b0;
    s_axi_wready_next = 1'b0;
    s_axi_bid_next = s_axi_bid_reg;
    s_axi_bresp_next = s_axi_bresp_reg;
    s_axi_buser_next = s_axi_buser_reg;
    s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
    m_axi_awid_next = m_axi_awid_reg;
    m_axi_awaddr_next = m_axi_awaddr_reg;
    m_axi_awlen_next = m_axi_awlen_reg;
    m_axi_awsize_next = m_axi_awsize_reg;
    m_axi_awburst_next = m_axi_awburst_reg;
    m_axi_awlock_next = m_axi_awlock_reg;
    m_axi_awcache_next = m_axi_awcache_reg;
    m_axi_awprot_next = m_axi_awprot_reg;
    m_axi_awqos_next = m_axi_awqos_reg;
    m_axi_awregion_next = m_axi_awregion_reg;
    m_axi_awuser_next = m_axi_awuser_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;
    m_axi_bready_next = 1'b0;

    if (SEGMENT_COUNT == 1) begin
        // 主端与从端位宽相同：直接传输，不做拆分/合并
        m_axi_wdata_int = s_axi_wdata;
        m_axi_wstrb_int = s_axi_wstrb;
        m_axi_wlast_int = s_axi_wlast;
        m_axi_wuser_int = s_axi_wuser;
        m_axi_wvalid_int = 1'b0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axi_awvalid;

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axi_awid_next = s_axi_awid;
                    m_axi_awaddr_next = s_axi_awaddr;
                    m_axi_awlen_next = s_axi_awlen;
                    m_axi_awsize_next = s_axi_awsize;
                    m_axi_awburst_next = s_axi_awburst;
                    m_axi_awlock_next = s_axi_awlock;
                    m_axi_awcache_next = s_axi_awcache;
                    m_axi_awprot_next = s_axi_awprot;
                    m_axi_awqos_next = s_axi_awqos;
                    m_axi_awregion_next = s_axi_awregion;
                    m_axi_awuser_next = s_axi_awuser;
                    m_axi_awvalid_next = 1'b1;
                    s_axi_wready_next = m_axi_wready_int_early;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输写数据
                s_axi_wready_next = m_axi_wready_int_early;

                if (s_axi_wready && s_axi_wvalid) begin
                    m_axi_wdata_int = s_axi_wdata;
                    m_axi_wstrb_int = s_axi_wstrb;
                    m_axi_wlast_int = s_axi_wlast;
                    m_axi_wuser_int = s_axi_wuser;
                    m_axi_wvalid_int = 1'b1;
                    if (s_axi_wlast) begin
                        // 最后一拍写数据，等待响应
                        s_axi_wready_next = 1'b0;
                        m_axi_bready_next = !s_axi_bvalid;
                        state_next = STATE_RESP;
                    end else begin
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_RESP: begin
                // 响应态：传输写响应
                m_axi_bready_next = !s_axi_bvalid;

                if (m_axi_bready && m_axi_bvalid) begin
                    m_axi_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    s_axi_bresp_next = m_axi_bresp;
                    s_axi_buser_next = m_axi_buser;
                    s_axi_bvalid_next = 1'b1;
                    s_axi_awready_next = !m_axi_awvalid;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_RESP;
                end
            end
        endcase
    end else if (EXPAND) begin
        // 主端输出更宽：执行写数据合并
        m_axi_wdata_int = {(M_WORD_WIDTH/S_WORD_WIDTH){s_axi_wdata}};
        m_axi_wstrb_int = s_axi_wstrb;
        m_axi_wlast_int = s_axi_wlast;
        m_axi_wuser_int = s_axi_wuser;
        m_axi_wvalid_int = 1'b0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axi_awvalid;

                data_next = {DATA_WIDTH{1'b0}};
                strb_next = {STRB_WIDTH{1'b0}};

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axi_awid_next = s_axi_awid;
                    m_axi_awaddr_next = s_axi_awaddr;
                    addr_next = s_axi_awaddr;
                    burst_next = s_axi_awlen;
                    burst_size_next = s_axi_awsize;
                    if (CONVERT_BURST && s_axi_awcache[1] && (CONVERT_NARROW_BURST || s_axi_awsize == S_BURST_SIZE)) begin
                        // 合并写数据
                        // 需开启 CONVERT_BURST 且 awcache[1] 置位
                        master_burst_size_next = M_BURST_SIZE;
                        if (CONVERT_NARROW_BURST) begin
                            m_axi_awlen_next = (({{S_ADDR_BIT_OFFSET+1{1'b0}}, s_axi_awlen} << s_axi_awsize) + s_axi_awaddr[M_ADDR_BIT_OFFSET-1:0]) >> M_BURST_SIZE;
                        end else begin
                            m_axi_awlen_next = ({1'b0, s_axi_awlen} + s_axi_awaddr[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]) >> $clog2(SEGMENT_COUNT);
                        end
                        m_axi_awsize_next = M_BURST_SIZE;
                        state_next = STATE_DATA_2;
                    end else begin
                        // 输出窄突发
                        master_burst_size_next = s_axi_awsize;
                        m_axi_awlen_next = s_axi_awlen;
                        m_axi_awsize_next = s_axi_awsize;
                        state_next = STATE_DATA;
                    end
                    m_axi_awburst_next = s_axi_awburst;
                    m_axi_awlock_next = s_axi_awlock;
                    m_axi_awcache_next = s_axi_awcache;
                    m_axi_awprot_next = s_axi_awprot;
                    m_axi_awqos_next = s_axi_awqos;
                    m_axi_awregion_next = s_axi_awregion;
                    m_axi_awuser_next = s_axi_awuser;
                    m_axi_awvalid_next = 1'b1;
                    s_axi_wready_next = m_axi_wready_int_early;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输写数据
                s_axi_wready_next = m_axi_wready_int_early;

                if (s_axi_wready && s_axi_wvalid) begin
                    m_axi_wdata_int = {(M_WORD_WIDTH/S_WORD_WIDTH){s_axi_wdata}};
                    m_axi_wstrb_int = s_axi_wstrb << (addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET] * S_STRB_WIDTH);
                    m_axi_wlast_int = s_axi_wlast;
                    m_axi_wuser_int = s_axi_wuser;
                    m_axi_wvalid_int = 1'b1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (s_axi_wlast) begin
                        s_axi_wready_next = 1'b0;
                        m_axi_bready_next = !s_axi_bvalid;
                        state_next = STATE_RESP;
                    end else begin
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_2: begin
                s_axi_wready_next = m_axi_wready_int_early;

                if (s_axi_wready && s_axi_wvalid) begin
                    if (CONVERT_NARROW_BURST) begin
                        for (i = 0; i < S_WORD_WIDTH; i = i + 1) begin
                            if (s_axi_wstrb[i]) begin
                                data_next[addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH+i*M_WORD_SIZE +: M_WORD_SIZE] = s_axi_wdata[i*M_WORD_SIZE +: M_WORD_SIZE];
                                strb_next[addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]*SEGMENT_STRB_WIDTH+i] = 1'b1;
                            end
                        end
                    end else begin
                        data_next[addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH +: SEGMENT_DATA_WIDTH] = s_axi_wdata;
                        strb_next[addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]*SEGMENT_STRB_WIDTH +: SEGMENT_STRB_WIDTH] = s_axi_wstrb;
                    end
                    m_axi_wdata_int = data_next;
                    m_axi_wstrb_int = strb_next;
                    m_axi_wlast_int = s_axi_wlast;
                    m_axi_wuser_int = s_axi_wuser;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        data_next = {DATA_WIDTH{1'b0}};
                        strb_next = {STRB_WIDTH{1'b0}};
                        m_axi_wvalid_int = 1'b1;
                    end
                    if (burst_reg == 0) begin
                        m_axi_wvalid_int = 1'b1;
                        s_axi_wready_next = 1'b0;
                        m_axi_bready_next = !s_axi_bvalid;
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
                m_axi_bready_next = !s_axi_bvalid;

                if (m_axi_bready && m_axi_bvalid) begin
                    m_axi_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    s_axi_bresp_next = m_axi_bresp;
                    s_axi_buser_next = m_axi_buser;
                    s_axi_bvalid_next = 1'b1;
                    s_axi_awready_next = !m_axi_awvalid;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_RESP;
                end
            end
        endcase
    end else begin
        // 主端输出更窄：执行写数据拆分，并可能拆分突发
        m_axi_wdata_int = data_reg;
        m_axi_wstrb_int = strb_reg;
        m_axi_wlast_int = 1'b0;
        m_axi_wuser_int = wuser_reg;
        m_axi_wvalid_int = 1'b0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_awready_next = !m_axi_awvalid;

                first_transfer_next = 1'b1;

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;
                    id_next = s_axi_awid;
                    m_axi_awid_next = s_axi_awid;
                    m_axi_awaddr_next = s_axi_awaddr;
                    addr_next = s_axi_awaddr;
                    burst_next = s_axi_awlen;
                    burst_size_next = s_axi_awsize;
                    burst_active_next = 1'b1;
                    if (s_axi_awsize > M_BURST_SIZE) begin
                        // 需要调整突发尺寸
                        if (s_axi_awlen >> (8+M_BURST_SIZE-s_axi_awsize) != 0) begin
                            // 将突发长度限制到最大值
                            master_burst_next = (8'd255 << (s_axi_awsize-M_BURST_SIZE)) | ((~s_axi_awaddr & (8'hff >> (8-s_axi_awsize))) >> M_BURST_SIZE);
                        end else begin
                            master_burst_next = (s_axi_awlen << (s_axi_awsize-M_BURST_SIZE)) | ((~s_axi_awaddr & (8'hff >> (8-s_axi_awsize))) >> M_BURST_SIZE);
                        end
                        master_burst_size_next = M_BURST_SIZE;
                        m_axi_awlen_next = master_burst_next;
                        m_axi_awsize_next = master_burst_size_next;
                    end else begin
                        // 直接透传足够窄的突发
                        master_burst_next = s_axi_awlen;
                        master_burst_size_next = s_axi_awsize;
                        m_axi_awlen_next = s_axi_awlen;
                        m_axi_awsize_next = s_axi_awsize;
                    end
                    m_axi_awburst_next = s_axi_awburst;
                    m_axi_awlock_next = s_axi_awlock;
                    m_axi_awcache_next = s_axi_awcache;
                    m_axi_awprot_next = s_axi_awprot;
                    m_axi_awqos_next = s_axi_awqos;
                    m_axi_awregion_next = s_axi_awregion;
                    m_axi_awuser_next = s_axi_awuser;
                    m_axi_awvalid_next = 1'b1;
                    s_axi_wready_next = m_axi_wready_int_early;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                s_axi_wready_next = m_axi_wready_int_early;

                if (s_axi_wready && s_axi_wvalid) begin
                    data_next = s_axi_wdata;
                    strb_next = s_axi_wstrb;
                    wuser_next = s_axi_wuser;
                    m_axi_wdata_int = s_axi_wdata >> (addr_reg[S_ADDR_BIT_OFFSET-1:M_ADDR_BIT_OFFSET] * M_DATA_WIDTH);
                    m_axi_wstrb_int = s_axi_wstrb >> (addr_reg[S_ADDR_BIT_OFFSET-1:M_ADDR_BIT_OFFSET] * M_STRB_WIDTH);
                    m_axi_wlast_int = 1'b0;
                    m_axi_wuser_int = s_axi_wuser;
                    m_axi_wvalid_int = 1'b1;
                    burst_next = burst_reg - 1;
                    burst_active_next = burst_reg != 0;
                    master_burst_next = master_burst_reg - 1;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    if (master_burst_reg == 0) begin
                        s_axi_wready_next = 1'b0;
                        m_axi_bready_next = !s_axi_bvalid && !s_axi_awvalid;
                        m_axi_wlast_int = 1'b1;
                        state_next = STATE_RESP;
                    end else if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                        state_next = STATE_DATA;
                    end else begin
                        s_axi_wready_next = 1'b0;
                        state_next = STATE_DATA_2;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_2: begin
                s_axi_wready_next = 1'b0;

                if (m_axi_wready_int_reg) begin
                    m_axi_wdata_int = data_reg >> (addr_reg[S_ADDR_BIT_OFFSET-1:M_ADDR_BIT_OFFSET] * M_DATA_WIDTH);
                    m_axi_wstrb_int = strb_reg >> (addr_reg[S_ADDR_BIT_OFFSET-1:M_ADDR_BIT_OFFSET] * M_STRB_WIDTH);
                    m_axi_wlast_int = 1'b0;
                    m_axi_wuser_int = wuser_reg;
                    m_axi_wvalid_int = 1'b1;
                    master_burst_next = master_burst_reg - 1;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    if (master_burst_reg == 0) begin
                        // 主端突发完成，转入响应处理
                        s_axi_wready_next = 1'b0;
                        m_axi_bready_next = !s_axi_bvalid && !m_axi_awvalid;
                        m_axi_wlast_int = 1'b1;
                        state_next = STATE_RESP;
                    end else if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                        state_next = STATE_DATA;
                    end else begin
                        s_axi_wready_next = 1'b0;
                        state_next = STATE_DATA_2;
                    end
                end else begin
                    state_next = STATE_DATA_2;
                end
            end
            STATE_RESP: begin
                // 响应态：传输写响应
                m_axi_bready_next = !s_axi_bvalid && !m_axi_awvalid;

                if (m_axi_bready && m_axi_bvalid) begin
                    first_transfer_next = 1'b0;
                    m_axi_bready_next = 1'b0;
                    s_axi_bid_next = id_reg;
                    if (first_transfer_reg || m_axi_bresp != 0) begin
                        s_axi_bresp_next = m_axi_bresp;
                    end

                    if (burst_reg >> (8+M_BURST_SIZE-burst_size_reg) != 0) begin
                        // 将突发长度限制到最大值
                        master_burst_next = 8'd255;
                    end else begin
                        master_burst_next = (burst_reg << (burst_size_reg-M_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> M_BURST_SIZE);
                    end
                    master_burst_size_next = M_BURST_SIZE;
                    m_axi_awaddr_next = addr_reg;
                    m_axi_awlen_next = master_burst_next;
                    m_axi_awsize_next = master_burst_size_next;
                    if (burst_active_reg) begin
                        // 从端突发仍在进行，启动新子突发
                        m_axi_awvalid_next = 1'b1;
                        state_next = STATE_DATA;
                    end else begin
                        // 从端突发结束，返回空闲态
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = !m_axi_awvalid;
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
    wuser_reg <= wuser_next;
    burst_reg <= burst_next;
    burst_size_reg <= burst_size_next;
    master_burst_reg <= master_burst_next;
    master_burst_size_reg <= master_burst_size_next;
    burst_active_reg <= burst_active_next;
    first_transfer_reg <= first_transfer_next;

    s_axi_awready_reg <= s_axi_awready_next;
    s_axi_wready_reg <= s_axi_wready_next;
    s_axi_bid_reg <= s_axi_bid_next;
    s_axi_bresp_reg <= s_axi_bresp_next;
    s_axi_buser_reg <= s_axi_buser_next;
    s_axi_bvalid_reg <= s_axi_bvalid_next;

    m_axi_awid_reg <= m_axi_awid_next;
    m_axi_awaddr_reg <= m_axi_awaddr_next;
    m_axi_awlen_reg <= m_axi_awlen_next;
    m_axi_awsize_reg <= m_axi_awsize_next;
    m_axi_awburst_reg <= m_axi_awburst_next;
    m_axi_awlock_reg <= m_axi_awlock_next;
    m_axi_awcache_reg <= m_axi_awcache_next;
    m_axi_awprot_reg <= m_axi_awprot_next;
    m_axi_awqos_reg <= m_axi_awqos_next;
    m_axi_awregion_reg <= m_axi_awregion_next;
    m_axi_awuser_reg <= m_axi_awuser_next;
    m_axi_awvalid_reg <= m_axi_awvalid_next;
    m_axi_bready_reg <= m_axi_bready_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axi_awready_reg <= 1'b0;
        s_axi_wready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;

        m_axi_awvalid_reg <= 1'b0;
        m_axi_bready_reg <= 1'b0;
    end
end

// 输出数据通路逻辑
reg [M_DATA_WIDTH-1:0] m_axi_wdata_reg  = {M_DATA_WIDTH{1'b0}}; // 最终输出 WDATA 寄存器。
reg [M_STRB_WIDTH-1:0] m_axi_wstrb_reg  = {M_STRB_WIDTH{1'b0}}; // 最终输出 WSTRB 寄存器。
reg                    m_axi_wlast_reg  = 1'b0; // 最终输出 WLAST 寄存器。
reg [WUSER_WIDTH-1:0]  m_axi_wuser_reg  = 1'b0; // 最终输出 WUSER 寄存器。
reg                    m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // 最终输出 WVALID 当前/下一状态。

reg [M_DATA_WIDTH-1:0] temp_m_axi_wdata_reg  = {M_DATA_WIDTH{1'b0}}; // 最终输出阻塞时的临时 WDATA。
reg [M_STRB_WIDTH-1:0] temp_m_axi_wstrb_reg  = {M_STRB_WIDTH{1'b0}}; // 临时 WSTRB。
reg                    temp_m_axi_wlast_reg  = 1'b0; // 临时 WLAST。
reg [WUSER_WIDTH-1:0]  temp_m_axi_wuser_reg  = 1'b0; // 临时 WUSER。
reg                    temp_m_axi_wvalid_reg = 1'b0, temp_m_axi_wvalid_next; // 临时 WVALID 当前/下一状态。

// 数据通路控制
reg store_axi_w_int_to_output; // 脉冲：将内部 W 拍写入最终输出寄存器。
reg store_axi_w_int_to_temp; // 脉冲：将内部 W 拍写入临时寄存器。
reg store_axi_w_temp_to_output; // 脉冲：将临时 W 拍提升到最终输出寄存器。

assign m_axi_wdata  = m_axi_wdata_reg;
assign m_axi_wstrb  = m_axi_wstrb_reg;
assign m_axi_wlast  = m_axi_wlast_reg;
assign m_axi_wuser  = WUSER_ENABLE ? m_axi_wuser_reg : {WUSER_WIDTH{1'b0}};
assign m_axi_wvalid = m_axi_wvalid_reg;

// 若输出就绪，或下一拍临时寄存器不会被写满（输出寄存器空/无输入），则下一拍拉高 ready
assign m_axi_wready_int_early = m_axi_wready | (~temp_m_axi_wvalid_reg & (~m_axi_wvalid_reg | ~m_axi_wvalid_int));

always @* begin
    // 将接收端 ready 状态传递到发送端
    m_axi_wvalid_next = m_axi_wvalid_reg;
    temp_m_axi_wvalid_next = temp_m_axi_wvalid_reg;

    store_axi_w_int_to_output = 1'b0;
    store_axi_w_int_to_temp = 1'b0;
    store_axi_w_temp_to_output = 1'b0;

    if (m_axi_wready_int_reg) begin
        // 输入端当前就绪
        if (m_axi_wready | ~m_axi_wvalid_reg) begin
            // 输出端就绪或当前无效，直接把数据写入输出寄存器
            m_axi_wvalid_next = m_axi_wvalid_int;
            store_axi_w_int_to_output = 1'b1;
        end else begin
            // 输出端未就绪，将输入暂存到临时寄存器
            temp_m_axi_wvalid_next = m_axi_wvalid_int;
            store_axi_w_int_to_temp = 1'b1;
        end
    end else if (m_axi_wready) begin
        // 输入端未就绪，但输出端就绪
        m_axi_wvalid_next = temp_m_axi_wvalid_reg;
        temp_m_axi_wvalid_next = 1'b0;
        store_axi_w_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axi_wvalid_reg <= 1'b0;
        m_axi_wready_int_reg <= 1'b0;
        temp_m_axi_wvalid_reg <= 1'b0;
    end else begin
        m_axi_wvalid_reg <= m_axi_wvalid_next;
        m_axi_wready_int_reg <= m_axi_wready_int_early;
        temp_m_axi_wvalid_reg <= temp_m_axi_wvalid_next;
    end

    // 数据通路寄存
    if (store_axi_w_int_to_output) begin
        m_axi_wdata_reg <= m_axi_wdata_int;
        m_axi_wstrb_reg <= m_axi_wstrb_int;
        m_axi_wlast_reg <= m_axi_wlast_int;
        m_axi_wuser_reg <= m_axi_wuser_int;
    end else if (store_axi_w_temp_to_output) begin
        m_axi_wdata_reg <= temp_m_axi_wdata_reg;
        m_axi_wstrb_reg <= temp_m_axi_wstrb_reg;
        m_axi_wlast_reg <= temp_m_axi_wlast_reg;
        m_axi_wuser_reg <= temp_m_axi_wuser_reg;
    end

    if (store_axi_w_int_to_temp) begin
        temp_m_axi_wdata_reg <= m_axi_wdata_int;
        temp_m_axi_wstrb_reg <= m_axi_wstrb_int;
        temp_m_axi_wlast_reg <= m_axi_wlast_int;
        temp_m_axi_wuser_reg <= m_axi_wuser_int;
    end
end

endmodule

`resetall
