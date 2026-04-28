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
 * AXI4 位宽适配器（读通道）
 *
 * 模块目录
 * 1) 在从端接收 AXI 读突发请求。
 * 2) 将突发形态/位宽转换为主端 AXI 读突发。
 * 3) 将返回数据按从端拍宽做重组或拆分。
 * 4) 最终 R 通道经过输出 skid buffer 级。
 */
module axi_adapter_rd #
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
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
    // 向更宽总线适配时，尽可能重打包为满宽突发，而不是透传窄突发
    parameter CONVERT_BURST = 1,
    // 向更宽总线适配时，对所有突发执行重打包，而不是透传窄突发
    parameter CONVERT_NARROW_BURST = 0,
    // 是否在适配器中透传 ID
    parameter FORWARD_ID = 0
)
(
    input  wire                     clk, // 读位宽适配器时钟。
    input  wire                     rst, // 转换 FSM 与输出数据通路寄存器同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // 从端 AR ID（读地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // 从端 AR 地址。
    input  wire [7:0]               s_axi_arlen, // 从端 AR 突发长度。
    input  wire [2:0]               s_axi_arsize, // 从端 AR 突发尺寸。
    input  wire [1:0]               s_axi_arburst, // 从端 AR 突发类型。
    input  wire                     s_axi_arlock, // 从端 AR 锁属性。
    input  wire [3:0]               s_axi_arcache, // 从端 AR cache 属性。
    input  wire [2:0]               s_axi_arprot, // 从端 AR 保护属性。
    input  wire [3:0]               s_axi_arqos, // 从端 AR QoS（服务质量字段）。
    input  wire [3:0]               s_axi_arregion, // 从端 AR region（区域属性字段）。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // 从端 AR 用户旁带。
    input  wire                     s_axi_arvalid, // 从端 ARVALID（读地址有效）。
    output wire                     s_axi_arready, // 从端 ARREADY（读地址就绪）。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 从端 R ID（读数据通道标识）。
    output wire [S_DATA_WIDTH-1:0]  s_axi_rdata, // 从端 R 数据（源位宽）。
    output wire [1:0]               s_axi_rresp, // 从端 R 响应码。
    output wire                     s_axi_rlast, // 从端 RLAST（读突发最后一拍）。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 从端 R 用户旁带。
    output wire                     s_axi_rvalid, // 从端 RVALID（读数据有效）。
    input  wire                     s_axi_rready, // 从端 RREADY（读数据就绪）。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_arid, // 主端 AR ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr, // 主端 AR 地址。
    output wire [7:0]               m_axi_arlen, // 主端 AR 突发长度。
    output wire [2:0]               m_axi_arsize, // 主端 AR 突发尺寸。
    output wire [1:0]               m_axi_arburst, // 主端 AR 突发类型。
    output wire                     m_axi_arlock, // 主端 AR 锁属性。
    output wire [3:0]               m_axi_arcache, // 主端 AR cache 属性。
    output wire [2:0]               m_axi_arprot, // 主端 AR 保护属性。
    output wire [3:0]               m_axi_arqos, // 主端 AR QoS。
    output wire [3:0]               m_axi_arregion, // 主端 AR region。
    output wire [ARUSER_WIDTH-1:0]  m_axi_aruser, // 主端 AR 用户旁带。
    output wire                     m_axi_arvalid, // 主端 ARVALID。
    input  wire                     m_axi_arready, // 主端 ARREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 主端 R ID。
    input  wire [M_DATA_WIDTH-1:0]  m_axi_rdata, // 主端 R 数据（目标位宽）。
    input  wire [1:0]               m_axi_rresp, // 主端 R 响应码。
    input  wire                     m_axi_rlast, // 主端 RLAST。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 主端 R 用户旁带。
    input  wire                     m_axi_rvalid, // 主端 RVALID。
    output wire                     m_axi_rready // 主端 RREADY。
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
    STATE_IDLE = 2'd0, // 等待新的从端 AR 请求。
    STATE_DATA = 2'd1, // 等宽或窄突发透传场景下直接传输。
    STATE_DATA_READ = 2'd2, // 先获取更宽主端读数据拍，再按从端位宽拆分。
    STATE_DATA_SPLIT = 2'd3; // 仅输出缓存拆分段，不再发起新主端读取。

reg [1:0] state_reg = STATE_IDLE, state_next; // 主读转换 FSM 状态。

reg [ID_WIDTH-1:0] id_reg = {ID_WIDTH{1'b0}}, id_next; // 当前从端突发锁存事务 ID。
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next; // 当前地址指针。
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next; // 缓存的合并/拆分读数据。
reg [1:0] resp_reg = 2'd0, resp_next; // 合并数据拍的累计响应状态。
reg [RUSER_WIDTH-1:0] ruser_reg = {RUSER_WIDTH{1'b0}}, ruser_next; // 合并数据拍的缓存 RUSER。
reg [7:0] burst_reg = 8'd0, burst_next; // 剩余从端数据拍计数。
reg [2:0] burst_size_reg = 3'd0, burst_size_next; // 从端突发尺寸（log2 bytes/beat）。
reg [7:0] master_burst_reg = 8'd0, master_burst_next; // 当前子突发剩余主端数据拍计数。
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next; // 主端有效突发尺寸。

reg s_axi_arready_reg = 1'b0, s_axi_arready_next; // 从端 ARREADY 状态。

reg [ID_WIDTH-1:0] m_axi_arid_reg = {ID_WIDTH{1'b0}}, m_axi_arid_next; // 主端 ARID 输出寄存器。
reg [ADDR_WIDTH-1:0] m_axi_araddr_reg = {ADDR_WIDTH{1'b0}}, m_axi_araddr_next; // 主端 ARADDR 输出寄存器。
reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next; // 主端 ARLEN 输出寄存器。
reg [2:0] m_axi_arsize_reg = 3'd0, m_axi_arsize_next; // 主端 ARSIZE 输出寄存器。
reg [1:0] m_axi_arburst_reg = 2'd0, m_axi_arburst_next; // 主端 ARBURST 输出寄存器。
reg m_axi_arlock_reg = 1'b0, m_axi_arlock_next; // 主端 ARLOCK 输出寄存器。
reg [3:0] m_axi_arcache_reg = 4'd0, m_axi_arcache_next; // 主端 ARCACHE 输出寄存器。
reg [2:0] m_axi_arprot_reg = 3'd0, m_axi_arprot_next; // 主端 ARPROT 输出寄存器。
reg [3:0] m_axi_arqos_reg = 4'd0, m_axi_arqos_next; // 主端 ARQOS 输出寄存器。
reg [3:0] m_axi_arregion_reg = 4'd0, m_axi_arregion_next; // 主端 ARREGION 输出寄存器。
reg [ARUSER_WIDTH-1:0] m_axi_aruser_reg = {ARUSER_WIDTH{1'b0}}, m_axi_aruser_next; // 主端 ARUSER 输出寄存器。
reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next; // 主端 ARVALID 状态。
reg m_axi_rready_reg = 1'b0, m_axi_rready_next; // 主端 RREADY 状态。

// 内部数据通路
reg  [ID_WIDTH-1:0]     s_axi_rid_int; // 来自转换 FSM 的内部预缓冲 RID。
reg  [S_DATA_WIDTH-1:0] s_axi_rdata_int; // 来自转换 FSM 的内部预缓冲 RDATA。
reg  [1:0]              s_axi_rresp_int; // 来自转换 FSM 的内部预缓冲 RRESP。
reg                     s_axi_rlast_int; // 来自转换 FSM 的内部预缓冲 RLAST。
reg  [RUSER_WIDTH-1:0]  s_axi_ruser_int; // 来自转换 FSM 的内部预缓冲 RUSER。
reg                     s_axi_rvalid_int; // 来自转换 FSM 的内部预缓冲 RVALID。
reg                     s_axi_rready_int_reg = 1'b0; // 内部到输出 skid 级的 ready 寄存器。
wire                    s_axi_rready_int_early; // 内部到输出 skid 级的前瞻 ready。

assign s_axi_arready = s_axi_arready_reg;

assign m_axi_arid = FORWARD_ID ? m_axi_arid_reg : {ID_WIDTH{1'b0}};
assign m_axi_araddr = m_axi_araddr_reg;
assign m_axi_arlen = m_axi_arlen_reg;
assign m_axi_arsize = m_axi_arsize_reg;
assign m_axi_arburst = m_axi_arburst_reg;
assign m_axi_arlock = m_axi_arlock_reg;
assign m_axi_arcache = m_axi_arcache_reg;
assign m_axi_arprot = m_axi_arprot_reg;
assign m_axi_arqos = m_axi_arqos_reg;
assign m_axi_arregion = m_axi_arregion_reg;
assign m_axi_aruser = ARUSER_ENABLE ? m_axi_aruser_reg : {ARUSER_WIDTH{1'b0}};
assign m_axi_arvalid = m_axi_arvalid_reg;
assign m_axi_rready = m_axi_rready_reg;

always @* begin
    state_next = STATE_IDLE;

    id_next = id_reg;
    addr_next = addr_reg;
    data_next = data_reg;
    resp_next = resp_reg;
    ruser_next = ruser_reg;
    burst_next = burst_reg;
    burst_size_next = burst_size_reg;
    master_burst_next = master_burst_reg;
    master_burst_size_next = master_burst_size_reg;

    s_axi_arready_next = 1'b0;
    m_axi_arid_next = m_axi_arid_reg;
    m_axi_araddr_next = m_axi_araddr_reg;
    m_axi_arlen_next = m_axi_arlen_reg;
    m_axi_arsize_next = m_axi_arsize_reg;
    m_axi_arburst_next = m_axi_arburst_reg;
    m_axi_arlock_next = m_axi_arlock_reg;
    m_axi_arcache_next = m_axi_arcache_reg;
    m_axi_arprot_next = m_axi_arprot_reg;
    m_axi_arqos_next = m_axi_arqos_reg;
    m_axi_arregion_next = m_axi_arregion_reg;
    m_axi_aruser_next = m_axi_aruser_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;
    m_axi_rready_next = 1'b0;

    if (SEGMENT_COUNT == 1) begin
        // 主端与从端位宽相同：直接传输，不做拆分/合并
        s_axi_rid_int = id_reg;
        s_axi_rdata_int = m_axi_rdata;
        s_axi_rresp_int = m_axi_rresp;
        s_axi_rlast_int = m_axi_rlast;
        s_axi_ruser_int = m_axi_ruser;
        s_axi_rvalid_int = 0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axi_arvalid;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axi_arid_next = s_axi_arid;
                    m_axi_araddr_next = s_axi_araddr;
                    m_axi_arlen_next = s_axi_arlen;
                    m_axi_arsize_next = s_axi_arsize;
                    m_axi_arburst_next = s_axi_arburst;
                    m_axi_arlock_next = s_axi_arlock;
                    m_axi_arcache_next = s_axi_arcache;
                    m_axi_arprot_next = s_axi_arprot;
                    m_axi_arqos_next = s_axi_arqos;
                    m_axi_arregion_next = s_axi_arregion;
                    m_axi_aruser_next = s_axi_aruser;
                    m_axi_arvalid_next = 1'b1;
                    m_axi_rready_next = s_axi_rready_int_early;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                // 数据态：传输读数据
                m_axi_rready_next = s_axi_rready_int_early;

                if (m_axi_rready && m_axi_rvalid) begin
                    s_axi_rid_int = id_reg;
                    s_axi_rdata_int = m_axi_rdata;
                    s_axi_rresp_int = m_axi_rresp;
                    s_axi_rlast_int = m_axi_rlast;
                    s_axi_ruser_int = m_axi_ruser;
                    s_axi_rvalid_int = 1'b1;
                    if (m_axi_rlast) begin
                        // 最后一拍数据，返回空闲态
                        m_axi_rready_next = 1'b0;
                        s_axi_arready_next = !m_axi_arvalid;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
        endcase
    end else if (EXPAND) begin
        // 主端输出更宽：执行读数据拆分
        s_axi_rid_int = id_reg;
        s_axi_rdata_int = m_axi_rdata;
        s_axi_rresp_int = m_axi_rresp;
        s_axi_rlast_int = m_axi_rlast;
        s_axi_ruser_int = m_axi_ruser;
        s_axi_rvalid_int = 0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axi_arvalid;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axi_arid_next = s_axi_arid;
                    m_axi_araddr_next = s_axi_araddr;
                    addr_next = s_axi_araddr;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    if (CONVERT_BURST && s_axi_arcache[1] && (CONVERT_NARROW_BURST || s_axi_arsize == S_BURST_SIZE)) begin
                        // 拆分读取
                        // 需开启 CONVERT_BURST 且 arcache[1] 置位
                        master_burst_size_next = M_BURST_SIZE;
                        if (CONVERT_NARROW_BURST) begin
                            m_axi_arlen_next = (({{S_ADDR_BIT_OFFSET+1{1'b0}}, s_axi_arlen} << s_axi_arsize) + s_axi_araddr[M_ADDR_BIT_OFFSET-1:0]) >> M_BURST_SIZE;
                        end else begin
                            m_axi_arlen_next = ({1'b0, s_axi_arlen} + s_axi_araddr[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET]) >> $clog2(SEGMENT_COUNT);
                        end
                        m_axi_arsize_next = M_BURST_SIZE;
                        state_next = STATE_DATA_READ;
                    end else begin
                        // 输出窄突发
                        master_burst_size_next = s_axi_arsize;
                        m_axi_arlen_next = s_axi_arlen;
                        m_axi_arsize_next = s_axi_arsize;
                        state_next = STATE_DATA;
                    end
                    m_axi_arburst_next = s_axi_arburst;
                    m_axi_arlock_next = s_axi_arlock;
                    m_axi_arcache_next = s_axi_arcache;
                    m_axi_arprot_next = s_axi_arprot;
                    m_axi_arqos_next = s_axi_arqos;
                    m_axi_arregion_next = s_axi_arregion;
                    m_axi_aruser_next = s_axi_aruser;
                    m_axi_arvalid_next = 1'b1;
                    m_axi_rready_next = s_axi_rready_int_early;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axi_rready_next = s_axi_rready_int_early;

                if (m_axi_rready && m_axi_rvalid) begin
                    s_axi_rid_int = id_reg;
                    s_axi_rdata_int = m_axi_rdata >> (addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET] * S_DATA_WIDTH);
                    s_axi_rresp_int = m_axi_rresp;
                    s_axi_rlast_int = m_axi_rlast;
                    s_axi_ruser_int = m_axi_ruser;
                    s_axi_rvalid_int = 1'b1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (m_axi_rlast) begin
                        m_axi_rready_next = 1'b0;
                        s_axi_arready_next = !m_axi_arvalid;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_DATA;
                end
            end
            STATE_DATA_READ: begin
                m_axi_rready_next = s_axi_rready_int_early;

                if (m_axi_rready && m_axi_rvalid) begin
                    s_axi_rid_int = id_reg;
                    data_next = m_axi_rdata;
                    resp_next = m_axi_rresp;
                    ruser_next = m_axi_ruser;
                    s_axi_rdata_int = m_axi_rdata >> (addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET] * S_DATA_WIDTH);
                    s_axi_rresp_int = m_axi_rresp;
                    s_axi_rlast_int = 1'b0;
                    s_axi_ruser_int = m_axi_ruser;
                    s_axi_rvalid_int = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        m_axi_rready_next = 1'b0;
                        s_axi_arready_next = !m_axi_arvalid;
                        s_axi_rlast_int = 1'b1;
                        state_next = STATE_IDLE;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        state_next = STATE_DATA_READ;
                    end else begin
                        m_axi_rready_next = 1'b0;
                        state_next = STATE_DATA_SPLIT;
                    end
                end else begin
                    state_next = STATE_DATA_READ;
                end
            end
            STATE_DATA_SPLIT: begin
                m_axi_rready_next = 1'b0;

                if (s_axi_rready_int_reg) begin
                    s_axi_rid_int = id_reg;
                    s_axi_rdata_int = data_reg >> (addr_reg[M_ADDR_BIT_OFFSET-1:S_ADDR_BIT_OFFSET] * S_DATA_WIDTH);
                    s_axi_rresp_int = resp_reg;
                    s_axi_rlast_int = 1'b0;
                    s_axi_ruser_int = ruser_reg;
                    s_axi_rvalid_int = 1'b1;
                    burst_next = burst_reg - 1;
                    addr_next = addr_reg + (1 << burst_size_reg);
                    if (burst_reg == 0) begin
                        s_axi_arready_next = !m_axi_arvalid;
                        s_axi_rlast_int = 1'b1;
                        state_next = STATE_IDLE;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        m_axi_rready_next = s_axi_rready_int_early;
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
        s_axi_rid_int = id_reg;
        s_axi_rdata_int = data_reg;
        s_axi_rresp_int = resp_reg;
        s_axi_rlast_int = 1'b0;
        s_axi_ruser_int = m_axi_ruser;
        s_axi_rvalid_int = 0;

        case (state_reg)
            STATE_IDLE: begin
                // 空闲态：等待新突发
                s_axi_arready_next = !m_axi_arvalid;

                resp_next = 2'd0;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;
                    id_next = s_axi_arid;
                    m_axi_arid_next = s_axi_arid;
                    m_axi_araddr_next = s_axi_araddr;
                    addr_next = s_axi_araddr;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    if (s_axi_arsize > M_BURST_SIZE) begin
                        // 需要调整突发尺寸
                        if (s_axi_arlen >> (8+M_BURST_SIZE-s_axi_arsize) != 0) begin
                            // 将突发长度限制到最大值
                            master_burst_next = (8'd255 << (s_axi_arsize-M_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> M_BURST_SIZE);
                        end else begin
                            master_burst_next = (s_axi_arlen << (s_axi_arsize-M_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> M_BURST_SIZE);
                        end
                        master_burst_size_next = M_BURST_SIZE;
                        m_axi_arlen_next = master_burst_next;
                        m_axi_arsize_next = master_burst_size_next;
                    end else begin
                        // 直接透传足够窄的突发
                        master_burst_next = s_axi_arlen;
                        master_burst_size_next = s_axi_arsize;
                        m_axi_arlen_next = s_axi_arlen;
                        m_axi_arsize_next = s_axi_arsize;
                    end
                    m_axi_arburst_next = s_axi_arburst;
                    m_axi_arlock_next = s_axi_arlock;
                    m_axi_arcache_next = s_axi_arcache;
                    m_axi_arprot_next = s_axi_arprot;
                    m_axi_arqos_next = s_axi_arqos;
                    m_axi_arregion_next = s_axi_arregion;
                    m_axi_aruser_next = s_axi_aruser;
                    m_axi_arvalid_next = 1'b1;
                    m_axi_rready_next = 1'b0;
                    state_next = STATE_DATA;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_DATA: begin
                m_axi_rready_next = s_axi_rready_int_early && !m_axi_arvalid;

                if (m_axi_rready && m_axi_rvalid) begin
                    data_next[addr_reg[S_ADDR_BIT_OFFSET-1:M_ADDR_BIT_OFFSET]*SEGMENT_DATA_WIDTH +: SEGMENT_DATA_WIDTH] = m_axi_rdata;
                    if (m_axi_rresp) begin
                        resp_next = m_axi_rresp;
                    end
                    s_axi_rid_int = id_reg;
                    s_axi_rdata_int = data_next;
                    s_axi_rresp_int = resp_next;
                    s_axi_rlast_int = 1'b0;
                    s_axi_ruser_int = m_axi_ruser;
                    s_axi_rvalid_int = 1'b0;
                    master_burst_next = master_burst_reg - 1;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    m_axi_araddr_next = addr_next;
                    if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                        data_next = {DATA_WIDTH{1'b0}};
                        burst_next = burst_reg - 1;
                        s_axi_rvalid_int = 1'b1;
                    end
                    if (master_burst_reg == 0) begin
                        if (burst_next >> (8+M_BURST_SIZE-burst_size_reg) != 0) begin
                            // 将突发长度限制到最大值
                            master_burst_next = 8'd255;
                        end else begin
                            master_burst_next = (burst_next << (burst_size_reg-M_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> M_BURST_SIZE);
                        end
                        m_axi_arlen_next = master_burst_next;

                        if (burst_reg == 0) begin
                            m_axi_rready_next = 1'b0;
                            s_axi_rlast_int = 1'b1;
                            s_axi_rvalid_int = 1'b1;
                            s_axi_arready_next = !m_axi_arvalid;
                            state_next = STATE_IDLE;
                        end else begin
                            // 启动新子突发
                            m_axi_arvalid_next = 1'b1;
                            m_axi_rready_next = 1'b0;
                            state_next = STATE_DATA;
                        end
                    end else begin
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
    ruser_reg <= ruser_next;
    burst_reg <= burst_next;
    burst_size_reg <= burst_size_next;
    master_burst_reg <= master_burst_next;
    master_burst_size_reg <= master_burst_size_next;

    s_axi_arready_reg <= s_axi_arready_next;

    m_axi_arid_reg <= m_axi_arid_next;
    m_axi_araddr_reg <= m_axi_araddr_next;
    m_axi_arlen_reg <= m_axi_arlen_next;
    m_axi_arsize_reg <= m_axi_arsize_next;
    m_axi_arburst_reg <= m_axi_arburst_next;
    m_axi_arlock_reg <= m_axi_arlock_next;
    m_axi_arcache_reg <= m_axi_arcache_next;
    m_axi_arprot_reg <= m_axi_arprot_next;
    m_axi_arqos_reg <= m_axi_arqos_next;
    m_axi_arregion_reg <= m_axi_arregion_next;
    m_axi_aruser_reg <= m_axi_aruser_next;
    m_axi_arvalid_reg <= m_axi_arvalid_next;
    m_axi_rready_reg <= m_axi_rready_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axi_arready_reg <= 1'b0;

        m_axi_arvalid_reg <= 1'b0;
        m_axi_rready_reg <= 1'b0;
    end
end

// 输出数据通路逻辑
reg [ID_WIDTH-1:0]     s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // 最终输出 RID 寄存器（经过 skid 级）。
reg [S_DATA_WIDTH-1:0] s_axi_rdata_reg  = {S_DATA_WIDTH{1'b0}}; // 最终输出 RDATA 寄存器。
reg [1:0]              s_axi_rresp_reg  = 2'd0; // 最终输出 RRESP 寄存器。
reg                    s_axi_rlast_reg  = 1'b0; // 最终输出 RLAST 寄存器。
reg [RUSER_WIDTH-1:0]  s_axi_ruser_reg  = 1'b0; // 最终输出 RUSER 寄存器。
reg                    s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // 最终输出 RVALID 当前/下一状态。

reg [ID_WIDTH-1:0]     temp_s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // 最终输出阻塞时的临时 RID。
reg [S_DATA_WIDTH-1:0] temp_s_axi_rdata_reg  = {S_DATA_WIDTH{1'b0}}; // 临时 RDATA。
reg [1:0]              temp_s_axi_rresp_reg  = 2'd0; // 临时 RRESP。
reg                    temp_s_axi_rlast_reg  = 1'b0; // 临时 RLAST。
reg [RUSER_WIDTH-1:0]  temp_s_axi_ruser_reg  = 1'b0; // 临时 RUSER。
reg                    temp_s_axi_rvalid_reg = 1'b0, temp_s_axi_rvalid_next; // 临时 RVALID 当前/下一状态。

// 数据通路控制
reg store_axi_r_int_to_output; // 脉冲：将内部 R 拍写入最终输出寄存器。
reg store_axi_r_int_to_temp; // 脉冲：将内部 R 拍写入临时寄存器。
reg store_axi_r_temp_to_output; // 脉冲：将临时 R 拍提升到最终输出寄存器。

assign s_axi_rid    = s_axi_rid_reg;
assign s_axi_rdata  = s_axi_rdata_reg;
assign s_axi_rresp  = s_axi_rresp_reg;
assign s_axi_rlast  = s_axi_rlast_reg;
assign s_axi_ruser  = RUSER_ENABLE ? s_axi_ruser_reg : {RUSER_WIDTH{1'b0}};
assign s_axi_rvalid = s_axi_rvalid_reg;

// 若输出就绪，或下一拍临时寄存器不会被写满（输出寄存器空/无输入），则下一拍拉高 ready
assign s_axi_rready_int_early = s_axi_rready | (~temp_s_axi_rvalid_reg & (~s_axi_rvalid_reg | ~s_axi_rvalid_int));

always @* begin
    // 将接收端 ready 状态传递到发送端
    s_axi_rvalid_next = s_axi_rvalid_reg;
    temp_s_axi_rvalid_next = temp_s_axi_rvalid_reg;

    store_axi_r_int_to_output = 1'b0;
    store_axi_r_int_to_temp = 1'b0;
    store_axi_r_temp_to_output = 1'b0;

    if (s_axi_rready_int_reg) begin
        // 输入端当前就绪
        if (s_axi_rready | ~s_axi_rvalid_reg) begin
            // 输出端就绪或当前无效，直接把数据写入输出寄存器
            s_axi_rvalid_next = s_axi_rvalid_int;
            store_axi_r_int_to_output = 1'b1;
        end else begin
            // 输出端未就绪，将输入暂存到临时寄存器
            temp_s_axi_rvalid_next = s_axi_rvalid_int;
            store_axi_r_int_to_temp = 1'b1;
        end
    end else if (s_axi_rready) begin
        // 输入端未就绪，但输出端就绪
        s_axi_rvalid_next = temp_s_axi_rvalid_reg;
        temp_s_axi_rvalid_next = 1'b0;
        store_axi_r_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        s_axi_rvalid_reg <= 1'b0;
        s_axi_rready_int_reg <= 1'b0;
        temp_s_axi_rvalid_reg <= 1'b0;
    end else begin
        s_axi_rvalid_reg <= s_axi_rvalid_next;
        s_axi_rready_int_reg <= s_axi_rready_int_early;
        temp_s_axi_rvalid_reg <= temp_s_axi_rvalid_next;
    end

    // 数据通路寄存
    if (store_axi_r_int_to_output) begin
        s_axi_rid_reg <= s_axi_rid_int;
        s_axi_rdata_reg <= s_axi_rdata_int;
        s_axi_rresp_reg <= s_axi_rresp_int;
        s_axi_rlast_reg <= s_axi_rlast_int;
        s_axi_ruser_reg <= s_axi_ruser_int;
    end else if (store_axi_r_temp_to_output) begin
        s_axi_rid_reg <= temp_s_axi_rid_reg;
        s_axi_rdata_reg <= temp_s_axi_rdata_reg;
        s_axi_rresp_reg <= temp_s_axi_rresp_reg;
        s_axi_rlast_reg <= temp_s_axi_rlast_reg;
        s_axi_ruser_reg <= temp_s_axi_ruser_reg;
    end

    if (store_axi_r_int_to_temp) begin
        temp_s_axi_rid_reg <= s_axi_rid_int;
        temp_s_axi_rdata_reg <= s_axi_rdata_int;
        temp_s_axi_rresp_reg <= s_axi_rresp_int;
        temp_s_axi_rlast_reg <= s_axi_rlast_int;
        temp_s_axi_ruser_reg <= s_axi_ruser_int;
    end
end

endmodule

`resetall
