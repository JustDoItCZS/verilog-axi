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
 * AXI4 DMA 读引擎
 *
 * 模块目录
 * 1) 接收读描述符，生成 AXI AR 突发，从内存搬运数据到 AXIS 输出。
 * 2) 处理未对齐读、末拍 keep/last 生成，以及读错误状态上报。
 * 3) 输出侧使用 FIFO 解耦 AXI R 通道与 AXIS 消费速率。
 */
module axi_dma_rd #
(
    // AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = 32,
    // AXI 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // AXI WSTRB 位宽（按字节）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 信号位宽
    parameter AXI_ID_WIDTH = 8,
    // 生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // AXI-Stream 接口数据位宽
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
    // 是否使用 AXI-Stream TKEEP
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI-Stream TKEEP 位宽（每拍字节数）
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // 是否使用 AXI-Stream TLAST
    parameter AXIS_LAST_ENABLE = 1,
    // 是否透传 AXI-Stream TID
    parameter AXIS_ID_ENABLE = 0,
    // AXI-Stream TID 位宽
    parameter AXIS_ID_WIDTH = 8,
    // 是否透传 AXI-Stream TDEST
    parameter AXIS_DEST_ENABLE = 0,
    // AXI-Stream TDEST 位宽
    parameter AXIS_DEST_WIDTH = 8,
    // 是否透传 AXI-Stream TUSER
    parameter AXIS_USER_ENABLE = 1,
    // AXI-Stream TUSER 位宽
    parameter AXIS_USER_WIDTH = 1,
    // 长度字段位宽
    parameter LEN_WIDTH = 20,
    // tag 字段位宽
    parameter TAG_WIDTH = 8,
    // 是否启用散列/聚集 DMA 支持
    // （每个 AXI-Stream 帧可含多个描述符）
    parameter ENABLE_SG = 0,
    // 是否启用非对齐传输支持
    parameter ENABLE_UNALIGNED = 0
)
(
    input  wire                       clk, // 读 DMA 时钟。
    input  wire                       rst, // 同步复位，高电平有效。

    /*
     * AXI 读描述符输入
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_read_desc_addr, // 读描述符源地址。
    input  wire [LEN_WIDTH-1:0]       s_axis_read_desc_len, // 读描述符长度。
    input  wire [TAG_WIDTH-1:0]       s_axis_read_desc_tag, // 读描述符 tag。
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_read_desc_id, // 读描述符附带 AXIS ID。
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_read_desc_dest, // 读描述符附带 AXIS DEST。
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_read_desc_user, // 读描述符附带 AXIS USER。
    input  wire                       s_axis_read_desc_valid, // 读描述符有效。
    output wire                       s_axis_read_desc_ready, // 模块可接收读描述符。

    /*
     * AXI 读描述符状态输出
     */
    output wire [TAG_WIDTH-1:0]       m_axis_read_desc_status_tag, // 读完成状态 tag。
    output wire [3:0]                 m_axis_read_desc_status_error, // 读完成状态错误码。
    output wire                       m_axis_read_desc_status_valid, // 读完成状态有效。

    /*
     * AXI-Stream 读数据输出
     */
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata, // AXIS 输出 tdata。
    output wire [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep, // AXIS 输出 tkeep。
    output wire                       m_axis_read_data_tvalid, // AXIS 输出 tvalid。
    input  wire                       m_axis_read_data_tready, // AXIS 输出 tready。
    output wire                       m_axis_read_data_tlast, // AXIS 输出 tlast。
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid, // AXIS 输出 tid。
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest, // AXIS 输出 tdest。
    output wire [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser, // AXIS 输出 tuser。

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid, // AXI 读地址 ID。
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr, // AXI 读地址。
    output wire [7:0]                 m_axi_arlen, // AXI 读突发长度。
    output wire [2:0]                 m_axi_arsize, // AXI 读 beat 大小。
    output wire [1:0]                 m_axi_arburst, // AXI 读突发类型。
    output wire                       m_axi_arlock, // AXI 读锁属性。
    output wire [3:0]                 m_axi_arcache, // AXI 读 cache 属性。
    output wire [2:0]                 m_axi_arprot, // AXI 读保护属性。
    output wire                       m_axi_arvalid, // AXI 读地址有效。
    input  wire                       m_axi_arready, // AXI 读地址 ready。
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid, // AXI 读响应 ID。
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata, // AXI 读响应数据。
    input  wire [1:0]                 m_axi_rresp, // AXI 读响应状态。
    input  wire                       m_axi_rlast, // AXI 读响应最后一拍。
    input  wire                       m_axi_rvalid, // AXI 读响应有效。
    output wire                       m_axi_rready, // AXI 读响应 ready。

    /*
     * 配置
     */
    input  wire                       enable // 读 DMA 总使能。
);

parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH;
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
parameter AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE;

parameter AXIS_KEEP_WIDTH_INT = AXIS_KEEP_ENABLE ? AXIS_KEEP_WIDTH : 1;
parameter AXIS_WORD_WIDTH = AXIS_KEEP_WIDTH_INT;
parameter AXIS_WORD_SIZE = AXIS_DATA_WIDTH/AXIS_WORD_WIDTH;

parameter OFFSET_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
parameter OFFSET_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_WIDTH{1'b1}} : 0;
parameter ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH);
parameter CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

parameter OUTPUT_FIFO_ADDR_WIDTH = 5;

// 总线位宽合法性检查
initial begin
    if (AXI_WORD_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXIS_WORD_SIZE * AXIS_KEEP_WIDTH_INT != AXIS_DATA_WIDTH) begin
        $error("Error: AXI stream data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXI_WORD_SIZE != AXIS_WORD_SIZE) begin
        $error("Error: word size mismatch (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_WORD_WIDTH) != AXI_WORD_WIDTH) begin
        $error("Error: AXI word width must be even power of two (instance %m)");
        $finish;
    end

    if (AXI_DATA_WIDTH != AXIS_DATA_WIDTH) begin
        $error("Error: AXI interface width must match AXI stream interface width (instance %m)");
        $finish;
    end

    if (AXI_MAX_BURST_LEN < 1 || AXI_MAX_BURST_LEN > 256) begin
        $error("Error: AXI_MAX_BURST_LEN must be between 1 and 256 (instance %m)");
        $finish;
    end

    if (ENABLE_SG) begin
        $error("Error: scatter/gather is not yet implemented (instance %m)");
        $finish;
    end
end

localparam [1:0]
    AXI_RESP_OKAY = 2'b00,
    AXI_RESP_EXOKAY = 2'b01,
    AXI_RESP_SLVERR = 2'b10,
    AXI_RESP_DECERR = 2'b11;

localparam [3:0]
    DMA_ERROR_NONE = 4'd0,
    DMA_ERROR_TIMEOUT = 4'd1,
    DMA_ERROR_PARITY = 4'd2,
    DMA_ERROR_AXI_RD_SLVERR = 4'd4,
    DMA_ERROR_AXI_RD_DECERR = 4'd5,
    DMA_ERROR_AXI_WR_SLVERR = 4'd6,
    DMA_ERROR_AXI_WR_DECERR = 4'd7,
    DMA_ERROR_PCIE_FLR = 4'd8,
    DMA_ERROR_PCIE_CPL_POISONED = 4'd9,
    DMA_ERROR_PCIE_CPL_STATUS_UR = 4'd10,
    DMA_ERROR_PCIE_CPL_STATUS_CA = 4'd11;

localparam [0:0]
    AXI_STATE_IDLE = 1'd0,
    AXI_STATE_START = 1'd1;

reg [0:0] axi_state_reg = AXI_STATE_IDLE, axi_state_next; // AXI 侧描述符拆包与 AR 生成状态机。

localparam [0:0]
    AXIS_STATE_IDLE = 1'd0,
    AXIS_STATE_READ = 1'd1;

reg [0:0] axis_state_reg = AXIS_STATE_IDLE, axis_state_next; // AXIS 侧数据重排与输出状态机。

// 数据通路控制信号
reg transfer_in_save; // 当前拍是否需把 AXI R 数据缓存到 save 寄存器。
reg axis_cmd_ready; // AXIS 命令寄存器是否可接受新命令。

reg [AXI_ADDR_WIDTH-1:0] addr_reg = {AXI_ADDR_WIDTH{1'b0}}, addr_next; // 当前 AXI 访问地址游标。
reg [LEN_WIDTH-1:0] op_word_count_reg = {LEN_WIDTH{1'b0}}, op_word_count_next; // 当前描述符剩余总字数计数。
reg [LEN_WIDTH-1:0] tr_word_count_reg = {LEN_WIDTH{1'b0}}, tr_word_count_next; // 当前突发剩余字数计数。

reg [OFFSET_WIDTH-1:0] axis_cmd_offset_reg = {OFFSET_WIDTH{1'b0}}, axis_cmd_offset_next; // 当前命令首拍字节偏移。
reg [OFFSET_WIDTH-1:0] axis_cmd_last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, axis_cmd_last_cycle_offset_next; // 当前命令末拍有效偏移。
reg [CYCLE_COUNT_WIDTH-1:0] axis_cmd_input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axis_cmd_input_cycle_count_next; // 需消费的 AXI 输入拍数。
reg [CYCLE_COUNT_WIDTH-1:0] axis_cmd_output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axis_cmd_output_cycle_count_next; // 需输出的 AXIS 输出拍数。
reg axis_cmd_bubble_cycle_reg = 1'b0, axis_cmd_bubble_cycle_next; // 命令是否插入气泡拍(未对齐场景)。
reg [TAG_WIDTH-1:0] axis_cmd_tag_reg = {TAG_WIDTH{1'b0}}, axis_cmd_tag_next; // 当前命令关联状态 tag。
reg [AXIS_ID_WIDTH-1:0] axis_cmd_axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_cmd_axis_id_next; // 当前命令附带 tid。
reg [AXIS_DEST_WIDTH-1:0] axis_cmd_axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_cmd_axis_dest_next; // 当前命令附带 tdest。
reg [AXIS_USER_WIDTH-1:0] axis_cmd_axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_cmd_axis_user_next; // 当前命令附带 tuser。
reg axis_cmd_valid_reg = 1'b0, axis_cmd_valid_next; // 命令寄存器有效位。

reg [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next; // 实际数据重排使用的当前偏移。
reg [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next; // 当前命令末拍偏移。
reg [CYCLE_COUNT_WIDTH-1:0] input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, input_cycle_count_next; // 剩余需读取 AXI R 拍数。
reg [CYCLE_COUNT_WIDTH-1:0] output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, output_cycle_count_next; // 剩余需输出 AXIS 拍数。
reg input_active_reg = 1'b0, input_active_next; // 输入侧是否处于活跃接收阶段。
reg output_active_reg = 1'b0, output_active_next; // 输出侧是否处于活跃发送阶段。
reg bubble_cycle_reg = 1'b0, bubble_cycle_next; // 当前是否处于气泡拍阶段。
reg first_cycle_reg = 1'b0, first_cycle_next; // 当前输出是否为命令首拍。
reg output_last_cycle_reg = 1'b0, output_last_cycle_next; // 当前输出是否为命令末拍。
reg [1:0] rresp_reg = AXI_RESP_OKAY, rresp_next; // 聚合的读响应错误状态。

reg [TAG_WIDTH-1:0] tag_reg = {TAG_WIDTH{1'b0}}, tag_next; // 当前输出事务的状态 tag。
reg [AXIS_ID_WIDTH-1:0] axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_id_next; // 当前输出事务的 tid。
reg [AXIS_DEST_WIDTH-1:0] axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_dest_next; // 当前输出事务的 tdest。
reg [AXIS_USER_WIDTH-1:0] axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_user_next; // 当前输出事务的 tuser。

reg s_axis_read_desc_ready_reg = 1'b0, s_axis_read_desc_ready_next; // 描述符输入 ready 寄存器。

reg [TAG_WIDTH-1:0] m_axis_read_desc_status_tag_reg = {TAG_WIDTH{1'b0}}, m_axis_read_desc_status_tag_next; // 状态输出 tag 寄存器。
reg [3:0] m_axis_read_desc_status_error_reg = 4'd0, m_axis_read_desc_status_error_next; // 状态输出错误码寄存器。
reg m_axis_read_desc_status_valid_reg = 1'b0, m_axis_read_desc_status_valid_next; // 状态输出 valid 寄存器。

reg [AXI_ADDR_WIDTH-1:0] m_axi_araddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_araddr_next; // AR 地址寄存器。
reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next; // AR 长度寄存器。
reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next; // ARVALID 寄存器。
reg m_axi_rready_reg = 1'b0, m_axi_rready_next; // RREADY 寄存器。

reg [AXI_DATA_WIDTH-1:0] save_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}}; // 未对齐拼接时缓存上一拍 AXI RDATA。

wire [AXI_DATA_WIDTH-1:0] shift_axi_rdata = {m_axi_rdata, save_axi_rdata_reg} >> ((AXI_STRB_WIDTH-offset_reg)*AXI_WORD_SIZE); // 根据偏移重排后的对齐数据。

// 内部数据通路
reg  [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata_int; // 内部待输出 tdata。
reg  [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep_int; // 内部待输出 tkeep。
reg                        m_axis_read_data_tvalid_int; // 内部待输出 tvalid。
wire                       m_axis_read_data_tready_int; // 内部输出通路 ready。
reg                        m_axis_read_data_tlast_int; // 内部待输出 tlast。
reg  [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid_int; // 内部待输出 tid。
reg  [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest_int; // 内部待输出 tdest。
reg  [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser_int; // 内部待输出 tuser。

assign s_axis_read_desc_ready = s_axis_read_desc_ready_reg;

assign m_axis_read_desc_status_tag = m_axis_read_desc_status_tag_reg;
assign m_axis_read_desc_status_error = m_axis_read_desc_status_error_reg;
assign m_axis_read_desc_status_valid = m_axis_read_desc_status_valid_reg;

assign m_axi_arid = {AXI_ID_WIDTH{1'b0}};
assign m_axi_araddr = m_axi_araddr_reg;
assign m_axi_arlen = m_axi_arlen_reg;
assign m_axi_arsize = AXI_BURST_SIZE;
assign m_axi_arburst = 2'b01;
assign m_axi_arlock = 1'b0;
assign m_axi_arcache = 4'b0011;
assign m_axi_arprot = 3'b010;
assign m_axi_arvalid = m_axi_arvalid_reg;
assign m_axi_rready = m_axi_rready_reg;

always @* begin
    axi_state_next = AXI_STATE_IDLE;

    s_axis_read_desc_ready_next = 1'b0;

    m_axi_araddr_next = m_axi_araddr_reg;
    m_axi_arlen_next = m_axi_arlen_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;

    addr_next = addr_reg;
    op_word_count_next = op_word_count_reg;
    tr_word_count_next = tr_word_count_reg;

    axis_cmd_offset_next = axis_cmd_offset_reg;
    axis_cmd_last_cycle_offset_next = axis_cmd_last_cycle_offset_reg;
    axis_cmd_input_cycle_count_next = axis_cmd_input_cycle_count_reg;
    axis_cmd_output_cycle_count_next = axis_cmd_output_cycle_count_reg;
    axis_cmd_bubble_cycle_next = axis_cmd_bubble_cycle_reg;
    axis_cmd_tag_next = axis_cmd_tag_reg;
    axis_cmd_axis_id_next = axis_cmd_axis_id_reg;
    axis_cmd_axis_dest_next = axis_cmd_axis_dest_reg;
    axis_cmd_axis_user_next = axis_cmd_axis_user_reg;
    axis_cmd_valid_next = axis_cmd_valid_reg && !axis_cmd_ready;

    case (axi_state_reg)
        AXI_STATE_IDLE: begin
            // 空闲态：装载新描述符并启动传输
            s_axis_read_desc_ready_next = !axis_cmd_valid_reg && enable;

            if (s_axis_read_desc_ready && s_axis_read_desc_valid) begin
                if (ENABLE_UNALIGNED) begin
                    addr_next = s_axis_read_desc_addr;
                    axis_cmd_offset_next = AXI_STRB_WIDTH > 1 ? AXI_STRB_WIDTH - (s_axis_read_desc_addr & OFFSET_MASK) : 0;
                    axis_cmd_bubble_cycle_next = axis_cmd_offset_next > 0;
                    axis_cmd_last_cycle_offset_next = s_axis_read_desc_len & OFFSET_MASK;
                end else begin
                    addr_next = s_axis_read_desc_addr & ADDR_MASK;
                    axis_cmd_offset_next = 0;
                    axis_cmd_bubble_cycle_next = 1'b0;
                    axis_cmd_last_cycle_offset_next = s_axis_read_desc_len & OFFSET_MASK;
                end
                axis_cmd_tag_next = s_axis_read_desc_tag;
                op_word_count_next = s_axis_read_desc_len;

                axis_cmd_axis_id_next = s_axis_read_desc_id;
                axis_cmd_axis_dest_next = s_axis_read_desc_dest;
                axis_cmd_axis_user_next = s_axis_read_desc_user;

                if (ENABLE_UNALIGNED) begin
                    axis_cmd_input_cycle_count_next = (op_word_count_next + (s_axis_read_desc_addr & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                end else begin
                    axis_cmd_input_cycle_count_next = (op_word_count_next - 1) >> AXI_BURST_SIZE;
                end
                axis_cmd_output_cycle_count_next = (op_word_count_next - 1) >> AXI_BURST_SIZE;

                axis_cmd_valid_next = 1'b1;

                s_axis_read_desc_ready_next = 1'b0;
                axi_state_next = AXI_STATE_START;
            end else begin
                axi_state_next = AXI_STATE_IDLE;
            end
        end
        AXI_STATE_START: begin
            // 启动态：发起新的 AXI 读突发
            if (!m_axi_arvalid) begin
                if (op_word_count_reg <= AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK) || AXI_MAX_BURST_SIZE >= 4096) begin
                    // 剩余包长小于最大突发长度
                    if (((addr_reg & 12'hfff) + (op_word_count_reg & 12'hfff)) >> 12 != 0 || op_word_count_reg >> 12 != 0) begin
                        // 跨越 4K 边界
                        tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        tr_word_count_next = op_word_count_reg;
                    end
                end else begin
                    // 剩余包长大于最大突发长度
                    if (((addr_reg & 12'hfff) + AXI_MAX_BURST_SIZE) >> 12 != 0) begin
                        // 跨越 4K 边界
                        tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        tr_word_count_next = AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK);
                    end
                end

                m_axi_araddr_next = addr_reg;
                if (ENABLE_UNALIGNED) begin
                    m_axi_arlen_next = (tr_word_count_next + (addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                end else begin
                    m_axi_arlen_next = (tr_word_count_next - 1) >> AXI_BURST_SIZE;
                end
                m_axi_arvalid_next = 1'b1;

                addr_next = addr_reg + tr_word_count_next;
                op_word_count_next = op_word_count_reg - tr_word_count_next;

                if (op_word_count_next > 0) begin
                    axi_state_next = AXI_STATE_START;
                end else begin
                    s_axis_read_desc_ready_next = !axis_cmd_valid_reg && enable;
                    axi_state_next = AXI_STATE_IDLE;
                end
            end else begin
                axi_state_next = AXI_STATE_START;
            end
        end
    endcase
end

always @* begin
    axis_state_next = AXIS_STATE_IDLE;

    m_axis_read_desc_status_tag_next = m_axis_read_desc_status_tag_reg;
    m_axis_read_desc_status_error_next = m_axis_read_desc_status_error_reg;
    m_axis_read_desc_status_valid_next = 1'b0;

    m_axis_read_data_tdata_int = shift_axi_rdata;
    m_axis_read_data_tkeep_int = {AXIS_KEEP_WIDTH{1'b1}};
    m_axis_read_data_tlast_int = 1'b0;
    m_axis_read_data_tvalid_int = 1'b0;
    m_axis_read_data_tid_int = axis_id_reg;
    m_axis_read_data_tdest_int = axis_dest_reg;
    m_axis_read_data_tuser_int = axis_user_reg;

    m_axi_rready_next = 1'b0;

    transfer_in_save = 1'b0;
    axis_cmd_ready = 1'b0;

    offset_next = offset_reg;
    last_cycle_offset_next = last_cycle_offset_reg;
    input_cycle_count_next = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next = input_active_reg;
    output_active_next = output_active_reg;
    bubble_cycle_next = bubble_cycle_reg;
    first_cycle_next = first_cycle_reg;
    output_last_cycle_next = output_last_cycle_reg;

    tag_next = tag_reg;
    axis_id_next = axis_id_reg;
    axis_dest_next = axis_dest_reg;
    axis_user_next = axis_user_reg;

    if (m_axi_rready && m_axi_rvalid && (m_axi_rresp == AXI_RESP_SLVERR || m_axi_rresp == AXI_RESP_DECERR)) begin
        rresp_next = m_axi_rresp;
    end else begin
        rresp_next = rresp_reg;
    end

    case (axis_state_reg)
        AXIS_STATE_IDLE: begin
            // 空闲态：等待并装载新命令
            m_axi_rready_next = 1'b0;

            // 锁存本次传输参数
            if (ENABLE_UNALIGNED) begin
                offset_next = axis_cmd_offset_reg;
            end else begin
                offset_next = 0;
            end
            last_cycle_offset_next = axis_cmd_last_cycle_offset_reg;
            input_cycle_count_next = axis_cmd_input_cycle_count_reg;
            output_cycle_count_next = axis_cmd_output_cycle_count_reg;
            bubble_cycle_next = axis_cmd_bubble_cycle_reg;
            tag_next = axis_cmd_tag_reg;
            axis_id_next = axis_cmd_axis_id_reg;
            axis_dest_next = axis_cmd_axis_dest_reg;
            axis_user_next = axis_cmd_axis_user_reg;

            output_last_cycle_next = output_cycle_count_next == 0;
            input_active_next = 1'b1;
            output_active_next = 1'b1;
            first_cycle_next = 1'b1;

            if (axis_cmd_valid_reg) begin
                axis_cmd_ready = 1'b1;
                m_axi_rready_next = m_axis_read_data_tready_int;
                axis_state_next = AXIS_STATE_READ;
            end
        end
        AXIS_STATE_READ: begin
            // 处理 AXI 读返回数据
            m_axi_rready_next = m_axis_read_data_tready_int && input_active_reg;

            if ((m_axi_rready && m_axi_rvalid) || !input_active_reg) begin
                // 接收一拍 AXI 读数据
                transfer_in_save = m_axi_rready && m_axi_rvalid;

                if (ENABLE_UNALIGNED && first_cycle_reg && bubble_cycle_reg) begin
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next = input_cycle_count_reg > 0;
                    end
                    bubble_cycle_next = 1'b0;
                    first_cycle_next = 1'b0;

                    m_axi_rready_next = m_axis_read_data_tready_int && input_active_next;
                    axis_state_next = AXIS_STATE_READ;
                end else begin
                    // 更新输入/输出计数器
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next = input_cycle_count_reg > 0;
                    end
                    if (output_active_reg) begin
                        output_cycle_count_next = output_cycle_count_reg - 1;
                        output_active_next = output_cycle_count_reg > 0;
                    end
                    output_last_cycle_next = output_cycle_count_next == 0;
                    bubble_cycle_next = 1'b0;
                    first_cycle_next = 1'b0;

                    // 输出对齐后的读数据
                    m_axis_read_data_tdata_int = shift_axi_rdata;
                    m_axis_read_data_tkeep_int = {AXIS_KEEP_WIDTH_INT{1'b1}};
                    m_axis_read_data_tvalid_int = 1'b1;

                    if (output_last_cycle_reg) begin
                        // 无剩余数据，结束本次操作
                        if (last_cycle_offset_reg > 0) begin
                            m_axis_read_data_tkeep_int = {AXIS_KEEP_WIDTH_INT{1'b1}} >> (AXIS_KEEP_WIDTH_INT - last_cycle_offset_reg);
                        end
                        m_axis_read_data_tlast_int = 1'b1;

                        m_axis_read_desc_status_tag_next = tag_reg;
                        if (rresp_next == AXI_RESP_SLVERR) begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_AXI_RD_SLVERR;
                        end else if (rresp_next == AXI_RESP_DECERR) begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_AXI_RD_DECERR;
                        end else begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_NONE;
                        end
                        m_axis_read_desc_status_valid_next = 1'b1;

                        rresp_next = AXI_RESP_OKAY;

                        m_axi_rready_next = 1'b0;
                        axis_state_next = AXIS_STATE_IDLE;
                    end else begin
                        // 仍有后续拍，继续传输
                        m_axi_rready_next = m_axis_read_data_tready_int && input_active_next;
                        axis_state_next = AXIS_STATE_READ;
                    end
                end
            end else begin
                axis_state_next = AXIS_STATE_READ;
            end
        end
    endcase
end

always @(posedge clk) begin
    axi_state_reg <= axi_state_next;
    axis_state_reg <= axis_state_next;

    s_axis_read_desc_ready_reg <= s_axis_read_desc_ready_next;

    m_axis_read_desc_status_tag_reg <= m_axis_read_desc_status_tag_next;
    m_axis_read_desc_status_error_reg <= m_axis_read_desc_status_error_next;
    m_axis_read_desc_status_valid_reg <= m_axis_read_desc_status_valid_next;

    m_axi_araddr_reg <= m_axi_araddr_next;
    m_axi_arlen_reg <= m_axi_arlen_next;
    m_axi_arvalid_reg <= m_axi_arvalid_next;
    m_axi_rready_reg <= m_axi_rready_next;

    addr_reg <= addr_next;
    op_word_count_reg <= op_word_count_next;
    tr_word_count_reg <= tr_word_count_next;

    axis_cmd_offset_reg <= axis_cmd_offset_next;
    axis_cmd_last_cycle_offset_reg <= axis_cmd_last_cycle_offset_next;
    axis_cmd_input_cycle_count_reg <= axis_cmd_input_cycle_count_next;
    axis_cmd_output_cycle_count_reg <= axis_cmd_output_cycle_count_next;
    axis_cmd_bubble_cycle_reg <= axis_cmd_bubble_cycle_next;
    axis_cmd_tag_reg <= axis_cmd_tag_next;
    axis_cmd_axis_id_reg <= axis_cmd_axis_id_next;
    axis_cmd_axis_dest_reg <= axis_cmd_axis_dest_next;
    axis_cmd_axis_user_reg <= axis_cmd_axis_user_next;
    axis_cmd_valid_reg <= axis_cmd_valid_next;

    offset_reg <= offset_next;
    last_cycle_offset_reg <= last_cycle_offset_next;
    input_cycle_count_reg <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg <= input_active_next;
    output_active_reg <= output_active_next;
    bubble_cycle_reg <= bubble_cycle_next;
    first_cycle_reg <= first_cycle_next;
    output_last_cycle_reg <= output_last_cycle_next;
    rresp_reg <= rresp_next;

    tag_reg <= tag_next;
    axis_id_reg <= axis_id_next;
    axis_dest_reg <= axis_dest_next;
    axis_user_reg <= axis_user_next;

    if (transfer_in_save) begin
        save_axi_rdata_reg <= m_axi_rdata;
    end

    if (rst) begin
        axi_state_reg <= AXI_STATE_IDLE;
        axis_state_reg <= AXIS_STATE_IDLE;

        axis_cmd_valid_reg <= 1'b0;

        s_axis_read_desc_ready_reg <= 1'b0;

        m_axis_read_desc_status_valid_reg <= 1'b0;
        m_axi_arvalid_reg <= 1'b0;
        m_axi_rready_reg <= 1'b0;

        rresp_reg <= AXI_RESP_OKAY;
    end
end

// 输出数据通路逻辑
reg [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata_reg  = {AXIS_DATA_WIDTH{1'b0}}; // AXIS 输出寄存器 tdata。
reg [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep_reg  = {AXIS_KEEP_WIDTH{1'b0}}; // AXIS 输出寄存器 tkeep。
reg                       m_axis_read_data_tvalid_reg = 1'b0; // AXIS 输出寄存器 tvalid。
reg                       m_axis_read_data_tlast_reg  = 1'b0; // AXIS 输出寄存器 tlast。
reg [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid_reg    = {AXIS_ID_WIDTH{1'b0}}; // AXIS 输出寄存器 tid。
reg [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest_reg  = {AXIS_DEST_WIDTH{1'b0}}; // AXIS 输出寄存器 tdest。
reg [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser_reg  = {AXIS_USER_WIDTH{1'b0}}; // AXIS 输出寄存器 tuser。

reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_wr_ptr_reg = 0; // 输出 FIFO 写指针(含额外位区分满/空)。
reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_rd_ptr_reg = 0; // 输出 FIFO 读指针(含额外位区分满/空)。
reg out_fifo_half_full_reg = 1'b0; // 输出 FIFO 半满标志(对上游产生背压)。

wire out_fifo_full = out_fifo_wr_ptr_reg == (out_fifo_rd_ptr_reg ^ {1'b1, {OUTPUT_FIFO_ADDR_WIDTH{1'b0}}}); // 输出 FIFO 满标志。
wire out_fifo_empty = out_fifo_wr_ptr_reg == out_fifo_rd_ptr_reg; // 输出 FIFO 空标志。

(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_DATA_WIDTH-1:0] out_fifo_tdata[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tdata。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_KEEP_WIDTH-1:0] out_fifo_tkeep[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tkeep。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg                       out_fifo_tlast[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tlast。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_ID_WIDTH-1:0]   out_fifo_tid[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tid。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_DEST_WIDTH-1:0] out_fifo_tdest[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tdest。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_USER_WIDTH-1:0] out_fifo_tuser[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：tuser。

assign m_axis_read_data_tready_int = !out_fifo_half_full_reg;

assign m_axis_read_data_tdata  = m_axis_read_data_tdata_reg;
assign m_axis_read_data_tkeep  = AXIS_KEEP_ENABLE ? m_axis_read_data_tkeep_reg : {AXIS_KEEP_WIDTH{1'b1}};
assign m_axis_read_data_tvalid = m_axis_read_data_tvalid_reg;
assign m_axis_read_data_tlast  = AXIS_LAST_ENABLE ? m_axis_read_data_tlast_reg : 1'b1;
assign m_axis_read_data_tid    = AXIS_ID_ENABLE   ? m_axis_read_data_tid_reg   : {AXIS_ID_WIDTH{1'b0}};
assign m_axis_read_data_tdest  = AXIS_DEST_ENABLE ? m_axis_read_data_tdest_reg : {AXIS_DEST_WIDTH{1'b0}};
assign m_axis_read_data_tuser  = AXIS_USER_ENABLE ? m_axis_read_data_tuser_reg : {AXIS_USER_WIDTH{1'b0}};

always @(posedge clk) begin
    m_axis_read_data_tvalid_reg <= m_axis_read_data_tvalid_reg && !m_axis_read_data_tready;

    out_fifo_half_full_reg <= $unsigned(out_fifo_wr_ptr_reg - out_fifo_rd_ptr_reg) >= 2**(OUTPUT_FIFO_ADDR_WIDTH-1);

    if (!out_fifo_full && m_axis_read_data_tvalid_int) begin
        out_fifo_tdata[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tdata_int;
        out_fifo_tkeep[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tkeep_int;
        out_fifo_tlast[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tlast_int;
        out_fifo_tid[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tid_int;
        out_fifo_tdest[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tdest_int;
        out_fifo_tuser[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tuser_int;
        out_fifo_wr_ptr_reg <= out_fifo_wr_ptr_reg + 1;
    end

    if (!out_fifo_empty && (!m_axis_read_data_tvalid_reg || m_axis_read_data_tready)) begin
        m_axis_read_data_tdata_reg <= out_fifo_tdata[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tkeep_reg <= out_fifo_tkeep[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tvalid_reg <= 1'b1;
        m_axis_read_data_tlast_reg <= out_fifo_tlast[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tid_reg <= out_fifo_tid[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tdest_reg <= out_fifo_tdest[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tuser_reg <= out_fifo_tuser[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        out_fifo_rd_ptr_reg <= out_fifo_rd_ptr_reg + 1;
    end

    if (rst) begin
        out_fifo_wr_ptr_reg <= 0;
        out_fifo_rd_ptr_reg <= 0;
        m_axis_read_data_tvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
