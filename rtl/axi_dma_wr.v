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
 * AXI4 DMA 写引擎
 *
 * 模块目录
 * 1) 接收写描述符与 AXIS 输入流，生成 AXI AW/W/B 完成内存写入。
 * 2) 处理未对齐写、tkeep 到 wstrb 映射、末拍/补拍控制和错误聚合。
 * 3) 通过状态 FIFO 把完成信息按描述符顺序回送到状态输出。
 */
module axi_dma_wr #
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
    input  wire                       clk, // 写 DMA 时钟。
    input  wire                       rst, // 同步复位，高电平有效。

    /*
     * AXI 写描述符输入
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_write_desc_addr, // 写描述符目的地址。
    input  wire [LEN_WIDTH-1:0]       s_axis_write_desc_len, // 写描述符长度。
    input  wire [TAG_WIDTH-1:0]       s_axis_write_desc_tag, // 写描述符 tag。
    input  wire                       s_axis_write_desc_valid, // 写描述符有效。
    output wire                       s_axis_write_desc_ready, // 模块可接收写描述符。

    /*
     * AXI 写描述符状态输出
     */
    output wire [LEN_WIDTH-1:0]       m_axis_write_desc_status_len, // 写完成状态长度。
    output wire [TAG_WIDTH-1:0]       m_axis_write_desc_status_tag, // 写完成状态 tag。
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_write_desc_status_id, // 写完成状态 tid。
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest, // 写完成状态 tdest。
    output wire [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user, // 写完成状态 tuser。
    output wire [3:0]                 m_axis_write_desc_status_error, // 写完成状态错误码。
    output wire                       m_axis_write_desc_status_valid, // 写完成状态有效。

    /*
     * AXI-Stream 写数据输入
     */
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_write_data_tdata, // AXIS 输入 tdata。
    input  wire [AXIS_KEEP_WIDTH-1:0] s_axis_write_data_tkeep, // AXIS 输入 tkeep。
    input  wire                       s_axis_write_data_tvalid, // AXIS 输入 tvalid。
    output wire                       s_axis_write_data_tready, // AXIS 输入 tready。
    input  wire                       s_axis_write_data_tlast, // AXIS 输入 tlast。
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_write_data_tid, // AXIS 输入 tid。
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_write_data_tdest, // AXIS 输入 tdest。
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_write_data_tuser, // AXIS 输入 tuser。

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid, // AXI 写地址 ID。
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr, // AXI 写地址。
    output wire [7:0]                 m_axi_awlen, // AXI 写突发长度。
    output wire [2:0]                 m_axi_awsize, // AXI 写 beat 大小。
    output wire [1:0]                 m_axi_awburst, // AXI 写突发类型。
    output wire                       m_axi_awlock, // AXI 写锁属性。
    output wire [3:0]                 m_axi_awcache, // AXI 写 cache 属性。
    output wire [2:0]                 m_axi_awprot, // AXI 写保护属性。
    output wire                       m_axi_awvalid, // AXI 写地址有效。
    input  wire                       m_axi_awready, // AXI 写地址 ready。
    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata, // AXI 写数据。
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb, // AXI 写字节使能。
    output wire                       m_axi_wlast, // AXI 写最后一拍。
    output wire                       m_axi_wvalid, // AXI 写数据有效。
    input  wire                       m_axi_wready, // AXI 写数据 ready。
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid, // AXI 写响应 ID。
    input  wire [1:0]                 m_axi_bresp, // AXI 写响应状态。
    input  wire                       m_axi_bvalid, // AXI 写响应有效。
    output wire                       m_axi_bready, // AXI 写响应 ready。

    /*
     * 配置
     */
    input  wire                       enable, // 写 DMA 总使能。
    input  wire                       abort // 写 DMA 中止请求。
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

parameter STATUS_FIFO_ADDR_WIDTH = 5;
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

localparam [2:0]
    STATE_IDLE = 3'd0,
    STATE_START = 3'd1,
    STATE_WRITE = 3'd2,
    STATE_FINISH_BURST = 3'd3,
    STATE_DROP_DATA = 3'd4;

reg [2:0] state_reg = STATE_IDLE, state_next; // 写 DMA 主状态机。

// 数据通路控制信号
reg transfer_in_save; // 当前拍是否把 AXIS 输入缓存到 save 寄存器。
reg flush_save; // 是否在末尾把 save 寄存器剩余数据刷出。
reg status_fifo_we; // 状态 FIFO 写使能。

integer i; // 组合逻辑中按字节展开循环索引。
reg [OFFSET_WIDTH:0] cycle_size; // 本次传输每拍有效字节数估计值。

reg [AXI_ADDR_WIDTH-1:0] addr_reg = {AXI_ADDR_WIDTH{1'b0}}, addr_next; // 当前 AXI 写地址游标。
reg [LEN_WIDTH-1:0] op_word_count_reg = {LEN_WIDTH{1'b0}}, op_word_count_next; // 当前描述符剩余总字数。
reg [LEN_WIDTH-1:0] tr_word_count_reg = {LEN_WIDTH{1'b0}}, tr_word_count_next; // 当前突发剩余字数。

reg [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next; // 当前写偏移(未对齐写处理)。
reg [AXI_STRB_WIDTH-1:0] strb_offset_mask_reg = {AXI_STRB_WIDTH{1'b1}}, strb_offset_mask_next; // 首拍/末拍掩码模板。
reg zero_offset_reg = 1'b1, zero_offset_next; // 当前偏移是否为 0。
reg [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next; // 末拍偏移。
reg [LEN_WIDTH-1:0] length_reg = {LEN_WIDTH{1'b0}}, length_next; // 当前描述符长度缓存。
reg [CYCLE_COUNT_WIDTH-1:0] input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, input_cycle_count_next; // 剩余待消费输入拍数。
reg [CYCLE_COUNT_WIDTH-1:0] output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, output_cycle_count_next; // 剩余待输出 AXI W 拍数。
reg input_active_reg = 1'b0, input_active_next; // 输入侧是否活跃。
reg first_cycle_reg = 1'b0, first_cycle_next; // 当前是否首拍。
reg input_last_cycle_reg = 1'b0, input_last_cycle_next; // 输入侧是否最后一拍。
reg output_last_cycle_reg = 1'b0, output_last_cycle_next; // 输出侧是否最后一拍。
reg last_transfer_reg = 1'b0, last_transfer_next; // 当前 burst 是否为描述符最后一次传输。
reg [1:0] bresp_reg = AXI_RESP_OKAY, bresp_next; // 累积写响应状态。

reg [TAG_WIDTH-1:0] tag_reg = {TAG_WIDTH{1'b0}}, tag_next; // 当前描述符 tag 缓存。
reg [AXIS_ID_WIDTH-1:0] axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_id_next; // 当前描述符 tid 缓存。
reg [AXIS_DEST_WIDTH-1:0] axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_dest_next; // 当前描述符 tdest 缓存。
reg [AXIS_USER_WIDTH-1:0] axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_user_next; // 当前描述符 tuser 缓存。

reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_wr_ptr_reg = 0; // 状态 FIFO 写指针。
reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_rd_ptr_reg = 0, status_fifo_rd_ptr_next; // 状态 FIFO 读指针。
reg [LEN_WIDTH-1:0] status_fifo_len[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：长度字段。
reg [TAG_WIDTH-1:0] status_fifo_tag[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：tag 字段。
reg [AXIS_ID_WIDTH-1:0] status_fifo_id[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：tid 字段。
reg [AXIS_DEST_WIDTH-1:0] status_fifo_dest[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：tdest 字段。
reg [AXIS_USER_WIDTH-1:0] status_fifo_user[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：tuser 字段。
reg status_fifo_last[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：该项是否最后一次传输标志。
reg [LEN_WIDTH-1:0] status_fifo_wr_len; // 本拍待写入状态 FIFO 的长度。
reg [TAG_WIDTH-1:0] status_fifo_wr_tag; // 本拍待写入状态 FIFO 的 tag。
reg [AXIS_ID_WIDTH-1:0] status_fifo_wr_id; // 本拍待写入状态 FIFO 的 tid。
reg [AXIS_DEST_WIDTH-1:0] status_fifo_wr_dest; // 本拍待写入状态 FIFO 的 tdest。
reg [AXIS_USER_WIDTH-1:0] status_fifo_wr_user; // 本拍待写入状态 FIFO 的 tuser。
reg status_fifo_wr_last; // 本拍待写入状态 FIFO 的 last 标志。

reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] active_count_reg = 0; // 在途写事务计数。
reg active_count_av_reg = 1'b1; // 在途计数是否低于可接收上限。
reg inc_active; // 在途计数加一事件。
reg dec_active; // 在途计数减一事件。

reg s_axis_write_desc_ready_reg = 1'b0, s_axis_write_desc_ready_next; // 描述符输入 ready 寄存器。

reg [LEN_WIDTH-1:0] m_axis_write_desc_status_len_reg = {LEN_WIDTH{1'b0}}, m_axis_write_desc_status_len_next; // 状态输出长度寄存器。
reg [TAG_WIDTH-1:0] m_axis_write_desc_status_tag_reg = {TAG_WIDTH{1'b0}}, m_axis_write_desc_status_tag_next; // 状态输出 tag 寄存器。
reg [AXIS_ID_WIDTH-1:0] m_axis_write_desc_status_id_reg = {AXIS_ID_WIDTH{1'b0}}, m_axis_write_desc_status_id_next; // 状态输出 tid 寄存器。
reg [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, m_axis_write_desc_status_dest_next; // 状态输出 tdest 寄存器。
reg [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user_reg = {AXIS_USER_WIDTH{1'b0}}, m_axis_write_desc_status_user_next; // 状态输出 tuser 寄存器。
reg [3:0] m_axis_write_desc_status_error_reg = 4'd0, m_axis_write_desc_status_error_next; // 状态输出错误码寄存器。
reg m_axis_write_desc_status_valid_reg = 1'b0, m_axis_write_desc_status_valid_next; // 状态输出 valid 寄存器。

reg [AXI_ADDR_WIDTH-1:0] m_axi_awaddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_awaddr_next; // AW 地址寄存器。
reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next; // AW 长度寄存器。
reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next; // AWVALID 寄存器。
reg m_axi_bready_reg = 1'b0, m_axi_bready_next; // BREADY 寄存器。

reg s_axis_write_data_tready_reg = 1'b0, s_axis_write_data_tready_next; // AXIS 输入 tready 寄存器。

reg [AXIS_DATA_WIDTH-1:0] save_axis_tdata_reg = {AXIS_DATA_WIDTH{1'b0}}; // 未对齐拼接时缓存上一拍 tdata。
reg [AXIS_KEEP_WIDTH_INT-1:0] save_axis_tkeep_reg = {AXIS_KEEP_WIDTH_INT{1'b0}}; // 未对齐拼接时缓存上一拍 tkeep。
reg save_axis_tlast_reg = 1'b0; // 未对齐拼接时缓存上一拍 tlast。

reg [AXIS_DATA_WIDTH-1:0] shift_axis_tdata; // 对齐重排后的 tdata。
reg [AXIS_KEEP_WIDTH_INT-1:0] shift_axis_tkeep; // 对齐重排后的 tkeep。
reg shift_axis_tvalid; // 对齐重排后的 tvalid。
reg shift_axis_tlast; // 对齐重排后的 tlast。
reg shift_axis_input_tready; // 重排逻辑对原始输入的 ready。
reg shift_axis_extra_cycle_reg = 1'b0; // 未对齐尾部是否需要额外补拍。

// 内部数据通路
reg  [AXI_DATA_WIDTH-1:0] m_axi_wdata_int; // 内部待输出 AXI WDATA。
reg  [AXI_STRB_WIDTH-1:0] m_axi_wstrb_int; // 内部待输出 AXI WSTRB。
reg                       m_axi_wlast_int; // 内部待输出 AXI WLAST。
reg                       m_axi_wvalid_int; // 内部待输出 AXI WVALID。
wire                      m_axi_wready_int; // 内部输出通路 ready。

assign s_axis_write_desc_ready = s_axis_write_desc_ready_reg;

assign m_axis_write_desc_status_len = m_axis_write_desc_status_len_reg;
assign m_axis_write_desc_status_tag = m_axis_write_desc_status_tag_reg;
assign m_axis_write_desc_status_id = m_axis_write_desc_status_id_reg;
assign m_axis_write_desc_status_dest = m_axis_write_desc_status_dest_reg;
assign m_axis_write_desc_status_user = m_axis_write_desc_status_user_reg;
assign m_axis_write_desc_status_error = m_axis_write_desc_status_error_reg;
assign m_axis_write_desc_status_valid = m_axis_write_desc_status_valid_reg;

assign s_axis_write_data_tready = s_axis_write_data_tready_reg;

assign m_axi_awid = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awaddr = m_axi_awaddr_reg;
assign m_axi_awlen = m_axi_awlen_reg;
assign m_axi_awsize = AXI_BURST_SIZE;
assign m_axi_awburst = 2'b01;
assign m_axi_awlock = 1'b0;
assign m_axi_awcache = 4'b0011;
assign m_axi_awprot = 3'b010;
assign m_axi_awvalid = m_axi_awvalid_reg;
assign m_axi_bready = m_axi_bready_reg;

always @* begin
    if (!ENABLE_UNALIGNED || zero_offset_reg) begin
        // 无拼接重叠时直接透传
        shift_axis_tdata = s_axis_write_data_tdata;
        shift_axis_tkeep = s_axis_write_data_tkeep;
        shift_axis_tvalid = s_axis_write_data_tvalid;
        shift_axis_tlast = AXIS_LAST_ENABLE && s_axis_write_data_tlast;
        shift_axis_input_tready = 1'b1;
    end else if (!AXIS_LAST_ENABLE) begin
        shift_axis_tdata = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep = {s_axis_write_data_tkeep, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid = s_axis_write_data_tvalid;
        shift_axis_tlast = 1'b0;
        shift_axis_input_tready = 1'b1;
    end else if (shift_axis_extra_cycle_reg) begin
        shift_axis_tdata = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep = {{AXIS_KEEP_WIDTH_INT{1'b0}}, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid = 1'b1;
        shift_axis_tlast = save_axis_tlast_reg;
        shift_axis_input_tready = flush_save;
    end else begin
        shift_axis_tdata = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep = {s_axis_write_data_tkeep, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid = s_axis_write_data_tvalid;
        shift_axis_tlast = (s_axis_write_data_tlast && ((s_axis_write_data_tkeep & ({AXIS_KEEP_WIDTH_INT{1'b1}} << (AXIS_KEEP_WIDTH_INT-offset_reg))) == 0));
        shift_axis_input_tready = !(s_axis_write_data_tlast && s_axis_write_data_tready && s_axis_write_data_tvalid);
    end
end

always @* begin
    state_next = STATE_IDLE;

    s_axis_write_desc_ready_next = 1'b0;

    m_axis_write_desc_status_len_next = m_axis_write_desc_status_len_reg;
    m_axis_write_desc_status_tag_next = m_axis_write_desc_status_tag_reg;
    m_axis_write_desc_status_id_next = m_axis_write_desc_status_id_reg;
    m_axis_write_desc_status_dest_next = m_axis_write_desc_status_dest_reg;
    m_axis_write_desc_status_user_next = m_axis_write_desc_status_user_reg;
    m_axis_write_desc_status_error_next = m_axis_write_desc_status_error_reg;
    m_axis_write_desc_status_valid_next = 1'b0;

    s_axis_write_data_tready_next = 1'b0;

    m_axi_awaddr_next = m_axi_awaddr_reg;
    m_axi_awlen_next = m_axi_awlen_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;
    m_axi_wdata_int = shift_axis_tdata;
    m_axi_wstrb_int = shift_axis_tkeep;
    m_axi_wlast_int = 1'b0;
    m_axi_wvalid_int = 1'b0;
    m_axi_bready_next = 1'b0;

    transfer_in_save = 1'b0;
    flush_save = 1'b0;
    status_fifo_we = 1'b0;

    cycle_size = AXIS_KEEP_WIDTH_INT;

    addr_next = addr_reg;
    offset_next = offset_reg;
    strb_offset_mask_next = strb_offset_mask_reg;
    zero_offset_next = zero_offset_reg;
    last_cycle_offset_next = last_cycle_offset_reg;
    length_next = length_reg;
    op_word_count_next = op_word_count_reg;
    tr_word_count_next = tr_word_count_reg;
    input_cycle_count_next = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next = input_active_reg;
    first_cycle_next = first_cycle_reg;
    input_last_cycle_next = input_last_cycle_reg;
    output_last_cycle_next = output_last_cycle_reg;
    last_transfer_next = last_transfer_reg;

    status_fifo_rd_ptr_next = status_fifo_rd_ptr_reg;

    inc_active = 1'b0;
    dec_active = 1'b0;

    tag_next = tag_reg;
    axis_id_next = axis_id_reg;
    axis_dest_next = axis_dest_reg;
    axis_user_next = axis_user_reg;

    status_fifo_wr_len = length_reg;
    status_fifo_wr_tag = tag_reg;
    status_fifo_wr_id = axis_id_reg;
    status_fifo_wr_dest = axis_dest_reg;
    status_fifo_wr_user = axis_user_reg;
    status_fifo_wr_last = 1'b0;

    if (m_axi_bready && m_axi_bvalid && (m_axi_bresp == AXI_RESP_SLVERR || m_axi_bresp == AXI_RESP_DECERR)) begin
        bresp_next = m_axi_bresp;
    end else begin
        bresp_next = bresp_reg;
    end

    case (state_reg)
        STATE_IDLE: begin
            // 空闲态：装载新描述符并启动传输
            flush_save = 1'b1;
            s_axis_write_desc_ready_next = enable && active_count_av_reg;

            if (ENABLE_UNALIGNED) begin
                addr_next = s_axis_write_desc_addr;
                offset_next = s_axis_write_desc_addr & OFFSET_MASK;
                strb_offset_mask_next = {AXI_STRB_WIDTH{1'b1}} << (s_axis_write_desc_addr & OFFSET_MASK);
                zero_offset_next = (s_axis_write_desc_addr & OFFSET_MASK) == 0;
                last_cycle_offset_next = offset_next + (s_axis_write_desc_len & OFFSET_MASK);
            end else begin
                addr_next = s_axis_write_desc_addr & ADDR_MASK;
                offset_next = 0;
                strb_offset_mask_next = {AXI_STRB_WIDTH{1'b1}};
                zero_offset_next = 1'b1;
                last_cycle_offset_next = offset_next + (s_axis_write_desc_len & OFFSET_MASK);
            end
            tag_next = s_axis_write_desc_tag;
            op_word_count_next = s_axis_write_desc_len;
            first_cycle_next = 1'b1;
            length_next = 0;

            if (s_axis_write_desc_ready && s_axis_write_desc_valid) begin
                s_axis_write_desc_ready_next = 1'b0;
                state_next = STATE_START;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_START: begin
            // 启动态：发起新的 AXI 写突发
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

            input_cycle_count_next = (tr_word_count_next - 1) >> $clog2(AXIS_KEEP_WIDTH_INT);
            input_last_cycle_next = input_cycle_count_next == 0;
            if (ENABLE_UNALIGNED) begin
                output_cycle_count_next = (tr_word_count_next + (addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
            end else begin
                output_cycle_count_next = (tr_word_count_next - 1) >> AXI_BURST_SIZE;
            end
            output_last_cycle_next = output_cycle_count_next == 0;
            last_transfer_next = tr_word_count_next == op_word_count_reg;
            input_active_next = 1'b1;

            if (ENABLE_UNALIGNED) begin
                if (!first_cycle_reg && last_transfer_next) begin
                    if (offset_reg >= last_cycle_offset_reg && last_cycle_offset_reg > 0) begin
                    // 最后一拍由缓存的部分数据补齐
                        input_active_next = input_cycle_count_next > 0;
                        input_cycle_count_next = input_cycle_count_next - 1;
                    end
                end
            end

            if (!m_axi_awvalid_reg && active_count_av_reg) begin
                m_axi_awaddr_next = addr_reg;
                m_axi_awlen_next = output_cycle_count_next;
                m_axi_awvalid_next = s_axis_write_data_tvalid || !first_cycle_reg;

                if (m_axi_awvalid_next) begin
                    addr_next = addr_reg + tr_word_count_next;
                    op_word_count_next = op_word_count_reg - tr_word_count_next;

                    s_axis_write_data_tready_next = m_axi_wready_int && input_active_next;

                    inc_active = 1'b1;

                    state_next = STATE_WRITE;
                end else begin
                    state_next = STATE_START;
                end
            end else begin
                state_next = STATE_START;
            end
        end
        STATE_WRITE: begin
            s_axis_write_data_tready_next = m_axi_wready_int && (last_transfer_reg || input_active_reg) && shift_axis_input_tready;

            if ((s_axis_write_data_tready && shift_axis_tvalid) || (!input_active_reg && !last_transfer_reg) || !shift_axis_input_tready) begin
                if (s_axis_write_data_tready && s_axis_write_data_tvalid) begin
                    transfer_in_save = 1'b1;

                    axis_id_next = s_axis_write_data_tid;
                    axis_dest_next = s_axis_write_data_tdest;
                    axis_user_next = s_axis_write_data_tuser;
                end

                // 更新输入/输出计数器
                if (first_cycle_reg) begin
                    length_next = length_reg + (AXIS_KEEP_WIDTH_INT - offset_reg);
                end else begin
                    length_next = length_reg + AXIS_KEEP_WIDTH_INT;
                end
                if (input_active_reg) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next = input_cycle_count_reg > 0;
                end
                input_last_cycle_next = input_cycle_count_next == 0;
                output_cycle_count_next = output_cycle_count_reg - 1;
                output_last_cycle_next = output_cycle_count_next == 0;
                first_cycle_next = 1'b0;
                strb_offset_mask_next = {AXI_STRB_WIDTH{1'b1}};

                m_axi_wdata_int = shift_axis_tdata;
                m_axi_wstrb_int = strb_offset_mask_reg;
                m_axi_wvalid_int = 1'b1;

                if (AXIS_LAST_ENABLE && s_axis_write_data_tlast) begin
                    // 到达输入帧结束
                    input_active_next = 1'b0;
                    s_axis_write_data_tready_next = 1'b0;
                end

                if (AXIS_LAST_ENABLE && shift_axis_tlast) begin
                    // 到达当前数据包结束

                    if (AXIS_KEEP_ENABLE) begin
                        cycle_size = AXIS_KEEP_WIDTH_INT;
                        for (i = AXIS_KEEP_WIDTH_INT-1; i >= 0; i = i - 1) begin
                            if (~shift_axis_tkeep & strb_offset_mask_reg & (1 << i)) begin
                                cycle_size = i;
                            end
                        end
                    end else begin
                        cycle_size = AXIS_KEEP_WIDTH_INT;
                    end

                    if (output_last_cycle_reg) begin
                        m_axi_wlast_int = 1'b1;

                        // 无剩余数据，结束本次操作
                        if (last_transfer_reg && last_cycle_offset_reg > 0) begin
                            if (AXIS_KEEP_ENABLE && !(shift_axis_tkeep & ~({AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg)))) begin
                                m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (cycle_size - offset_reg);
                                end else begin
                                    length_next = length_reg + cycle_size;
                                end
                            end else begin
                                m_axi_wstrb_int = strb_offset_mask_reg & {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg);
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (last_cycle_offset_reg - offset_reg);
                                end else begin
                                    length_next = length_reg + last_cycle_offset_reg;
                                end
                            end
                        end else begin
                            if (AXIS_KEEP_ENABLE) begin
                                m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (cycle_size - offset_reg);
                                end else begin
                                    length_next = length_reg + cycle_size;
                                end
                            end
                        end

                        // 向状态 FIFO 写入完成记录
                        status_fifo_we = 1'b1;
                        status_fifo_wr_len = length_next;
                        status_fifo_wr_tag = tag_reg;
                        status_fifo_wr_id = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1;

                        s_axis_write_data_tready_next = 1'b0;
                        s_axis_write_desc_ready_next = enable && active_count_av_reg;
                        state_next = STATE_IDLE;
                    end else begin
                        // 当前 burst 仍有剩余拍，先收尾
                        if (AXIS_KEEP_ENABLE) begin
                            m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                            if (first_cycle_reg) begin
                                length_next = length_reg + (cycle_size - offset_reg);
                            end else begin
                                length_next = length_reg + cycle_size;
                            end
                        end

                        // 向状态 FIFO 写入完成记录
                        status_fifo_we = 1'b1;
                        status_fifo_wr_len = length_next;
                        status_fifo_wr_tag = tag_reg;
                        status_fifo_wr_id = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1;

                        s_axis_write_data_tready_next = 1'b0;
                        state_next = STATE_FINISH_BURST;
                    end

                end else if (output_last_cycle_reg) begin
                    m_axi_wlast_int = 1'b1;

                    if (op_word_count_reg > 0) begin
                        // 当前 AXI 传输完成，但描述符仍有剩余数据
                        // 向状态 FIFO 写入完成记录
                        status_fifo_we = 1'b1;
                        status_fifo_wr_len = length_next;
                        status_fifo_wr_tag = tag_reg;
                        status_fifo_wr_id = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b0;

                        s_axis_write_data_tready_next = 1'b0;
                        state_next = STATE_START;
                    end else begin
                        // 无剩余数据，结束本次操作
                        if (last_cycle_offset_reg > 0) begin
                            m_axi_wstrb_int = strb_offset_mask_reg & {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg);
                            if (first_cycle_reg) begin
                                length_next = length_reg + (last_cycle_offset_reg - offset_reg);
                            end else begin
                                length_next = length_reg + last_cycle_offset_reg;
                            end
                        end

                        // 向状态 FIFO 写入完成记录
                        status_fifo_we = 1'b1;
                        status_fifo_wr_len = length_next;
                        status_fifo_wr_tag = tag_reg;
                        status_fifo_wr_id = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1;

                        if (AXIS_LAST_ENABLE) begin
                            // 未到包尾，丢弃剩余输入数据
                            s_axis_write_data_tready_next = shift_axis_input_tready;
                            state_next = STATE_DROP_DATA;
                        end else begin
                            // 无帧边界模式，直接回到空闲态
                            s_axis_write_data_tready_next = 1'b0;
                            s_axis_write_desc_ready_next = enable && active_count_av_reg;
                            state_next = STATE_IDLE;
                        end
                    end
                end else begin
                    s_axis_write_data_tready_next = m_axi_wready_int && (last_transfer_reg || input_active_next) && shift_axis_input_tready;
                    state_next = STATE_WRITE;
                end
            end else begin
                state_next = STATE_WRITE;
            end
        end
        STATE_FINISH_BURST: begin
            // 收尾当前 AXI burst

            if (m_axi_wready_int) begin
                // 更新计数器
                if (input_active_reg) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next = input_cycle_count_reg > 0;
                end
                input_last_cycle_next = input_cycle_count_next == 0;
                output_cycle_count_next = output_cycle_count_reg - 1;
                output_last_cycle_next = output_cycle_count_next == 0;

                m_axi_wdata_int = {AXI_DATA_WIDTH{1'b0}};
                m_axi_wstrb_int = {AXI_STRB_WIDTH{1'b0}};
                m_axi_wvalid_int = 1'b1;

                if (output_last_cycle_reg) begin
                    // 无剩余数据，结束本次操作
                    m_axi_wlast_int = 1'b1;

                    s_axis_write_data_tready_next = 1'b0;
                    s_axis_write_desc_ready_next = enable && active_count_av_reg;
                    state_next = STATE_IDLE;
                end else begin
                    // AXI 传输仍有后续拍
                    state_next = STATE_FINISH_BURST;
                end
            end else begin
                state_next = STATE_FINISH_BURST;
            end
        end
        STATE_DROP_DATA: begin
            // 丢弃多余的 AXI-Stream 输入数据
            s_axis_write_data_tready_next = shift_axis_input_tready;

            if (shift_axis_tvalid) begin
                if (s_axis_write_data_tready && s_axis_write_data_tvalid) begin
                    transfer_in_save = 1'b1;
                end

                if (shift_axis_tlast) begin
                    s_axis_write_data_tready_next = 1'b0;
                    s_axis_write_desc_ready_next = enable && active_count_av_reg;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_DROP_DATA;
                end
            end else begin
                state_next = STATE_DROP_DATA;
            end
        end
    endcase

    if (status_fifo_rd_ptr_reg != status_fifo_wr_ptr_reg) begin
        // 状态 FIFO 非空
        if (m_axi_bready && m_axi_bvalid) begin
            // 收到写完成响应，出队并回传状态
            m_axis_write_desc_status_len_next = status_fifo_len[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_tag_next = status_fifo_tag[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_id_next = status_fifo_id[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_dest_next = status_fifo_dest[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_user_next = status_fifo_user[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            if (bresp_next == AXI_RESP_SLVERR) begin
                m_axis_write_desc_status_error_next = DMA_ERROR_AXI_WR_SLVERR;
            end else if (bresp_next == AXI_RESP_DECERR) begin
                m_axis_write_desc_status_error_next = DMA_ERROR_AXI_WR_DECERR;
            end else begin
                m_axis_write_desc_status_error_next = DMA_ERROR_NONE;
            end
            m_axis_write_desc_status_valid_next = status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            status_fifo_rd_ptr_next = status_fifo_rd_ptr_reg + 1;
            m_axi_bready_next = 1'b0;

            if (status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]) begin
                bresp_next = AXI_RESP_OKAY;
            end

            dec_active = 1'b1;
        end else begin
            // 等待写完成响应
            m_axi_bready_next = 1'b1;
        end
    end
end

always @(posedge clk) begin
    state_reg <= state_next;

    s_axis_write_desc_ready_reg <= s_axis_write_desc_ready_next;

    m_axis_write_desc_status_len_reg <= m_axis_write_desc_status_len_next;
    m_axis_write_desc_status_tag_reg <= m_axis_write_desc_status_tag_next;
    m_axis_write_desc_status_id_reg <= m_axis_write_desc_status_id_next;
    m_axis_write_desc_status_dest_reg <= m_axis_write_desc_status_dest_next;
    m_axis_write_desc_status_user_reg <= m_axis_write_desc_status_user_next;
    m_axis_write_desc_status_error_reg <= m_axis_write_desc_status_error_next;
    m_axis_write_desc_status_valid_reg <= m_axis_write_desc_status_valid_next;

    s_axis_write_data_tready_reg <= s_axis_write_data_tready_next;

    m_axi_awaddr_reg <= m_axi_awaddr_next;
    m_axi_awlen_reg <= m_axi_awlen_next;
    m_axi_awvalid_reg <= m_axi_awvalid_next;
    m_axi_bready_reg <= m_axi_bready_next;

    addr_reg <= addr_next;
    offset_reg <= offset_next;
    strb_offset_mask_reg <= strb_offset_mask_next;
    zero_offset_reg <= zero_offset_next;
    last_cycle_offset_reg <= last_cycle_offset_next;
    length_reg <= length_next;
    op_word_count_reg <= op_word_count_next;
    tr_word_count_reg <= tr_word_count_next;
    input_cycle_count_reg <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg <= input_active_next;
    first_cycle_reg <= first_cycle_next;
    input_last_cycle_reg <= input_last_cycle_next;
    output_last_cycle_reg <= output_last_cycle_next;
    last_transfer_reg <= last_transfer_next;
    bresp_reg <= bresp_next;

    tag_reg <= tag_next;
    axis_id_reg <= axis_id_next;
    axis_dest_reg <= axis_dest_next;
    axis_user_reg <= axis_user_next;

    // 数据通路寄存
    if (flush_save) begin
        save_axis_tkeep_reg <= {AXIS_KEEP_WIDTH_INT{1'b0}};
        save_axis_tlast_reg <= 1'b0;
        shift_axis_extra_cycle_reg <= 1'b0;
    end else if (transfer_in_save) begin
        save_axis_tdata_reg <= s_axis_write_data_tdata;
        save_axis_tkeep_reg <= AXIS_KEEP_ENABLE ? s_axis_write_data_tkeep : {AXIS_KEEP_WIDTH_INT{1'b1}};
        save_axis_tlast_reg <= s_axis_write_data_tlast;
        shift_axis_extra_cycle_reg <= s_axis_write_data_tlast & ((s_axis_write_data_tkeep >> (AXIS_KEEP_WIDTH_INT-offset_reg)) != 0);
    end

    if (status_fifo_we) begin
        status_fifo_len[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_len;
        status_fifo_tag[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_tag;
        status_fifo_id[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_id;
        status_fifo_dest[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_dest;
        status_fifo_user[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_user;
        status_fifo_last[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_last;
        status_fifo_wr_ptr_reg <= status_fifo_wr_ptr_reg + 1;
    end
    status_fifo_rd_ptr_reg <= status_fifo_rd_ptr_next;

    if (active_count_reg < 2**STATUS_FIFO_ADDR_WIDTH && inc_active && !dec_active) begin
        active_count_reg <= active_count_reg + 1;
        active_count_av_reg <= active_count_reg < (2**STATUS_FIFO_ADDR_WIDTH-1);
    end else if (active_count_reg > 0 && !inc_active && dec_active) begin
        active_count_reg <= active_count_reg - 1;
        active_count_av_reg <= 1'b1;
    end else begin
        active_count_av_reg <= active_count_reg < 2**STATUS_FIFO_ADDR_WIDTH;
    end

    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axis_write_desc_ready_reg <= 1'b0;
        m_axis_write_desc_status_valid_reg <= 1'b0;

        s_axis_write_data_tready_reg <= 1'b0;

        m_axi_awvalid_reg <= 1'b0;
        m_axi_bready_reg <= 1'b0;

        bresp_reg <= AXI_RESP_OKAY;

        save_axis_tlast_reg <= 1'b0;
        shift_axis_extra_cycle_reg <= 1'b0;

        status_fifo_wr_ptr_reg <= 0;
        status_fifo_rd_ptr_reg <= 0;

        active_count_reg <= 0;
        active_count_av_reg <= 1'b1;
    end
end

// 输出数据通路逻辑
reg [AXI_DATA_WIDTH-1:0] m_axi_wdata_reg  = {AXI_DATA_WIDTH{1'b0}}; // AXI W 输出寄存器：wdata。
reg [AXI_STRB_WIDTH-1:0] m_axi_wstrb_reg  = {AXI_STRB_WIDTH{1'b0}}; // AXI W 输出寄存器：wstrb。
reg                      m_axi_wlast_reg  = 1'b0; // AXI W 输出寄存器：wlast。
reg                      m_axi_wvalid_reg = 1'b0; // AXI W 输出寄存器：wvalid。

reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_wr_ptr_reg = 0; // 输出 FIFO 写指针(含额外位区分满/空)。
reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_rd_ptr_reg = 0; // 输出 FIFO 读指针(含额外位区分满/空)。
reg out_fifo_half_full_reg = 1'b0; // 输出 FIFO 半满标志(对上游产生背压)。

wire out_fifo_full = out_fifo_wr_ptr_reg == (out_fifo_rd_ptr_reg ^ {1'b1, {OUTPUT_FIFO_ADDR_WIDTH{1'b0}}}); // 输出 FIFO 满标志。
wire out_fifo_empty = out_fifo_wr_ptr_reg == out_fifo_rd_ptr_reg; // 输出 FIFO 空标志。

(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXI_DATA_WIDTH-1:0] out_fifo_wdata[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：wdata。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXI_STRB_WIDTH-1:0] out_fifo_wstrb[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：wstrb。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg                      out_fifo_wlast[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：wlast。

assign m_axi_wready_int = !out_fifo_half_full_reg;

assign m_axi_wdata  = m_axi_wdata_reg;
assign m_axi_wstrb  = m_axi_wstrb_reg;
assign m_axi_wvalid = m_axi_wvalid_reg;
assign m_axi_wlast  = m_axi_wlast_reg;

always @(posedge clk) begin
    m_axi_wvalid_reg <= m_axi_wvalid_reg && !m_axi_wready;

    out_fifo_half_full_reg <= $unsigned(out_fifo_wr_ptr_reg - out_fifo_rd_ptr_reg) >= 2**(OUTPUT_FIFO_ADDR_WIDTH-1);

    if (!out_fifo_full && m_axi_wvalid_int) begin
        out_fifo_wdata[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wdata_int;
        out_fifo_wstrb[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wstrb_int;
        out_fifo_wlast[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wlast_int;
        out_fifo_wr_ptr_reg <= out_fifo_wr_ptr_reg + 1;
    end

    if (!out_fifo_empty && (!m_axi_wvalid_reg || m_axi_wready)) begin
        m_axi_wdata_reg <= out_fifo_wdata[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wstrb_reg <= out_fifo_wstrb[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wlast_reg <= out_fifo_wlast[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wvalid_reg <= 1'b1;
        out_fifo_rd_ptr_reg <= out_fifo_rd_ptr_reg + 1;
    end

    if (rst) begin
        out_fifo_wr_ptr_reg <= 0;
        out_fifo_rd_ptr_reg <= 0;
        m_axi_wvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
