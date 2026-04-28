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
 * AXI4 中央 DMA
 *
 * 模块目录
 * 1) 直接在 AXI 读写主口之间搬运数据：从 read_addr 读，再写到 write_addr。
 * 2) 读侧负责 AR/R 获取，写侧负责 AW/W/B 提交，中间进行对齐重排。
 * 3) 用状态 FIFO 跟踪事务完成与错误，回送描述符状态。
 */
module axi_cdma #
(
    // 数据总线位宽
    parameter AXI_DATA_WIDTH = 32,
    // 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节 lane）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 信号位宽
    parameter AXI_ID_WIDTH = 8,
    // 允许生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // 长度字段位宽
    parameter LEN_WIDTH = 20,
    // 标签字段位宽
    parameter TAG_WIDTH = 8,
    // 使能非对齐传输支持
    parameter ENABLE_UNALIGNED = 0
)
(
    input  wire                       clk, // CDMA 时钟。
    input  wire                       rst, // 同步复位，高电平有效。

    /*
     * AXI 描述符输入
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_desc_read_addr, // 描述符源地址。
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_desc_write_addr, // 描述符目的地址。
    input  wire [LEN_WIDTH-1:0]       s_axis_desc_len, // 描述符长度。
    input  wire [TAG_WIDTH-1:0]       s_axis_desc_tag, // 描述符 tag。
    input  wire                       s_axis_desc_valid, // 描述符有效。
    output wire                       s_axis_desc_ready, // 模块可接收描述符。

    /*
     * AXI 描述符状态输出
     */
    output wire [TAG_WIDTH-1:0]       m_axis_desc_status_tag, // 完成状态 tag。
    output wire [3:0]                 m_axis_desc_status_error, // 完成状态错误码。
    output wire                       m_axis_desc_status_valid, // 完成状态有效。

    /*
     * AXI 写主接口
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
     * AXI 读主接口
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
    input  wire                       enable // CDMA 使能。
);

parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH;
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
parameter AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE;

parameter OFFSET_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
parameter OFFSET_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_WIDTH{1'b1}} : 0;
parameter ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH);
parameter CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

parameter STATUS_FIFO_ADDR_WIDTH = 5;
parameter OUTPUT_FIFO_ADDR_WIDTH = 5;

// 总线位宽断言检查
initial begin
    if (AXI_WORD_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisble (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_WORD_WIDTH) != AXI_WORD_WIDTH) begin
        $error("Error: AXI word width must be even power of two (instance %m)");
        $finish;
    end

    if (AXI_MAX_BURST_LEN < 1 || AXI_MAX_BURST_LEN > 256) begin
        $error("Error: AXI_MAX_BURST_LEN must be between 1 and 256 (instance %m)");
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

localparam [1:0]
    READ_STATE_IDLE = 2'd0,
    READ_STATE_START = 2'd1,
    READ_STATE_REQ = 2'd2;

reg [1:0] read_state_reg = READ_STATE_IDLE, read_state_next; // 读描述符拆分与 AR 生成状态机。

localparam [0:0]
    AXI_STATE_IDLE = 1'd0,
    AXI_STATE_WRITE = 1'd1;

reg [0:0] axi_state_reg = AXI_STATE_IDLE, axi_state_next; // 写数据重排与 W 输出状态机。

// 数据通路控制信号
reg transfer_in_save; // 当前拍是否把 AXI R 数据缓存到 save 寄存器。
reg axi_cmd_ready; // 写侧命令寄存器是否可接收新命令。
reg status_fifo_we; // 状态 FIFO 写使能。

reg [AXI_ADDR_WIDTH-1:0] read_addr_reg = {AXI_ADDR_WIDTH{1'b0}}, read_addr_next; // 当前读地址游标。
reg [AXI_ADDR_WIDTH-1:0] write_addr_reg = {AXI_ADDR_WIDTH{1'b0}}, write_addr_next; // 当前写地址游标。
reg [LEN_WIDTH-1:0] op_word_count_reg = {LEN_WIDTH{1'b0}}, op_word_count_next; // 当前描述符剩余总字数。
reg [LEN_WIDTH-1:0] tr_word_count_reg = {LEN_WIDTH{1'b0}}, tr_word_count_next; // 当前突发剩余字数。
reg [LEN_WIDTH-1:0] axi_word_count_reg = {LEN_WIDTH{1'b0}}, axi_word_count_next; // 当前次 AR 计划读取字数。

reg [AXI_ADDR_WIDTH-1:0] axi_cmd_addr_reg = {AXI_ADDR_WIDTH{1'b0}}, axi_cmd_addr_next; // 送往写侧的起始地址命令。
reg [OFFSET_WIDTH-1:0] axi_cmd_offset_reg = {OFFSET_WIDTH{1'b0}}, axi_cmd_offset_next; // 写侧命令偏移。
reg [OFFSET_WIDTH-1:0] axi_cmd_first_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, axi_cmd_first_cycle_offset_next; // 写侧命令首拍偏移。
reg [OFFSET_WIDTH-1:0] axi_cmd_last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, axi_cmd_last_cycle_offset_next; // 写侧命令末拍偏移。
reg [CYCLE_COUNT_WIDTH-1:0] axi_cmd_input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axi_cmd_input_cycle_count_next; // 写侧命令输入拍数。
reg [CYCLE_COUNT_WIDTH-1:0] axi_cmd_output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axi_cmd_output_cycle_count_next; // 写侧命令输出拍数。
reg axi_cmd_bubble_cycle_reg = 1'b0, axi_cmd_bubble_cycle_next; // 写侧命令是否插入气泡拍。
reg axi_cmd_last_transfer_reg = 1'b0, axi_cmd_last_transfer_next; // 写侧命令是否最后一次传输。
reg [TAG_WIDTH-1:0] axi_cmd_tag_reg = {TAG_WIDTH{1'b0}}, axi_cmd_tag_next; // 写侧命令关联 tag。
reg axi_cmd_valid_reg = 1'b0, axi_cmd_valid_next; // 写侧命令有效位。

reg [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next; // 当前数据重排偏移。
reg [OFFSET_WIDTH-1:0] first_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, first_cycle_offset_next; // 当前事务首拍偏移。
reg [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next; // 当前事务末拍偏移。
reg [CYCLE_COUNT_WIDTH-1:0] input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, input_cycle_count_next; // 剩余输入拍数。
reg [CYCLE_COUNT_WIDTH-1:0] output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, output_cycle_count_next; // 剩余输出拍数。
reg input_active_reg = 1'b0, input_active_next; // 输入侧是否活跃。
reg output_active_reg = 1'b0, output_active_next; // 输出侧是否活跃。
reg bubble_cycle_reg = 1'b0, bubble_cycle_next; // 当前是否气泡拍。
reg first_input_cycle_reg = 1'b0, first_input_cycle_next; // 当前输入是否首拍。
reg first_output_cycle_reg = 1'b0, first_output_cycle_next; // 当前输出是否首拍。
reg output_last_cycle_reg = 1'b0, output_last_cycle_next; // 当前输出是否末拍。
reg last_transfer_reg = 1'b0, last_transfer_next; // 当前突发是否描述符最后一次传输。
reg [1:0] rresp_reg = AXI_RESP_OKAY, rresp_next; // 聚合读响应状态。
reg [1:0] bresp_reg = AXI_RESP_OKAY, bresp_next; // 聚合写响应状态。

reg [TAG_WIDTH-1:0] tag_reg = {TAG_WIDTH{1'b0}}, tag_next; // 当前描述符 tag 缓存。

reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_wr_ptr_reg = 0; // 状态 FIFO 写指针。
reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_rd_ptr_reg = 0, status_fifo_rd_ptr_next; // 状态 FIFO 读指针。
reg [TAG_WIDTH-1:0] status_fifo_tag[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：tag 字段。
reg [1:0] status_fifo_resp[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：响应码字段。
reg status_fifo_last[(2**STATUS_FIFO_ADDR_WIDTH)-1:0]; // 状态 FIFO：last 标志字段。
reg [TAG_WIDTH-1:0] status_fifo_wr_tag; // 本拍待写入状态 FIFO 的 tag。
reg [1:0] status_fifo_wr_resp; // 本拍待写入状态 FIFO 的响应码。
reg status_fifo_wr_last; // 本拍待写入状态 FIFO 的 last 标志。

reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] active_count_reg = 0; // 在途事务计数。
reg active_count_av_reg = 1'b1; // 在途计数是否低于可接收上限。
reg inc_active; // 在途计数加一事件。
reg dec_active; // 在途计数减一事件。

reg s_axis_desc_ready_reg = 1'b0, s_axis_desc_ready_next; // 描述符输入 ready 寄存器。

reg [TAG_WIDTH-1:0] m_axis_desc_status_tag_reg = {TAG_WIDTH{1'b0}}, m_axis_desc_status_tag_next; // 状态输出 tag 寄存器。
reg [3:0] m_axis_desc_status_error_reg = 4'd0, m_axis_desc_status_error_next; // 状态输出错误码寄存器。
reg m_axis_desc_status_valid_reg = 1'b0, m_axis_desc_status_valid_next; // 状态输出 valid 寄存器。

reg [AXI_ADDR_WIDTH-1:0] m_axi_araddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_araddr_next; // AR 地址寄存器。
reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next; // AR 长度寄存器。
reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next; // ARVALID 寄存器。
reg m_axi_rready_reg = 1'b0, m_axi_rready_next; // RREADY 寄存器。

reg [AXI_ADDR_WIDTH-1:0] m_axi_awaddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_awaddr_next; // AW 地址寄存器。
reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next; // AW 长度寄存器。
reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next; // AWVALID 寄存器。
reg m_axi_bready_reg = 1'b0, m_axi_bready_next; // BREADY 寄存器。

reg [AXI_DATA_WIDTH-1:0] save_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}}; // 未对齐拼接时缓存上一拍 RDATA。

wire [AXI_DATA_WIDTH-1:0] shift_axi_rdata = {m_axi_rdata, save_axi_rdata_reg} >> ((AXI_STRB_WIDTH-offset_reg)*AXI_WORD_SIZE); // 根据偏移重排后的对齐数据。

// 内部数据通路
reg  [AXI_DATA_WIDTH-1:0] m_axi_wdata_int; // 内部待输出 WDATA。
reg  [AXI_STRB_WIDTH-1:0] m_axi_wstrb_int; // 内部待输出 WSTRB。
reg                       m_axi_wlast_int; // 内部待输出 WLAST。
reg                       m_axi_wvalid_int; // 内部待输出 WVALID。
wire                      m_axi_wready_int; // 内部输出通路 ready。

assign s_axis_desc_ready = s_axis_desc_ready_reg;

assign m_axis_desc_status_tag = m_axis_desc_status_tag_reg;
assign m_axis_desc_status_error = m_axis_desc_status_error_reg;
assign m_axis_desc_status_valid = m_axis_desc_status_valid_reg;

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
    read_state_next = READ_STATE_IDLE;

    s_axis_desc_ready_next = 1'b0;

    m_axi_araddr_next = m_axi_araddr_reg;
    m_axi_arlen_next = m_axi_arlen_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;

    read_addr_next = read_addr_reg;
    write_addr_next = write_addr_reg;
    op_word_count_next = op_word_count_reg;
    tr_word_count_next = tr_word_count_reg;
    axi_word_count_next = axi_word_count_reg;

    axi_cmd_addr_next = axi_cmd_addr_reg;
    axi_cmd_offset_next = axi_cmd_offset_reg;
    axi_cmd_first_cycle_offset_next = axi_cmd_first_cycle_offset_reg;
    axi_cmd_last_cycle_offset_next = axi_cmd_last_cycle_offset_reg;
    axi_cmd_input_cycle_count_next = axi_cmd_input_cycle_count_reg;
    axi_cmd_output_cycle_count_next = axi_cmd_output_cycle_count_reg;
    axi_cmd_bubble_cycle_next = axi_cmd_bubble_cycle_reg;
    axi_cmd_last_transfer_next = axi_cmd_last_transfer_reg;
    axi_cmd_tag_next = axi_cmd_tag_reg;
    axi_cmd_valid_next = axi_cmd_valid_reg && !axi_cmd_ready;

    inc_active = 1'b0;

    case (read_state_reg)
        READ_STATE_IDLE: begin
            // 空闲态：装载新描述符并启动操作
            s_axis_desc_ready_next = !axi_cmd_valid_reg && enable && active_count_av_reg;

            if (s_axis_desc_ready && s_axis_desc_valid) begin
                if (ENABLE_UNALIGNED) begin
                    read_addr_next = s_axis_desc_read_addr;
                    write_addr_next = s_axis_desc_write_addr;
                end else begin
                    read_addr_next = s_axis_desc_read_addr & ADDR_MASK;
                    write_addr_next = s_axis_desc_write_addr & ADDR_MASK;
                end
                axi_cmd_tag_next = s_axis_desc_tag;
                op_word_count_next = s_axis_desc_len;

                s_axis_desc_ready_next = 1'b0;
                read_state_next = READ_STATE_START;
            end else begin
                read_state_next = READ_STATE_IDLE;
            end
        end
        READ_STATE_START: begin
            // 启动态：计算写传输长度
            if (!axi_cmd_valid_reg && active_count_av_reg) begin
                if (op_word_count_reg <= AXI_MAX_BURST_SIZE - (write_addr_reg & OFFSET_MASK)) begin
                    // 数据包小于最大突发长度
                    if (((write_addr_reg & 12'hfff) + (op_word_count_reg & 12'hfff)) >> 12 != 0 || op_word_count_reg >> 12 != 0) begin
                        // 跨越 4K 边界
                        axi_word_count_next = 13'h1000 - (write_addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        axi_word_count_next = op_word_count_reg;
                    end
                end else begin
                    // 数据包大于最大突发长度
                    if (((write_addr_reg & 12'hfff) + AXI_MAX_BURST_SIZE) >> 12 != 0) begin
                        // 跨越 4K 边界
                        axi_word_count_next = 13'h1000 - (write_addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        axi_word_count_next = AXI_MAX_BURST_SIZE - (write_addr_reg & OFFSET_MASK);
                    end
                end

                write_addr_next = write_addr_reg + axi_word_count_next;
                op_word_count_next = op_word_count_reg - axi_word_count_next;

                axi_cmd_addr_next = write_addr_reg;
                if (ENABLE_UNALIGNED) begin
                    axi_cmd_input_cycle_count_next = (axi_word_count_next + (read_addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                    axi_cmd_output_cycle_count_next = (axi_word_count_next + (write_addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                    axi_cmd_offset_next = (write_addr_reg & OFFSET_MASK) - (read_addr_reg & OFFSET_MASK);
                    axi_cmd_bubble_cycle_next = (read_addr_reg & OFFSET_MASK) > (write_addr_reg & OFFSET_MASK);
                    axi_cmd_first_cycle_offset_next = write_addr_reg & OFFSET_MASK;
                    axi_cmd_last_cycle_offset_next = axi_cmd_first_cycle_offset_next + axi_word_count_next & OFFSET_MASK;
                end else begin
                    axi_cmd_input_cycle_count_next = (axi_word_count_next - 1) >> AXI_BURST_SIZE;
                    axi_cmd_output_cycle_count_next = (axi_word_count_next - 1) >> AXI_BURST_SIZE;
                    axi_cmd_offset_next = 0;
                    axi_cmd_bubble_cycle_next = 0;
                    axi_cmd_first_cycle_offset_next = 0;
                    axi_cmd_last_cycle_offset_next = axi_word_count_next & OFFSET_MASK;
                end
                axi_cmd_last_transfer_next = op_word_count_next == 0;
                axi_cmd_valid_next = 1'b1;

                inc_active = 1'b1;

                read_state_next = READ_STATE_REQ;
            end else begin
                read_state_next = READ_STATE_START;
            end
        end
        READ_STATE_REQ: begin
            // 请求态：发起 AXI 读请求
            if (!m_axi_arvalid) begin
                if (axi_word_count_reg <= AXI_MAX_BURST_SIZE - (read_addr_reg & OFFSET_MASK)) begin
                    // 数据包小于最大突发长度
                    if (((read_addr_reg & 12'hfff) + (axi_word_count_reg & 12'hfff)) >> 12 != 0 || axi_word_count_reg >> 12 != 0) begin
                        // 跨越 4K 边界
                        tr_word_count_next = 13'h1000 - (read_addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        tr_word_count_next = axi_word_count_reg;
                    end
                end else begin
                    // 数据包大于最大突发长度
                    if (((read_addr_reg & 12'hfff) + AXI_MAX_BURST_SIZE) >> 12 != 0) begin
                        // 跨越 4K 边界
                        tr_word_count_next = 13'h1000 - (read_addr_reg & 12'hfff);
                    end else begin
                        // 不跨越 4K 边界
                        tr_word_count_next = AXI_MAX_BURST_SIZE - (read_addr_reg & OFFSET_MASK);
                    end
                end

                m_axi_araddr_next = read_addr_reg;
                if (ENABLE_UNALIGNED) begin
                    m_axi_arlen_next = (tr_word_count_next + (read_addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                end else begin
                    m_axi_arlen_next = (tr_word_count_next - 1) >> AXI_BURST_SIZE;
                end
                m_axi_arvalid_next = 1'b1;

                read_addr_next = read_addr_reg + tr_word_count_next;
                axi_word_count_next = axi_word_count_reg - tr_word_count_next;

                if (axi_word_count_next > 0) begin
                    read_state_next = READ_STATE_REQ;
                end else if (op_word_count_next > 0) begin
                    read_state_next = READ_STATE_START;
                end else begin
                    s_axis_desc_ready_next = !axi_cmd_valid_reg && enable && active_count_av_reg;
                    read_state_next = READ_STATE_IDLE;
                end
            end else begin
                read_state_next = READ_STATE_REQ;
            end
        end
    endcase
end

always @* begin
    axi_state_next = AXI_STATE_IDLE;

    m_axis_desc_status_tag_next = m_axis_desc_status_tag_reg;
    m_axis_desc_status_error_next = m_axis_desc_status_error_reg;
    m_axis_desc_status_valid_next = 1'b0;

    m_axi_awaddr_next = m_axi_awaddr_reg;
    m_axi_awlen_next = m_axi_awlen_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;
    m_axi_wdata_int = shift_axi_rdata;
    m_axi_wstrb_int = {AXI_STRB_WIDTH{1'b0}};
    m_axi_wlast_int = 1'b0;
    m_axi_wvalid_int = 1'b0;
    m_axi_bready_next = 1'b0;

    m_axi_rready_next = 1'b0;

    transfer_in_save = 1'b0;
    axi_cmd_ready = 1'b0;
    status_fifo_we = 1'b0;

    offset_next = offset_reg;
    first_cycle_offset_next = first_cycle_offset_reg;
    last_cycle_offset_next = last_cycle_offset_reg;
    input_cycle_count_next = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next = input_active_reg;
    output_active_next = output_active_reg;
    bubble_cycle_next = bubble_cycle_reg;
    first_input_cycle_next = first_input_cycle_reg;
    first_output_cycle_next = first_output_cycle_reg;
    output_last_cycle_next = output_last_cycle_reg;
    last_transfer_next = last_transfer_reg;

    tag_next = tag_reg;

    status_fifo_rd_ptr_next = status_fifo_rd_ptr_reg;

    dec_active = 1'b0;

    if (m_axi_rready && m_axi_rvalid && (m_axi_rresp == AXI_RESP_SLVERR || m_axi_rresp == AXI_RESP_DECERR)) begin
        rresp_next = m_axi_rresp;
    end else begin
        rresp_next = rresp_reg;
    end

    if (m_axi_bready && m_axi_bvalid && (m_axi_bresp == AXI_RESP_SLVERR || m_axi_bresp == AXI_RESP_DECERR)) begin
        bresp_next = m_axi_bresp;
    end else begin
        bresp_next = bresp_reg;
    end

    status_fifo_wr_tag = tag_reg;
    status_fifo_wr_resp = rresp_next;
    status_fifo_wr_last = 1'b0;

    case (axi_state_reg)
        AXI_STATE_IDLE: begin
            // 空闲态：装载新描述符并启动操作
            m_axi_rready_next = 1'b0;

            // 保存传输参数
            if (ENABLE_UNALIGNED) begin
                offset_next = axi_cmd_offset_reg;
                first_cycle_offset_next = axi_cmd_first_cycle_offset_reg;
            end else begin
                offset_next = 0;
                first_cycle_offset_next = 0;
            end
            last_cycle_offset_next = axi_cmd_last_cycle_offset_reg;
            input_cycle_count_next = axi_cmd_input_cycle_count_reg;
            output_cycle_count_next = axi_cmd_output_cycle_count_reg;
            bubble_cycle_next = axi_cmd_bubble_cycle_reg;
            last_transfer_next = axi_cmd_last_transfer_reg;
            tag_next = axi_cmd_tag_reg;

            output_last_cycle_next = output_cycle_count_next == 0;
            input_active_next = 1'b1;
            output_active_next = 1'b1;
            first_input_cycle_next = 1'b1;
            first_output_cycle_next = 1'b1;

            if (!m_axi_awvalid && axi_cmd_valid_reg) begin
                axi_cmd_ready = 1'b1;

                m_axi_awaddr_next = axi_cmd_addr_reg;
                m_axi_awlen_next = axi_cmd_output_cycle_count_reg;
                m_axi_awvalid_next = 1'b1;

                m_axi_rready_next = m_axi_wready_int;
                axi_state_next = AXI_STATE_WRITE;
            end
        end
        AXI_STATE_WRITE: begin
            // 处理 AXI 读返回数据
            m_axi_rready_next = m_axi_wready_int && input_active_reg;

            if ((m_axi_rready && m_axi_rvalid) || !input_active_reg) begin
                // 接收并搬运 AXI 读数据
                transfer_in_save = m_axi_rready && m_axi_rvalid;

                if (ENABLE_UNALIGNED && first_input_cycle_reg && bubble_cycle_reg) begin
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next = input_cycle_count_reg > 0;
                    end
                    bubble_cycle_next = 1'b0;
                    first_input_cycle_next = 1'b0;

                    m_axi_rready_next = m_axi_wready_int && input_active_next;
                    axi_state_next = AXI_STATE_WRITE;
                end else begin
                    // 更新计数器
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
                    first_input_cycle_next = 1'b0;
                    first_output_cycle_next = 1'b0;

                    // 直通读数据
                    m_axi_wdata_int = shift_axi_rdata;
                    if (first_output_cycle_reg) begin
                        m_axi_wstrb_int = {AXI_STRB_WIDTH{1'b1}} << first_cycle_offset_reg;
                    end else begin
                        m_axi_wstrb_int = {AXI_STRB_WIDTH{1'b1}};
                    end
                    m_axi_wvalid_int = 1'b1;

                    if (output_last_cycle_reg) begin
                        // 无剩余数据需传输，结束操作
                        if (last_cycle_offset_reg > 0) begin
                            m_axi_wstrb_int = m_axi_wstrb_int & {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg);
                        end
                        m_axi_wlast_int = 1'b1;

                        status_fifo_we = 1'b1;
                        status_fifo_wr_tag = tag_reg;
                        status_fifo_wr_resp = rresp_next;
                        status_fifo_wr_last = last_transfer_reg;

                        if (last_transfer_reg) begin
                            rresp_next = AXI_RESP_OKAY;
                        end

                        m_axi_rready_next = 1'b0;
                        axi_state_next = AXI_STATE_IDLE;
                    end else begin
                        // AXI 传输仍有后续数据拍
                        axi_state_next = AXI_STATE_WRITE;
                    end
                end
            end else begin
                axi_state_next = AXI_STATE_WRITE;
            end
        end
    endcase

    if (status_fifo_rd_ptr_reg != status_fifo_wr_ptr_reg) begin
        // 状态 FIFO 非空
        if (m_axi_bready && m_axi_bvalid) begin
            // 收到写完成，出队并返回状态
            m_axis_desc_status_tag_next = status_fifo_tag[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            if (status_fifo_resp[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] == AXI_RESP_SLVERR) begin
                m_axis_desc_status_error_next = DMA_ERROR_AXI_RD_SLVERR;
            end else if (status_fifo_resp[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] == AXI_RESP_DECERR) begin
                m_axis_desc_status_error_next = DMA_ERROR_AXI_RD_DECERR;
            end else if (bresp_next == AXI_RESP_SLVERR) begin
                m_axis_desc_status_error_next = DMA_ERROR_AXI_WR_SLVERR;
            end else if (bresp_next == AXI_RESP_DECERR) begin
                m_axis_desc_status_error_next = DMA_ERROR_AXI_WR_DECERR;
            end else begin
                m_axis_desc_status_error_next = DMA_ERROR_NONE;
            end
            m_axis_desc_status_valid_next = status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            status_fifo_rd_ptr_next = status_fifo_rd_ptr_reg + 1;
            m_axi_bready_next = 1'b0;

            if (status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]) begin
                bresp_next = AXI_RESP_OKAY;
            end

            dec_active = 1'b1;
        end else begin
            // 等待写完成
            m_axi_bready_next = 1'b1;
        end
    end
end

always @(posedge clk) begin
    read_state_reg <= read_state_next;
    axi_state_reg <= axi_state_next;

    s_axis_desc_ready_reg <= s_axis_desc_ready_next;

    m_axis_desc_status_tag_reg <= m_axis_desc_status_tag_next;
    m_axis_desc_status_error_reg <= m_axis_desc_status_error_next;
    m_axis_desc_status_valid_reg <= m_axis_desc_status_valid_next;

    m_axi_awaddr_reg <= m_axi_awaddr_next;
    m_axi_awlen_reg <= m_axi_awlen_next;
    m_axi_awvalid_reg <= m_axi_awvalid_next;
    m_axi_bready_reg <= m_axi_bready_next;
    m_axi_araddr_reg <= m_axi_araddr_next;
    m_axi_arlen_reg <= m_axi_arlen_next;
    m_axi_arvalid_reg <= m_axi_arvalid_next;
    m_axi_rready_reg <= m_axi_rready_next;

    read_addr_reg <= read_addr_next;
    write_addr_reg <= write_addr_next;
    op_word_count_reg <= op_word_count_next;
    tr_word_count_reg <= tr_word_count_next;
    axi_word_count_reg <= axi_word_count_next;

    axi_cmd_addr_reg <= axi_cmd_addr_next;
    axi_cmd_offset_reg <= axi_cmd_offset_next;
    axi_cmd_first_cycle_offset_reg <= axi_cmd_first_cycle_offset_next;
    axi_cmd_last_cycle_offset_reg <= axi_cmd_last_cycle_offset_next;
    axi_cmd_input_cycle_count_reg <= axi_cmd_input_cycle_count_next;
    axi_cmd_output_cycle_count_reg <= axi_cmd_output_cycle_count_next;
    axi_cmd_bubble_cycle_reg <= axi_cmd_bubble_cycle_next;
    axi_cmd_last_transfer_reg <= axi_cmd_last_transfer_next;
    axi_cmd_tag_reg <= axi_cmd_tag_next;
    axi_cmd_valid_reg <= axi_cmd_valid_next;

    offset_reg <= offset_next;
    first_cycle_offset_reg <= first_cycle_offset_next;
    last_cycle_offset_reg <= last_cycle_offset_next;
    input_cycle_count_reg <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg <= input_active_next;
    output_active_reg <= output_active_next;
    bubble_cycle_reg <= bubble_cycle_next;
    first_input_cycle_reg <= first_input_cycle_next;
    first_output_cycle_reg <= first_output_cycle_next;
    output_last_cycle_reg <= output_last_cycle_next;
    last_transfer_reg <= last_transfer_next;
    rresp_reg <= rresp_next;
    bresp_reg <= bresp_next;

    tag_reg <= tag_next;

    if (transfer_in_save) begin
        save_axi_rdata_reg <= m_axi_rdata;
    end

    if (status_fifo_we) begin
        status_fifo_tag[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_tag;
        status_fifo_resp[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_resp;
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
        read_state_reg <= READ_STATE_IDLE;
        axi_state_reg <= AXI_STATE_IDLE;

        s_axis_desc_ready_reg <= 1'b0;
        m_axis_desc_status_valid_reg <= 1'b0;

        m_axi_awvalid_reg <= 1'b0;
        m_axi_bready_reg <= 1'b0;
        m_axi_arvalid_reg <= 1'b0;
        m_axi_rready_reg <= 1'b0;

        axi_cmd_valid_reg <= 1'b0;

        rresp_reg <= AXI_RESP_OKAY;
        bresp_reg <= AXI_RESP_OKAY;

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
reg out_fifo_half_full_reg = 1'b0; // 输出 FIFO 半满标志(给前级背压)。

wire out_fifo_full = out_fifo_wr_ptr_reg == (out_fifo_rd_ptr_reg ^ {1'b1, {OUTPUT_FIFO_ADDR_WIDTH{1'b0}}}); // 输出 FIFO 满标志。
wire out_fifo_empty = out_fifo_wr_ptr_reg == out_fifo_rd_ptr_reg; // 输出 FIFO 空标志。

(* ram_style = "distributed" *)
reg [AXI_DATA_WIDTH-1:0] out_fifo_wdata[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：wdata。
(* ram_style = "distributed" *)
reg [AXI_STRB_WIDTH-1:0] out_fifo_wstrb[2**OUTPUT_FIFO_ADDR_WIDTH-1:0]; // 输出 FIFO 存储：wstrb。
(* ram_style = "distributed" *)
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
