/*

Copyright (c) 2023 Alex Forencich

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
 * AXI4 虚拟 FIFO（解码器）
 *
 * 模块目录
 * 1) 解析虚拟 FIFO 分段控制头，把分段数据重组为标准 AXIS 帧输出。
 * 2) 处理 SOP/EOP、跨段边界和末拍 keep/last 计算。
 * 3) 使用控制 FIFO 与输出 FIFO 隔离输入与 AXIS 下游回压。
 */
module axi_vfifo_dec #
(
    // 输入分段位宽
    parameter SEG_WIDTH = 32,
    // 分段数量
    parameter SEG_CNT = 2,
    // AXI-Stream 接口位宽
    parameter AXIS_DATA_WIDTH = SEG_WIDTH*SEG_CNT/2,
    // 是否启用 AXI-Stream tkeep
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI-Stream tkeep 位宽（每拍字节数）
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // 是否启用 AXI-Stream tlast
    parameter AXIS_LAST_ENABLE = 1,
    // 是否透传 AXI-Stream tid
    parameter AXIS_ID_ENABLE = 0,
    // AXI-Stream tid 位宽
    parameter AXIS_ID_WIDTH = 8,
    // 是否透传 AXI-Stream tdest
    parameter AXIS_DEST_ENABLE = 0,
    // AXI-Stream tdest 位宽
    parameter AXIS_DEST_WIDTH = 8,
    // 是否透传 AXI-Stream tuser
    parameter AXIS_USER_ENABLE = 1,
    // AXI-Stream tuser 位宽
    parameter AXIS_USER_WIDTH = 1
)
(
    input  wire                          clk, // 解码器时钟。
    input  wire                          rst, // 同步复位，高电平有效。

    /*
     * 分段数据输入（来自虚拟 FIFO 通道）
     */
    input  wire                          fifo_rst_in, // 来自虚拟 FIFO 通道的同步复位请求。
    input  wire [SEG_CNT*SEG_WIDTH-1:0]  input_data, // 分段数据输入总线。
    input  wire [SEG_CNT-1:0]            input_valid, // 分段数据有效位。
    output wire [SEG_CNT-1:0]            input_ready, // 分段数据就绪位。
    input  wire [SEG_CNT*SEG_WIDTH-1:0]  input_ctrl_data, // 分段控制头输入总线。
    input  wire [SEG_CNT-1:0]            input_ctrl_valid, // 分段控制头有效位。
    output wire [SEG_CNT-1:0]            input_ctrl_ready, // 分段控制头就绪位。

    /*
     * AXI-Stream 数据输出
     */
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata, // AXIS 输出 tdata。
    output wire [AXIS_KEEP_WIDTH-1:0]    m_axis_tkeep, // AXIS 输出 tkeep。
    output wire                          m_axis_tvalid, // AXIS 输出 tvalid。
    input  wire                          m_axis_tready, // AXIS 输出 tready。
    output wire                          m_axis_tlast, // AXIS 输出 tlast。
    output wire [AXIS_ID_WIDTH-1:0]      m_axis_tid, // AXIS 输出 tid。
    output wire [AXIS_DEST_WIDTH-1:0]    m_axis_tdest, // AXIS 输出 tdest。
    output wire [AXIS_USER_WIDTH-1:0]    m_axis_tuser, // AXIS 输出 tuser。

    /*
     * 状态
     */
    output wire                          sts_hdr_parity_err // 头部奇偶校验错误状态。
);

parameter AXIS_KEEP_WIDTH_INT = AXIS_KEEP_ENABLE ? AXIS_KEEP_WIDTH : 1;
parameter AXIS_BYTE_LANES = AXIS_KEEP_WIDTH_INT;
parameter AXIS_BYTE_SIZE = AXIS_DATA_WIDTH/AXIS_BYTE_LANES;
parameter AXIS_BYTE_IDX_WIDTH = $clog2(AXIS_BYTE_LANES);

parameter BYTE_SIZE = AXIS_BYTE_SIZE;

parameter SEG_BYTE_LANES = SEG_WIDTH / BYTE_SIZE;

parameter EXPAND_INPUT = SEG_CNT < 2;

parameter SEG_CNT_INT = EXPAND_INPUT ? SEG_CNT*2 : SEG_CNT;

parameter SEG_IDX_WIDTH = $clog2(SEG_CNT_INT);
parameter SEG_BYTE_IDX_WIDTH = $clog2(SEG_BYTE_LANES);

parameter AXIS_SEG_CNT = (AXIS_DATA_WIDTH + SEG_WIDTH-1) / SEG_WIDTH;
parameter AXIS_SEG_IDX_WIDTH = AXIS_SEG_CNT > 1 ? $clog2(AXIS_SEG_CNT) : 1;
parameter AXIS_LEN_MASK = AXIS_BYTE_LANES-1;

parameter OUT_OFFS_WIDTH = AXIS_SEG_IDX_WIDTH;

parameter META_ID_OFFSET = 0;
parameter META_DEST_OFFSET = META_ID_OFFSET + (AXIS_ID_ENABLE ? AXIS_ID_WIDTH : 0);
parameter META_USER_OFFSET = META_DEST_OFFSET + (AXIS_DEST_ENABLE ? AXIS_DEST_WIDTH : 0);
parameter META_WIDTH = META_USER_OFFSET + (AXIS_USER_ENABLE ? AXIS_USER_WIDTH : 0);
parameter HDR_SIZE = (16 + META_WIDTH + BYTE_SIZE-1) / BYTE_SIZE;
parameter HDR_WIDTH = HDR_SIZE * BYTE_SIZE;

parameter HDR_LEN_WIDTH = 12;
parameter HDR_SEG_LEN_WIDTH = HDR_LEN_WIDTH-SEG_BYTE_IDX_WIDTH;

parameter CTRL_FIFO_ADDR_WIDTH = 5;
parameter OUTPUT_FIFO_ADDR_WIDTH = 5;

parameter CTRL_FIFO_PTR_WIDTH = CTRL_FIFO_ADDR_WIDTH + SEG_IDX_WIDTH;

// 参数检查
initial begin
    if (AXIS_BYTE_SIZE * AXIS_KEEP_WIDTH_INT != AXIS_DATA_WIDTH) begin
        $error("Error: AXI stream data width not evenly divisible (instance %m)");
        $finish;
    end

    if (AXIS_SEG_CNT * SEG_WIDTH != AXIS_DATA_WIDTH) begin
        $error("Error: AXI stream data width not evenly divisible into segments (instance %m)");
        $finish;
    end

    if (SEG_WIDTH < HDR_WIDTH) begin
        $error("Error: Segment smaller than header (instance %m)");
        $finish;
    end
end

reg frame_reg = 1'b0, frame_next, frame_cyc; // 头部解析状态：当前是否处于帧内。
reg last_reg = 1'b0, last_next, last_cyc; // 当前帧是否标记为最后一个 frame。
reg extra_cycle_reg = 1'b0, extra_cycle_next, extra_cycle_cyc; // 是否需要额外输出拍(跨段尾部)。
reg last_straddle_reg = 1'b0, last_straddle_next, last_straddle_cyc; // 最后段是否跨界(straddle)。
reg [HDR_SEG_LEN_WIDTH-1:0] seg_cnt_reg = 0, seg_cnt_next, seg_cnt_cyc; // 当前帧剩余 segment 计数。
reg hdr_parity_err_reg = 1'b0, hdr_parity_err_next, hdr_parity_err_cyc; // 头部奇偶校验错误锁存。

reg out_frame_reg = 1'b0, out_frame_next, out_frame_cyc; // 数据重组阶段是否处于帧内。
reg [SEG_IDX_WIDTH-1:0] out_seg_offset_reg = 0, out_seg_offset_next, out_seg_offset_cyc; // 输出 segment 偏移。
reg [OUT_OFFS_WIDTH-1:0] output_offset_reg = 0, output_offset_next, output_offset_cyc; // 输出字偏移。
reg [SEG_CNT_INT-1:0] out_seg_consumed; // 本拍每段消费标志。
reg [SEG_CNT_INT-1:0] out_seg_consumed_reg = 0, out_seg_consumed_next; // 累积段消费寄存器。
reg out_valid, out_valid_straddle, out_frame, out_last, out_abort, out_done; // 输出构造过程中的组合状态标志。

reg [SEG_CNT_INT-1:0] seg_valid; // 解析出的段有效标志。
reg [SEG_CNT_INT-1:0] seg_valid_straddle; // straddle 对齐后的段有效标志。
reg [SEG_CNT_INT-1:0] seg_hdr_start_pkt; // 每段头部 SOP 标志。
reg [SEG_CNT_INT-1:0] seg_hdr_last; // 每段头部 LAST 标志。
reg [SEG_CNT_INT-1:0] seg_hdr_last_straddle; // 每段头部 LAST 的 straddle 标志。
reg [SEG_CNT_INT-1:0] seg_hdr_parity_err; // 每段头部奇偶校验错误标志。
reg [HDR_LEN_WIDTH-1:0] seg_hdr_len[SEG_CNT_INT-1:0]; // 每段头部长度字段。
reg [HDR_SEG_LEN_WIDTH-1:0] seg_hdr_seg_cnt[SEG_CNT_INT-1:0]; // 每段头部换算后的段计数。

reg [SEG_CNT_INT-1:0] shift_out_seg_valid; // 应用输出偏移后的 valid 位图。
reg [SEG_CNT_INT-1:0] shift_out_seg_valid_straddle; // 应用偏移后的 straddle valid 位图。
reg [SEG_CNT_INT-1:0] shift_out_seg_sop; // 应用偏移后的 SOP 位图。
reg [SEG_CNT_INT-1:0] shift_out_seg_eop; // 应用偏移后的 EOP 位图。
reg [SEG_CNT_INT-1:0] shift_out_seg_end; // 应用偏移后的 END 位图。
reg [SEG_CNT_INT-1:0] shift_out_seg_last; // 应用偏移后的 LAST 位图。

reg [SEG_CNT-1:0] input_ready_cmb; // 数据输入 ready 组合信号。
reg [SEG_CNT-1:0] input_ctrl_ready_cmb; // 控制输入 ready 组合信号。

reg [SEG_CNT*SEG_WIDTH-1:0] input_data_int_reg = 0, input_data_int_next; // 扩展输入模式下缓存上一拍数据。
reg [SEG_CNT-1:0] input_valid_int_reg = 0, input_valid_int_next; // 扩展输入模式下缓存上一拍 valid。

wire [SEG_CNT_INT*SEG_WIDTH*2-1:0] input_data_full = EXPAND_INPUT ? {2{{input_data, input_data_int_reg}}} : {2{input_data}}; // 标准化后的双倍宽输入数据窗口。
wire [SEG_CNT_INT-1:0] input_valid_full = EXPAND_INPUT ? {input_valid, input_valid_int_reg} : input_valid; // 标准化后的输入 valid 窗口。

reg out_ctrl_en_reg = 0, out_ctrl_en_next; // 输出控制信息使能。
reg out_ctrl_hdr_reg = 0, out_ctrl_hdr_next; // 输出控制信息是否为头部拍。
reg out_ctrl_last_reg = 0, out_ctrl_last_next; // 输出控制信息是否为帧末拍。
reg [AXIS_BYTE_IDX_WIDTH-1:0] out_ctrl_last_len_reg = 0, out_ctrl_last_len_next; // 输出控制信息末拍有效字节数。
reg [SEG_IDX_WIDTH-1:0] out_ctrl_seg_offset_reg = 0, out_ctrl_seg_offset_next; // 输出控制信息段偏移。

reg [AXIS_ID_WIDTH-1:0] axis_tid_reg = 0, axis_tid_next; // 当前帧 tid 缓存。
reg [AXIS_DEST_WIDTH-1:0] axis_tdest_reg = 0, axis_tdest_next; // 当前帧 tdest 缓存。
reg [AXIS_USER_WIDTH-1:0] axis_tuser_reg = 0, axis_tuser_next; // 当前帧 tuser 缓存。

// 内部数据通路
reg  [AXIS_DATA_WIDTH-1:0] m_axis_tdata_int; // 内部待输出 tdata。
reg  [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep_int; // 内部待输出 tkeep。
reg                        m_axis_tvalid_int; // 内部待输出 tvalid。
wire                       m_axis_tready_int; // 内部输出通路就绪。
reg                        m_axis_tlast_int; // 内部待输出 tlast。
reg  [AXIS_ID_WIDTH-1:0]   m_axis_tid_int; // 内部待输出 tid。
reg  [AXIS_DEST_WIDTH-1:0] m_axis_tdest_int; // 内部待输出 tdest。
reg  [AXIS_USER_WIDTH-1:0] m_axis_tuser_int; // 内部待输出 tuser。

assign input_ready = input_ready_cmb;
assign input_ctrl_ready = input_ctrl_ready_cmb;

assign sts_hdr_parity_err = hdr_parity_err_reg;

// 分段控制 FIFO
reg [CTRL_FIFO_PTR_WIDTH+1-1:0] ctrl_fifo_wr_ptr_reg = 0, ctrl_fifo_wr_ptr_next; // 控制 FIFO 写指针。
reg [CTRL_FIFO_PTR_WIDTH+1-1:0] ctrl_fifo_rd_ptr_reg = 0, ctrl_fifo_rd_ptr_next; // 控制 FIFO 读指针。

reg [SEG_CNT-1:0] ctrl_mem_rd_data_valid_reg = 0, ctrl_mem_rd_data_valid_next; // 控制 FIFO 读数据有效位。

reg [SEG_CNT-1:0] ctrl_fifo_wr_sop; // 控制 FIFO 写入 SOP 位图。
reg [SEG_CNT-1:0] ctrl_fifo_wr_eop; // 控制 FIFO 写入 EOP 位图。
reg [SEG_CNT-1:0] ctrl_fifo_wr_end; // 控制 FIFO 写入 END 位图。
reg [SEG_CNT-1:0] ctrl_fifo_wr_last; // 控制 FIFO 写入 LAST 位图。
reg [SEG_CNT*AXIS_BYTE_IDX_WIDTH-1:0] ctrl_fifo_wr_last_len; // 控制 FIFO 写入末拍长度字段。
reg [SEG_CNT-1:0] ctrl_fifo_wr_en; // 控制 FIFO 每段写使能。

wire [SEG_CNT-1:0] ctrl_fifo_rd_sop; // 控制 FIFO 读出 SOP 位图。
wire [SEG_CNT-1:0] ctrl_fifo_rd_eop; // 控制 FIFO 读出 EOP 位图。
wire [SEG_CNT-1:0] ctrl_fifo_rd_end; // 控制 FIFO 读出 END 位图。
wire [SEG_CNT-1:0] ctrl_fifo_rd_last; // 控制 FIFO 读出 LAST 位图。
wire [SEG_CNT*AXIS_BYTE_IDX_WIDTH-1:0] ctrl_fifo_rd_last_len; // 控制 FIFO 读出末拍长度字段。
wire [SEG_CNT-1:0] ctrl_fifo_rd_valid; // 控制 FIFO 每段读有效位。
reg [SEG_CNT-1:0] ctrl_fifo_rd_en; // 控制 FIFO 每段读使能。

wire [SEG_CNT-1:0] ctrl_fifo_seg_full; // 控制 FIFO 每段满标志。
wire [SEG_CNT-1:0] ctrl_fifo_seg_half_full; // 控制 FIFO 每段半满标志。
wire [SEG_CNT-1:0] ctrl_fifo_seg_empty; // 控制 FIFO 每段空标志。

wire ctrl_fifo_full = |ctrl_fifo_seg_full; // 任一段满则整体视为满。
wire ctrl_fifo_half_full = |ctrl_fifo_seg_half_full; // 任一段半满则整体半满。
wire ctrl_fifo_empty = |ctrl_fifo_seg_empty; // 任一段空则整体视为空(保持段对齐)。

generate

genvar n;

for (n = 0; n < SEG_CNT; n = n + 1) begin : ctrl_fifo_seg

    reg [CTRL_FIFO_ADDR_WIDTH+1-1:0] seg_wr_ptr_reg = 0; // 当前段控制 FIFO 写指针。
    reg [CTRL_FIFO_ADDR_WIDTH+1-1:0] seg_rd_ptr_reg = 0; // 当前段控制 FIFO 读指针。

    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg seg_mem_sop[2**CTRL_FIFO_ADDR_WIDTH-1:0]; // 当前段控制 FIFO 存储：SOP。
    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg seg_mem_eop[2**CTRL_FIFO_ADDR_WIDTH-1:0]; // 当前段控制 FIFO 存储：EOP。
    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg seg_mem_end[2**CTRL_FIFO_ADDR_WIDTH-1:0]; // 当前段控制 FIFO 存储：END。
    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg seg_mem_last[2**CTRL_FIFO_ADDR_WIDTH-1:0]; // 当前段控制 FIFO 存储：LAST。
    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg [AXIS_BYTE_IDX_WIDTH-1:0] seg_mem_last_len[2**CTRL_FIFO_ADDR_WIDTH-1:0]; // 当前段控制 FIFO 存储：末拍长度。

    reg seg_rd_sop_reg = 0; // 当前段控制 FIFO 读寄存：SOP。
    reg seg_rd_eop_reg = 0; // 当前段控制 FIFO 读寄存：EOP。
    reg seg_rd_end_reg = 0; // 当前段控制 FIFO 读寄存：END。
    reg seg_rd_last_reg = 0; // 当前段控制 FIFO 读寄存：LAST。
    reg [AXIS_BYTE_IDX_WIDTH-1:0] seg_rd_last_len_reg = 0; // 当前段控制 FIFO 读寄存：末拍长度。
    reg seg_rd_valid_reg = 0; // 当前段控制 FIFO 读寄存有效位。

    reg seg_half_full_reg = 1'b0; // 当前段控制 FIFO 半满标志。

    assign ctrl_fifo_rd_sop[n] = seg_rd_sop_reg;
    assign ctrl_fifo_rd_eop[n] = seg_rd_eop_reg;
    assign ctrl_fifo_rd_end[n] = seg_rd_end_reg;
    assign ctrl_fifo_rd_last[n] = seg_rd_last_reg;
    assign ctrl_fifo_rd_last_len[AXIS_BYTE_IDX_WIDTH*n +: AXIS_BYTE_IDX_WIDTH] = seg_rd_last_len_reg;
    assign ctrl_fifo_rd_valid[n] = seg_rd_valid_reg;

    wire seg_full = seg_wr_ptr_reg == (seg_rd_ptr_reg ^ {1'b1, {CTRL_FIFO_ADDR_WIDTH{1'b0}}}); // 当前段控制 FIFO 满标志。
    wire seg_empty = seg_wr_ptr_reg == seg_rd_ptr_reg; // 当前段控制 FIFO 空标志。

    assign ctrl_fifo_seg_full[n] = seg_full;
    assign ctrl_fifo_seg_half_full[n] = seg_half_full_reg;
    assign ctrl_fifo_seg_empty[n] = seg_empty;

    always @(posedge clk) begin
        seg_rd_valid_reg <= seg_rd_valid_reg && !ctrl_fifo_rd_en[n];

        seg_half_full_reg <= $unsigned(seg_wr_ptr_reg - seg_rd_ptr_reg) >= 2**(CTRL_FIFO_ADDR_WIDTH-1);

        if (ctrl_fifo_wr_en[n]) begin
            seg_mem_sop[seg_wr_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]] <= ctrl_fifo_wr_sop[n];
            seg_mem_eop[seg_wr_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]] <= ctrl_fifo_wr_eop[n];
            seg_mem_end[seg_wr_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]] <= ctrl_fifo_wr_end[n];
            seg_mem_last[seg_wr_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]] <= ctrl_fifo_wr_last[n];
            seg_mem_last_len[seg_wr_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]] <= ctrl_fifo_wr_last_len[AXIS_BYTE_IDX_WIDTH*n +: AXIS_BYTE_IDX_WIDTH];

            seg_wr_ptr_reg <= seg_wr_ptr_reg + 1;
        end

        if (!seg_empty && (!seg_rd_valid_reg || ctrl_fifo_rd_en[n])) begin
            seg_rd_sop_reg <= seg_mem_sop[seg_rd_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_eop_reg <= seg_mem_eop[seg_rd_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_end_reg <= seg_mem_end[seg_rd_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_last_reg <= seg_mem_last[seg_rd_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_last_len_reg <= seg_mem_last_len[seg_rd_ptr_reg[CTRL_FIFO_ADDR_WIDTH-1:0]];
            seg_rd_valid_reg <= 1'b1;

            seg_rd_ptr_reg <= seg_rd_ptr_reg + 1;
        end

        if (rst || fifo_rst_in) begin
            seg_wr_ptr_reg <= 0;
            seg_rd_ptr_reg <= 0;
            seg_rd_valid_reg <= 1'b0;
        end
    end

end

endgenerate

// 解析分段头信息
integer seg; // 头部解析循环索引(段号)。

always @* begin
    input_ctrl_ready_cmb = 0;

    frame_next = frame_reg;
    frame_cyc = frame_reg;
    last_next = last_reg;
    last_cyc = last_reg;
    extra_cycle_next = extra_cycle_reg;
    extra_cycle_cyc = extra_cycle_reg;
    last_straddle_next = last_straddle_reg;
    last_straddle_cyc = last_straddle_reg;
    seg_cnt_next = seg_cnt_reg;
    seg_cnt_cyc = seg_cnt_reg;
    hdr_parity_err_next = 1'b0;
    hdr_parity_err_cyc = 1'b0;

    ctrl_fifo_wr_sop = 0;
    ctrl_fifo_wr_eop = 0;
    ctrl_fifo_wr_end = 0;
    ctrl_fifo_wr_last = 0;
    ctrl_fifo_wr_last_len = 0;
    ctrl_fifo_wr_en = 0;

    // 解码分段头信息
    for (seg = 0; seg < SEG_CNT; seg = seg + 1) begin
        seg_valid[seg] = input_ctrl_valid[seg];
        seg_hdr_start_pkt[seg] = input_ctrl_data[SEG_WIDTH*seg + 0 +: 1];
        seg_hdr_last[seg] = input_ctrl_data[SEG_WIDTH*seg + 1 +: 1];
        seg_hdr_len[seg] = input_ctrl_data[SEG_WIDTH*seg + 4 +: 12];
        seg_hdr_seg_cnt[seg] = (seg_hdr_len[seg] + SEG_BYTE_LANES) >> SEG_BYTE_IDX_WIDTH;
        seg_hdr_last_straddle[seg] = ((seg_hdr_len[seg] & (SEG_BYTE_LANES-1)) + HDR_SIZE) >> SEG_BYTE_IDX_WIDTH != 0;
        seg_hdr_parity_err[seg] = ^input_ctrl_data[SEG_WIDTH*seg + 0 +: 3] || ^input_ctrl_data[SEG_WIDTH*seg + 3 +: 13];
    end
    seg_valid_straddle = {2{seg_valid}} >> 1;

    for (seg = 0; seg < SEG_CNT; seg = seg + 1) begin
        if (!frame_cyc) begin
            if (seg_valid[seg]) begin
                if (seg_hdr_start_pkt[seg]) begin
                    // 帧起始
                    last_cyc = seg_hdr_last[seg];
                    extra_cycle_cyc = 1'b0;
                    last_straddle_cyc = seg_hdr_last_straddle[seg];
                    seg_cnt_cyc = seg_hdr_seg_cnt[seg];

                    ctrl_fifo_wr_sop[seg] = 1'b1;
                    ctrl_fifo_wr_last_len[AXIS_BYTE_IDX_WIDTH*seg +: AXIS_BYTE_IDX_WIDTH] = seg_hdr_len[seg];

                    frame_cyc = 1'b1;
                end else  begin
                    // 消耗空分段
                end

                if (seg_hdr_parity_err[seg]) begin
                    hdr_parity_err_cyc = 1'b1;
                end
            end
        end

        if (frame_cyc) begin
            if (extra_cycle_cyc) begin
                // 额外拍
                frame_cyc = 0;
                extra_cycle_cyc = 0;

                ctrl_fifo_wr_eop[seg] = 1'b1;
            end else if (seg_cnt_cyc == 1) begin
                // 最后一拍输出
                if (last_cyc) begin
                    ctrl_fifo_wr_last[seg] = 1'b1;
                end

                if (last_straddle_cyc) begin
                    // 最后一拍输出，且跨分段边界
                    extra_cycle_cyc = 1'b1;

                    ctrl_fifo_wr_end[seg] = 1'b1;
                end else begin
                    // 最后一拍输出，不跨分段边界
                    frame_cyc = 0;

                    ctrl_fifo_wr_eop[seg] = 1'b1;
                    ctrl_fifo_wr_end[seg] = 1'b1;
                end
            end else begin
                // 中间拍
            end
        end

        seg_cnt_cyc = seg_cnt_cyc - 1;
    end

    if (&seg_valid && !ctrl_fifo_half_full) begin
        input_ctrl_ready_cmb = {SEG_CNT{1'b1}};

        ctrl_fifo_wr_en = {SEG_CNT{1'b1}};

        frame_next = frame_cyc;
        last_next = last_cyc;
        extra_cycle_next = extra_cycle_cyc;
        last_straddle_next = last_straddle_cyc;
        seg_cnt_next = seg_cnt_cyc;
        hdr_parity_err_next = hdr_parity_err_cyc;
    end
end

// 重新打包数据
integer out_seg; // 数据重组循环索引(输出段号)。
reg [SEG_IDX_WIDTH-1:0] out_cur_seg; // 当前处理的输入段索引(含偏移)。

always @* begin
    input_ready_cmb = 0;

    out_frame_next = out_frame_reg;
    out_frame_cyc = out_frame_reg;
    out_seg_offset_next = out_seg_offset_reg;
    out_seg_offset_cyc = out_seg_offset_reg;
    output_offset_next = output_offset_reg;
    // output_offset_cyc = output_offset_reg;  // 调试保留：输出偏移寄存值
    output_offset_cyc = 0;
    out_seg_consumed_next = 0;


    out_ctrl_en_next = 0;
    out_ctrl_hdr_next = 0;
    out_ctrl_last_next = 0;
    out_ctrl_last_len_next = out_ctrl_last_len_reg;
    out_ctrl_seg_offset_next = out_ctrl_seg_offset_reg;

    axis_tid_next = axis_tid_reg;
    axis_tdest_next = axis_tdest_reg;
    axis_tuser_next = axis_tuser_reg;

    input_data_int_next = input_data_int_reg;
    input_valid_int_next = input_valid_int_reg;

    ctrl_fifo_rd_en = 0;

    // 应用分段偏移
    shift_out_seg_valid = {2{ctrl_fifo_rd_valid}} >> out_seg_offset_reg;
    shift_out_seg_valid_straddle = {2{ctrl_fifo_rd_valid}} >> (out_seg_offset_reg+1);
    shift_out_seg_valid_straddle[SEG_CNT-1] = 1'b0; // 发生回绕，因此不可消费
    shift_out_seg_sop = {2{ctrl_fifo_rd_sop}} >> out_seg_offset_reg;
    shift_out_seg_eop = {2{ctrl_fifo_rd_eop}} >> out_seg_offset_reg;
    shift_out_seg_end = {2{ctrl_fifo_rd_end}} >> out_seg_offset_reg;
    shift_out_seg_last = {2{ctrl_fifo_rd_last}} >> out_seg_offset_reg;

    // 提取数据
    out_valid = 0;
    out_valid_straddle = 0;
    out_frame = out_frame_cyc;
    out_abort = 0;
    out_done = 0;
    out_seg_consumed = 0;

    out_ctrl_seg_offset_next = out_seg_offset_reg;

    out_cur_seg = out_seg_offset_reg;
    for (out_seg = 0; out_seg < SEG_CNT; out_seg = out_seg + 1) begin
        out_seg_offset_cyc = out_seg_offset_cyc + 1;

        // 检查连续有效分段
        out_valid = (~shift_out_seg_valid & ({SEG_CNT{1'b1}} >> (SEG_CNT-1 - out_seg))) == 0;
        out_valid_straddle = shift_out_seg_valid_straddle[0];

        if (!out_frame_cyc) begin
            if (out_valid) begin
                if (shift_out_seg_sop[0]) begin
                    // 帧起始
                    out_frame_cyc = 1'b1;

                    if (!out_done) begin
                        out_ctrl_hdr_next = 1'b1;
                        out_ctrl_last_len_next = ctrl_fifo_rd_last_len[AXIS_BYTE_IDX_WIDTH*out_cur_seg +: AXIS_BYTE_IDX_WIDTH];
                        out_ctrl_seg_offset_next = out_cur_seg;
                    end
                end else if (!out_abort) begin
                    // 消耗空分段
                    out_seg_consumed[out_cur_seg] = 1'b1;
                    out_seg_consumed_next = out_seg_consumed;
                    ctrl_fifo_rd_en = out_seg_consumed;

                    out_seg_offset_next = out_seg_offset_cyc;
                end
            end
        end
        out_frame = out_frame_cyc;

        if (out_frame && !out_done) begin
            if (shift_out_seg_end[0]) begin
                // 最后一拍输出
                out_frame_cyc = 0;
                out_done = 1;

                if (shift_out_seg_last[0]) begin
                    out_ctrl_last_next = 1'b1;
                end

                if (out_valid && (out_valid_straddle || shift_out_seg_eop[0]) && m_axis_tready_int) begin
                    out_ctrl_en_next = 1'b1;
                    out_seg_consumed[out_cur_seg] = 1'b1;
                    out_seg_consumed_next = out_seg_consumed;
                    ctrl_fifo_rd_en = out_seg_consumed;
                    out_frame_next = out_frame_cyc;
                    out_seg_offset_next = out_seg_offset_cyc;
                end else begin
                    out_abort = 1'b1;
                end
            end else if (output_offset_cyc == AXIS_SEG_CNT-1) begin
                // 输出已满
                out_done = 1;

                if (out_valid && out_valid_straddle && m_axis_tready_int) begin
                    out_ctrl_en_next = 1'b1;
                    out_seg_consumed[out_cur_seg] = 1'b1;
                    out_seg_consumed_next = out_seg_consumed;
                    ctrl_fifo_rd_en = out_seg_consumed;
                    out_frame_next = out_frame_cyc;
                    out_seg_offset_next = out_seg_offset_cyc;
                end else begin
                    out_abort = 1'b1;
                end
            end else begin
                // 中间拍

                if (out_valid && out_valid_straddle && m_axis_tready_int) begin
                    out_seg_consumed[out_cur_seg] = 1'b1;
                end else begin
                    out_abort = 1'b1;
                end
            end

            if (output_offset_cyc == AXIS_SEG_CNT-1) begin
                output_offset_cyc = 0;
            end else begin
                output_offset_cyc = output_offset_cyc + 1;
            end
        end

        out_cur_seg = out_cur_seg + 1;

        // shift_out_seg_valid = shift_out_seg_valid >> 1;  // 调试保留：有效位图右移
        shift_out_seg_valid_straddle = shift_out_seg_valid_straddle >> 1;
        shift_out_seg_sop = shift_out_seg_sop >> 1;
        shift_out_seg_eop = shift_out_seg_eop >> 1;
        shift_out_seg_end = shift_out_seg_end >> 1;
        shift_out_seg_last = shift_out_seg_last >> 1;
    end

    // 构造输出
    input_ready_cmb = out_seg_consumed_reg;

    m_axis_tdata_int = input_data_full >> (SEG_WIDTH*out_ctrl_seg_offset_reg + HDR_WIDTH);

    if (out_ctrl_last_reg) begin
        m_axis_tkeep_int = {AXIS_KEEP_WIDTH{1'b1}} >> (AXIS_KEEP_WIDTH-1 - out_ctrl_last_len_reg);
    end else begin
        m_axis_tkeep_int = {AXIS_KEEP_WIDTH{1'b1}};
    end
    m_axis_tlast_int = out_ctrl_last_reg;

    if (out_ctrl_hdr_reg) begin
        axis_tid_next = input_data_full >> (SEG_WIDTH*out_ctrl_seg_offset_reg + 16 + META_ID_OFFSET);
        axis_tdest_next = input_data_full >> (SEG_WIDTH*out_ctrl_seg_offset_reg + 16 + META_DEST_OFFSET);
        axis_tuser_next = input_data_full >> (SEG_WIDTH*out_ctrl_seg_offset_reg + 16 + META_USER_OFFSET);
    end

    m_axis_tvalid_int = out_ctrl_en_reg;

    m_axis_tid_int = axis_tid_next;
    m_axis_tdest_int = axis_tdest_next;
    m_axis_tuser_int = axis_tuser_next;

    if (EXPAND_INPUT) begin
        for (seg = 0; seg < SEG_CNT; seg = seg + 1) begin
            if (input_ready[seg] && input_valid[seg]) begin
                input_data_int_next[SEG_WIDTH*seg +: SEG_WIDTH] = input_data[SEG_WIDTH*seg +: SEG_WIDTH];
                input_valid_int_next[seg] = 1'b1;
            end
        end
    end
end

always @(posedge clk) begin
    frame_reg <= frame_next;
    last_reg <= last_next;
    extra_cycle_reg <= extra_cycle_next;
    last_straddle_reg <= last_straddle_next;
    seg_cnt_reg <= seg_cnt_next;
    hdr_parity_err_reg <= hdr_parity_err_next;

    out_frame_reg <= out_frame_next;
    out_seg_offset_reg <= out_seg_offset_next;
    output_offset_reg <= output_offset_next;
    out_seg_consumed_reg <= out_seg_consumed_next;

    input_data_int_reg <= input_data_int_next;
    input_valid_int_reg <= input_valid_int_next;

    out_ctrl_en_reg <= out_ctrl_en_next;
    out_ctrl_hdr_reg <= out_ctrl_hdr_next;
    out_ctrl_last_reg <= out_ctrl_last_next;
    out_ctrl_last_len_reg <= out_ctrl_last_len_next;
    out_ctrl_seg_offset_reg <= out_ctrl_seg_offset_next;

    axis_tid_reg <= axis_tid_next;
    axis_tdest_reg <= axis_tdest_next;
    axis_tuser_reg <= axis_tuser_next;

    if (rst || fifo_rst_in) begin
        frame_reg <= 1'b0;
        hdr_parity_err_reg <= 1'b0;
        out_frame_reg <= 1'b0;
        out_seg_offset_reg <= 0;
        output_offset_reg <= 0;
        out_seg_consumed_reg <= 0;
        input_valid_int_next <= 1'b0;
        out_ctrl_en_reg <= 1'b0;
    end
end

// 输出数据通路逻辑
reg [AXIS_DATA_WIDTH-1:0] m_axis_tdata_reg  = {AXIS_DATA_WIDTH{1'b0}}; // AXIS 输出寄存器：tdata。
reg [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep_reg  = {AXIS_KEEP_WIDTH{1'b0}}; // AXIS 输出寄存器：tkeep。
reg                       m_axis_tvalid_reg = 1'b0; // AXIS 输出寄存器：tvalid。
reg                       m_axis_tlast_reg  = 1'b0; // AXIS 输出寄存器：tlast。
reg [AXIS_ID_WIDTH-1:0]   m_axis_tid_reg    = {AXIS_ID_WIDTH{1'b0}}; // AXIS 输出寄存器：tid。
reg [AXIS_DEST_WIDTH-1:0] m_axis_tdest_reg  = {AXIS_DEST_WIDTH{1'b0}}; // AXIS 输出寄存器：tdest。
reg [AXIS_USER_WIDTH-1:0] m_axis_tuser_reg  = {AXIS_USER_WIDTH{1'b0}}; // AXIS 输出寄存器：tuser。

reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_wr_ptr_reg = 0; // 输出 FIFO 写指针(含额外位区分满/空)。
reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_rd_ptr_reg = 0; // 输出 FIFO 读指针(含额外位区分满/空)。
reg out_fifo_half_full_reg = 1'b0; // 输出 FIFO 半满标志(给前级背压)。

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

assign m_axis_tready_int = !out_fifo_half_full_reg;

assign m_axis_tdata  = m_axis_tdata_reg;
assign m_axis_tkeep  = AXIS_KEEP_ENABLE ? m_axis_tkeep_reg : {AXIS_KEEP_WIDTH{1'b1}};
assign m_axis_tvalid = m_axis_tvalid_reg;
assign m_axis_tlast  = AXIS_LAST_ENABLE ? m_axis_tlast_reg : 1'b1;
assign m_axis_tid    = AXIS_ID_ENABLE   ? m_axis_tid_reg   : {AXIS_ID_WIDTH{1'b0}};
assign m_axis_tdest  = AXIS_DEST_ENABLE ? m_axis_tdest_reg : {AXIS_DEST_WIDTH{1'b0}};
assign m_axis_tuser  = AXIS_USER_ENABLE ? m_axis_tuser_reg : {AXIS_USER_WIDTH{1'b0}};

always @(posedge clk) begin
    m_axis_tvalid_reg <= m_axis_tvalid_reg && !m_axis_tready;

    out_fifo_half_full_reg <= $unsigned(out_fifo_wr_ptr_reg - out_fifo_rd_ptr_reg) >= 2**(OUTPUT_FIFO_ADDR_WIDTH-1);

    if (!out_fifo_full && m_axis_tvalid_int) begin
        out_fifo_tdata[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tdata_int;
        out_fifo_tkeep[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tkeep_int;
        out_fifo_tlast[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tlast_int;
        out_fifo_tid[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tid_int;
        out_fifo_tdest[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tdest_int;
        out_fifo_tuser[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_tuser_int;
        out_fifo_wr_ptr_reg <= out_fifo_wr_ptr_reg + 1;
    end

    if (!out_fifo_empty && (!m_axis_tvalid_reg || m_axis_tready)) begin
        m_axis_tdata_reg <= out_fifo_tdata[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_tkeep_reg <= out_fifo_tkeep[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_tvalid_reg <= 1'b1;
        m_axis_tlast_reg <= out_fifo_tlast[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_tid_reg <= out_fifo_tid[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_tdest_reg <= out_fifo_tdest[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_tuser_reg <= out_fifo_tuser[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        out_fifo_rd_ptr_reg <= out_fifo_rd_ptr_reg + 1;
    end

    if (rst || fifo_rst_in) begin
        out_fifo_wr_ptr_reg <= 0;
        out_fifo_rd_ptr_reg <= 0;
        m_axis_tvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
