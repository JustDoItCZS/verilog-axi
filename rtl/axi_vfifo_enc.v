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
 * AXI4 虚拟 FIFO（编码器）
 *
 * 模块目录
 * 1) 把 AXIS 输入帧按段编码为虚拟 FIFO 通道数据与控制头。
 * 2) 维护输入数据 FIFO 与头信息 FIFO，支持按块提交与超时拆包。
 * 3) 负责把 tid/tdest/tuser 元数据压入头部并在输出阶段恢复控制结构。
 */
module axi_vfifo_enc #
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
    input  wire                          clk, // 编码器时钟。
    input  wire                          rst, // 同步复位，高电平有效。

    /*
     * AXI-Stream 数据输入
     */
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata, // AXIS 输入 tdata。
    input  wire [AXIS_KEEP_WIDTH-1:0]    s_axis_tkeep, // AXIS 输入 tkeep。
    input  wire                          s_axis_tvalid, // AXIS 输入 tvalid。
    output wire                          s_axis_tready, // AXIS 输入 tready。
    input  wire                          s_axis_tlast, // AXIS 输入 tlast。
    input  wire [AXIS_ID_WIDTH-1:0]      s_axis_tid, // AXIS 输入 tid。
    input  wire [AXIS_DEST_WIDTH-1:0]    s_axis_tdest, // AXIS 输入 tdest。
    input  wire [AXIS_USER_WIDTH-1:0]    s_axis_tuser, // AXIS 输入 tuser。

    /*
     * 分段数据输出（到虚拟 FIFO 通道）
     */
    input  wire                          fifo_rst_in, // 来自虚拟 FIFO 通道的同步复位请求。
    output wire [SEG_CNT*SEG_WIDTH-1:0]  output_data, // 分段数据输出总线。
    output wire [SEG_CNT-1:0]            output_valid, // 分段数据有效位。
    input  wire                          fifo_watermark_in // 下游水位告警输入(触发提前提交)。
);

parameter AXIS_KEEP_WIDTH_INT = AXIS_KEEP_ENABLE ? AXIS_KEEP_WIDTH : 1;
parameter AXIS_BYTE_LANES = AXIS_KEEP_WIDTH_INT;
parameter AXIS_BYTE_SIZE = AXIS_DATA_WIDTH/AXIS_BYTE_LANES;
parameter CL_AXIS_BYTE_LANES = $clog2(AXIS_BYTE_LANES);

parameter BYTE_SIZE = AXIS_BYTE_SIZE;

parameter SEG_BYTE_LANES = SEG_WIDTH / BYTE_SIZE;

parameter EXPAND_OUTPUT = SEG_CNT < 2;

parameter SEG_CNT_INT = EXPAND_OUTPUT ? SEG_CNT*2 : SEG_CNT;

parameter SEG_IDX_WIDTH = $clog2(SEG_CNT_INT);
parameter SEG_BYTE_IDX_WIDTH = $clog2(SEG_BYTE_LANES);

parameter AXIS_SEG_CNT = (AXIS_DATA_WIDTH + SEG_WIDTH-1) / SEG_WIDTH;
parameter AXIS_SEG_IDX_WIDTH = AXIS_SEG_CNT > 1 ? $clog2(AXIS_SEG_CNT) : 1;
parameter AXIS_LEN_MASK = AXIS_BYTE_LANES-1;

parameter IN_OFFS_WIDTH = AXIS_SEG_IDX_WIDTH;

parameter META_ID_OFFSET = 0;
parameter META_DEST_OFFSET = META_ID_OFFSET + (AXIS_ID_ENABLE ? AXIS_ID_WIDTH : 0);
parameter META_USER_OFFSET = META_DEST_OFFSET + (AXIS_DEST_ENABLE ? AXIS_DEST_WIDTH : 0);
parameter META_WIDTH = META_USER_OFFSET + (AXIS_USER_ENABLE ? AXIS_USER_WIDTH : 0);
parameter HDR_SIZE = (16 + META_WIDTH + BYTE_SIZE-1) / BYTE_SIZE;
parameter HDR_WIDTH = HDR_SIZE * BYTE_SIZE;

parameter HDR_LEN_WIDTH = 12;
parameter HDR_SEG_LEN_WIDTH = HDR_LEN_WIDTH-SEG_BYTE_IDX_WIDTH;

parameter INPUT_FIFO_ADDR_WIDTH = 5;
parameter HDR_FIFO_ADDR_WIDTH = INPUT_FIFO_ADDR_WIDTH + SEG_IDX_WIDTH;

parameter INPUT_FIFO_PTR_WIDTH = INPUT_FIFO_ADDR_WIDTH + SEG_IDX_WIDTH;
parameter HDR_FIFO_PTR_WIDTH = HDR_FIFO_ADDR_WIDTH;

parameter INPUT_FIFO_SIZE = SEG_BYTE_LANES * SEG_CNT_INT * 2**INPUT_FIFO_ADDR_WIDTH;

parameter MAX_BLOCK_LEN = INPUT_FIFO_SIZE / 2 > 4096 ? 4096 : INPUT_FIFO_SIZE / 2;

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

    if (SEG_WIDTH < HDR_SIZE*BYTE_SIZE) begin
        $error("Error: Segment smaller than header (instance %m)");
        $finish;
    end
end

reg [INPUT_FIFO_PTR_WIDTH+1-1:0] input_fifo_wr_ptr_reg = 0, input_fifo_wr_ptr_next; // 输入数据 FIFO 写指针。
reg [INPUT_FIFO_PTR_WIDTH+1-1:0] input_fifo_rd_ptr_reg = 0, input_fifo_rd_ptr_next; // 输入数据 FIFO 读指针。
reg [HDR_FIFO_PTR_WIDTH+1-1:0] hdr_fifo_wr_ptr_reg = 0, hdr_fifo_wr_ptr_next; // 头信息 FIFO 写指针。
reg [HDR_FIFO_PTR_WIDTH+1-1:0] hdr_fifo_rd_ptr_reg = 0, hdr_fifo_rd_ptr_next; // 头信息 FIFO 读指针。

reg [SEG_CNT_INT-1:0] mem_rd_data_valid_reg = 0, mem_rd_data_valid_next; // 输入段 RAM 读数据有效标志。
reg hdr_mem_rd_data_valid_reg = 0, hdr_mem_rd_data_valid_next; // 头信息 RAM 读数据有效标志。

reg [AXIS_DATA_WIDTH-1:0] int_seg_data; // 按分段展开的输入数据。
reg [AXIS_SEG_CNT-1:0] int_seg_valid; // 按分段展开的输入有效位。

reg [SEG_CNT_INT*SEG_WIDTH-1:0] seg_mem_wr_data; // 段 RAM 写数据总线。
reg [SEG_CNT_INT-1:0] seg_mem_wr_valid; // 段 RAM 每段写有效。
reg [SEG_CNT_INT*INPUT_FIFO_ADDR_WIDTH-1:0] seg_mem_wr_addr_reg = 0, seg_mem_wr_addr_next; // 段 RAM 每段写地址。
reg [SEG_CNT_INT-1:0] seg_mem_wr_en; // 段 RAM 每段写使能。
reg [SEG_CNT_INT*SEG_IDX_WIDTH-1:0] seg_mem_wr_sel; // 段 RAM 写数据源段选择。

wire [SEG_CNT_INT*SEG_WIDTH-1:0] seg_mem_rd_data; // 段 RAM 读数据总线。
reg [SEG_CNT_INT*INPUT_FIFO_ADDR_WIDTH-1:0] seg_mem_rd_addr_reg = 0, seg_mem_rd_addr_next; // 段 RAM 每段读地址。
reg [SEG_CNT_INT-1:0] seg_mem_rd_en; // 段 RAM 每段读使能。

reg [HDR_LEN_WIDTH-1:0] hdr_mem_wr_len; // 头 RAM 写入长度字段。
reg hdr_mem_wr_last; // 头 RAM 写入 last 标志。
reg [META_WIDTH-1:0] hdr_mem_wr_meta; // 头 RAM 写入元数据。
reg [HDR_FIFO_ADDR_WIDTH-1:0] hdr_mem_wr_addr; // 头 RAM 写地址。
reg hdr_mem_wr_en; // 头 RAM 写使能。

wire [HDR_LEN_WIDTH-1:0] hdr_mem_rd_len; // 头 RAM 读出长度字段。
wire hdr_mem_rd_last; // 头 RAM 读出 last 标志。
wire [META_WIDTH-1:0] hdr_mem_rd_meta; // 头 RAM 读出元数据。
reg [HDR_FIFO_ADDR_WIDTH-1:0] hdr_mem_rd_addr_reg = 0, hdr_mem_rd_addr_next; // 头 RAM 读地址寄存器。
reg hdr_mem_rd_en; // 头 RAM 读使能。

reg input_fifo_full_reg = 1'b0; // 输入数据 FIFO 满标志。
reg input_fifo_half_full_reg = 1'b0; // 输入数据 FIFO 半满标志。
reg input_fifo_empty_reg = 1'b1; // 输入数据 FIFO 空标志。
reg [INPUT_FIFO_PTR_WIDTH+1-1:0] input_fifo_count_reg = 0; // 输入数据 FIFO 占用计数。
reg hdr_fifo_full_reg = 1'b0; // 头信息 FIFO 满标志。
reg hdr_fifo_half_full_reg = 1'b0; // 头信息 FIFO 半满标志。
reg hdr_fifo_empty_reg = 1'b1; // 头信息 FIFO 空标志。
reg [HDR_FIFO_PTR_WIDTH+1-1:0] hdr_fifo_count_reg = 0; // 头信息 FIFO 占用计数。

reg [SEG_CNT*SEG_WIDTH-1:0] output_data_reg = 0, output_data_next; // 分段输出数据寄存器。
reg [SEG_CNT-1:0] output_valid_reg = 0, output_valid_next; // 分段输出有效寄存器。

assign s_axis_tready = !input_fifo_full_reg && !hdr_fifo_full_reg && !fifo_rst_in;

assign output_data = output_data_reg;
assign output_valid = output_valid_reg;

generate

genvar n;

for (n = 0; n < SEG_CNT_INT; n = n + 1) begin : seg_ram

    (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
    reg [SEG_WIDTH-1:0] seg_mem_data[2**INPUT_FIFO_ADDR_WIDTH-1:0]; // 第 n 段输入数据 RAM。

    wire wr_en = seg_mem_wr_en[n]; // 第 n 段写使能。
    wire [INPUT_FIFO_ADDR_WIDTH-1:0] wr_addr = seg_mem_wr_addr_reg[n*INPUT_FIFO_ADDR_WIDTH +: INPUT_FIFO_ADDR_WIDTH]; // 第 n 段写地址。
    wire [SEG_WIDTH-1:0] wr_data = seg_mem_wr_data[n*SEG_WIDTH +: SEG_WIDTH]; // 第 n 段写数据。

    wire rd_en = seg_mem_rd_en[n]; // 第 n 段读使能。
    wire [INPUT_FIFO_ADDR_WIDTH-1:0] rd_addr = seg_mem_rd_addr_reg[n*INPUT_FIFO_ADDR_WIDTH +: INPUT_FIFO_ADDR_WIDTH]; // 第 n 段读地址。
    reg [SEG_WIDTH-1:0] rd_data_reg = 0; // 第 n 段读数据寄存器。

    assign seg_mem_rd_data[n*SEG_WIDTH +: SEG_WIDTH] = rd_data_reg;

    always @(posedge clk) begin
        if (wr_en) begin
            seg_mem_data[wr_addr] <= wr_data;
        end
    end

    always @(posedge clk) begin
        if (rd_en) begin
            rd_data_reg <= seg_mem_data[rd_addr];
        end
    end

end

endgenerate

(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [HDR_LEN_WIDTH-1:0] hdr_mem_len[2**HDR_FIFO_ADDR_WIDTH-1:0]; // 头 RAM：长度字段。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg hdr_mem_last[2**HDR_FIFO_ADDR_WIDTH-1:0]; // 头 RAM：last 标志。
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [META_WIDTH-1:0] hdr_mem_meta[2**HDR_FIFO_ADDR_WIDTH-1:0]; // 头 RAM：元数据字段。

reg [HDR_LEN_WIDTH-1:0] hdr_mem_rd_len_reg = 0; // 头 RAM 读寄存：长度。
reg hdr_mem_rd_last_reg = 1'b0; // 头 RAM 读寄存：last。
reg [META_WIDTH-1:0] hdr_mem_rd_meta_reg = 0; // 头 RAM 读寄存：元数据。

assign hdr_mem_rd_len = hdr_mem_rd_len_reg;
assign hdr_mem_rd_last = hdr_mem_rd_last_reg;
assign hdr_mem_rd_meta = hdr_mem_rd_meta_reg;

always @(posedge clk) begin
    if (hdr_mem_wr_en) begin
        hdr_mem_len[hdr_mem_wr_addr] <= hdr_mem_wr_len;
        hdr_mem_last[hdr_mem_wr_addr] <= hdr_mem_wr_last;
        hdr_mem_meta[hdr_mem_wr_addr] <= hdr_mem_wr_meta;
    end
end

always @(posedge clk) begin
    if (hdr_mem_rd_en) begin
        hdr_mem_rd_len_reg <= hdr_mem_len[hdr_mem_rd_addr_reg];
        hdr_mem_rd_last_reg <= hdr_mem_last[hdr_mem_rd_addr_reg];
        hdr_mem_rd_meta_reg <= hdr_mem_meta[hdr_mem_rd_addr_reg];
    end
end

// 限制条件计算
always @(posedge clk) begin
    input_fifo_full_reg <= $unsigned(input_fifo_wr_ptr_reg - input_fifo_rd_ptr_reg) >= (2**INPUT_FIFO_ADDR_WIDTH*SEG_CNT_INT)-SEG_CNT_INT*2;
    input_fifo_half_full_reg <= $unsigned(input_fifo_wr_ptr_reg - input_fifo_rd_ptr_reg) >= (2**INPUT_FIFO_ADDR_WIDTH*SEG_CNT_INT)/2;
    hdr_fifo_full_reg <= $unsigned(hdr_fifo_wr_ptr_reg - hdr_fifo_rd_ptr_reg) >= 2**HDR_FIFO_ADDR_WIDTH-4;
    hdr_fifo_half_full_reg <= $unsigned(hdr_fifo_wr_ptr_reg - hdr_fifo_rd_ptr_reg) >= 2**HDR_FIFO_ADDR_WIDTH/2;

    if (rst) begin
        input_fifo_full_reg <= 1'b0;
        input_fifo_half_full_reg <= 1'b0;
        hdr_fifo_full_reg <= 1'b0;
        hdr_fifo_half_full_reg <= 1'b0;
    end
end

// 输入分段拆分
integer si; // 输入 AXIS 段拆分循环索引。

always @* begin
    int_seg_data = s_axis_tdata;
    int_seg_valid = 0;

    if (s_axis_tvalid) begin
        if (s_axis_tlast) begin
            for (si = 0; si < AXIS_SEG_CNT; si = si + 1) begin
                int_seg_valid[si] = s_axis_tkeep[SEG_BYTE_LANES*si +: SEG_BYTE_LANES] != 0;
            end
        end else begin
            int_seg_valid = {AXIS_SEG_CNT{1'b1}};
        end
    end else begin
        int_seg_valid = 0;
    end
end

// 写入控制逻辑
integer seg, k; // 写路径组合处理循环索引（seg: 段, k: 字节通道）。
reg [SEG_IDX_WIDTH+1-1:0] seg_count; // 本拍有效段数量计数。
reg [SEG_IDX_WIDTH-1:0] cur_seg; // 当前写入段位置索引。

reg frame_reg = 1'b0, frame_next; // 当前是否处于帧聚合状态。
reg [HDR_LEN_WIDTH-1:0] len_reg = 0, len_next; // 当前聚合帧长度计数。

reg cycle_valid_reg = 1'b0, cycle_valid_next; // 缓存拍是否有效。
reg cycle_last_reg = 1'b0, cycle_last_next; // 缓存拍是否 tlast。
reg [CL_AXIS_BYTE_LANES+1-1:0] cycle_len_reg = 0, cycle_len_next; // 缓存拍有效字节数。
reg [META_WIDTH-1:0] cycle_meta_reg = 0, cycle_meta_next; // 缓存拍元数据。

reg [CL_AXIS_BYTE_LANES+1-1:0] cycle_len; // 当前输入拍有效字节数(组合计算)。

reg [HDR_LEN_WIDTH-1:0] hdr_len_reg = 0, hdr_len_next; // 待写头信息长度字段。
reg [META_WIDTH-1:0] hdr_meta_reg = 0, hdr_meta_next; // 待写头信息元数据字段。
reg hdr_last_reg = 0, hdr_last_next; // 待写头信息的最后一拍标志。
reg hdr_commit_reg = 0, hdr_commit_next; // 当前头是否提交。
reg hdr_commit_prev_reg = 0, hdr_commit_prev_next; // 前一头是否提交(元数据切换场景)。
reg hdr_valid_reg = 0, hdr_valid_next; // 待写头信息有效标志。

wire [META_WIDTH-1:0] s_axis_meta; // 从 AXIS 侧带信号打包后的元数据字段。

generate

if (AXIS_ID_ENABLE) assign s_axis_meta[META_ID_OFFSET +: AXIS_ID_WIDTH] = s_axis_tid;
if (AXIS_DEST_ENABLE) assign s_axis_meta[META_DEST_OFFSET +: AXIS_DEST_WIDTH] = s_axis_tdest;
if (AXIS_USER_ENABLE) assign s_axis_meta[META_USER_OFFSET +: AXIS_USER_WIDTH] = s_axis_tuser;

endgenerate

always @* begin
    input_fifo_wr_ptr_next = input_fifo_wr_ptr_reg;
    hdr_fifo_wr_ptr_next = hdr_fifo_wr_ptr_reg;

    if (AXIS_KEEP_ENABLE) begin
        cycle_len = 0;
        for (k = 0; k < AXIS_BYTE_LANES; k = k + 1) begin
            cycle_len = cycle_len + s_axis_tkeep[k];
        end
    end else begin
        cycle_len = AXIS_BYTE_LANES;
    end

    // 打包分段
    seg_mem_wr_valid = 0;
    seg_mem_wr_sel = 0;
    cur_seg = input_fifo_wr_ptr_reg[SEG_IDX_WIDTH-1:0];
    seg_count = 0;
    for (seg = 0; seg < AXIS_SEG_CNT; seg = seg + 1) begin
        if (int_seg_valid[seg]) begin
            seg_mem_wr_valid[cur_seg +: 1] = 1'b1;
            seg_mem_wr_sel[cur_seg*SEG_IDX_WIDTH +: SEG_IDX_WIDTH] = seg;
            cur_seg = cur_seg + 1;
            seg_count = seg_count + 1;
        end
    end

    for (seg = 0; seg < SEG_CNT_INT; seg = seg + 1) begin
        seg_mem_wr_data[seg*SEG_WIDTH +: SEG_WIDTH] = int_seg_data[seg_mem_wr_sel[seg*SEG_IDX_WIDTH +: SEG_IDX_WIDTH]*SEG_WIDTH +: SEG_WIDTH];
    end

    seg_mem_wr_addr_next = seg_mem_wr_addr_reg;
    seg_mem_wr_en = 0;

    hdr_mem_wr_len = hdr_len_reg;
    hdr_mem_wr_last = hdr_last_reg;
    hdr_mem_wr_meta = hdr_meta_reg;
    hdr_mem_wr_addr = hdr_fifo_wr_ptr_reg;
    hdr_mem_wr_en = 1'b0;

    frame_next = frame_reg;
    len_next = len_reg;

    cycle_valid_next = 1'b0;
    cycle_last_next = cycle_last_reg;
    cycle_len_next = cycle_len_reg;
    cycle_meta_next = cycle_meta_reg;

    hdr_len_next = len_reg;
    hdr_meta_next = cycle_meta_reg;
    hdr_last_next = cycle_last_reg;
    hdr_commit_next = 1'b0;
    hdr_commit_prev_next = 1'b0;
    hdr_valid_next = 1'b0;

    if (s_axis_tvalid && s_axis_tready) begin
        // 传输数据
        seg_mem_wr_en = seg_mem_wr_valid;
        input_fifo_wr_ptr_next = input_fifo_wr_ptr_reg + seg_count;
        for (seg = 0; seg < SEG_CNT_INT; seg = seg + 1) begin
            seg_mem_wr_addr_next[seg*INPUT_FIFO_ADDR_WIDTH +: INPUT_FIFO_ADDR_WIDTH] = (input_fifo_wr_ptr_next + (SEG_CNT_INT-1 - seg)) >> SEG_IDX_WIDTH;
        end

        cycle_valid_next = 1'b1;
        cycle_last_next = s_axis_tlast;
        cycle_len_next = cycle_len;
        cycle_meta_next = s_axis_meta;
    end

    if (cycle_valid_reg) begin
        // 处理数据包
        if (!frame_reg) begin
            frame_next = 1'b1;

            if (cycle_last_reg) begin
                len_next = cycle_len_reg;
            end else begin
                len_next = AXIS_BYTE_LANES;
            end

            hdr_len_next = len_next-1;
            hdr_meta_next = cycle_meta_reg;
            hdr_last_next = cycle_last_reg;
            hdr_valid_next = 1'b1;

            if (cycle_last_reg) begin
                // 帧结束

                hdr_commit_next = 1'b1;

                frame_next = 1'b0;
            end
        end else begin
            if (cycle_meta_reg != hdr_meta_reg) begin
                if (cycle_last_reg) begin
                    len_next = cycle_len_reg;
                end else begin
                    len_next = AXIS_BYTE_LANES;
                end
            end else begin
                if (cycle_last_reg) begin
                    len_next = len_reg + cycle_len_reg;
                end else begin
                    len_next = len_reg + AXIS_BYTE_LANES;
                end
            end

            hdr_len_next = len_next-1;
            hdr_meta_next = cycle_meta_reg;
            hdr_last_next = cycle_last_reg;
            hdr_valid_next = 1'b1;

            if (cycle_meta_reg != hdr_meta_reg) begin
                // 元数据变化

                hdr_commit_prev_next = 1'b1;

                if (cycle_last_reg) begin
                    hdr_commit_next = 1'b1;
                    frame_next = 1'b0;
                end
            end else if (cycle_last_reg || len_next >= MAX_BLOCK_LEN) begin
                // 帧结束或当前块已满

                hdr_commit_next = 1'b1;

                frame_next = 1'b0;
            end
        end
    end

    if (hdr_valid_reg) begin
        hdr_mem_wr_len = hdr_len_reg;
        hdr_mem_wr_last = hdr_last_reg;
        hdr_mem_wr_meta = hdr_meta_reg;
        hdr_mem_wr_addr = hdr_fifo_wr_ptr_reg;
        hdr_mem_wr_en = 1'b1;

        if (hdr_commit_prev_reg) begin
            if (hdr_commit_reg) begin
                hdr_fifo_wr_ptr_next = hdr_fifo_wr_ptr_reg + 2;
                hdr_mem_wr_addr = hdr_fifo_wr_ptr_reg + 1;
            end else begin
                hdr_fifo_wr_ptr_next = hdr_fifo_wr_ptr_reg + 1;
                hdr_mem_wr_addr = hdr_fifo_wr_ptr_reg + 1;
            end
        end else begin
            if (hdr_commit_reg) begin
                hdr_fifo_wr_ptr_next = hdr_fifo_wr_ptr_reg + 1;
                hdr_mem_wr_addr = hdr_fifo_wr_ptr_reg;
            end
        end
    end
end

always @(posedge clk) begin
    input_fifo_wr_ptr_reg <= input_fifo_wr_ptr_next;
    hdr_fifo_wr_ptr_reg <= hdr_fifo_wr_ptr_next;

    seg_mem_wr_addr_reg <= seg_mem_wr_addr_next;

    frame_reg <= frame_next;
    len_reg <= len_next;

    cycle_valid_reg <= cycle_valid_next;
    cycle_last_reg <= cycle_last_next;
    cycle_len_reg <= cycle_len_next;
    cycle_meta_reg <= cycle_meta_next;

    hdr_len_reg <= hdr_len_next;
    hdr_meta_reg <= hdr_meta_next;
    hdr_last_reg <= hdr_last_next;
    hdr_commit_reg <= hdr_commit_next;
    hdr_commit_prev_reg <= hdr_commit_prev_next;
    hdr_valid_reg <= hdr_valid_next;

    if (rst || fifo_rst_in) begin
        input_fifo_wr_ptr_reg <= 0;
        hdr_fifo_wr_ptr_reg <= 0;

        seg_mem_wr_addr_reg <= 0;

        frame_reg <= 1'b0;

        cycle_valid_reg <= 1'b0;
        hdr_valid_reg <= 1'b0;
    end
end

// 读出控制逻辑
integer rd_seg; // 读路径段处理循环索引。
reg [SEG_IDX_WIDTH-1:0] cur_rd_seg; // 当前读取段索引。
reg rd_valid; // 当前读段是否有效。

reg out_frame_reg = 1'b0, out_frame_next; // 输出组包是否处于帧内。
reg [HDR_LEN_WIDTH-1:0] out_len_reg = 0, out_len_next; // 当前输出帧剩余长度。
reg out_split1_reg = 1'b0, out_split1_next; // 当前输出是否处于 split1 阶段。
reg [HDR_SEG_LEN_WIDTH-1:0] out_seg_cnt_in_reg = 0, out_seg_cnt_in_next; // 输入段计数缓存。
reg out_seg_last_straddle_reg = 1'b0, out_seg_last_straddle_next; // 最后一段是否跨越分段边界。
reg [SEG_IDX_WIDTH-1:0] out_seg_offset_reg = 0, out_seg_offset_next; // 输出段偏移。
reg [SEG_IDX_WIDTH-1:0] out_seg_fifo_offset_reg = 0, out_seg_fifo_offset_next; // FIFO 段偏移。
reg [SEG_IDX_WIDTH+1-1:0] out_seg_count_reg = 0, out_seg_count_next; // 本拍输出段计数。

reg [HDR_WIDTH-1:0] out_hdr_reg = 0, out_hdr_next; // 当前输出头缓存。

reg [SEG_CNT_INT-1:0] out_ctl_seg_hdr_reg = 0, out_ctl_seg_hdr_next, out_ctl_seg_hdr_raw; // 每段是否输出头数据标志。
reg [SEG_CNT_INT-1:0] out_ctl_seg_split1_reg = 0, out_ctl_seg_split1_next, out_ctl_seg_split1_raw; // 每段 split1 阶段标志。
reg [SEG_CNT_INT-1:0] out_ctl_seg_en_reg = 0, out_ctl_seg_en_next, out_ctl_seg_en_raw; // 每段输出使能标志。
reg [SEG_IDX_WIDTH-1:0] out_ctl_seg_idx_reg[SEG_CNT_INT-1:0], out_ctl_seg_idx_next[SEG_CNT_INT-1:0]; // 每段对应的输入段索引。
reg [SEG_IDX_WIDTH-1:0] out_ctl_seg_offset_reg = 0, out_ctl_seg_offset_next; // 输出段偏移控制寄存器。

reg [HDR_WIDTH-1:0] out_shift_reg = 0, out_shift_next; // 头部跨段移位缓存。

reg [7:0] block_timeout_count_reg = 0, block_timeout_count_next; // 部分块超时计数器。
reg block_timeout_reg = 0, block_timeout_next; // 部分块超时触发标志。

always @* begin
    input_fifo_rd_ptr_next = input_fifo_rd_ptr_reg;
    hdr_fifo_rd_ptr_next = hdr_fifo_rd_ptr_reg;

    mem_rd_data_valid_next = mem_rd_data_valid_reg;
    hdr_mem_rd_data_valid_next = hdr_mem_rd_data_valid_reg;

    output_data_next = output_data_reg;
    output_valid_next = 0;

    seg_mem_rd_addr_next = seg_mem_rd_addr_reg;
    seg_mem_rd_en = 0;

    hdr_mem_rd_addr_next = hdr_mem_rd_addr_reg;
    hdr_mem_rd_en = 0;

    out_frame_next = out_frame_reg;
    out_len_next = out_len_reg;
    out_split1_next = out_split1_reg;
    out_seg_cnt_in_next = out_seg_cnt_in_reg;
    out_seg_last_straddle_next = out_seg_last_straddle_reg;
    out_seg_offset_next = out_seg_offset_reg;
    out_seg_fifo_offset_next = out_seg_fifo_offset_reg;

    out_hdr_next = out_hdr_reg;

    out_ctl_seg_hdr_raw = 0;
    out_ctl_seg_hdr_next = 0;
    out_ctl_seg_split1_raw = 0;
    out_ctl_seg_split1_next = 0;
    out_ctl_seg_en_raw = 0;
    out_ctl_seg_en_next = 0;
    out_ctl_seg_offset_next = out_seg_offset_reg;

    for (seg = 0; seg < SEG_CNT_INT; seg = seg + 1) begin
        out_ctl_seg_idx_next[seg] = out_seg_fifo_offset_reg - out_seg_offset_reg + seg;
    end

    // 部分块超时处理
    block_timeout_count_next = block_timeout_count_reg;
    block_timeout_next = block_timeout_count_reg == 0;
    if (output_valid || out_seg_offset_reg == 0) begin
        block_timeout_count_next = 8'hff;
        block_timeout_next = 1'b0;
    end else if (block_timeout_count_reg > 0) begin
        block_timeout_count_next = block_timeout_count_reg - 1;
    end

    // 处理头信息并生成输出控制命令
    if (!fifo_watermark_in) begin
        if (out_frame_reg) begin
            if (out_seg_cnt_in_next >= SEG_CNT_INT) begin
                out_frame_next = out_seg_last_straddle_next || out_seg_cnt_in_next > SEG_CNT_INT;
                out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}};
                out_seg_offset_next = out_seg_offset_reg + SEG_CNT_INT;
                out_seg_fifo_offset_next = out_seg_fifo_offset_reg + SEG_CNT_INT;
            end else begin
                out_frame_next = 1'b0;
                if (out_seg_last_straddle_next) begin
                    out_ctl_seg_split1_raw = 1 << out_seg_cnt_in_next;
                    if (out_seg_cnt_in_next == SEG_CNT_INT-1) begin
                        out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}};
                    end else begin
                        out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}} >> (SEG_CNT_INT - (out_seg_cnt_in_next+1));
                    end
                    out_seg_offset_next = out_seg_offset_reg + out_seg_cnt_in_next+1;
                end else begin
                    out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}} >> (SEG_CNT_INT - out_seg_cnt_in_next);
                    out_seg_offset_next = out_seg_offset_reg + out_seg_cnt_in_next;
                end
                out_seg_fifo_offset_next = out_seg_fifo_offset_reg + out_seg_cnt_in_next;
            end

            out_seg_cnt_in_next = out_seg_cnt_in_next - SEG_CNT_INT;
        end else begin
            out_len_next = hdr_mem_rd_len;
            out_seg_cnt_in_next = (hdr_mem_rd_len + SEG_BYTE_LANES) >> SEG_BYTE_IDX_WIDTH;
            out_seg_last_straddle_next = ((hdr_mem_rd_len & (SEG_BYTE_LANES-1)) + HDR_SIZE) >> SEG_BYTE_IDX_WIDTH != 0;
            out_hdr_next = 0;
            out_hdr_next[0] = 1'b1;
            out_hdr_next[1] = hdr_mem_rd_last;
            out_hdr_next[2] = !hdr_mem_rd_last;
            out_hdr_next[15:4] = hdr_mem_rd_len;
            out_hdr_next[3] = ^hdr_mem_rd_len;
            if (META_WIDTH > 0) begin
                out_hdr_next[16 +: META_WIDTH] = hdr_mem_rd_meta;
            end

            out_ctl_seg_hdr_raw = 1;

            if (hdr_mem_rd_data_valid_reg) begin
                if (out_seg_cnt_in_next >= SEG_CNT_INT) begin
                    out_frame_next = out_seg_last_straddle_next || out_seg_cnt_in_next > SEG_CNT_INT;
                    out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}};
                    out_seg_offset_next = out_seg_offset_reg + SEG_CNT_INT;
                    out_seg_fifo_offset_next = out_seg_fifo_offset_reg + SEG_CNT_INT;
                end else begin
                    out_frame_next = 1'b0;
                    if (out_seg_last_straddle_next) begin
                        out_ctl_seg_split1_raw = 1 << out_seg_cnt_in_next;
                        if (out_seg_cnt_in_next == SEG_CNT_INT-1) begin
                            out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}};
                        end else begin
                            out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}} >> (SEG_CNT_INT - (out_seg_cnt_in_next+1));
                        end
                        out_seg_offset_next = out_seg_offset_reg + out_seg_cnt_in_next+1;
                    end else begin
                        out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}} >> (SEG_CNT_INT - out_seg_cnt_in_next);
                        out_seg_offset_next = out_seg_offset_reg + out_seg_cnt_in_next;
                    end
                    out_seg_fifo_offset_next = out_seg_fifo_offset_reg + out_seg_cnt_in_next;
                end

                out_seg_cnt_in_next = out_seg_cnt_in_next - SEG_CNT_INT;

                hdr_mem_rd_data_valid_next = 1'b0;
            end else if (block_timeout_reg && out_seg_offset_reg) begin
                // 插入填充
                out_hdr_next[15:0] = 0;

                out_ctl_seg_en_raw = {SEG_CNT_INT{1'b1}} >> out_seg_offset_reg;
                out_ctl_seg_hdr_raw = {SEG_CNT_INT{1'b1}};
                out_ctl_seg_split1_raw = {SEG_CNT_INT{1'b1}};

                out_seg_offset_next = 0;
            end
        end
    end

    out_ctl_seg_hdr_next = {2{out_ctl_seg_hdr_raw}} >> (SEG_CNT_INT - out_seg_offset_reg);
    out_ctl_seg_split1_next = {2{out_ctl_seg_split1_raw}} >> (SEG_CNT_INT - out_seg_offset_reg);
    out_ctl_seg_en_next = {2{out_ctl_seg_en_raw}} >> (SEG_CNT_INT - out_seg_offset_reg);

    out_shift_next = out_shift_reg;

    // 分段复用
    cur_rd_seg = out_ctl_seg_offset_reg;
    for (rd_seg = 0; rd_seg < SEG_CNT_INT; rd_seg = rd_seg + 1) begin
        output_data_next[cur_rd_seg*SEG_WIDTH +: SEG_WIDTH] = out_shift_next;
        output_data_next[cur_rd_seg*SEG_WIDTH+HDR_WIDTH +: SEG_WIDTH-HDR_WIDTH] = seg_mem_rd_data[out_ctl_seg_idx_reg[cur_rd_seg]*SEG_WIDTH +: SEG_WIDTH-HDR_WIDTH];

        if (out_ctl_seg_hdr_reg[cur_rd_seg]) begin
            output_data_next[cur_rd_seg*SEG_WIDTH +: HDR_WIDTH] = out_hdr_reg;
        end

        output_valid_next[cur_rd_seg] = out_ctl_seg_en_reg[cur_rd_seg];

        if (out_ctl_seg_en_reg[cur_rd_seg] && !out_ctl_seg_split1_reg[cur_rd_seg]) begin
            mem_rd_data_valid_next[out_ctl_seg_idx_reg[cur_rd_seg]] = 1'b0;
        end

        if (out_ctl_seg_en_reg[cur_rd_seg]) begin
            out_shift_next = seg_mem_rd_data[(out_ctl_seg_idx_reg[cur_rd_seg]+1)*SEG_WIDTH-HDR_WIDTH +: HDR_WIDTH];
        end

        cur_rd_seg = cur_rd_seg + 1;
    end

    // 读取分段数据
    cur_rd_seg = input_fifo_rd_ptr_reg[SEG_IDX_WIDTH-1:0];
    rd_valid = 1;
    for (rd_seg = 0; rd_seg < SEG_CNT_INT; rd_seg = rd_seg + 1) begin
        if (!mem_rd_data_valid_next[cur_rd_seg] && input_fifo_count_reg > rd_seg && rd_valid) begin
            input_fifo_rd_ptr_next = input_fifo_rd_ptr_reg + rd_seg+1;
            seg_mem_rd_en[cur_rd_seg] = 1'b1;
            seg_mem_rd_addr_next[cur_rd_seg*INPUT_FIFO_ADDR_WIDTH +: INPUT_FIFO_ADDR_WIDTH] = ((input_fifo_rd_ptr_reg + rd_seg) >> SEG_IDX_WIDTH) + 1;
            mem_rd_data_valid_next[cur_rd_seg] = 1'b1;
        end else begin
            rd_valid = 0;
        end
        cur_rd_seg = cur_rd_seg + 1;
    end

    // 读取头信息
    if (!hdr_mem_rd_data_valid_next && !hdr_fifo_empty_reg) begin
        hdr_fifo_rd_ptr_next = hdr_fifo_rd_ptr_reg + 1;
        hdr_mem_rd_en = 1'b1;
        hdr_mem_rd_addr_next = hdr_fifo_rd_ptr_next;
        hdr_mem_rd_data_valid_next = 1'b1;
    end
end

integer i; // 输出打包时的段循环索引。

always @(posedge clk) begin
    input_fifo_rd_ptr_reg <= input_fifo_rd_ptr_next;
    input_fifo_count_reg <= input_fifo_wr_ptr_next - input_fifo_rd_ptr_next;
    input_fifo_empty_reg <= input_fifo_wr_ptr_next == input_fifo_rd_ptr_next;
    hdr_fifo_rd_ptr_reg <= hdr_fifo_rd_ptr_next;
    hdr_fifo_count_reg <= hdr_fifo_wr_ptr_next - hdr_fifo_rd_ptr_next;
    hdr_fifo_empty_reg <= hdr_fifo_wr_ptr_next == hdr_fifo_rd_ptr_next;

    seg_mem_rd_addr_reg <= seg_mem_rd_addr_next;
    hdr_mem_rd_addr_reg <= hdr_mem_rd_addr_next;

    mem_rd_data_valid_reg <= mem_rd_data_valid_next;
    hdr_mem_rd_data_valid_reg <= hdr_mem_rd_data_valid_next;

    output_data_reg <= output_data_next;
    output_valid_reg <= output_valid_next;

    out_frame_reg <= out_frame_next;
    out_len_reg <= out_len_next;
    out_split1_reg <= out_split1_next;
    out_seg_cnt_in_reg <= out_seg_cnt_in_next;
    out_seg_last_straddle_reg <= out_seg_last_straddle_next;
    out_seg_offset_reg <= out_seg_offset_next;
    out_seg_fifo_offset_reg <= out_seg_fifo_offset_next;

    out_hdr_reg <= out_hdr_next;

    out_ctl_seg_hdr_reg <= out_ctl_seg_hdr_next;
    out_ctl_seg_split1_reg <= out_ctl_seg_split1_next;
    out_ctl_seg_en_reg <= out_ctl_seg_en_next;
    for (i = 0; i < SEG_CNT_INT; i = i + 1) begin
        out_ctl_seg_idx_reg[i] <= out_ctl_seg_idx_next[i];
    end
    out_ctl_seg_offset_reg <= out_ctl_seg_offset_next;

    out_shift_reg <= out_shift_next;

    block_timeout_count_reg <= block_timeout_count_next;
    block_timeout_reg <= block_timeout_next;

    if (rst || fifo_rst_in) begin
        input_fifo_rd_ptr_reg <= 0;
        input_fifo_count_reg <= 0;
        input_fifo_empty_reg <= 1'b1;
        hdr_fifo_rd_ptr_reg <= 0;
        hdr_fifo_count_reg <= 0;
        hdr_fifo_empty_reg <= 1'b1;

        seg_mem_rd_addr_reg <= 0;
        hdr_mem_rd_addr_reg <= 0;

        mem_rd_data_valid_reg <= 0;
        hdr_mem_rd_data_valid_reg <= 0;

        out_frame_reg <= 1'b0;
        out_len_reg <= 0;
        out_split1_reg <= 0;
        out_seg_offset_reg <= 0;
        out_seg_fifo_offset_reg <= 0;
        out_seg_count_reg <= 0;
    end
end

endmodule

`resetall
