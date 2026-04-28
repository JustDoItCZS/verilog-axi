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
 * AXI4 交叉开关地址解码与接纳控制
 *
 * 模块目录
 * 1) 针对单个 S 口的地址请求做目标 M 口/region 解码。
 * 2) 维护“按 ID 分线程”的并发配额，限制同一 S 口未完成事务数量。
 * 3) 输出两类命令：写通道路由命令(WC)和读/写通用返回命令(RC)。
 */
module axi_crossbar_addr #
(
    // 从接口索引
    parameter S = 0,
    // AXI 输入端口数量（从接口数量）
    parameter S_COUNT = 4,
    // AXI 输出端口数量（主接口数量）
    parameter M_COUNT = 4,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // ID 字段位宽
    parameter ID_WIDTH = 8,
    // 可并发唯一 ID 数量
    parameter S_THREADS = 32'd2,
    // 可并发事务数量
    parameter S_ACCEPT = 32'd16,
    // 每个主接口地址区域数量
    parameter M_REGIONS = 1,
    // 主接口基地址表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 ADDR_WIDTH 位字段
    // 置 0 时按 M_ADDR_WIDTH 自动生成默认地址映射
    parameter M_BASE_ADDR = 0,
    // 主接口地址宽度表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 32 位字段
    parameter M_ADDR_WIDTH = {M_COUNT{{M_REGIONS{32'd24}}}},
    // 接口间连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT = {M_COUNT{{S_COUNT{1'b1}}}},
    // 安全主端口配置（基于 awprot/arprot 拒绝访问）
    // M_COUNT 位
    parameter M_SECURE = {M_COUNT{1'b0}},
    // 使能写命令输出
    parameter WC_OUTPUT = 0
)
(
    input  wire                       clk, // 模块时钟。
    input  wire                       rst, // 同步复位，高电平有效。

    /*
     * 地址输入
     */
    input  wire [ID_WIDTH-1:0]        s_axi_aid, // 当前地址请求的 ID。
    input  wire [ADDR_WIDTH-1:0]      s_axi_aaddr, // 当前地址请求的地址。
    input  wire [2:0]                 s_axi_aprot, // 当前地址请求的保护属性(含 secure 位)。
    input  wire [3:0]                 s_axi_aqos, // 当前地址请求的 QoS(本模块仅透传匹配语义)。
    input  wire                       s_axi_avalid, // 地址请求有效。
    output wire                       s_axi_aready, // 模块可接受地址请求。

    /*
     * 地址输出
     */
    output wire [3:0]                 m_axi_aregion, // 解码后的目标 region。
    output wire [$clog2(M_COUNT)-1:0] m_select, // 解码后的目标 M 口编号。
    output wire                       m_axi_avalid, // 地址输出命令有效。
    input  wire                       m_axi_aready, // 下游地址命令被接受。

    /*
     * 写命令输出
     */
    output wire [$clog2(M_COUNT)-1:0] m_wc_select, // 写通道路由命令中的目标 M 口。
    output wire                       m_wc_decerr, // 写命令是否解码错误(DECERR)。
    output wire                       m_wc_valid, // 写命令有效。
    input  wire                       m_wc_ready, // 写命令被后级接收。

    /*
     * 响应命令输出
     */
    output wire                       m_rc_decerr, // 返回命令是否解码错误(DECERR)。
    output wire                       m_rc_valid, // 返回命令有效。
    input  wire                       m_rc_ready, // 返回命令被后级接收。

    /*
     * 完成通知输入
     */
    input  wire [ID_WIDTH-1:0]        s_cpl_id, // 已完成事务的 ID。
    input  wire                       s_cpl_valid // 完成事件有效。
);

parameter CL_S_COUNT = $clog2(S_COUNT);
parameter CL_M_COUNT = $clog2(M_COUNT);

parameter S_INT_THREADS = S_THREADS > S_ACCEPT ? S_ACCEPT : S_THREADS;
parameter CL_S_INT_THREADS = $clog2(S_INT_THREADS);
parameter CL_S_ACCEPT = $clog2(S_ACCEPT);

// 默认地址映射计算
function [M_COUNT*M_REGIONS*ADDR_WIDTH-1:0] calcBaseAddrs(input [31:0] dummy);
    integer i; // 地址区域遍历索引。
    reg [ADDR_WIDTH-1:0] base; // 自动分配基地址游标。
    reg [ADDR_WIDTH-1:0] width; // 当前区域地址宽度。
    reg [ADDR_WIDTH-1:0] size; // 当前区域大小。
    reg [ADDR_WIDTH-1:0] mask; // 当前区域掩码。
    begin
        calcBaseAddrs = {M_COUNT*M_REGIONS*ADDR_WIDTH{1'b0}};
        base = 0;
        for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
            width = M_ADDR_WIDTH[i*32 +: 32];
            mask = {ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - width);
            size = mask + 1;
            if (width > 0) begin
                if ((base & mask) != 0) begin
                   base = base + size - (base & mask); // 对齐到该区域边界
                end
                calcBaseAddrs[i * ADDR_WIDTH +: ADDR_WIDTH] = base;
                base = base + size; // 推进到下一段基地址
            end
        end
    end
endfunction

parameter M_BASE_ADDR_INT = M_BASE_ADDR ? M_BASE_ADDR : calcBaseAddrs(0);

integer i, j; // 配置检查与地址匹配循环索引。

// 配置合法性检查
initial begin
    if (S_ACCEPT < 1) begin
        $error("Error: need at least 1 accept (instance %m)");
        $finish;
    end

    if (S_THREADS < 1) begin
        $error("Error: need at least 1 thread (instance %m)");
        $finish;
    end

    if (S_THREADS > S_ACCEPT) begin
        $warning("Warning: requested thread count larger than accept count; limiting thread count to accept count (instance %m)");
    end

    if (M_REGIONS < 1) begin
        $error("Error: need at least 1 region (instance %m)");
        $finish;
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && (M_ADDR_WIDTH[i*32 +: 32] < 12 || M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH)) begin
            $error("Error: address width out of range (instance %m)");
            $finish;
        end
    end

    $display("Addressing configuration for axi_crossbar_addr instance %m");
    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32]) begin
            $display("%2d (%2d): %x / %02d -- %x-%x",
                i/M_REGIONS, i%M_REGIONS,
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH],
                M_ADDR_WIDTH[i*32 +: 32],
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[i*32 +: 32]),
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[i*32 +: 32]))
            );
        end
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if ((M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] & (2**M_ADDR_WIDTH[i*32 +: 32]-1)) != 0) begin
            $display("Region not aligned:");
            $display("%2d (%2d): %x / %2d -- %x-%x",
                i/M_REGIONS, i%M_REGIONS,
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH],
                M_ADDR_WIDTH[i*32 +: 32],
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[i*32 +: 32]),
                M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[i*32 +: 32]))
            );
            $error("Error: address range not aligned (instance %m)");
            $finish;
        end
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        for (j = i+1; j < M_COUNT*M_REGIONS; j = j + 1) begin
            if (M_ADDR_WIDTH[i*32 +: 32] && M_ADDR_WIDTH[j*32 +: 32]) begin
                if (((M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[i*32 +: 32])) <= (M_BASE_ADDR_INT[j*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[j*32 +: 32]))))
                        && ((M_BASE_ADDR_INT[j*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[j*32 +: 32])) <= (M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[i*32 +: 32]))))) begin
                    $display("Overlapping regions:");
                    $display("%2d (%2d): %x / %2d -- %x-%x",
                        i/M_REGIONS, i%M_REGIONS,
                        M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH],
                        M_ADDR_WIDTH[i*32 +: 32],
                        M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[i*32 +: 32]),
                        M_BASE_ADDR_INT[i*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[i*32 +: 32]))
                    );
                    $display("%2d (%2d): %x / %2d -- %x-%x",
                        j/M_REGIONS, j%M_REGIONS,
                        M_BASE_ADDR_INT[j*ADDR_WIDTH +: ADDR_WIDTH],
                        M_ADDR_WIDTH[j*32 +: 32],
                        M_BASE_ADDR_INT[j*ADDR_WIDTH +: ADDR_WIDTH] & ({ADDR_WIDTH{1'b1}} << M_ADDR_WIDTH[j*32 +: 32]),
                        M_BASE_ADDR_INT[j*ADDR_WIDTH +: ADDR_WIDTH] | ({ADDR_WIDTH{1'b1}} >> (ADDR_WIDTH - M_ADDR_WIDTH[j*32 +: 32]))
                    );
                    $error("Error: address ranges overlap (instance %m)");
                    $finish;
                end
            end
        end
    end
end

localparam [2:0]
    STATE_IDLE = 3'd0, // 等待新地址请求。
    STATE_DECODE = 3'd1; // 执行地址解码并产生命令。

reg [2:0] state_reg = STATE_IDLE, state_next; // 状态机寄存器。

reg s_axi_aready_reg = 0, s_axi_aready_next; // 输入地址握手 ready 寄存器。

reg [3:0] m_axi_aregion_reg = 4'd0, m_axi_aregion_next; // 输出 region 寄存器。
reg [CL_M_COUNT-1:0] m_select_reg = 0, m_select_next; // 输出目标 M 口寄存器。
reg m_axi_avalid_reg = 1'b0, m_axi_avalid_next; // 地址命令 valid 寄存器。
reg m_decerr_reg = 1'b0, m_decerr_next; // 当前事务是否解码错误。
reg m_wc_valid_reg = 1'b0, m_wc_valid_next; // 写命令 valid 寄存器。
reg m_rc_valid_reg = 1'b0, m_rc_valid_next; // 返回命令 valid 寄存器。

assign s_axi_aready = s_axi_aready_reg;

assign m_axi_aregion = m_axi_aregion_reg;
assign m_select = m_select_reg;
assign m_axi_avalid = m_axi_avalid_reg;

assign m_wc_select = m_select_reg;
assign m_wc_decerr = m_decerr_reg;
assign m_wc_valid = m_wc_valid_reg;

assign m_rc_decerr = m_decerr_reg;
assign m_rc_valid = m_rc_valid_reg;

reg match; // 地址是否命中可达目标。
reg trans_start; // 新事务开始事件。
reg trans_complete; // 事务完成事件。

reg [$clog2(S_ACCEPT+1)-1:0] trans_count_reg = 0; // 当前 S 口在途事务总数。
wire trans_limit = trans_count_reg >= S_ACCEPT && !trans_complete; // 是否达到在途事务上限。

// 传输 ID 线程跟踪
reg [ID_WIDTH-1:0] thread_id_reg[S_INT_THREADS-1:0]; // 每个线程槽绑定的事务 ID。
reg [CL_M_COUNT-1:0] thread_m_reg[S_INT_THREADS-1:0]; // 每个线程槽绑定的目标 M 口。
reg [3:0] thread_region_reg[S_INT_THREADS-1:0]; // 每个线程槽绑定的目标 region。
reg [$clog2(S_ACCEPT+1)-1:0] thread_count_reg[S_INT_THREADS-1:0]; // 每个线程槽在途事务计数。

wire [S_INT_THREADS-1:0] thread_active; // 线程槽是否活跃(计数非零)。
wire [S_INT_THREADS-1:0] thread_match; // 线程槽 ID 是否匹配当前输入 ID。
wire [S_INT_THREADS-1:0] thread_match_dest; // 匹配 ID 且目标口/region 一致。
wire [S_INT_THREADS-1:0] thread_cpl_match; // 线程槽 ID 是否匹配完成事件 ID。
wire [S_INT_THREADS-1:0] thread_trans_start; // 对应线程槽是否在本拍分配新事务。
wire [S_INT_THREADS-1:0] thread_trans_complete; // 对应线程槽是否在本拍完成事务。

generate
    genvar n;

    for (n = 0; n < S_INT_THREADS; n = n + 1) begin
        initial begin
            thread_count_reg[n] <= 0;
        end

        assign thread_active[n] = thread_count_reg[n] != 0;
        assign thread_match[n] = thread_active[n] && thread_id_reg[n] == s_axi_aid;
        assign thread_match_dest[n] = thread_match[n] && thread_m_reg[n] == m_select_next && (M_REGIONS < 2 || thread_region_reg[n] == m_axi_aregion_next);
        assign thread_cpl_match[n] = thread_active[n] && thread_id_reg[n] == s_cpl_id;
        assign thread_trans_start[n] = (thread_match[n] || (!thread_active[n] && !thread_match && !(thread_trans_start & ({S_INT_THREADS{1'b1}} >> (S_INT_THREADS-n))))) && trans_start;
        assign thread_trans_complete[n] = thread_cpl_match[n] && trans_complete;

        always @(posedge clk) begin
            if (rst) begin
                thread_count_reg[n] <= 0;
            end else begin
                if (thread_trans_start[n] && !thread_trans_complete[n]) begin
                    thread_count_reg[n] <= thread_count_reg[n] + 1;
                end else if (!thread_trans_start[n] && thread_trans_complete[n]) begin
                    thread_count_reg[n] <= thread_count_reg[n] - 1;
                end
            end

            if (thread_trans_start[n]) begin
                thread_id_reg[n] <= s_axi_aid;
                thread_m_reg[n] <= m_select_next;
                thread_region_reg[n] <= m_axi_aregion_next;
            end
        end
    end
endgenerate

always @* begin
    state_next = STATE_IDLE;

    match = 1'b0;
    trans_start = 1'b0;
    trans_complete = 1'b0;

    s_axi_aready_next = 1'b0;

    m_axi_aregion_next = m_axi_aregion_reg;
    m_select_next = m_select_reg;
    m_axi_avalid_next = m_axi_avalid_reg && !m_axi_aready;
    m_decerr_next = m_decerr_reg;
    m_wc_valid_next = m_wc_valid_reg && !m_wc_ready;
    m_rc_valid_next = m_rc_valid_reg && !m_rc_ready;

    case (state_reg)
        STATE_IDLE: begin
            // 空闲态：锁存输入信息
            s_axi_aready_next = 1'b0;

            if (s_axi_avalid && !s_axi_aready) begin
                match = 1'b0;
                for (i = 0; i < M_COUNT; i = i + 1) begin
                    for (j = 0; j < M_REGIONS; j = j + 1) begin
                        if (M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32] && (!M_SECURE[i] || !s_axi_aprot[1]) && (M_CONNECT & (1 << (S+i*S_COUNT))) && (s_axi_aaddr >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32]) == (M_BASE_ADDR_INT[(i*M_REGIONS+j)*ADDR_WIDTH +: ADDR_WIDTH] >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32])) begin
                            m_select_next = i;
                            m_axi_aregion_next = j;
                            match = 1'b1;
                        end
                    end
                end

                if (match) begin
                    // 地址解码成功
                    if (!trans_limit && (thread_match_dest || (!(&thread_active) && !thread_match))) begin
                        // 未达到事务上限
                        m_axi_avalid_next = 1'b1;
                        m_decerr_next = 1'b0;
                        m_wc_valid_next = WC_OUTPUT;
                        m_rc_valid_next = 1'b0;
                        trans_start = 1'b1;
                        state_next = STATE_DECODE;
                    end else begin
                        // 已达到事务上限；保持空闲阻塞
                        state_next = STATE_IDLE;
                    end
                end else begin
                    // 地址解码失败
                    m_axi_avalid_next = 1'b0;
                    m_decerr_next = 1'b1;
                    m_wc_valid_next = WC_OUTPUT;
                    m_rc_valid_next = 1'b1;
                    state_next = STATE_DECODE;
                end
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_DECODE: begin
            if (!m_axi_avalid_next && (!m_wc_valid_next || !WC_OUTPUT) && !m_rc_valid_next) begin
                s_axi_aready_next = 1'b1;
                state_next = STATE_IDLE;
            end else begin
                state_next = STATE_DECODE;
            end
        end
    endcase

    // 完成事务管理
    trans_complete = s_cpl_valid;
end

always @(posedge clk) begin
    if (rst) begin
        state_reg <= STATE_IDLE;
        s_axi_aready_reg <= 1'b0;
        m_axi_avalid_reg <= 1'b0;
        m_wc_valid_reg <= 1'b0;
        m_rc_valid_reg <= 1'b0;

        trans_count_reg <= 0;
    end else begin
        state_reg <= state_next;
        s_axi_aready_reg <= s_axi_aready_next;
        m_axi_avalid_reg <= m_axi_avalid_next;
        m_wc_valid_reg <= m_wc_valid_next;
        m_rc_valid_reg <= m_rc_valid_next;

        if (trans_start && !trans_complete) begin
            trans_count_reg <= trans_count_reg + 1;
        end else if (!trans_start && trans_complete) begin
            trans_count_reg <= trans_count_reg - 1;
        end
    end

    m_axi_aregion_reg <= m_axi_aregion_next;
    m_select_reg <= m_select_next;
    m_decerr_reg <= m_decerr_next;
end

endmodule

`resetall
