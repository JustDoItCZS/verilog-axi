/*

Copyright (c) 2021 Alex Forencich

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
 * AXI4-Lite 交叉开关地址解码与接纳控制
 *
 * 模块目录
 * 1) 接收来自单个从端口的一路地址通道流。
 * 2) 进行地址区域比较与权限检查，选择一个目标主端口。
 * 3) 输出：
 *    - 即时地址授予（m_select + m_axil_avalid）
 *    - 可选写命令记账通道（m_wc_*）
 *    - 回程路由用响应命令记账通道（m_rc_*）
 */
module axil_crossbar_addr #
(
    // 从接口索引
    parameter S = 0,
    // AXI 输入端口数量（从接口数量）
    parameter S_COUNT = 4,
    // AXI 输出端口数量（主接口数量）
    parameter M_COUNT = 4,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // 每个主接口的地址区域数量
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
    // 使能写命令输出通道
    parameter WC_OUTPUT = 0
)
(
    input  wire                       clk, // 解码与控制时钟。
    input  wire                       rst, // 同步复位，清空状态与有效标志。

    /*
     * 地址输入
     */
    input  wire [ADDR_WIDTH-1:0]      s_axil_aaddr, // 输入地址，与所有配置区域进行匹配解码。
    input  wire [2:0]                 s_axil_aprot, // 输入保护位；bit[1] 参与安全过滤。
    input  wire                       s_axil_avalid, // 上游地址通道输入有效。
    output wire                       s_axil_aready, // 返回上游就绪；解码器可接收新请求时拉高。

    /*
     * 地址输出
     */
    output wire [$clog2(M_COUNT)-1:0] m_select, // 当前请求解码得到的目标主端索引。
    output wire                       m_axil_avalid, // 解码目标有效；保持到下游接收完成。
    input  wire                       m_axil_aready, // 下游地址通路返回握手就绪。

    /*
     * 写命令输出
     */
    output wire [$clog2(M_COUNT)-1:0] m_wc_select, // 写数据路径记账使用的目标索引。
    output wire                       m_wc_decerr, // 与 m_wc_select 配对的解码错误标记。
    output wire                       m_wc_valid, // 写命令记账有效；仅 WC_OUTPUT 使能时有效。
    input  wire                       m_wc_ready, // 写命令记账通道反压输入。

    /*
     * 响应命令输出
     */
    output wire [$clog2(M_COUNT)-1:0] m_rc_select, // 写入响应路由 FIFO 的目标索引。
    output wire                       m_rc_decerr, // 回程路径合成 DECERR 用的解码错误标记。
    output wire                       m_rc_valid, // 响应命令有效；每次地址解码接纳后置位。
    input  wire                       m_rc_ready // 响应命令消费者/FIFO 写端返回反压。
);

parameter CL_S_COUNT = $clog2(S_COUNT); // 编码源从端索引位宽（用于连通掩码）。
parameter CL_M_COUNT = $clog2(M_COUNT); // 编码解码后主端选择位宽。

// 默认地址映射计算
function [M_COUNT*M_REGIONS*ADDR_WIDTH-1:0] calcBaseAddrs(input [31:0] dummy);
    integer i; // 默认基地址生成的区域循环变量。
    reg [ADDR_WIDTH-1:0] base; // 综合地址映射时逐步推进的基地址指针。
    reg [ADDR_WIDTH-1:0] width; // 当前区域大小指数（地址宽度配置项）。
    reg [ADDR_WIDTH-1:0] size; // 当前宽度对应的区域字节大小。
    reg [ADDR_WIDTH-1:0] mask; // 由宽度推导的对齐掩码，用于对齐与范围计算。
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

parameter M_BASE_ADDR_INT = M_BASE_ADDR ? M_BASE_ADDR : calcBaseAddrs(0); // 可选自动生成后的有效区域基地址表。

integer i, j; // 配置校验与解码比较循环变量。

// 配置合法性检查
initial begin
    if (M_REGIONS < 1) begin
        $error("Error: need at least 1 region (instance %m)");
        $finish;
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH) begin
            $error("Error: address width out of range (instance %m)");
            $finish;
        end
    end

    $display("Addressing configuration for axil_crossbar_addr instance %m");
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
    STATE_IDLE = 3'd0, // 空闲态：等待新的 s_axil_avalid 事务。
    STATE_DECODE = 3'd1; // 解码态：保持解码元数据有效直至输出握手完成。

reg [2:0] state_reg = STATE_IDLE, state_next; // 解码控制 FSM 当前/下一状态。

reg s_axil_aready_reg = 0, s_axil_aready_next; // 输入 ready 寄存器；当前事务排空后重新拉高。

reg [CL_M_COUNT-1:0] m_select_reg = 0, m_select_next; // 锁存的主端口选择索引。
reg m_axil_avalid_reg = 1'b0, m_axil_avalid_next; // 地址输出有效标志；m_axil_aready 握手后清零。
reg m_decerr_reg = 1'b0, m_decerr_next; // 与当前事务元数据配对锁存的解码错误标志。
reg m_wc_valid_reg = 1'b0, m_wc_valid_next; // 写命令元数据有效；随 m_wc_ready 握手变化。
reg m_rc_valid_reg = 1'b0, m_rc_valid_next; // 响应命令元数据有效；随 m_rc_ready 握手变化。

assign s_axil_aready = s_axil_aready_reg;

assign m_select = m_select_reg;
assign m_axil_avalid = m_axil_avalid_reg;

assign m_wc_select = m_select_reg;
assign m_wc_decerr = m_decerr_reg;
assign m_wc_valid = m_wc_valid_reg;

assign m_rc_select = m_select_reg;
assign m_rc_decerr = m_decerr_reg;
assign m_rc_valid = m_rc_valid_reg;

reg match; // 组合解码命中标志；任一区域比较成功时置位。

always @* begin
    state_next = STATE_IDLE;

    match = 1'b0;

    s_axil_aready_next = 1'b0;

    m_select_next = m_select_reg;
    m_axil_avalid_next = m_axil_avalid_reg && !m_axil_aready;
    m_decerr_next = m_decerr_reg;
    m_wc_valid_next = m_wc_valid_reg && !m_wc_ready;
    m_rc_valid_next = m_rc_valid_reg && !m_rc_ready;

    case (state_reg)
        STATE_IDLE: begin
            // 空闲态：锁存输入信息
            s_axil_aready_next = 1'b0;

            if (s_axil_avalid && !s_axil_aready) begin
                match = 1'b0;
                for (i = 0; i < M_COUNT; i = i + 1) begin
                    for (j = 0; j < M_REGIONS; j = j + 1) begin
                        if (M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32] && (!M_SECURE[i] || !s_axil_aprot[1]) && (M_CONNECT & (1 << (S+i*S_COUNT))) && (s_axil_aaddr >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32]) == (M_BASE_ADDR_INT[(i*M_REGIONS+j)*ADDR_WIDTH +: ADDR_WIDTH] >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32])) begin
                            m_select_next = i;
                            match = 1'b1;
                        end
                    end
                end

                if (match) begin
                    // 地址解码成功
                    m_axil_avalid_next = 1'b1;
                    m_decerr_next = 1'b0;
                    m_wc_valid_next = WC_OUTPUT;
                    m_rc_valid_next = 1'b1;
                    state_next = STATE_DECODE;
                end else begin
                    // 地址解码失败
                    m_axil_avalid_next = 1'b0;
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
            if (!m_axil_avalid_next && (!m_wc_valid_next || !WC_OUTPUT) && !m_rc_valid_next) begin
                s_axil_aready_next = 1'b1;
                state_next = STATE_IDLE;
            end else begin
                state_next = STATE_DECODE;
            end
        end
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_reg <= STATE_IDLE;
        s_axil_aready_reg <= 1'b0;
        m_axil_avalid_reg <= 1'b0;
        m_wc_valid_reg <= 1'b0;
        m_rc_valid_reg <= 1'b0;
    end else begin
        state_reg <= state_next;
        s_axil_aready_reg <= s_axil_aready_next;
        m_axil_avalid_reg <= m_axil_avalid_next;
        m_wc_valid_reg <= m_wc_valid_next;
        m_rc_valid_reg <= m_rc_valid_next;
    end

    m_select_reg <= m_select_next;
    m_decerr_reg <= m_decerr_next;
end

endmodule

`resetall
