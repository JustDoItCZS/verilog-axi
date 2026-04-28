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
 * AXI4-Lite 互连模块
 *
 * 模块目录
 * 1) 采用单事务共享数据通路，由全局 FSM 统一控制。
 * 2) 仲裁器从所有从端读写请求中选一路，解码器确定目标主端口。
 * 3) 事务端到端转发，响应再路由回请求来源从端。
 * 4) 任意时刻仅允许一个在途事务（相比全交叉开关面积更小）。
 */
module axil_interconnect #
(
    // AXI 输入端口数量（从接口数量）
    parameter S_COUNT = 4,
    // AXI 输出端口数量（主接口数量）
    parameter M_COUNT = 4,
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 每个主接口的地址区域数量
    parameter M_REGIONS = 1,
    // 主接口基地址表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 ADDR_WIDTH 位字段
    // 置 0 时按 M_ADDR_WIDTH 自动生成默认地址映射
    parameter M_BASE_ADDR = 0,
    // 主接口地址宽度表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 32 位字段
    parameter M_ADDR_WIDTH = {M_COUNT{{M_REGIONS{32'd24}}}},
    // 接口间读通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT_READ = {M_COUNT{{S_COUNT{1'b1}}}},
    // 接口间写通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT_WRITE = {M_COUNT{{S_COUNT{1'b1}}}},
    // 安全主端口配置（基于 awprot/arprot 拒绝访问）
    // M_COUNT 位
    parameter M_SECURE = {M_COUNT{1'b0}}
)
(
    input  wire                           clk, // 互连核心时钟。
    input  wire                           rst, // 同步复位，清空仲裁与路由 FSM 状态。

    /*
     * AXI-Lite 从接口
     */
    input  wire [S_COUNT*ADDR_WIDTH-1:0]  s_axil_awaddr, // 从端 AW 地址向量（按端口打包）。
    input  wire [S_COUNT*3-1:0]           s_axil_awprot, // 从端 AW 保护属性向量。
    input  wire [S_COUNT-1:0]             s_axil_awvalid, // 从端 AWVALID 向量。
    output wire [S_COUNT-1:0]             s_axil_awready, // 从端 AWREADY 向量（由当前选中源握手驱动）。
    input  wire [S_COUNT*DATA_WIDTH-1:0]  s_axil_wdata, // 从端写数据向量。
    input  wire [S_COUNT*STRB_WIDTH-1:0]  s_axil_wstrb, // 从端写字节使能向量。
    input  wire [S_COUNT-1:0]             s_axil_wvalid, // 从端 WVALID 向量。
    output wire [S_COUNT-1:0]             s_axil_wready, // 从端 WREADY 向量。
    output wire [S_COUNT*2-1:0]           s_axil_bresp, // 从端 BRESP 向量（路由值或 DECERR）。
    output wire [S_COUNT-1:0]             s_axil_bvalid, // 从端 BVALID 向量。
    input  wire [S_COUNT-1:0]             s_axil_bready, // 从端 BREADY 向量。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]  s_axil_araddr, // 从端 AR 地址向量（按端口打包）。
    input  wire [S_COUNT*3-1:0]           s_axil_arprot, // 从端 AR 保护属性向量。
    input  wire [S_COUNT-1:0]             s_axil_arvalid, // 从端 ARVALID 向量。
    output wire [S_COUNT-1:0]             s_axil_arready, // 从端 ARREADY 向量。
    output wire [S_COUNT*DATA_WIDTH-1:0]  s_axil_rdata, // 从端读数据向量。
    output wire [S_COUNT*2-1:0]           s_axil_rresp, // 从端读响应向量。
    output wire [S_COUNT-1:0]             s_axil_rvalid, // 从端 RVALID 向量。
    input  wire [S_COUNT-1:0]             s_axil_rready, // 从端 RREADY 向量。

    /*
     * AXI-Lite 主接口
     */
    output wire [M_COUNT*ADDR_WIDTH-1:0]  m_axil_awaddr, // 主端 AW 地址向量（复制自当前事务上下文）。
    output wire [M_COUNT*3-1:0]           m_axil_awprot, // 主端 AW 保护属性向量。
    output wire [M_COUNT-1:0]             m_axil_awvalid, // 主端 AWVALID，目标端口 one-hot 有效。
    input  wire [M_COUNT-1:0]             m_axil_awready, // 主端 AWREADY 向量。
    output wire [M_COUNT*DATA_WIDTH-1:0]  m_axil_wdata, // 主端写数据向量（复制自当前事务上下文）。
    output wire [M_COUNT*STRB_WIDTH-1:0]  m_axil_wstrb, // 主端写字节使能向量（复制自当前事务上下文）。
    output wire [M_COUNT-1:0]             m_axil_wvalid, // 主端 WVALID，目标端口 one-hot 有效。
    input  wire [M_COUNT-1:0]             m_axil_wready, // 主端 WREADY 向量。
    input  wire [M_COUNT*2-1:0]           m_axil_bresp, // 主端 BRESP 向量。
    input  wire [M_COUNT-1:0]             m_axil_bvalid, // 主端 BVALID 向量。
    output wire [M_COUNT-1:0]             m_axil_bready, // 主端 BREADY，指向目标端口 one-hot。
    output wire [M_COUNT*ADDR_WIDTH-1:0]  m_axil_araddr, // 主端 AR 地址向量（复制自当前事务上下文）。
    output wire [M_COUNT*3-1:0]           m_axil_arprot, // 主端 AR 保护属性向量。
    output wire [M_COUNT-1:0]             m_axil_arvalid, // 主端 ARVALID，目标端口 one-hot 有效。
    input  wire [M_COUNT-1:0]             m_axil_arready, // 主端 ARREADY 向量。
    input  wire [M_COUNT*DATA_WIDTH-1:0]  m_axil_rdata, // 主端读数据向量。
    input  wire [M_COUNT*2-1:0]           m_axil_rresp, // 主端读响应向量。
    input  wire [M_COUNT-1:0]             m_axil_rvalid, // 主端 RVALID 向量。
    output wire [M_COUNT-1:0]             m_axil_rready // 主端 RREADY，指向目标端口 one-hot。
);

parameter CL_S_COUNT = $clog2(S_COUNT); // 编码源从端索引所需位宽。
parameter CL_M_COUNT = $clog2(M_COUNT); // 编码目标主端索引所需位宽。

// 默认地址映射计算
function [M_COUNT*M_REGIONS*ADDR_WIDTH-1:0] calcBaseAddrs(input [31:0] dummy);
    integer i; // 自动映射生成时的区域循环变量。
    reg [ADDR_WIDTH-1:0] base; // 构建默认映射时的当前基地址指针。
    reg [ADDR_WIDTH-1:0] width; // 当前区域宽度字段。
    reg [ADDR_WIDTH-1:0] size; // 由宽度推导得到的区域大小。
    reg [ADDR_WIDTH-1:0] mask; // 区域对齐掩码。
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
                base = base + size; // 移动到下一段基地址
            end
        end
    end
endfunction

parameter M_BASE_ADDR_INT = M_BASE_ADDR ? M_BASE_ADDR : calcBaseAddrs(0); // 解码实际使用的基地址表。

integer i, j; // 配置校验与运行期解码循环变量。

// 配置合法性检查
initial begin
    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && (M_ADDR_WIDTH[i*32 +: 32] < $clog2(STRB_WIDTH) || M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH)) begin
            $error("Error: address width out of range (instance %m)");
            $finish;
        end
    end

    $display("Addressing configuration for axil_interconnect instance %m");
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
    STATE_IDLE = 3'd0, // 空闲态：等待仲裁器授予一路从端读/写请求。
    STATE_DECODE = 3'd1, // 解码态：将选中地址解码为目标主端（或 DECERR）。
    STATE_WRITE = 3'd2, // 写转发态：向目标主端转发写地址/写数据。
    STATE_WRITE_RESP = 3'd3, // 写响应态：等待并采样目标主端写响应。
    STATE_WRITE_DROP = 3'd4, // 写丢弃态：解码失败时吞吐写数据并返回 DECERR。
    STATE_READ = 3'd5, // 读转发态：转发读地址并等待读响应。
    STATE_WAIT_IDLE = 3'd6; // 等待释放态：等待源端响应握手完成后解除仲裁阻塞。

reg [2:0] state_reg = STATE_IDLE, state_next; // 互连主控制 FSM 当前/下一状态。

reg match; // 当前请求地址命中某目标区域的标志。

reg [CL_M_COUNT-1:0] m_select_reg = 2'd0, m_select_next; // 当前在途事务选中的目标主端索引。
reg [ADDR_WIDTH-1:0] axil_addr_reg = {ADDR_WIDTH{1'b0}}, axil_addr_next; // 锁存的在途事务地址。
reg axil_addr_valid_reg = 1'b0, axil_addr_valid_next; // 地址相位是否仍需向主端转发。
reg [2:0] axil_prot_reg = 3'b000, axil_prot_next; // 锁存的在途事务保护属性。
reg [DATA_WIDTH-1:0] axil_data_reg = {DATA_WIDTH{1'b0}}, axil_data_next; // 锁存的写数据/读返回数据载荷。
reg [STRB_WIDTH-1:0] axil_wstrb_reg = {STRB_WIDTH{1'b0}}, axil_wstrb_next; // 锁存的在途写事务字节使能。
reg [1:0] axil_resp_reg = 2'b00, axil_resp_next; // 锁存并回送给源从端的响应码。

reg [S_COUNT-1:0] s_axil_awready_reg = 0, s_axil_awready_next; // 各从端 AWREADY 输出寄存器。
reg [S_COUNT-1:0] s_axil_wready_reg = 0, s_axil_wready_next; // 各从端 WREADY 输出寄存器。
reg [S_COUNT-1:0] s_axil_bvalid_reg = 0, s_axil_bvalid_next; // 各从端 BVALID 输出寄存器。
reg [S_COUNT-1:0] s_axil_arready_reg = 0, s_axil_arready_next; // 各从端 ARREADY 输出寄存器。
reg [S_COUNT-1:0] s_axil_rvalid_reg = 0, s_axil_rvalid_next; // 各从端 RVALID 输出寄存器。

reg [M_COUNT-1:0] m_axil_awvalid_reg = 0, m_axil_awvalid_next; // 各主端 AWVALID 输出寄存器。
reg [M_COUNT-1:0] m_axil_wvalid_reg = 0, m_axil_wvalid_next; // 各主端 WVALID 输出寄存器。
reg [M_COUNT-1:0] m_axil_bready_reg = 0, m_axil_bready_next; // 各主端 BREADY 输出寄存器。
reg [M_COUNT-1:0] m_axil_arvalid_reg = 0, m_axil_arvalid_next; // 各主端 ARVALID 输出寄存器。
reg [M_COUNT-1:0] m_axil_rready_reg = 0, m_axil_rready_next; // 各主端 RREADY 输出寄存器。

assign s_axil_awready = s_axil_awready_reg;
assign s_axil_wready = s_axil_wready_reg;
assign s_axil_bresp = {S_COUNT{axil_resp_reg}};
assign s_axil_bvalid = s_axil_bvalid_reg;
assign s_axil_arready = s_axil_arready_reg;
assign s_axil_rdata = {S_COUNT{axil_data_reg}};
assign s_axil_rresp = {S_COUNT{axil_resp_reg}};
assign s_axil_rvalid = s_axil_rvalid_reg;

assign m_axil_awaddr = {M_COUNT{axil_addr_reg}};
assign m_axil_awprot = {M_COUNT{axil_prot_reg}};
assign m_axil_awvalid = m_axil_awvalid_reg;
assign m_axil_wdata = {M_COUNT{axil_data_reg}};
assign m_axil_wstrb = {M_COUNT{axil_wstrb_reg}};
assign m_axil_wvalid = m_axil_wvalid_reg;
assign m_axil_bready = m_axil_bready_reg;
assign m_axil_araddr = {M_COUNT{axil_addr_reg}};
assign m_axil_arprot = {M_COUNT{axil_prot_reg}};
assign m_axil_arvalid = m_axil_arvalid_reg;
assign m_axil_rready = m_axil_rready_reg;

// 从端侧复用
wire [(CL_S_COUNT > 0 ? CL_S_COUNT-1 : 0):0] s_select; // 当前被授予的源从端编码索引。

wire [ADDR_WIDTH-1:0] current_s_axil_awaddr  = s_axil_awaddr[s_select*ADDR_WIDTH +: ADDR_WIDTH]; // 选中源从端的 AW 地址。
wire [2:0]            current_s_axil_awprot  = s_axil_awprot[s_select*3 +: 3]; // 选中源从端的 AW 保护属性。
wire                  current_s_axil_awvalid = s_axil_awvalid[s_select]; // 选中源从端的 AWVALID。
wire                  current_s_axil_awready = s_axil_awready[s_select]; // 返回给选中源从端的 AWREADY。
wire [DATA_WIDTH-1:0] current_s_axil_wdata   = s_axil_wdata[s_select*DATA_WIDTH +: DATA_WIDTH]; // 选中源从端的 WDATA。
wire [STRB_WIDTH-1:0] current_s_axil_wstrb   = s_axil_wstrb[s_select*STRB_WIDTH +: STRB_WIDTH]; // 选中源从端的 WSTRB。
wire                  current_s_axil_wvalid  = s_axil_wvalid[s_select]; // 选中源从端的 WVALID。
wire                  current_s_axil_wready  = s_axil_wready[s_select]; // 返回给选中源从端的 WREADY。
wire [1:0]            current_s_axil_bresp   = s_axil_bresp[s_select*2 +: 2]; // 选中源从端可见的 BRESP。
wire                  current_s_axil_bvalid  = s_axil_bvalid[s_select]; // 选中源从端的 BVALID。
wire                  current_s_axil_bready  = s_axil_bready[s_select]; // 选中源从端输入的 BREADY。
wire [ADDR_WIDTH-1:0] current_s_axil_araddr  = s_axil_araddr[s_select*ADDR_WIDTH +: ADDR_WIDTH]; // 选中源从端的 AR 地址。
wire [2:0]            current_s_axil_arprot  = s_axil_arprot[s_select*3 +: 3]; // 选中源从端的 AR 保护属性。
wire                  current_s_axil_arvalid = s_axil_arvalid[s_select]; // 选中源从端的 ARVALID。
wire                  current_s_axil_arready = s_axil_arready[s_select]; // 返回给选中源从端的 ARREADY。
wire [DATA_WIDTH-1:0] current_s_axil_rdata   = s_axil_rdata[s_select*DATA_WIDTH +: DATA_WIDTH]; // 选中源从端可见的 RDATA。
wire [1:0]            current_s_axil_rresp   = s_axil_rresp[s_select*2 +: 2]; // 选中源从端可见的 RRESP。
wire                  current_s_axil_rvalid  = s_axil_rvalid[s_select]; // 选中源从端的 RVALID。
wire                  current_s_axil_rready  = s_axil_rready[s_select]; // 选中源从端输入的 RREADY。

// 主端侧复用
wire [ADDR_WIDTH-1:0] current_m_axil_awaddr  = m_axil_awaddr[m_select_reg*ADDR_WIDTH +: ADDR_WIDTH]; // 选中目标主端口槽位上的 AW 地址。
wire [2:0]            current_m_axil_awprot  = m_axil_awprot[m_select_reg*3 +: 3]; // 选中目标槽位上的 AW 保护属性。
wire                  current_m_axil_awvalid = m_axil_awvalid[m_select_reg]; // 发往选中目标主端口的 AWVALID。
wire                  current_m_axil_awready = m_axil_awready[m_select_reg]; // 选中目标主端口返回的 AWREADY。
wire [DATA_WIDTH-1:0] current_m_axil_wdata   = m_axil_wdata[m_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 发往选中目标主端口的 WDATA。
wire [STRB_WIDTH-1:0] current_m_axil_wstrb   = m_axil_wstrb[m_select_reg*STRB_WIDTH +: STRB_WIDTH]; // 发往选中目标主端口的 WSTRB。
wire                  current_m_axil_wvalid  = m_axil_wvalid[m_select_reg]; // 发往选中目标主端口的 WVALID。
wire                  current_m_axil_wready  = m_axil_wready[m_select_reg]; // 选中目标主端口返回的 WREADY。
wire [1:0]            current_m_axil_bresp   = m_axil_bresp[m_select_reg*2 +: 2]; // 来自选中目标主端口的 BRESP。
wire                  current_m_axil_bvalid  = m_axil_bvalid[m_select_reg]; // 来自选中目标主端口的 BVALID。
wire                  current_m_axil_bready  = m_axil_bready[m_select_reg]; // 发往选中目标主端口的 BREADY。
wire [ADDR_WIDTH-1:0] current_m_axil_araddr  = m_axil_araddr[m_select_reg*ADDR_WIDTH +: ADDR_WIDTH]; // 发往选中目标主端口的 AR 地址。
wire [2:0]            current_m_axil_arprot  = m_axil_arprot[m_select_reg*3 +: 3]; // 发往选中目标主端口的 AR 保护属性。
wire                  current_m_axil_arvalid = m_axil_arvalid[m_select_reg]; // 发往选中目标主端口的 ARVALID。
wire                  current_m_axil_arready = m_axil_arready[m_select_reg]; // 选中目标主端口返回的 ARREADY。
wire [DATA_WIDTH-1:0] current_m_axil_rdata   = m_axil_rdata[m_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 来自选中目标主端口的 RDATA。
wire [1:0]            current_m_axil_rresp   = m_axil_rresp[m_select_reg*2 +: 2]; // 来自选中目标主端口的 RRESP。
wire                  current_m_axil_rvalid  = m_axil_rvalid[m_select_reg]; // 来自选中目标主端口的 RVALID。
wire                  current_m_axil_rready  = m_axil_rready[m_select_reg]; // 发往选中目标主端口的 RREADY。

// 仲裁器实例
wire [S_COUNT*2-1:0] request; // 仲裁请求位：每个从端偶数位=写请求，奇数位=读请求。
wire [S_COUNT*2-1:0] acknowledge; // 授予槽位在响应握手后回传的确认位。
wire [S_COUNT*2-1:0] grant; // 仲裁器对所有读写请求槽位输出的 one-hot 授予。
wire grant_valid; // 仲裁器当前存在有效获胜者。
wire [CL_S_COUNT:0] grant_encoded; // 获胜者编码索引；最低位 1=读，0=写。

wire read = grant_encoded[0]; // 从仲裁获胜编码提取事务类型位。
assign s_select = grant_encoded >> 1;

arbiter #(
    .PORTS(S_COUNT*2),
    .ARB_TYPE_ROUND_ROBIN(1),
    .ARB_BLOCK(1),
    .ARB_BLOCK_ACK(1),
    .ARB_LSB_HIGH_PRIORITY(1)
)
arb_inst (
    .clk(clk),
    .rst(rst),
    .request(request),
    .acknowledge(acknowledge),
    .grant(grant),
    .grant_valid(grant_valid),
    .grant_encoded(grant_encoded)
);

genvar n;

// 请求信号生成
generate
for (n = 0; n < S_COUNT; n = n + 1) begin
    assign request[2*n]   = s_axil_awvalid[n];
    assign request[2*n+1] = s_axil_arvalid[n];
end
endgenerate

// 确认信号生成
generate
for (n = 0; n < S_COUNT; n = n + 1) begin
    assign acknowledge[2*n]   = grant[2*n]   && s_axil_bvalid[n] && s_axil_bready[n];
    assign acknowledge[2*n+1] = grant[2*n+1] && s_axil_rvalid[n] && s_axil_rready[n];
end
endgenerate

always @* begin
    state_next = STATE_IDLE;

    match = 1'b0;

    m_select_next = m_select_reg;
    axil_addr_next = axil_addr_reg;
    axil_addr_valid_next = axil_addr_valid_reg;
    axil_prot_next = axil_prot_reg;
    axil_data_next = axil_data_reg;
    axil_wstrb_next = axil_wstrb_reg;
    axil_resp_next = axil_resp_reg;

    s_axil_awready_next = 0;
    s_axil_wready_next = 0;
    s_axil_bvalid_next = s_axil_bvalid_reg & ~s_axil_bready;
    s_axil_arready_next = 0;
    s_axil_rvalid_next = s_axil_rvalid_reg & ~s_axil_rready;

    m_axil_awvalid_next = m_axil_awvalid_reg & ~m_axil_awready;
    m_axil_wvalid_next = m_axil_wvalid_reg & ~m_axil_wready;
    m_axil_bready_next = 0;
    m_axil_arvalid_next = m_axil_arvalid_reg & ~m_axil_arready;
    m_axil_rready_next = 0;

    case (state_reg)
        STATE_IDLE: begin
            // 空闲态：等待仲裁结果

            if (grant_valid) begin

                axil_addr_valid_next = 1'b1;

                if (read) begin
                    // 读事务
                    axil_addr_next = current_s_axil_araddr;
                    axil_prot_next = current_s_axil_arprot;
                    s_axil_arready_next[s_select] = 1'b1;
                end else  begin
                    // 写事务
                    axil_addr_next = current_s_axil_awaddr;
                    axil_prot_next = current_s_axil_awprot;
                    s_axil_awready_next[s_select] = 1'b1;
                end

                state_next = STATE_DECODE;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_DECODE: begin
            // 解码态：确定目标主接口

            match = 1'b0;
            for (i = 0; i < M_COUNT; i = i + 1) begin
                for (j = 0; j < M_REGIONS; j = j + 1) begin
                    if (M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32] && (!M_SECURE[i] || !axil_prot_reg[1]) && ((read ? M_CONNECT_READ : M_CONNECT_WRITE) & (1 << (s_select+i*S_COUNT))) && (axil_addr_reg >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32]) == (M_BASE_ADDR_INT[(i*M_REGIONS+j)*ADDR_WIDTH +: ADDR_WIDTH] >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32])) begin
                        m_select_next = i;
                        match = 1'b1;
                    end
                end
            end

            if (match) begin
                if (read) begin
                    // 读事务
                    m_axil_rready_next[m_select_next] = 1'b1;
                    state_next = STATE_READ;
                end else begin
                    // 写事务
                    s_axil_wready_next[s_select] = 1'b1;
                    state_next = STATE_WRITE;
                end
            end else begin
                // 未命中任何区域：返回 DECERR
                axil_data_next = {DATA_WIDTH{1'b0}};
                axil_resp_next = 2'b11;
                if (read) begin
                    // 读事务
                    s_axil_rvalid_next[s_select] = 1'b1;
                    state_next = STATE_WAIT_IDLE;
                end else begin
                    // 写事务
                    s_axil_wready_next[s_select] = 1'b1;
                    state_next = STATE_WRITE_DROP;
                end
            end
        end
        STATE_WRITE: begin
            // 写态：缓存并转发写数据
            s_axil_wready_next[s_select] = 1'b1;

            if (axil_addr_valid_reg) begin
                m_axil_awvalid_next[m_select_reg] = 1'b1;
            end
            axil_addr_valid_next = 1'b0;

            if (current_s_axil_wready && current_s_axil_wvalid) begin
                s_axil_wready_next[s_select] = 1'b0;
                axil_data_next = current_s_axil_wdata;
                axil_wstrb_next = current_s_axil_wstrb;
                m_axil_wvalid_next[m_select_reg] = 1'b1;
                m_axil_bready_next[m_select_reg] = 1'b1;
                state_next = STATE_WRITE_RESP;
            end else begin
                state_next = STATE_WRITE;
            end
        end
        STATE_WRITE_RESP: begin
            // 写响应态：缓存并转发写响应
            m_axil_bready_next[m_select_reg] = 1'b1;

            if (current_m_axil_bready && current_m_axil_bvalid) begin
                m_axil_bready_next[m_select_reg] = 1'b0;
                axil_resp_next = current_m_axil_bresp;
                s_axil_bvalid_next[s_select] = 1'b1;
                state_next = STATE_WAIT_IDLE;
            end else begin
                state_next = STATE_WRITE_RESP;
            end
        end
        STATE_WRITE_DROP: begin
            // 写丢弃态：吞吐并丢弃写数据
            s_axil_wready_next[s_select] = 1'b1;

            axil_addr_valid_next = 1'b0;

            if (current_s_axil_wready && current_s_axil_wvalid) begin
                s_axil_wready_next[s_select] = 1'b0;
                s_axil_bvalid_next[s_select] = 1'b1;
                state_next = STATE_WAIT_IDLE;
            end else begin
                state_next = STATE_WRITE_DROP;
            end
        end
        STATE_READ: begin
            // 读态：缓存并转发读响应
            m_axil_rready_next[m_select_reg] = 1'b1;

            if (axil_addr_valid_reg) begin
                m_axil_arvalid_next[m_select_reg] = 1'b1;
            end
            axil_addr_valid_next = 1'b0;

            if (current_m_axil_rready && current_m_axil_rvalid) begin
                m_axil_rready_next[m_select_reg] = 1'b0;
                axil_data_next = current_m_axil_rdata;
                axil_resp_next = current_m_axil_rresp;
                s_axil_rvalid_next[s_select] = 1'b1;
                state_next = STATE_WAIT_IDLE;
            end else begin
                state_next = STATE_READ;
            end
        end
        STATE_WAIT_IDLE: begin
            // 等待空闲态：等待 grant_valid 拉低后释放

            if (!grant_valid || acknowledge) begin
                state_next = STATE_IDLE;
            end else begin
                state_next = STATE_WAIT_IDLE;
            end
        end
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axil_awready_reg <= 0;
        s_axil_wready_reg <= 0;
        s_axil_bvalid_reg <= 0;
        s_axil_arready_reg <= 0;
        s_axil_rvalid_reg <= 0;

        m_axil_awvalid_reg <= 0;
        m_axil_wvalid_reg <= 0;
        m_axil_bready_reg <= 0;
        m_axil_arvalid_reg <= 0;
        m_axil_rready_reg <= 0;
    end else begin
        state_reg <= state_next;

        s_axil_awready_reg <= s_axil_awready_next;
        s_axil_wready_reg <= s_axil_wready_next;
        s_axil_bvalid_reg <= s_axil_bvalid_next;
        s_axil_arready_reg <= s_axil_arready_next;
        s_axil_rvalid_reg <= s_axil_rvalid_next;

        m_axil_awvalid_reg <= m_axil_awvalid_next;
        m_axil_wvalid_reg <= m_axil_wvalid_next;
        m_axil_bready_reg <= m_axil_bready_next;
        m_axil_arvalid_reg <= m_axil_arvalid_next;
        m_axil_rready_reg <= m_axil_rready_next;
    end

    m_select_reg <= m_select_next;
    axil_addr_reg <= axil_addr_next;
    axil_addr_valid_reg <= axil_addr_valid_next;
    axil_prot_reg <= axil_prot_next;
    axil_data_reg <= axil_data_next;
    axil_wstrb_reg <= axil_wstrb_next;
    axil_resp_reg <= axil_resp_next;
end

endmodule

`resetall
