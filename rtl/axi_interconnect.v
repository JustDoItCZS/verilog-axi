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
 * AXI4 互连模块
 *
 * 模块目录
 * 1) 在多个 AXI 发起端(S 口)和多个目标端(M 口)之间做地址解码与时分复用。
 * 2) 同一时刻仅服务一次读或写事务，保证通道关联简单、时序可控。
 * 3) 内部包含仲裁、地址匹配、错误响应生成，以及 R/W 通道 skid buffer。
 */
module axi_interconnect #
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
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
    // 是否透传 ID 字段
    parameter FORWARD_ID = 0,
    // 每个主接口地址区域数量
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
    input  wire                            clk, // 互连主时钟。
    input  wire                            rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [S_COUNT*ID_WIDTH-1:0]     s_axi_awid, // 所有 S 口拼接的 AWID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]   s_axi_awaddr, // 所有 S 口拼接的 AWADDR。
    input  wire [S_COUNT*8-1:0]            s_axi_awlen, // 所有 S 口拼接的 AWLEN。
    input  wire [S_COUNT*3-1:0]            s_axi_awsize, // 所有 S 口拼接的 AWSIZE。
    input  wire [S_COUNT*2-1:0]            s_axi_awburst, // 所有 S 口拼接的 AWBURST。
    input  wire [S_COUNT-1:0]              s_axi_awlock, // 所有 S 口 AWLOCK。
    input  wire [S_COUNT*4-1:0]            s_axi_awcache, // 所有 S 口拼接的 AWCACHE。
    input  wire [S_COUNT*3-1:0]            s_axi_awprot, // 所有 S 口拼接的 AWPROT。
    input  wire [S_COUNT*4-1:0]            s_axi_awqos, // 所有 S 口拼接的 AWQOS。
    input  wire [S_COUNT*AWUSER_WIDTH-1:0] s_axi_awuser, // 所有 S 口拼接的 AWUSER。
    input  wire [S_COUNT-1:0]              s_axi_awvalid, // 所有 S 口 AWVALID。
    output wire [S_COUNT-1:0]              s_axi_awready, // 所有 S 口 AWREADY。
    input  wire [S_COUNT*DATA_WIDTH-1:0]   s_axi_wdata, // 所有 S 口拼接的 WDATA。
    input  wire [S_COUNT*STRB_WIDTH-1:0]   s_axi_wstrb, // 所有 S 口拼接的 WSTRB。
    input  wire [S_COUNT-1:0]              s_axi_wlast, // 所有 S 口 WLAST。
    input  wire [S_COUNT*WUSER_WIDTH-1:0]  s_axi_wuser, // 所有 S 口拼接的 WUSER。
    input  wire [S_COUNT-1:0]              s_axi_wvalid, // 所有 S 口 WVALID。
    output wire [S_COUNT-1:0]              s_axi_wready, // 所有 S 口 WREADY。
    output wire [S_COUNT*ID_WIDTH-1:0]     s_axi_bid, // 所有 S 口拼接的 BID。
    output wire [S_COUNT*2-1:0]            s_axi_bresp, // 所有 S 口拼接的 BRESP。
    output wire [S_COUNT*BUSER_WIDTH-1:0]  s_axi_buser, // 所有 S 口拼接的 BUSER。
    output wire [S_COUNT-1:0]              s_axi_bvalid, // 所有 S 口 BVALID。
    input  wire [S_COUNT-1:0]              s_axi_bready, // 所有 S 口 BREADY。
    input  wire [S_COUNT*ID_WIDTH-1:0]     s_axi_arid, // 所有 S 口拼接的 ARID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]   s_axi_araddr, // 所有 S 口拼接的 ARADDR。
    input  wire [S_COUNT*8-1:0]            s_axi_arlen, // 所有 S 口拼接的 ARLEN。
    input  wire [S_COUNT*3-1:0]            s_axi_arsize, // 所有 S 口拼接的 ARSIZE。
    input  wire [S_COUNT*2-1:0]            s_axi_arburst, // 所有 S 口拼接的 ARBURST。
    input  wire [S_COUNT-1:0]              s_axi_arlock, // 所有 S 口 ARLOCK。
    input  wire [S_COUNT*4-1:0]            s_axi_arcache, // 所有 S 口拼接的 ARCACHE。
    input  wire [S_COUNT*3-1:0]            s_axi_arprot, // 所有 S 口拼接的 ARPROT。
    input  wire [S_COUNT*4-1:0]            s_axi_arqos, // 所有 S 口拼接的 ARQOS。
    input  wire [S_COUNT*ARUSER_WIDTH-1:0] s_axi_aruser, // 所有 S 口拼接的 ARUSER。
    input  wire [S_COUNT-1:0]              s_axi_arvalid, // 所有 S 口 ARVALID。
    output wire [S_COUNT-1:0]              s_axi_arready, // 所有 S 口 ARREADY。
    output wire [S_COUNT*ID_WIDTH-1:0]     s_axi_rid, // 所有 S 口拼接的 RID。
    output wire [S_COUNT*DATA_WIDTH-1:0]   s_axi_rdata, // 所有 S 口拼接的 RDATA。
    output wire [S_COUNT*2-1:0]            s_axi_rresp, // 所有 S 口拼接的 RRESP。
    output wire [S_COUNT-1:0]              s_axi_rlast, // 所有 S 口 RLAST。
    output wire [S_COUNT*RUSER_WIDTH-1:0]  s_axi_ruser, // 所有 S 口拼接的 RUSER。
    output wire [S_COUNT-1:0]              s_axi_rvalid, // 所有 S 口 RVALID。
    input  wire [S_COUNT-1:0]              s_axi_rready, // 所有 S 口 RREADY。

    /*
     * AXI 主接口
     */
    output wire [M_COUNT*ID_WIDTH-1:0]     m_axi_awid, // 所有 M 口拼接的 AWID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]   m_axi_awaddr, // 所有 M 口拼接的 AWADDR 输出。
    output wire [M_COUNT*8-1:0]            m_axi_awlen, // 所有 M 口拼接的 AWLEN 输出。
    output wire [M_COUNT*3-1:0]            m_axi_awsize, // 所有 M 口拼接的 AWSIZE 输出。
    output wire [M_COUNT*2-1:0]            m_axi_awburst, // 所有 M 口拼接的 AWBURST 输出。
    output wire [M_COUNT-1:0]              m_axi_awlock, // 所有 M 口 AWLOCK 输出。
    output wire [M_COUNT*4-1:0]            m_axi_awcache, // 所有 M 口拼接的 AWCACHE 输出。
    output wire [M_COUNT*3-1:0]            m_axi_awprot, // 所有 M 口拼接的 AWPROT 输出。
    output wire [M_COUNT*4-1:0]            m_axi_awqos, // 所有 M 口拼接的 AWQOS 输出。
    output wire [M_COUNT*4-1:0]            m_axi_awregion, // 所有 M 口拼接的 AWREGION 输出。
    output wire [M_COUNT*AWUSER_WIDTH-1:0] m_axi_awuser, // 所有 M 口拼接的 AWUSER 输出。
    output wire [M_COUNT-1:0]              m_axi_awvalid, // 所有 M 口 AWVALID 输出。
    input  wire [M_COUNT-1:0]              m_axi_awready, // 所有 M 口 AWREADY 输入。
    output wire [M_COUNT*DATA_WIDTH-1:0]   m_axi_wdata, // 所有 M 口拼接的 WDATA 输出。
    output wire [M_COUNT*STRB_WIDTH-1:0]   m_axi_wstrb, // 所有 M 口拼接的 WSTRB 输出。
    output wire [M_COUNT-1:0]              m_axi_wlast, // 所有 M 口 WLAST 输出。
    output wire [M_COUNT*WUSER_WIDTH-1:0]  m_axi_wuser, // 所有 M 口拼接的 WUSER 输出。
    output wire [M_COUNT-1:0]              m_axi_wvalid, // 所有 M 口 WVALID 输出。
    input  wire [M_COUNT-1:0]              m_axi_wready, // 所有 M 口 WREADY 输入。
    input  wire [M_COUNT*ID_WIDTH-1:0]     m_axi_bid, // 所有 M 口拼接的 BID 输入。
    input  wire [M_COUNT*2-1:0]            m_axi_bresp, // 所有 M 口拼接的 BRESP 输入。
    input  wire [M_COUNT*BUSER_WIDTH-1:0]  m_axi_buser, // 所有 M 口拼接的 BUSER 输入。
    input  wire [M_COUNT-1:0]              m_axi_bvalid, // 所有 M 口 BVALID 输入。
    output wire [M_COUNT-1:0]              m_axi_bready, // 所有 M 口 BREADY 输出。
    output wire [M_COUNT*ID_WIDTH-1:0]     m_axi_arid, // 所有 M 口拼接的 ARID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]   m_axi_araddr, // 所有 M 口拼接的 ARADDR 输出。
    output wire [M_COUNT*8-1:0]            m_axi_arlen, // 所有 M 口拼接的 ARLEN 输出。
    output wire [M_COUNT*3-1:0]            m_axi_arsize, // 所有 M 口拼接的 ARSIZE 输出。
    output wire [M_COUNT*2-1:0]            m_axi_arburst, // 所有 M 口拼接的 ARBURST 输出。
    output wire [M_COUNT-1:0]              m_axi_arlock, // 所有 M 口 ARLOCK 输出。
    output wire [M_COUNT*4-1:0]            m_axi_arcache, // 所有 M 口拼接的 ARCACHE 输出。
    output wire [M_COUNT*3-1:0]            m_axi_arprot, // 所有 M 口拼接的 ARPROT 输出。
    output wire [M_COUNT*4-1:0]            m_axi_arqos, // 所有 M 口拼接的 ARQOS 输出。
    output wire [M_COUNT*4-1:0]            m_axi_arregion, // 所有 M 口拼接的 ARREGION 输出。
    output wire [M_COUNT*ARUSER_WIDTH-1:0] m_axi_aruser, // 所有 M 口拼接的 ARUSER 输出。
    output wire [M_COUNT-1:0]              m_axi_arvalid, // 所有 M 口 ARVALID 输出。
    input  wire [M_COUNT-1:0]              m_axi_arready, // 所有 M 口 ARREADY 输入。
    input  wire [M_COUNT*ID_WIDTH-1:0]     m_axi_rid, // 所有 M 口拼接的 RID 输入。
    input  wire [M_COUNT*DATA_WIDTH-1:0]   m_axi_rdata, // 所有 M 口拼接的 RDATA 输入。
    input  wire [M_COUNT*2-1:0]            m_axi_rresp, // 所有 M 口拼接的 RRESP 输入。
    input  wire [M_COUNT-1:0]              m_axi_rlast, // 所有 M 口 RLAST 输入。
    input  wire [M_COUNT*RUSER_WIDTH-1:0]  m_axi_ruser, // 所有 M 口拼接的 RUSER 输入。
    input  wire [M_COUNT-1:0]              m_axi_rvalid, // 所有 M 口 RVALID 输入。
    output wire [M_COUNT-1:0]              m_axi_rready // 所有 M 口 RREADY 输出。
);

parameter CL_S_COUNT = $clog2(S_COUNT);
parameter CL_M_COUNT = $clog2(M_COUNT);

parameter AUSER_WIDTH = AWUSER_WIDTH > ARUSER_WIDTH ? AWUSER_WIDTH : ARUSER_WIDTH;

// 默认地址映射计算
function [M_COUNT*M_REGIONS*ADDR_WIDTH-1:0] calcBaseAddrs(input [31:0] dummy);
    integer i; // 地址区域遍历索引。
    reg [ADDR_WIDTH-1:0] base; // 当前自动分配的基地址游标。
    reg [ADDR_WIDTH-1:0] width; // 当前区域地址宽度。
    reg [ADDR_WIDTH-1:0] size; // 当前区域大小(字节空间)。
    reg [ADDR_WIDTH-1:0] mask; // 当前区域地址掩码。
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

integer i, j; // 配置检查/地址解码循环索引。

// 配置合法性检查
initial begin
    if (M_REGIONS < 1 || M_REGIONS > 16) begin
        $error("Error: M_REGIONS must be between 1 and 16 (instance %m)");
        $finish;
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && (M_ADDR_WIDTH[i*32 +: 32] < 12 || M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH)) begin
            $error("Error: address width out of range (instance %m)");
            $finish;
        end
    end

    $display("Addressing configuration for axi_interconnect instance %m");
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
    STATE_IDLE = 3'd0, // 空闲，等待仲裁决定本轮服务的 S 口和方向。
    STATE_DECODE = 3'd1, // 地址解码，决定目标 M 口和 region。
    STATE_WRITE = 3'd2, // 正常写数据转发阶段。
    STATE_WRITE_RESP = 3'd3, // 等待并转发 B 响应。
    STATE_WRITE_DROP = 3'd4, // 地址未命中时丢弃 W 并回送 DECERR。
    STATE_READ = 3'd5, // 正常读数据转发阶段。
    STATE_READ_DROP = 3'd6, // 地址未命中时构造 DECERR 读响应。
    STATE_WAIT_IDLE = 3'd7; // 等待仲裁器 grant 撤销，避免同事务重复进入。

reg [2:0] state_reg = STATE_IDLE, state_next; // 主状态机寄存器。

reg match; // 地址解码是否命中任一可达 M 口区域。

reg [CL_M_COUNT-1:0] m_select_reg = 2'd0, m_select_next; // 当前事务选中的目标 M 口编号。
reg [ID_WIDTH-1:0] axi_id_reg = {ID_WIDTH{1'b0}}, axi_id_next; // 当前事务 ID(返回给源端)。
reg [ADDR_WIDTH-1:0] axi_addr_reg = {ADDR_WIDTH{1'b0}}, axi_addr_next; // 当前事务地址。
reg axi_addr_valid_reg = 1'b0, axi_addr_valid_next; // 地址是否待下发到目标 M 口。
reg [7:0] axi_len_reg = 8'd0, axi_len_next; // 当前 burst 长度。
reg [2:0] axi_size_reg = 3'd0, axi_size_next; // 当前 burst beat 大小。
reg [1:0] axi_burst_reg = 2'd0, axi_burst_next; // 当前 burst 类型。
reg axi_lock_reg = 1'b0, axi_lock_next; // 当前事务 lock 属性。
reg [3:0] axi_cache_reg = 4'd0, axi_cache_next; // 当前事务 cache 属性。
reg [2:0] axi_prot_reg = 3'b000, axi_prot_next; // 当前事务保护属性(含 secure 检查位)。
reg [3:0] axi_qos_reg = 4'd0, axi_qos_next; // 当前事务 QoS。
reg [3:0] axi_region_reg = 4'd0, axi_region_next; // 目标 region 编号(映射到 AW/ARREGION)。
reg [AUSER_WIDTH-1:0] axi_auser_reg = {AUSER_WIDTH{1'b0}}, axi_auser_next; // 当前地址通道 user 信息。
reg [1:0] axi_bresp_reg = 2'b00, axi_bresp_next; // 写事务返回给源端的 BRESP。
reg [BUSER_WIDTH-1:0] axi_buser_reg = {BUSER_WIDTH{1'b0}}, axi_buser_next; // 写事务返回给源端的 BUSER。

reg [S_COUNT-1:0] s_axi_awready_reg = 0, s_axi_awready_next; // 各 S 口 AWREADY 输出寄存器。
reg [S_COUNT-1:0] s_axi_wready_reg = 0, s_axi_wready_next; // 各 S 口 WREADY 输出寄存器。
reg [S_COUNT-1:0] s_axi_bvalid_reg = 0, s_axi_bvalid_next; // 各 S 口 BVALID 输出寄存器。
reg [S_COUNT-1:0] s_axi_arready_reg = 0, s_axi_arready_next; // 各 S 口 ARREADY 输出寄存器。

reg [M_COUNT-1:0] m_axi_awvalid_reg = 0, m_axi_awvalid_next; // 各 M 口 AWVALID 输出寄存器。
reg [M_COUNT-1:0] m_axi_bready_reg = 0, m_axi_bready_next; // 各 M 口 BREADY 输出寄存器。
reg [M_COUNT-1:0] m_axi_arvalid_reg = 0, m_axi_arvalid_next; // 各 M 口 ARVALID 输出寄存器。
reg [M_COUNT-1:0] m_axi_rready_reg = 0, m_axi_rready_next; // 各 M 口 RREADY 输出寄存器。

// 内部数据通路
reg  [ID_WIDTH-1:0]    s_axi_rid_int; // R 通道内部待输出 RID。
reg  [DATA_WIDTH-1:0]  s_axi_rdata_int; // R 通道内部待输出 RDATA。
reg  [1:0]             s_axi_rresp_int; // R 通道内部待输出 RRESP。
reg                    s_axi_rlast_int; // R 通道内部待输出 RLAST。
reg  [RUSER_WIDTH-1:0] s_axi_ruser_int; // R 通道内部待输出 RUSER。
reg                    s_axi_rvalid_int; // R 通道内部待输出有效标志。
reg                    s_axi_rready_int_reg = 1'b0; // R 输出级对内部数据源的 ready 寄存。
wire                   s_axi_rready_int_early; // R 输出级组合 ready 预测。

reg  [DATA_WIDTH-1:0]  m_axi_wdata_int; // W 通道内部待输出 WDATA。
reg  [STRB_WIDTH-1:0]  m_axi_wstrb_int; // W 通道内部待输出 WSTRB。
reg                    m_axi_wlast_int; // W 通道内部待输出 WLAST。
reg  [WUSER_WIDTH-1:0] m_axi_wuser_int; // W 通道内部待输出 WUSER。
reg                    m_axi_wvalid_int; // W 通道内部待输出有效标志。
reg                    m_axi_wready_int_reg = 1'b0; // W 输出级对内部数据源的 ready 寄存。
wire                   m_axi_wready_int_early; // W 输出级组合 ready 预测。

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = s_axi_wready_reg;
assign s_axi_bid = {S_COUNT{axi_id_reg}};
assign s_axi_bresp = {S_COUNT{axi_bresp_reg}};
assign s_axi_buser = {S_COUNT{BUSER_ENABLE ? axi_buser_reg : {BUSER_WIDTH{1'b0}}}};
assign s_axi_bvalid = s_axi_bvalid_reg;
assign s_axi_arready = s_axi_arready_reg;

assign m_axi_awid = {M_COUNT{FORWARD_ID ? axi_id_reg : {ID_WIDTH{1'b0}}}};
assign m_axi_awaddr = {M_COUNT{axi_addr_reg}};
assign m_axi_awlen = {M_COUNT{axi_len_reg}};
assign m_axi_awsize = {M_COUNT{axi_size_reg}};
assign m_axi_awburst = {M_COUNT{axi_burst_reg}};
assign m_axi_awlock = {M_COUNT{axi_lock_reg}};
assign m_axi_awcache = {M_COUNT{axi_cache_reg}};
assign m_axi_awprot = {M_COUNT{axi_prot_reg}};
assign m_axi_awqos = {M_COUNT{axi_qos_reg}};
assign m_axi_awregion = {M_COUNT{axi_region_reg}};
assign m_axi_awuser = {M_COUNT{AWUSER_ENABLE ? axi_auser_reg[AWUSER_WIDTH-1:0] : {AWUSER_WIDTH{1'b0}}}};
assign m_axi_awvalid = m_axi_awvalid_reg;
assign m_axi_bready = m_axi_bready_reg;
assign m_axi_arid = {M_COUNT{FORWARD_ID ? axi_id_reg : {ID_WIDTH{1'b0}}}};
assign m_axi_araddr = {M_COUNT{axi_addr_reg}};
assign m_axi_arlen = {M_COUNT{axi_len_reg}};
assign m_axi_arsize = {M_COUNT{axi_size_reg}};
assign m_axi_arburst = {M_COUNT{axi_burst_reg}};
assign m_axi_arlock = {M_COUNT{axi_lock_reg}};
assign m_axi_arcache = {M_COUNT{axi_cache_reg}};
assign m_axi_arprot = {M_COUNT{axi_prot_reg}};
assign m_axi_arqos = {M_COUNT{axi_qos_reg}};
assign m_axi_arregion = {M_COUNT{axi_region_reg}};
assign m_axi_aruser = {M_COUNT{ARUSER_ENABLE ? axi_auser_reg[ARUSER_WIDTH-1:0] : {ARUSER_WIDTH{1'b0}}}};
assign m_axi_arvalid = m_axi_arvalid_reg;
assign m_axi_rready = m_axi_rready_reg;

// 从端侧复用
wire [(CL_S_COUNT > 0 ? CL_S_COUNT-1 : 0):0] s_select; // 当前被仲裁选中的源 S 口编号。

wire [ID_WIDTH-1:0]     current_s_axi_awid      = s_axi_awid[s_select*ID_WIDTH +: ID_WIDTH]; // 当前 S 口 AWID。
wire [ADDR_WIDTH-1:0]   current_s_axi_awaddr    = s_axi_awaddr[s_select*ADDR_WIDTH +: ADDR_WIDTH]; // 当前 S 口 AWADDR。
wire [7:0]              current_s_axi_awlen     = s_axi_awlen[s_select*8 +: 8]; // 当前 S 口 AWLEN。
wire [2:0]              current_s_axi_awsize    = s_axi_awsize[s_select*3 +: 3]; // 当前 S 口 AWSIZE。
wire [1:0]              current_s_axi_awburst   = s_axi_awburst[s_select*2 +: 2]; // 当前 S 口 AWBURST。
wire                    current_s_axi_awlock    = s_axi_awlock[s_select]; // 当前 S 口 AWLOCK。
wire [3:0]              current_s_axi_awcache   = s_axi_awcache[s_select*4 +: 4]; // 当前 S 口 AWCACHE。
wire [2:0]              current_s_axi_awprot    = s_axi_awprot[s_select*3 +: 3]; // 当前 S 口 AWPROT。
wire [3:0]              current_s_axi_awqos     = s_axi_awqos[s_select*4 +: 4]; // 当前 S 口 AWQOS。
wire [AWUSER_WIDTH-1:0] current_s_axi_awuser    = s_axi_awuser[s_select*AWUSER_WIDTH +: AWUSER_WIDTH]; // 当前 S 口 AWUSER。
wire                    current_s_axi_awvalid   = s_axi_awvalid[s_select]; // 当前 S 口 AWVALID。
wire                    current_s_axi_awready   = s_axi_awready[s_select]; // 当前 S 口 AWREADY。
wire [DATA_WIDTH-1:0]   current_s_axi_wdata     = s_axi_wdata[s_select*DATA_WIDTH +: DATA_WIDTH]; // 当前 S 口 WDATA。
wire [STRB_WIDTH-1:0]   current_s_axi_wstrb     = s_axi_wstrb[s_select*STRB_WIDTH +: STRB_WIDTH]; // 当前 S 口 WSTRB。
wire                    current_s_axi_wlast     = s_axi_wlast[s_select]; // 当前 S 口 WLAST。
wire [WUSER_WIDTH-1:0]  current_s_axi_wuser     = s_axi_wuser[s_select*WUSER_WIDTH +: WUSER_WIDTH]; // 当前 S 口 WUSER。
wire                    current_s_axi_wvalid    = s_axi_wvalid[s_select]; // 当前 S 口 WVALID。
wire                    current_s_axi_wready    = s_axi_wready[s_select]; // 当前 S 口 WREADY。
wire [ID_WIDTH-1:0]     current_s_axi_bid       = s_axi_bid[s_select*ID_WIDTH +: ID_WIDTH]; // 当前 S 口 BID。
wire [1:0]              current_s_axi_bresp     = s_axi_bresp[s_select*2 +: 2]; // 当前 S 口 BRESP。
wire [BUSER_WIDTH-1:0]  current_s_axi_buser     = s_axi_buser[s_select*BUSER_WIDTH +: BUSER_WIDTH]; // 当前 S 口 BUSER。
wire                    current_s_axi_bvalid    = s_axi_bvalid[s_select]; // 当前 S 口 BVALID。
wire                    current_s_axi_bready    = s_axi_bready[s_select]; // 当前 S 口 BREADY。
wire [ID_WIDTH-1:0]     current_s_axi_arid      = s_axi_arid[s_select*ID_WIDTH +: ID_WIDTH]; // 当前 S 口 ARID。
wire [ADDR_WIDTH-1:0]   current_s_axi_araddr    = s_axi_araddr[s_select*ADDR_WIDTH +: ADDR_WIDTH]; // 当前 S 口 ARADDR。
wire [7:0]              current_s_axi_arlen     = s_axi_arlen[s_select*8 +: 8]; // 当前 S 口 ARLEN。
wire [2:0]              current_s_axi_arsize    = s_axi_arsize[s_select*3 +: 3]; // 当前 S 口 ARSIZE。
wire [1:0]              current_s_axi_arburst   = s_axi_arburst[s_select*2 +: 2]; // 当前 S 口 ARBURST。
wire                    current_s_axi_arlock    = s_axi_arlock[s_select]; // 当前 S 口 ARLOCK。
wire [3:0]              current_s_axi_arcache   = s_axi_arcache[s_select*4 +: 4]; // 当前 S 口 ARCACHE。
wire [2:0]              current_s_axi_arprot    = s_axi_arprot[s_select*3 +: 3]; // 当前 S 口 ARPROT。
wire [3:0]              current_s_axi_arqos     = s_axi_arqos[s_select*4 +: 4]; // 当前 S 口 ARQOS。
wire [ARUSER_WIDTH-1:0] current_s_axi_aruser    = s_axi_aruser[s_select*ARUSER_WIDTH +: ARUSER_WIDTH]; // 当前 S 口 ARUSER。
wire                    current_s_axi_arvalid   = s_axi_arvalid[s_select]; // 当前 S 口 ARVALID。
wire                    current_s_axi_arready   = s_axi_arready[s_select]; // 当前 S 口 ARREADY。
wire [ID_WIDTH-1:0]     current_s_axi_rid       = s_axi_rid[s_select*ID_WIDTH +: ID_WIDTH]; // 当前 S 口 RID。
wire [DATA_WIDTH-1:0]   current_s_axi_rdata     = s_axi_rdata[s_select*DATA_WIDTH +: DATA_WIDTH]; // 当前 S 口 RDATA。
wire [1:0]              current_s_axi_rresp     = s_axi_rresp[s_select*2 +: 2]; // 当前 S 口 RRESP。
wire                    current_s_axi_rlast     = s_axi_rlast[s_select]; // 当前 S 口 RLAST。
wire [RUSER_WIDTH-1:0]  current_s_axi_ruser     = s_axi_ruser[s_select*RUSER_WIDTH +: RUSER_WIDTH]; // 当前 S 口 RUSER。
wire                    current_s_axi_rvalid    = s_axi_rvalid[s_select]; // 当前 S 口 RVALID。
wire                    current_s_axi_rready    = s_axi_rready[s_select]; // 当前 S 口 RREADY。

// 主端侧复用
wire [ID_WIDTH-1:0]     current_m_axi_awid      = m_axi_awid[m_select_reg*ID_WIDTH +: ID_WIDTH]; // 当前目标 M 口 AWID。
wire [ADDR_WIDTH-1:0]   current_m_axi_awaddr    = m_axi_awaddr[m_select_reg*ADDR_WIDTH +: ADDR_WIDTH]; // 当前目标 M 口 AWADDR。
wire [7:0]              current_m_axi_awlen     = m_axi_awlen[m_select_reg*8 +: 8]; // 当前目标 M 口 AWLEN。
wire [2:0]              current_m_axi_awsize    = m_axi_awsize[m_select_reg*3 +: 3]; // 当前目标 M 口 AWSIZE。
wire [1:0]              current_m_axi_awburst   = m_axi_awburst[m_select_reg*2 +: 2]; // 当前目标 M 口 AWBURST。
wire                    current_m_axi_awlock    = m_axi_awlock[m_select_reg]; // 当前目标 M 口 AWLOCK。
wire [3:0]              current_m_axi_awcache   = m_axi_awcache[m_select_reg*4 +: 4]; // 当前目标 M 口 AWCACHE。
wire [2:0]              current_m_axi_awprot    = m_axi_awprot[m_select_reg*3 +: 3]; // 当前目标 M 口 AWPROT。
wire [3:0]              current_m_axi_awqos     = m_axi_awqos[m_select_reg*4 +: 4]; // 当前目标 M 口 AWQOS。
wire [3:0]              current_m_axi_awregion  = m_axi_awregion[m_select_reg*4 +: 4]; // 当前目标 M 口 AWREGION。
wire [AWUSER_WIDTH-1:0] current_m_axi_awuser    = m_axi_awuser[m_select_reg*AWUSER_WIDTH +: AWUSER_WIDTH]; // 当前目标 M 口 AWUSER。
wire                    current_m_axi_awvalid   = m_axi_awvalid[m_select_reg]; // 当前目标 M 口 AWVALID。
wire                    current_m_axi_awready   = m_axi_awready[m_select_reg]; // 当前目标 M 口 AWREADY。
wire [DATA_WIDTH-1:0]   current_m_axi_wdata     = m_axi_wdata[m_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 当前目标 M 口 WDATA。
wire [STRB_WIDTH-1:0]   current_m_axi_wstrb     = m_axi_wstrb[m_select_reg*STRB_WIDTH +: STRB_WIDTH]; // 当前目标 M 口 WSTRB。
wire                    current_m_axi_wlast     = m_axi_wlast[m_select_reg]; // 当前目标 M 口 WLAST。
wire [WUSER_WIDTH-1:0]  current_m_axi_wuser     = m_axi_wuser[m_select_reg*WUSER_WIDTH +: WUSER_WIDTH]; // 当前目标 M 口 WUSER。
wire                    current_m_axi_wvalid    = m_axi_wvalid[m_select_reg]; // 当前目标 M 口 WVALID。
wire                    current_m_axi_wready    = m_axi_wready[m_select_reg]; // 当前目标 M 口 WREADY。
wire [ID_WIDTH-1:0]     current_m_axi_bid       = m_axi_bid[m_select_reg*ID_WIDTH +: ID_WIDTH]; // 当前目标 M 口 BID。
wire [1:0]              current_m_axi_bresp     = m_axi_bresp[m_select_reg*2 +: 2]; // 当前目标 M 口 BRESP。
wire [BUSER_WIDTH-1:0]  current_m_axi_buser     = m_axi_buser[m_select_reg*BUSER_WIDTH +: BUSER_WIDTH]; // 当前目标 M 口 BUSER。
wire                    current_m_axi_bvalid    = m_axi_bvalid[m_select_reg]; // 当前目标 M 口 BVALID。
wire                    current_m_axi_bready    = m_axi_bready[m_select_reg]; // 当前目标 M 口 BREADY。
wire [ID_WIDTH-1:0]     current_m_axi_arid      = m_axi_arid[m_select_reg*ID_WIDTH +: ID_WIDTH]; // 当前目标 M 口 ARID。
wire [ADDR_WIDTH-1:0]   current_m_axi_araddr    = m_axi_araddr[m_select_reg*ADDR_WIDTH +: ADDR_WIDTH]; // 当前目标 M 口 ARADDR。
wire [7:0]              current_m_axi_arlen     = m_axi_arlen[m_select_reg*8 +: 8]; // 当前目标 M 口 ARLEN。
wire [2:0]              current_m_axi_arsize    = m_axi_arsize[m_select_reg*3 +: 3]; // 当前目标 M 口 ARSIZE。
wire [1:0]              current_m_axi_arburst   = m_axi_arburst[m_select_reg*2 +: 2]; // 当前目标 M 口 ARBURST。
wire                    current_m_axi_arlock    = m_axi_arlock[m_select_reg]; // 当前目标 M 口 ARLOCK。
wire [3:0]              current_m_axi_arcache   = m_axi_arcache[m_select_reg*4 +: 4]; // 当前目标 M 口 ARCACHE。
wire [2:0]              current_m_axi_arprot    = m_axi_arprot[m_select_reg*3 +: 3]; // 当前目标 M 口 ARPROT。
wire [3:0]              current_m_axi_arqos     = m_axi_arqos[m_select_reg*4 +: 4]; // 当前目标 M 口 ARQOS。
wire [3:0]              current_m_axi_arregion  = m_axi_arregion[m_select_reg*4 +: 4]; // 当前目标 M 口 ARREGION。
wire [ARUSER_WIDTH-1:0] current_m_axi_aruser    = m_axi_aruser[m_select_reg*ARUSER_WIDTH +: ARUSER_WIDTH]; // 当前目标 M 口 ARUSER。
wire                    current_m_axi_arvalid   = m_axi_arvalid[m_select_reg]; // 当前目标 M 口 ARVALID。
wire                    current_m_axi_arready   = m_axi_arready[m_select_reg]; // 当前目标 M 口 ARREADY。
wire [ID_WIDTH-1:0]     current_m_axi_rid       = m_axi_rid[m_select_reg*ID_WIDTH +: ID_WIDTH]; // 当前目标 M 口 RID。
wire [DATA_WIDTH-1:0]   current_m_axi_rdata     = m_axi_rdata[m_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 当前目标 M 口 RDATA。
wire [1:0]              current_m_axi_rresp     = m_axi_rresp[m_select_reg*2 +: 2]; // 当前目标 M 口 RRESP。
wire                    current_m_axi_rlast     = m_axi_rlast[m_select_reg]; // 当前目标 M 口 RLAST。
wire [RUSER_WIDTH-1:0]  current_m_axi_ruser     = m_axi_ruser[m_select_reg*RUSER_WIDTH +: RUSER_WIDTH]; // 当前目标 M 口 RUSER。
wire                    current_m_axi_rvalid    = m_axi_rvalid[m_select_reg]; // 当前目标 M 口 RVALID。
wire                    current_m_axi_rready    = m_axi_rready[m_select_reg]; // 当前目标 M 口 RREADY。

// 仲裁器实例
wire [S_COUNT*2-1:0] request; // 仲裁请求向量(每个 S 口包含写请求+读请求)。
wire [S_COUNT*2-1:0] acknowledge; // 仲裁完成应答向量(事务真正结束时拉高)。
wire [S_COUNT*2-1:0] grant; // 仲裁授权 one-hot 向量。
wire grant_valid; // 当前是否存在有效授权。
wire [CL_S_COUNT:0] grant_encoded; // 编码后的授权值，最低位区分读/写。

wire read = grant_encoded[0]; // 1 表示本轮服务读事务，0 表示写事务。
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
    assign request[2*n]   = s_axi_awvalid[n];
    assign request[2*n+1] = s_axi_arvalid[n];
end
endgenerate

// 确认信号生成
generate
for (n = 0; n < S_COUNT; n = n + 1) begin
    assign acknowledge[2*n]   = grant[2*n]   && s_axi_bvalid[n] && s_axi_bready[n];
    assign acknowledge[2*n+1] = grant[2*n+1] && s_axi_rvalid[n] && s_axi_rready[n] && s_axi_rlast[n];
end
endgenerate

always @* begin
    state_next = STATE_IDLE;

    match = 1'b0;

    m_select_next = m_select_reg;
    axi_id_next = axi_id_reg;
    axi_addr_next = axi_addr_reg;
    axi_addr_valid_next = axi_addr_valid_reg;
    axi_len_next = axi_len_reg;
    axi_size_next = axi_size_reg;
    axi_burst_next = axi_burst_reg;
    axi_lock_next = axi_lock_reg;
    axi_cache_next = axi_cache_reg;
    axi_prot_next = axi_prot_reg;
    axi_qos_next = axi_qos_reg;
    axi_region_next = axi_region_reg;
    axi_auser_next = axi_auser_reg;
    axi_bresp_next = axi_bresp_reg;
    axi_buser_next = axi_buser_reg;

    s_axi_awready_next = 0;
    s_axi_wready_next = 0;
    s_axi_bvalid_next = s_axi_bvalid_reg & ~s_axi_bready;
    s_axi_arready_next = 0;

    m_axi_awvalid_next = m_axi_awvalid_reg & ~m_axi_awready;
    m_axi_bready_next = 0;
    m_axi_arvalid_next = m_axi_arvalid_reg & ~m_axi_arready;
    m_axi_rready_next = 0;

    s_axi_rid_int = axi_id_reg;
    s_axi_rdata_int = current_m_axi_rdata;
    s_axi_rresp_int = current_m_axi_rresp;
    s_axi_rlast_int = current_m_axi_rlast;
    s_axi_ruser_int = current_m_axi_ruser;
    s_axi_rvalid_int = 1'b0;

    m_axi_wdata_int = current_s_axi_wdata;
    m_axi_wstrb_int = current_s_axi_wstrb;
    m_axi_wlast_int = current_s_axi_wlast;
    m_axi_wuser_int = current_s_axi_wuser;
    m_axi_wvalid_int = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            // 空闲态：等待仲裁结果

            if (grant_valid) begin

                axi_addr_valid_next = 1'b1;

                if (read) begin
                    // 读事务
                    axi_addr_next = current_s_axi_araddr;
                    axi_prot_next = current_s_axi_arprot;
                    axi_id_next = current_s_axi_arid;
                    axi_addr_next = current_s_axi_araddr;
                    axi_len_next = current_s_axi_arlen;
                    axi_size_next = current_s_axi_arsize;
                    axi_burst_next = current_s_axi_arburst;
                    axi_lock_next = current_s_axi_arlock;
                    axi_cache_next = current_s_axi_arcache;
                    axi_prot_next = current_s_axi_arprot;
                    axi_qos_next = current_s_axi_arqos;
                    axi_auser_next = current_s_axi_aruser;
                    s_axi_arready_next[s_select] = 1'b1;
                end else  begin
                    // 写事务
                    axi_addr_next = current_s_axi_awaddr;
                    axi_prot_next = current_s_axi_awprot;
                    axi_id_next = current_s_axi_awid;
                    axi_addr_next = current_s_axi_awaddr;
                    axi_len_next = current_s_axi_awlen;
                    axi_size_next = current_s_axi_awsize;
                    axi_burst_next = current_s_axi_awburst;
                    axi_lock_next = current_s_axi_awlock;
                    axi_cache_next = current_s_axi_awcache;
                    axi_prot_next = current_s_axi_awprot;
                    axi_qos_next = current_s_axi_awqos;
                    axi_auser_next = current_s_axi_awuser;
                    s_axi_awready_next[s_select] = 1'b1;
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
                    if (M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32] && (!M_SECURE[i] || !axi_prot_reg[1]) && ((read ? M_CONNECT_READ : M_CONNECT_WRITE) & (1 << (s_select+i*S_COUNT))) && (axi_addr_reg >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32]) == (M_BASE_ADDR_INT[(i*M_REGIONS+j)*ADDR_WIDTH +: ADDR_WIDTH] >> M_ADDR_WIDTH[(i*M_REGIONS+j)*32 +: 32])) begin
                        m_select_next = i;
                        axi_region_next = j;
                        match = 1'b1;
                    end
                end
            end

            if (match) begin
                if (read) begin
                    // 读事务
                    m_axi_rready_next[m_select_reg] = s_axi_rready_int_early;
                    state_next = STATE_READ;
                end else begin
                    // 写事务
                    s_axi_wready_next[s_select] = m_axi_wready_int_early;
                    state_next = STATE_WRITE;
                end
            end else begin
                // 未命中任何区域：返回 DECERR
                if (read) begin
                    // 读事务
                    state_next = STATE_READ_DROP;
                end else begin
                    // 写事务
                    axi_bresp_next = 2'b11;
                    s_axi_wready_next[s_select] = 1'b1;
                    state_next = STATE_WRITE_DROP;
                end
            end
        end
        STATE_WRITE: begin
            // 写态：缓存并转发写数据
            s_axi_wready_next[s_select] = m_axi_wready_int_early;

            if (axi_addr_valid_reg) begin
                m_axi_awvalid_next[m_select_reg] = 1'b1;
            end
            axi_addr_valid_next = 1'b0;

            if (current_s_axi_wready && current_s_axi_wvalid) begin
                m_axi_wdata_int = current_s_axi_wdata;
                m_axi_wstrb_int = current_s_axi_wstrb;
                m_axi_wlast_int = current_s_axi_wlast;
                m_axi_wuser_int = current_s_axi_wuser;
                m_axi_wvalid_int = 1'b1;

                if (current_s_axi_wlast) begin
                    s_axi_wready_next[s_select] = 1'b0;
                    m_axi_bready_next[m_select_reg] = 1'b1;
                    state_next = STATE_WRITE_RESP;
                end else begin
                    state_next = STATE_WRITE;
                end
            end else begin
                state_next = STATE_WRITE;
            end
        end
        STATE_WRITE_RESP: begin
            // 写响应态：缓存并转发写响应
            m_axi_bready_next[m_select_reg] = 1'b1;

            if (current_m_axi_bready && current_m_axi_bvalid) begin
                m_axi_bready_next[m_select_reg] = 1'b0;
                axi_bresp_next = current_m_axi_bresp;
                s_axi_bvalid_next[s_select] = 1'b1;
                state_next = STATE_WAIT_IDLE;
            end else begin
                state_next = STATE_WRITE_RESP;
            end
        end
        STATE_WRITE_DROP: begin
            // 写丢弃态：吞吐并丢弃写数据
            s_axi_wready_next[s_select] = 1'b1;

            axi_addr_valid_next = 1'b0;

            if (current_s_axi_wready && current_s_axi_wvalid && current_s_axi_wlast) begin
                s_axi_wready_next[s_select] = 1'b0;
                s_axi_bvalid_next[s_select] = 1'b1;
                state_next = STATE_WAIT_IDLE;
            end else begin
                state_next = STATE_WRITE_DROP;
            end
        end
        STATE_READ: begin
            // 读态：缓存并转发读响应
            m_axi_rready_next[m_select_reg] = s_axi_rready_int_early;

            if (axi_addr_valid_reg) begin
                m_axi_arvalid_next[m_select_reg] = 1'b1;
            end
            axi_addr_valid_next = 1'b0;

            if (current_m_axi_rready && current_m_axi_rvalid) begin
                s_axi_rid_int = axi_id_reg;
                s_axi_rdata_int = current_m_axi_rdata;
                s_axi_rresp_int = current_m_axi_rresp;
                s_axi_rlast_int = current_m_axi_rlast;
                s_axi_ruser_int = current_m_axi_ruser;
                s_axi_rvalid_int = 1'b1;

                if (current_m_axi_rlast) begin
                    m_axi_rready_next[m_select_reg] = 1'b0;
                    state_next = STATE_WAIT_IDLE;
                end else begin
                    state_next = STATE_READ;
                end
            end else begin
                state_next = STATE_READ;
            end
        end
        STATE_READ_DROP: begin
            // 读丢弃态：生成 DECERR 读响应

            s_axi_rid_int = axi_id_reg;
            s_axi_rdata_int = {DATA_WIDTH{1'b0}};
            s_axi_rresp_int = 2'b11;
            s_axi_rlast_int = axi_len_reg == 0;
            s_axi_ruser_int = {RUSER_WIDTH{1'b0}};
            s_axi_rvalid_int = 1'b1;

            if (s_axi_rready_int_reg) begin
                axi_len_next = axi_len_reg - 1;
                if (axi_len_reg == 0) begin
                    state_next = STATE_WAIT_IDLE;
                end else begin
                    state_next = STATE_READ_DROP;
                end
            end else begin
                state_next = STATE_READ_DROP;
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

        s_axi_awready_reg <= 0;
        s_axi_wready_reg <= 0;
        s_axi_bvalid_reg <= 0;
        s_axi_arready_reg <= 0;

        m_axi_awvalid_reg <= 0;
        m_axi_bready_reg <= 0;
        m_axi_arvalid_reg <= 0;
        m_axi_rready_reg <= 0;
    end else begin
        state_reg <= state_next;

        s_axi_awready_reg <= s_axi_awready_next;
        s_axi_wready_reg <= s_axi_wready_next;
        s_axi_bvalid_reg <= s_axi_bvalid_next;
        s_axi_arready_reg <= s_axi_arready_next;

        m_axi_awvalid_reg <= m_axi_awvalid_next;
        m_axi_bready_reg <= m_axi_bready_next;
        m_axi_arvalid_reg <= m_axi_arvalid_next;
        m_axi_rready_reg <= m_axi_rready_next;
    end

    m_select_reg <= m_select_next;
    axi_id_reg <= axi_id_next;
    axi_addr_reg <= axi_addr_next;
    axi_addr_valid_reg <= axi_addr_valid_next;
    axi_len_reg <= axi_len_next;
    axi_size_reg <= axi_size_next;
    axi_burst_reg <= axi_burst_next;
    axi_lock_reg <= axi_lock_next;
    axi_cache_reg <= axi_cache_next;
    axi_prot_reg <= axi_prot_next;
    axi_qos_reg <= axi_qos_next;
    axi_region_reg <= axi_region_next;
    axi_auser_reg <= axi_auser_next;
    axi_bresp_reg <= axi_bresp_next;
    axi_buser_reg <= axi_buser_next;
end

// 输出数据通路逻辑（R 通道）
reg [ID_WIDTH-1:0]    s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // R 输出寄存器中的 RID。
reg [DATA_WIDTH-1:0]  s_axi_rdata_reg  = {DATA_WIDTH{1'b0}}; // R 输出寄存器中的 RDATA。
reg [1:0]             s_axi_rresp_reg  = 2'd0; // R 输出寄存器中的 RRESP。
reg                   s_axi_rlast_reg  = 1'b0; // R 输出寄存器中的 RLAST。
reg [RUSER_WIDTH-1:0] s_axi_ruser_reg  = 1'b0; // R 输出寄存器中的 RUSER。
reg [S_COUNT-1:0]     s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // 各 S 口 RVALID 输出寄存器。

reg [ID_WIDTH-1:0]    temp_s_axi_rid_reg    = {ID_WIDTH{1'b0}}; // R 临时缓冲中的 RID。
reg [DATA_WIDTH-1:0]  temp_s_axi_rdata_reg  = {DATA_WIDTH{1'b0}}; // R 临时缓冲中的 RDATA。
reg [1:0]             temp_s_axi_rresp_reg  = 2'd0; // R 临时缓冲中的 RRESP。
reg                   temp_s_axi_rlast_reg  = 1'b0; // R 临时缓冲中的 RLAST。
reg [RUSER_WIDTH-1:0] temp_s_axi_ruser_reg  = 1'b0; // R 临时缓冲中的 RUSER。
reg                   temp_s_axi_rvalid_reg = 1'b0, temp_s_axi_rvalid_next; // R 临时缓冲有效标志。

// 数据通路控制
reg store_axi_r_int_to_output; // 将内部输入直接写入主输出寄存器。
reg store_axi_r_int_to_temp; // 将内部输入写入临时缓冲。
reg store_axi_r_temp_to_output; // 将临时缓冲回填到主输出寄存器。

assign s_axi_rid = {S_COUNT{s_axi_rid_reg}};
assign s_axi_rdata = {S_COUNT{s_axi_rdata_reg}};
assign s_axi_rresp = {S_COUNT{s_axi_rresp_reg}};
assign s_axi_rlast = {S_COUNT{s_axi_rlast_reg}};
assign s_axi_ruser = {S_COUNT{RUSER_ENABLE ? s_axi_ruser_reg : {RUSER_WIDTH{1'b0}}}};
assign s_axi_rvalid = s_axi_rvalid_reg;

// 若输出就绪，或下一拍临时寄存器不会被写满（输出寄存器空/无输入），则下一拍拉高 ready
assign s_axi_rready_int_early = current_s_axi_rready | (~temp_s_axi_rvalid_reg & (~current_s_axi_rvalid | ~s_axi_rvalid_int));

always @* begin
    // 将接收端 ready 状态传递到发送端
    s_axi_rvalid_next = s_axi_rvalid_reg;
    temp_s_axi_rvalid_next = temp_s_axi_rvalid_reg;

    store_axi_r_int_to_output = 1'b0;
    store_axi_r_int_to_temp = 1'b0;
    store_axi_r_temp_to_output = 1'b0;

    if (s_axi_rready_int_reg) begin
        // 输入端当前就绪
        if (current_s_axi_rready | ~current_s_axi_rvalid) begin
            // 输出端就绪或当前无效，直接把数据送到输出寄存器
            s_axi_rvalid_next[s_select] = s_axi_rvalid_int;
            store_axi_r_int_to_output = 1'b1;
        end else begin
            // 输出端未就绪，将输入暂存到临时寄存器
            temp_s_axi_rvalid_next = s_axi_rvalid_int;
            store_axi_r_int_to_temp = 1'b1;
        end
    end else if (current_s_axi_rready) begin
        // 输入端未就绪，但输出端就绪
        s_axi_rvalid_next[s_select] = temp_s_axi_rvalid_reg;
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

// 输出数据通路逻辑（W 通道）
reg [DATA_WIDTH-1:0]  m_axi_wdata_reg  = {DATA_WIDTH{1'b0}}; // W 输出寄存器中的 WDATA。
reg [STRB_WIDTH-1:0]  m_axi_wstrb_reg  = {STRB_WIDTH{1'b0}}; // W 输出寄存器中的 WSTRB。
reg                   m_axi_wlast_reg  = 1'b0; // W 输出寄存器中的 WLAST。
reg [WUSER_WIDTH-1:0] m_axi_wuser_reg  = 1'b0; // W 输出寄存器中的 WUSER。
reg [M_COUNT-1:0]     m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // 各 M 口 WVALID 输出寄存器。

reg [DATA_WIDTH-1:0]  temp_m_axi_wdata_reg  = {DATA_WIDTH{1'b0}}; // W 临时缓冲中的 WDATA。
reg [STRB_WIDTH-1:0]  temp_m_axi_wstrb_reg  = {STRB_WIDTH{1'b0}}; // W 临时缓冲中的 WSTRB。
reg                   temp_m_axi_wlast_reg  = 1'b0; // W 临时缓冲中的 WLAST。
reg [WUSER_WIDTH-1:0] temp_m_axi_wuser_reg  = 1'b0; // W 临时缓冲中的 WUSER。
reg                   temp_m_axi_wvalid_reg = 1'b0, temp_m_axi_wvalid_next; // W 临时缓冲有效标志。

// 数据通路控制
reg store_axi_w_int_to_output; // 将内部输入直接写入主输出寄存器。
reg store_axi_w_int_to_temp; // 将内部输入写入临时缓冲。
reg store_axi_w_temp_to_output; // 将临时缓冲回填到主输出寄存器。

assign m_axi_wdata = {M_COUNT{m_axi_wdata_reg}};
assign m_axi_wstrb = {M_COUNT{m_axi_wstrb_reg}};
assign m_axi_wlast = {M_COUNT{m_axi_wlast_reg}};
assign m_axi_wuser = {M_COUNT{WUSER_ENABLE ? m_axi_wuser_reg : {WUSER_WIDTH{1'b0}}}};
assign m_axi_wvalid = m_axi_wvalid_reg;

// 若输出就绪，或下一拍临时寄存器不会被写满（输出寄存器空/无输入），则下一拍拉高 ready
assign m_axi_wready_int_early = current_m_axi_wready | (~temp_m_axi_wvalid_reg & (~current_m_axi_wvalid | ~m_axi_wvalid_int));

always @* begin
    // 将接收端 ready 状态传递到发送端
    m_axi_wvalid_next = m_axi_wvalid_reg;
    temp_m_axi_wvalid_next = temp_m_axi_wvalid_reg;

    store_axi_w_int_to_output = 1'b0;
    store_axi_w_int_to_temp = 1'b0;
    store_axi_w_temp_to_output = 1'b0;

    if (m_axi_wready_int_reg) begin
        // 输入端当前就绪
        if (current_m_axi_wready | ~current_m_axi_wvalid) begin
            // 输出端就绪或当前无效，直接把数据送到输出寄存器
            m_axi_wvalid_next[m_select_reg] = m_axi_wvalid_int;
            store_axi_w_int_to_output = 1'b1;
        end else begin
            // 输出端未就绪，将输入暂存到临时寄存器
            temp_m_axi_wvalid_next = m_axi_wvalid_int;
            store_axi_w_int_to_temp = 1'b1;
        end
    end else if (current_m_axi_wready) begin
        // 输入端未就绪，但输出端就绪
        m_axi_wvalid_next[m_select_reg] = temp_m_axi_wvalid_reg;
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
