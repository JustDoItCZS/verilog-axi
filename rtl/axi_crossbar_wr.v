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
 * AXI4 交叉开关（写通道）
 *
 * 模块目录
 * 1) 每个 S 口先经 `axi_crossbar_addr` 做写地址解码和准入控制。
 * 2) 每个 M 口独立做 AW 仲裁和 W 路由，支持并行写通路。
 * 3) B 响应按 BID 高位携带的源口索引回送到对应 S 口，并支持 DECERR 注入。
 */
module axi_crossbar_wr #
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
    // 输入 ID 位宽（来自 AXI 主设备）
    parameter S_ID_WIDTH = 8,
    // 输出 ID 位宽（发往 AXI 从设备）
    // 包含响应路由所需附加位
    parameter M_ID_WIDTH = S_ID_WIDTH+$clog2(S_COUNT),
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
    // 每个从接口可并发唯一 ID 数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_THREADS = {S_COUNT{32'd2}},
    // 每个从接口可并发事务数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_ACCEPT = {S_COUNT{32'd16}},
    // 每个主接口地址区域数量
    parameter M_REGIONS = 1,
    // 主接口基地址表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 ADDR_WIDTH 位字段
    // 置 0 时按 M_ADDR_WIDTH 自动生成默认地址映射
    parameter M_BASE_ADDR = 0,
    // 主接口地址宽度表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 32 位字段
    parameter M_ADDR_WIDTH = {M_COUNT{{M_REGIONS{32'd24}}}},
    // 接口间写通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT = {M_COUNT{{S_COUNT{1'b1}}}},
    // 每个主接口可并发事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd4}},
    // 安全主端口配置（基于 awprot/arprot 拒绝访问）
    // M_COUNT 位
    parameter M_SECURE = {M_COUNT{1'b0}},
    // 从接口 AW 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_AW_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 W 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_W_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 B 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_B_REG_TYPE = {S_COUNT{2'd1}},
    // 主接口 AW 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_AW_REG_TYPE = {M_COUNT{2'd1}},
    // 主接口 W 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_W_REG_TYPE = {M_COUNT{2'd2}},
    // 主接口 B 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_B_REG_TYPE = {M_COUNT{2'd0}}
)
(
    input  wire                             clk, // 写 crossbar 时钟。
    input  wire                             rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_awid, // 所有 S 口拼接的 AWID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axi_awaddr, // 所有 S 口拼接的 AWADDR。
    input  wire [S_COUNT*8-1:0]             s_axi_awlen, // 所有 S 口拼接的 AWLEN。
    input  wire [S_COUNT*3-1:0]             s_axi_awsize, // 所有 S 口拼接的 AWSIZE。
    input  wire [S_COUNT*2-1:0]             s_axi_awburst, // 所有 S 口拼接的 AWBURST。
    input  wire [S_COUNT-1:0]               s_axi_awlock, // 所有 S 口 AWLOCK。
    input  wire [S_COUNT*4-1:0]             s_axi_awcache, // 所有 S 口拼接的 AWCACHE。
    input  wire [S_COUNT*3-1:0]             s_axi_awprot, // 所有 S 口拼接的 AWPROT。
    input  wire [S_COUNT*4-1:0]             s_axi_awqos, // 所有 S 口拼接的 AWQOS。
    input  wire [S_COUNT*AWUSER_WIDTH-1:0]  s_axi_awuser, // 所有 S 口拼接的 AWUSER。
    input  wire [S_COUNT-1:0]               s_axi_awvalid, // 所有 S 口 AWVALID。
    output wire [S_COUNT-1:0]               s_axi_awready, // 所有 S 口 AWREADY。
    input  wire [S_COUNT*DATA_WIDTH-1:0]    s_axi_wdata, // 所有 S 口拼接的 WDATA。
    input  wire [S_COUNT*STRB_WIDTH-1:0]    s_axi_wstrb, // 所有 S 口拼接的 WSTRB。
    input  wire [S_COUNT-1:0]               s_axi_wlast, // 所有 S 口 WLAST。
    input  wire [S_COUNT*WUSER_WIDTH-1:0]   s_axi_wuser, // 所有 S 口拼接的 WUSER。
    input  wire [S_COUNT-1:0]               s_axi_wvalid, // 所有 S 口 WVALID。
    output wire [S_COUNT-1:0]               s_axi_wready, // 所有 S 口 WREADY。
    output wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_bid, // 所有 S 口拼接的 BID。
    output wire [S_COUNT*2-1:0]             s_axi_bresp, // 所有 S 口拼接的 BRESP。
    output wire [S_COUNT*BUSER_WIDTH-1:0]   s_axi_buser, // 所有 S 口拼接的 BUSER。
    output wire [S_COUNT-1:0]               s_axi_bvalid, // 所有 S 口 BVALID。
    input  wire [S_COUNT-1:0]               s_axi_bready, // 所有 S 口 BREADY。

    /*
     * AXI 主接口
     */
    output wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_awid, // 所有 M 口拼接的 AWID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axi_awaddr, // 所有 M 口拼接的 AWADDR 输出。
    output wire [M_COUNT*8-1:0]             m_axi_awlen, // 所有 M 口拼接的 AWLEN 输出。
    output wire [M_COUNT*3-1:0]             m_axi_awsize, // 所有 M 口拼接的 AWSIZE 输出。
    output wire [M_COUNT*2-1:0]             m_axi_awburst, // 所有 M 口拼接的 AWBURST 输出。
    output wire [M_COUNT-1:0]               m_axi_awlock, // 所有 M 口 AWLOCK 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awcache, // 所有 M 口拼接的 AWCACHE 输出。
    output wire [M_COUNT*3-1:0]             m_axi_awprot, // 所有 M 口拼接的 AWPROT 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awqos, // 所有 M 口拼接的 AWQOS 输出。
    output wire [M_COUNT*4-1:0]             m_axi_awregion, // 所有 M 口拼接的 AWREGION 输出。
    output wire [M_COUNT*AWUSER_WIDTH-1:0]  m_axi_awuser, // 所有 M 口拼接的 AWUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_awvalid, // 所有 M 口 AWVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_awready, // 所有 M 口 AWREADY 输入。
    output wire [M_COUNT*DATA_WIDTH-1:0]    m_axi_wdata, // 所有 M 口拼接的 WDATA 输出。
    output wire [M_COUNT*STRB_WIDTH-1:0]    m_axi_wstrb, // 所有 M 口拼接的 WSTRB 输出。
    output wire [M_COUNT-1:0]               m_axi_wlast, // 所有 M 口 WLAST 输出。
    output wire [M_COUNT*WUSER_WIDTH-1:0]   m_axi_wuser, // 所有 M 口拼接的 WUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_wvalid, // 所有 M 口 WVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_wready, // 所有 M 口 WREADY 输入。
    input  wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_bid, // 所有 M 口拼接的 BID 输入。
    input  wire [M_COUNT*2-1:0]             m_axi_bresp, // 所有 M 口拼接的 BRESP 输入。
    input  wire [M_COUNT*BUSER_WIDTH-1:0]   m_axi_buser, // 所有 M 口拼接的 BUSER 输入。
    input  wire [M_COUNT-1:0]               m_axi_bvalid, // 所有 M 口 BVALID 输入。
    output wire [M_COUNT-1:0]               m_axi_bready // 所有 M 口 BREADY 输出。
);

parameter CL_S_COUNT = $clog2(S_COUNT);
parameter CL_M_COUNT = $clog2(M_COUNT);
parameter M_COUNT_P1 = M_COUNT+1;
parameter CL_M_COUNT_P1 = $clog2(M_COUNT_P1);

integer i; // 配置检查循环索引。

// 配置合法性检查
initial begin
    if (M_ID_WIDTH < S_ID_WIDTH+$clog2(S_COUNT)) begin
        $error("Error: M_ID_WIDTH must be at least $clog2(S_COUNT) larger than S_ID_WIDTH (instance %m)");
        $finish;
    end

    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && (M_ADDR_WIDTH[i*32 +: 32] < 12 || M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH)) begin
            $error("Error: value out of range (instance %m)");
            $finish;
        end
    end
end

wire [S_COUNT*S_ID_WIDTH-1:0]    int_s_axi_awid; // S 侧寄存后内部 AWID 总线。
wire [S_COUNT*ADDR_WIDTH-1:0]    int_s_axi_awaddr; // S 侧寄存后内部 AWADDR 总线。
wire [S_COUNT*8-1:0]             int_s_axi_awlen; // S 侧寄存后内部 AWLEN 总线。
wire [S_COUNT*3-1:0]             int_s_axi_awsize; // S 侧寄存后内部 AWSIZE 总线。
wire [S_COUNT*2-1:0]             int_s_axi_awburst; // S 侧寄存后内部 AWBURST 总线。
wire [S_COUNT-1:0]               int_s_axi_awlock; // S 侧寄存后内部 AWLOCK。
wire [S_COUNT*4-1:0]             int_s_axi_awcache; // S 侧寄存后内部 AWCACHE 总线。
wire [S_COUNT*3-1:0]             int_s_axi_awprot; // S 侧寄存后内部 AWPROT 总线。
wire [S_COUNT*4-1:0]             int_s_axi_awqos; // S 侧寄存后内部 AWQOS 总线。
wire [S_COUNT*4-1:0]             int_s_axi_awregion; // 地址解码后写入的 AWREGION 总线。
wire [S_COUNT*AWUSER_WIDTH-1:0]  int_s_axi_awuser; // S 侧寄存后内部 AWUSER 总线。
wire [S_COUNT-1:0]               int_s_axi_awvalid; // 各 S 口内部 AWVALID。
wire [S_COUNT-1:0]               int_s_axi_awready; // 各 S 口内部 AWREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axi_awvalid; // S->M 的 AW 路由有效矩阵(按源口展开)。
wire [M_COUNT*S_COUNT-1:0]       int_axi_awready; // M->S 的 AW 路由 ready 矩阵(按目标口展开)。

wire [S_COUNT*DATA_WIDTH-1:0]    int_s_axi_wdata; // S 侧寄存后内部 WDATA 总线。
wire [S_COUNT*STRB_WIDTH-1:0]    int_s_axi_wstrb; // S 侧寄存后内部 WSTRB 总线。
wire [S_COUNT-1:0]               int_s_axi_wlast; // S 侧寄存后内部 WLAST。
wire [S_COUNT*WUSER_WIDTH-1:0]   int_s_axi_wuser; // S 侧寄存后内部 WUSER 总线。
wire [S_COUNT-1:0]               int_s_axi_wvalid; // 各 S 口内部 WVALID。
wire [S_COUNT-1:0]               int_s_axi_wready; // 各 S 口内部 WREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axi_wvalid; // S->M 的 W 路由有效矩阵。
wire [M_COUNT*S_COUNT-1:0]       int_axi_wready; // M->S 的 W 路由 ready 矩阵。

wire [M_COUNT*M_ID_WIDTH-1:0]    int_m_axi_bid; // M 侧寄存后内部 BID 总线。
wire [M_COUNT*2-1:0]             int_m_axi_bresp; // M 侧寄存后内部 BRESP 总线。
wire [M_COUNT*BUSER_WIDTH-1:0]   int_m_axi_buser; // M 侧寄存后内部 BUSER 总线。
wire [M_COUNT-1:0]               int_m_axi_bvalid; // 各 M 口内部 BVALID。
wire [M_COUNT-1:0]               int_m_axi_bready; // 各 M 口内部 BREADY。

wire [M_COUNT*S_COUNT-1:0]       int_axi_bvalid; // M->S 的 B 路由有效矩阵。
wire [S_COUNT*M_COUNT-1:0]       int_axi_bready; // S->M 的 B 路由 ready 矩阵。

generate

    genvar m, n;

    for (m = 0; m < S_COUNT; m = m + 1) begin : s_ifaces
        // 地址解码与接纳控制
        wire [CL_M_COUNT-1:0] a_select; // 当前 S 口地址解码得到的目标 M 口。

        wire m_axi_avalid; // 地址输出命令有效。
        wire m_axi_aready; // 地址输出命令被目标侧接受。

        wire [CL_M_COUNT-1:0] m_wc_select; // 写数据路由命令中的目标 M 口。
        wire m_wc_decerr; // 写命令是否为解码错误。
        wire m_wc_valid; // 写路由命令有效。
        wire m_wc_ready; // 写路由命令被本地 W 路由逻辑接收。

        wire m_rc_decerr; // 返回命令是否为解码错误。
        wire m_rc_valid; // 返回命令有效。
        wire m_rc_ready; // 返回命令被本地 B 处理逻辑接收。

        wire [S_ID_WIDTH-1:0] s_cpl_id; // 给准入控制器的完成事务 ID。
        wire s_cpl_valid; // 完成事务事件有效。

        axi_crossbar_addr #(
            .S(m),
            .S_COUNT(S_COUNT),
            .M_COUNT(M_COUNT),
            .ADDR_WIDTH(ADDR_WIDTH),
            .ID_WIDTH(S_ID_WIDTH),
            .S_THREADS(S_THREADS[m*32 +: 32]),
            .S_ACCEPT(S_ACCEPT[m*32 +: 32]),
            .M_REGIONS(M_REGIONS),
            .M_BASE_ADDR(M_BASE_ADDR),
            .M_ADDR_WIDTH(M_ADDR_WIDTH),
            .M_CONNECT(M_CONNECT),
            .M_SECURE(M_SECURE),
            .WC_OUTPUT(1)
        )
        addr_inst (
            .clk(clk),
            .rst(rst),

            /*
             * 地址输入
             */
            .s_axi_aid(int_s_axi_awid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_aaddr(int_s_axi_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axi_aprot(int_s_axi_awprot[m*3 +: 3]),
            .s_axi_aqos(int_s_axi_awqos[m*4 +: 4]),
            .s_axi_avalid(int_s_axi_awvalid[m]),
            .s_axi_aready(int_s_axi_awready[m]),

            /*
             * 地址输出
             */
            .m_axi_aregion(int_s_axi_awregion[m*4 +: 4]),
            .m_select(a_select),
            .m_axi_avalid(m_axi_avalid),
            .m_axi_aready(m_axi_aready),

            /*
             * 写命令输出
             */
            .m_wc_select(m_wc_select),
            .m_wc_decerr(m_wc_decerr),
            .m_wc_valid(m_wc_valid),
            .m_wc_ready(m_wc_ready),

            /*
             * 响应命令输出
             */
            .m_rc_decerr(m_rc_decerr),
            .m_rc_valid(m_rc_valid),
            .m_rc_ready(m_rc_ready),

            /*
             * 完成通知输入
             */
            .s_cpl_id(s_cpl_id),
            .s_cpl_valid(s_cpl_valid)
        );

        assign int_axi_awvalid[m*M_COUNT +: M_COUNT] = m_axi_avalid << a_select;
        assign m_axi_aready = int_axi_awready[a_select*S_COUNT+m];

        // 写命令处理
        reg [CL_M_COUNT-1:0] w_select_reg = 0, w_select_next; // 当前 W 数据应发往的目标 M 口。
        reg w_drop_reg = 1'b0, w_drop_next; // 当前写事务是否需丢弃数据(DECERR 路径)。
        reg w_select_valid_reg = 1'b0, w_select_valid_next; // 当前 W 路由选择是否有效。

        assign m_wc_ready = !w_select_valid_reg;

        always @* begin
            w_select_next = w_select_reg;
            w_drop_next = w_drop_reg && !(int_s_axi_wvalid[m] && int_s_axi_wready[m] && int_s_axi_wlast[m]);
            w_select_valid_next = w_select_valid_reg && !(int_s_axi_wvalid[m] && int_s_axi_wready[m] && int_s_axi_wlast[m]);

            if (m_wc_valid && !w_select_valid_reg) begin
                w_select_next = m_wc_select;
                w_drop_next = m_wc_decerr;
                w_select_valid_next = m_wc_valid;
            end
        end

        always @(posedge clk) begin
            if (rst) begin
                w_select_valid_reg <= 1'b0;
            end else begin
                w_select_valid_reg <= w_select_valid_next;
            end

            w_select_reg <= w_select_next;
            w_drop_reg <= w_drop_next;
        end

        // 写数据转发
        assign int_axi_wvalid[m*M_COUNT +: M_COUNT] = (int_s_axi_wvalid[m] && w_select_valid_reg && !w_drop_reg) << w_select_reg;
        assign int_s_axi_wready[m] = int_axi_wready[w_select_reg*S_COUNT+m] || w_drop_reg;

        // 解码错误处理
        reg [S_ID_WIDTH-1:0]  decerr_m_axi_bid_reg = {S_ID_WIDTH{1'b0}}, decerr_m_axi_bid_next; // DECERR 虚拟响应的 BID。
        reg                   decerr_m_axi_bvalid_reg = 1'b0, decerr_m_axi_bvalid_next; // DECERR 虚拟响应有效位。
        wire                  decerr_m_axi_bready; // DECERR 虚拟响应被消费。

        assign m_rc_ready = !decerr_m_axi_bvalid_reg;

        always @* begin
            decerr_m_axi_bid_next = decerr_m_axi_bid_reg;
            decerr_m_axi_bvalid_next = decerr_m_axi_bvalid_reg;

            if (decerr_m_axi_bvalid_reg) begin
                if (decerr_m_axi_bready) begin
                    decerr_m_axi_bvalid_next = 1'b0;
                end
            end else if (m_rc_valid && m_rc_ready) begin
                decerr_m_axi_bid_next = int_s_axi_awid[m*S_ID_WIDTH +: S_ID_WIDTH];
                decerr_m_axi_bvalid_next = 1'b1;
            end
        end

        always @(posedge clk) begin
            if (rst) begin
                decerr_m_axi_bvalid_reg <= 1'b0;
            end else begin
                decerr_m_axi_bvalid_reg <= decerr_m_axi_bvalid_next;
            end

            decerr_m_axi_bid_reg <= decerr_m_axi_bid_next;
        end

        // 写响应仲裁
        wire [M_COUNT_P1-1:0] b_request; // B 仲裁请求(含一个 DECERR 虚拟端口)。
        wire [M_COUNT_P1-1:0] b_acknowledge; // B 仲裁完成应答。
        wire [M_COUNT_P1-1:0] b_grant; // B 仲裁授权 one-hot。
        wire b_grant_valid; // B 仲裁是否有有效授权。
        wire [CL_M_COUNT_P1-1:0] b_grant_encoded; // B 仲裁授权编码值。

        arbiter #(
            .PORTS(M_COUNT_P1),
            .ARB_TYPE_ROUND_ROBIN(1),
            .ARB_BLOCK(1),
            .ARB_BLOCK_ACK(1),
            .ARB_LSB_HIGH_PRIORITY(1)
        )
        b_arb_inst (
            .clk(clk),
            .rst(rst),
            .request(b_request),
            .acknowledge(b_acknowledge),
            .grant(b_grant),
            .grant_valid(b_grant_valid),
            .grant_encoded(b_grant_encoded)
        );

        // 写响应复用
        wire [S_ID_WIDTH-1:0]  m_axi_bid_mux    = {decerr_m_axi_bid_reg, int_m_axi_bid} >> b_grant_encoded*M_ID_WIDTH; // 复用后的返回 BID(剥离路由位后给 S 侧)。
        wire [1:0]             m_axi_bresp_mux  = {2'b11, int_m_axi_bresp} >> b_grant_encoded*2; // 复用后的 BRESP(DECERR 或真实响应)。
        wire [BUSER_WIDTH-1:0] m_axi_buser_mux  = {{BUSER_WIDTH{1'b0}}, int_m_axi_buser} >> b_grant_encoded*BUSER_WIDTH; // 复用后的 BUSER。
        wire                   m_axi_bvalid_mux = ({decerr_m_axi_bvalid_reg, int_m_axi_bvalid} >> b_grant_encoded) & b_grant_valid; // 复用后的 BVALID。
        wire                   m_axi_bready_mux; // S 侧寄存器返回的 BREADY。

        assign int_axi_bready[m*M_COUNT +: M_COUNT] = (b_grant_valid && m_axi_bready_mux) << b_grant_encoded;
        assign decerr_m_axi_bready = (b_grant_valid && m_axi_bready_mux) && (b_grant_encoded == M_COUNT_P1-1);

        for (n = 0; n < M_COUNT; n = n + 1) begin
            assign b_request[n] = int_axi_bvalid[n*S_COUNT+m] && !b_grant[n];
            assign b_acknowledge[n] = b_grant[n] && int_axi_bvalid[n*S_COUNT+m] && m_axi_bready_mux;
        end

        assign b_request[M_COUNT_P1-1] = decerr_m_axi_bvalid_reg && !b_grant[M_COUNT_P1-1];
        assign b_acknowledge[M_COUNT_P1-1] = b_grant[M_COUNT_P1-1] && decerr_m_axi_bvalid_reg && m_axi_bready_mux;

        assign s_cpl_id = m_axi_bid_mux;
        assign s_cpl_valid = m_axi_bvalid_mux && m_axi_bready_mux;

        // S 侧寄存器切片
        axi_register_wr #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .ID_WIDTH(S_ID_WIDTH),
            .AWUSER_ENABLE(AWUSER_ENABLE),
            .AWUSER_WIDTH(AWUSER_WIDTH),
            .WUSER_ENABLE(WUSER_ENABLE),
            .WUSER_WIDTH(WUSER_WIDTH),
            .BUSER_ENABLE(BUSER_ENABLE),
            .BUSER_WIDTH(BUSER_WIDTH),
            .AW_REG_TYPE(S_AW_REG_TYPE[m*2 +: 2]),
            .W_REG_TYPE(S_W_REG_TYPE[m*2 +: 2]),
            .B_REG_TYPE(S_B_REG_TYPE[m*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axi_awid(s_axi_awid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_awaddr(s_axi_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axi_awlen(s_axi_awlen[m*8 +: 8]),
            .s_axi_awsize(s_axi_awsize[m*3 +: 3]),
            .s_axi_awburst(s_axi_awburst[m*2 +: 2]),
            .s_axi_awlock(s_axi_awlock[m]),
            .s_axi_awcache(s_axi_awcache[m*4 +: 4]),
            .s_axi_awprot(s_axi_awprot[m*3 +: 3]),
            .s_axi_awqos(s_axi_awqos[m*4 +: 4]),
            .s_axi_awregion(4'd0),
            .s_axi_awuser(s_axi_awuser[m*AWUSER_WIDTH +: AWUSER_WIDTH]),
            .s_axi_awvalid(s_axi_awvalid[m]),
            .s_axi_awready(s_axi_awready[m]),
            .s_axi_wdata(s_axi_wdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .s_axi_wstrb(s_axi_wstrb[m*STRB_WIDTH +: STRB_WIDTH]),
            .s_axi_wlast(s_axi_wlast[m]),
            .s_axi_wuser(s_axi_wuser[m*WUSER_WIDTH +: WUSER_WIDTH]),
            .s_axi_wvalid(s_axi_wvalid[m]),
            .s_axi_wready(s_axi_wready[m]),
            .s_axi_bid(s_axi_bid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_bresp(s_axi_bresp[m*2 +: 2]),
            .s_axi_buser(s_axi_buser[m*BUSER_WIDTH +: BUSER_WIDTH]),
            .s_axi_bvalid(s_axi_bvalid[m]),
            .s_axi_bready(s_axi_bready[m]),
            .m_axi_awid(int_s_axi_awid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .m_axi_awaddr(int_s_axi_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axi_awlen(int_s_axi_awlen[m*8 +: 8]),
            .m_axi_awsize(int_s_axi_awsize[m*3 +: 3]),
            .m_axi_awburst(int_s_axi_awburst[m*2 +: 2]),
            .m_axi_awlock(int_s_axi_awlock[m]),
            .m_axi_awcache(int_s_axi_awcache[m*4 +: 4]),
            .m_axi_awprot(int_s_axi_awprot[m*3 +: 3]),
            .m_axi_awqos(int_s_axi_awqos[m*4 +: 4]),
            .m_axi_awregion(),
            .m_axi_awuser(int_s_axi_awuser[m*AWUSER_WIDTH +: AWUSER_WIDTH]),
            .m_axi_awvalid(int_s_axi_awvalid[m]),
            .m_axi_awready(int_s_axi_awready[m]),
            .m_axi_wdata(int_s_axi_wdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .m_axi_wstrb(int_s_axi_wstrb[m*STRB_WIDTH +: STRB_WIDTH]),
            .m_axi_wlast(int_s_axi_wlast[m]),
            .m_axi_wuser(int_s_axi_wuser[m*WUSER_WIDTH +: WUSER_WIDTH]),
            .m_axi_wvalid(int_s_axi_wvalid[m]),
            .m_axi_wready(int_s_axi_wready[m]),
            .m_axi_bid(m_axi_bid_mux),
            .m_axi_bresp(m_axi_bresp_mux),
            .m_axi_buser(m_axi_buser_mux),
            .m_axi_bvalid(m_axi_bvalid_mux),
            .m_axi_bready(m_axi_bready_mux)
        );
    end // 从端接口循环

    for (n = 0; n < M_COUNT; n = n + 1) begin : m_ifaces
        // 在途事务计数
        wire trans_start; // 该 M 口本拍新启动一个写事务。
        wire trans_complete; // 该 M 口本拍完成一个写事务(B 握手成功)。
        reg [$clog2(M_ISSUE[n*32 +: 32]+1)-1:0] trans_count_reg = 0; // 该 M 口在途写事务计数。

        wire trans_limit = trans_count_reg >= M_ISSUE[n*32 +: 32] && !trans_complete; // 该 M 口在途上限命中。

        always @(posedge clk) begin
            if (rst) begin
                trans_count_reg <= 0;
            end else begin
                if (trans_start && !trans_complete) begin
                    trans_count_reg <= trans_count_reg + 1;
                end else if (!trans_start && trans_complete) begin
                    trans_count_reg <= trans_count_reg - 1;
                end
            end
        end

        // 地址仲裁
        reg [CL_S_COUNT-1:0] w_select_reg = 0, w_select_next; // 当前由哪个 S 口向该 M 口发送 W。
        reg w_select_valid_reg = 1'b0, w_select_valid_next; // 当前 W 源选择是否有效。
        reg w_select_new_reg = 1'b0, w_select_new_next; // 是否允许锁存新的 AW 授权作为 W 源。

        wire [S_COUNT-1:0] a_request; // 该 M 口对各 S 口的 AW 仲裁请求。
        wire [S_COUNT-1:0] a_acknowledge; // 该 M 口 AW 仲裁应答。
        wire [S_COUNT-1:0] a_grant; // 该 M 口 AW 仲裁授权 one-hot。
        wire a_grant_valid; // 该 M 口 AW 仲裁是否有有效授权。
        wire [CL_S_COUNT-1:0] a_grant_encoded; // 该 M 口 AW 仲裁授权编码值。

        arbiter #(
            .PORTS(S_COUNT),
            .ARB_TYPE_ROUND_ROBIN(1),
            .ARB_BLOCK(1),
            .ARB_BLOCK_ACK(1),
            .ARB_LSB_HIGH_PRIORITY(1)
        )
        a_arb_inst (
            .clk(clk),
            .rst(rst),
            .request(a_request),
            .acknowledge(a_acknowledge),
            .grant(a_grant),
            .grant_valid(a_grant_valid),
            .grant_encoded(a_grant_encoded)
        );

        // 地址复用
        wire [M_ID_WIDTH-1:0]   s_axi_awid_mux     = int_s_axi_awid[a_grant_encoded*S_ID_WIDTH +: S_ID_WIDTH] | (a_grant_encoded << S_ID_WIDTH); // AWID 复用并拼接源口编号用于返回路由。
        wire [ADDR_WIDTH-1:0]   s_axi_awaddr_mux   = int_s_axi_awaddr[a_grant_encoded*ADDR_WIDTH +: ADDR_WIDTH]; // 复用后的 AWADDR。
        wire [7:0]              s_axi_awlen_mux    = int_s_axi_awlen[a_grant_encoded*8 +: 8]; // 复用后的 AWLEN。
        wire [2:0]              s_axi_awsize_mux   = int_s_axi_awsize[a_grant_encoded*3 +: 3]; // 复用后的 AWSIZE。
        wire [1:0]              s_axi_awburst_mux  = int_s_axi_awburst[a_grant_encoded*2 +: 2]; // 复用后的 AWBURST。
        wire                    s_axi_awlock_mux   = int_s_axi_awlock[a_grant_encoded]; // 复用后的 AWLOCK。
        wire [3:0]              s_axi_awcache_mux  = int_s_axi_awcache[a_grant_encoded*4 +: 4]; // 复用后的 AWCACHE。
        wire [2:0]              s_axi_awprot_mux   = int_s_axi_awprot[a_grant_encoded*3 +: 3]; // 复用后的 AWPROT。
        wire [3:0]              s_axi_awqos_mux    = int_s_axi_awqos[a_grant_encoded*4 +: 4]; // 复用后的 AWQOS。
        wire [3:0]              s_axi_awregion_mux = int_s_axi_awregion[a_grant_encoded*4 +: 4]; // 复用后的 AWREGION。
        wire [AWUSER_WIDTH-1:0] s_axi_awuser_mux   = int_s_axi_awuser[a_grant_encoded*AWUSER_WIDTH +: AWUSER_WIDTH]; // 复用后的 AWUSER。
        wire                    s_axi_awvalid_mux  = int_axi_awvalid[a_grant_encoded*M_COUNT+n] && a_grant_valid; // 复用后的 AWVALID。
        wire                    s_axi_awready_mux; // 目标 M 口寄存器返回的 AWREADY。

        assign int_axi_awready[n*S_COUNT +: S_COUNT] = (a_grant_valid && s_axi_awready_mux) << a_grant_encoded;

        for (m = 0; m < S_COUNT; m = m + 1) begin
            assign a_request[m] = int_axi_awvalid[m*M_COUNT+n] && !a_grant[m] && !trans_limit && !w_select_valid_next;
            assign a_acknowledge[m] = a_grant[m] && int_axi_awvalid[m*M_COUNT+n] && s_axi_awready_mux;
        end

        assign trans_start = s_axi_awvalid_mux && s_axi_awready_mux && a_grant_valid;

        // 写数据复用
        wire [DATA_WIDTH-1:0]  s_axi_wdata_mux   = int_s_axi_wdata[w_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 当前选中 S 口的 WDATA。
        wire [STRB_WIDTH-1:0]  s_axi_wstrb_mux   = int_s_axi_wstrb[w_select_reg*STRB_WIDTH +: STRB_WIDTH]; // 当前选中 S 口的 WSTRB。
        wire                   s_axi_wlast_mux   = int_s_axi_wlast[w_select_reg]; // 当前选中 S 口的 WLAST。
        wire [WUSER_WIDTH-1:0] s_axi_wuser_mux   = int_s_axi_wuser[w_select_reg*WUSER_WIDTH +: WUSER_WIDTH]; // 当前选中 S 口的 WUSER。
        wire                   s_axi_wvalid_mux  = int_axi_wvalid[w_select_reg*M_COUNT+n] && w_select_valid_reg; // 当前选中 S 口到该 M 口的 WVALID。
        wire                   s_axi_wready_mux; // 目标 M 口寄存器返回的 WREADY。

        assign int_axi_wready[n*S_COUNT +: S_COUNT] = (w_select_valid_reg && s_axi_wready_mux) << w_select_reg;

        // 写数据路由
        always @* begin
            w_select_next = w_select_reg;
            w_select_valid_next = w_select_valid_reg && !(s_axi_wvalid_mux && s_axi_wready_mux && s_axi_wlast_mux);
            w_select_new_next = w_select_new_reg || !a_grant_valid || a_acknowledge;

            if (a_grant_valid && !w_select_valid_reg && w_select_new_reg) begin
                w_select_next = a_grant_encoded;
                w_select_valid_next = a_grant_valid;
                w_select_new_next = 1'b0;
            end
        end

        always @(posedge clk) begin
            if (rst) begin
                w_select_valid_reg <= 1'b0;
                w_select_new_reg <= 1'b1;
            end else begin
                w_select_valid_reg <= w_select_valid_next;
                w_select_new_reg <= w_select_new_next;
            end

            w_select_reg <= w_select_next;
        end

        // 写响应回传
        wire [CL_S_COUNT-1:0] b_select = m_axi_bid[n*M_ID_WIDTH +: M_ID_WIDTH] >> S_ID_WIDTH; // 从 BID 高位提取源 S 口索引。

        assign int_axi_bvalid[n*S_COUNT +: S_COUNT] = int_m_axi_bvalid[n] << b_select;
        assign int_m_axi_bready[n] = int_axi_bready[b_select*M_COUNT+n];

        assign trans_complete = int_m_axi_bvalid[n] && int_m_axi_bready[n];

        // M 侧寄存器切片
        axi_register_wr #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .ID_WIDTH(M_ID_WIDTH),
            .AWUSER_ENABLE(AWUSER_ENABLE),
            .AWUSER_WIDTH(AWUSER_WIDTH),
            .WUSER_ENABLE(WUSER_ENABLE),
            .WUSER_WIDTH(WUSER_WIDTH),
            .BUSER_ENABLE(BUSER_ENABLE),
            .BUSER_WIDTH(BUSER_WIDTH),
            .AW_REG_TYPE(M_AW_REG_TYPE[n*2 +: 2]),
            .W_REG_TYPE(M_W_REG_TYPE[n*2 +: 2]),
            .B_REG_TYPE(M_B_REG_TYPE[n*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axi_awid(s_axi_awid_mux),
            .s_axi_awaddr(s_axi_awaddr_mux),
            .s_axi_awlen(s_axi_awlen_mux),
            .s_axi_awsize(s_axi_awsize_mux),
            .s_axi_awburst(s_axi_awburst_mux),
            .s_axi_awlock(s_axi_awlock_mux),
            .s_axi_awcache(s_axi_awcache_mux),
            .s_axi_awprot(s_axi_awprot_mux),
            .s_axi_awqos(s_axi_awqos_mux),
            .s_axi_awregion(s_axi_awregion_mux),
            .s_axi_awuser(s_axi_awuser_mux),
            .s_axi_awvalid(s_axi_awvalid_mux),
            .s_axi_awready(s_axi_awready_mux),
            .s_axi_wdata(s_axi_wdata_mux),
            .s_axi_wstrb(s_axi_wstrb_mux),
            .s_axi_wlast(s_axi_wlast_mux),
            .s_axi_wuser(s_axi_wuser_mux),
            .s_axi_wvalid(s_axi_wvalid_mux),
            .s_axi_wready(s_axi_wready_mux),
            .s_axi_bid(int_m_axi_bid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .s_axi_bresp(int_m_axi_bresp[n*2 +: 2]),
            .s_axi_buser(int_m_axi_buser[n*BUSER_WIDTH +: BUSER_WIDTH]),
            .s_axi_bvalid(int_m_axi_bvalid[n]),
            .s_axi_bready(int_m_axi_bready[n]),
            .m_axi_awid(m_axi_awid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .m_axi_awaddr(m_axi_awaddr[n*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axi_awlen(m_axi_awlen[n*8 +: 8]),
            .m_axi_awsize(m_axi_awsize[n*3 +: 3]),
            .m_axi_awburst(m_axi_awburst[n*2 +: 2]),
            .m_axi_awlock(m_axi_awlock[n]),
            .m_axi_awcache(m_axi_awcache[n*4 +: 4]),
            .m_axi_awprot(m_axi_awprot[n*3 +: 3]),
            .m_axi_awqos(m_axi_awqos[n*4 +: 4]),
            .m_axi_awregion(m_axi_awregion[n*4 +: 4]),
            .m_axi_awuser(m_axi_awuser[n*AWUSER_WIDTH +: AWUSER_WIDTH]),
            .m_axi_awvalid(m_axi_awvalid[n]),
            .m_axi_awready(m_axi_awready[n]),
            .m_axi_wdata(m_axi_wdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .m_axi_wstrb(m_axi_wstrb[n*STRB_WIDTH +: STRB_WIDTH]),
            .m_axi_wlast(m_axi_wlast[n]),
            .m_axi_wuser(m_axi_wuser[n*WUSER_WIDTH +: WUSER_WIDTH]),
            .m_axi_wvalid(m_axi_wvalid[n]),
            .m_axi_wready(m_axi_wready[n]),
            .m_axi_bid(m_axi_bid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .m_axi_bresp(m_axi_bresp[n*2 +: 2]),
            .m_axi_buser(m_axi_buser[n*BUSER_WIDTH +: BUSER_WIDTH]),
            .m_axi_bvalid(m_axi_bvalid[n]),
            .m_axi_bready(m_axi_bready[n])
        );
    end // 主端接口循环

endgenerate

endmodule

`resetall
