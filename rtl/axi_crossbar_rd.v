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
 * AXI4 交叉开关（读通道）
 *
 * 模块目录
 * 1) 每个 S 口通过 `axi_crossbar_addr` 进行 AR 地址解码和准入控制。
 * 2) 每个 M 口独立仲裁 AR 请求，支持多目标并行读。
 * 3) R 响应按 RID 高位携带的源口索引回送到对应 S 口，并支持 DECERR 读返回。
 */
module axi_crossbar_rd #
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
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
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
    // 接口间读通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT = {M_COUNT{{S_COUNT{1'b1}}}},
    // 每个主接口可并发事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd4}},
    // 安全主端口配置（基于 awprot/arprot 拒绝访问）
    // M_COUNT 位
    parameter M_SECURE = {M_COUNT{1'b0}},
    // 从接口 AR 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_AR_REG_TYPE = {S_COUNT{2'd0}},
    // 从接口 R 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter S_R_REG_TYPE = {S_COUNT{2'd2}},
    // 主接口 AR 通道寄存器类型（输出侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_AR_REG_TYPE = {M_COUNT{2'd1}},
    // 主接口 R 通道寄存器类型（输入侧）
    // 0=直通，1=简单缓冲，2=skid buffer
    parameter M_R_REG_TYPE = {M_COUNT{2'd0}}
)
(
    input  wire                             clk, // 读 crossbar 时钟。
    input  wire                             rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_arid, // 所有 S 口拼接的 ARID。
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axi_araddr, // 所有 S 口拼接的 ARADDR。
    input  wire [S_COUNT*8-1:0]             s_axi_arlen, // 所有 S 口拼接的 ARLEN。
    input  wire [S_COUNT*3-1:0]             s_axi_arsize, // 所有 S 口拼接的 ARSIZE。
    input  wire [S_COUNT*2-1:0]             s_axi_arburst, // 所有 S 口拼接的 ARBURST。
    input  wire [S_COUNT-1:0]               s_axi_arlock, // 所有 S 口 ARLOCK。
    input  wire [S_COUNT*4-1:0]             s_axi_arcache, // 所有 S 口拼接的 ARCACHE。
    input  wire [S_COUNT*3-1:0]             s_axi_arprot, // 所有 S 口拼接的 ARPROT。
    input  wire [S_COUNT*4-1:0]             s_axi_arqos, // 所有 S 口拼接的 ARQOS。
    input  wire [S_COUNT*ARUSER_WIDTH-1:0]  s_axi_aruser, // 所有 S 口拼接的 ARUSER。
    input  wire [S_COUNT-1:0]               s_axi_arvalid, // 所有 S 口 ARVALID。
    output wire [S_COUNT-1:0]               s_axi_arready, // 所有 S 口 ARREADY。
    output wire [S_COUNT*S_ID_WIDTH-1:0]    s_axi_rid, // 所有 S 口拼接的 RID。
    output wire [S_COUNT*DATA_WIDTH-1:0]    s_axi_rdata, // 所有 S 口拼接的 RDATA。
    output wire [S_COUNT*2-1:0]             s_axi_rresp, // 所有 S 口拼接的 RRESP。
    output wire [S_COUNT-1:0]               s_axi_rlast, // 所有 S 口 RLAST。
    output wire [S_COUNT*RUSER_WIDTH-1:0]   s_axi_ruser, // 所有 S 口拼接的 RUSER。
    output wire [S_COUNT-1:0]               s_axi_rvalid, // 所有 S 口 RVALID。
    input  wire [S_COUNT-1:0]               s_axi_rready, // 所有 S 口 RREADY。

    /*
     * AXI 主接口
     */
    output wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_arid, // 所有 M 口拼接的 ARID 输出。
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axi_araddr, // 所有 M 口拼接的 ARADDR 输出。
    output wire [M_COUNT*8-1:0]             m_axi_arlen, // 所有 M 口拼接的 ARLEN 输出。
    output wire [M_COUNT*3-1:0]             m_axi_arsize, // 所有 M 口拼接的 ARSIZE 输出。
    output wire [M_COUNT*2-1:0]             m_axi_arburst, // 所有 M 口拼接的 ARBURST 输出。
    output wire [M_COUNT-1:0]               m_axi_arlock, // 所有 M 口 ARLOCK 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arcache, // 所有 M 口拼接的 ARCACHE 输出。
    output wire [M_COUNT*3-1:0]             m_axi_arprot, // 所有 M 口拼接的 ARPROT 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arqos, // 所有 M 口拼接的 ARQOS 输出。
    output wire [M_COUNT*4-1:0]             m_axi_arregion, // 所有 M 口拼接的 ARREGION 输出。
    output wire [M_COUNT*ARUSER_WIDTH-1:0]  m_axi_aruser, // 所有 M 口拼接的 ARUSER 输出。
    output wire [M_COUNT-1:0]               m_axi_arvalid, // 所有 M 口 ARVALID 输出。
    input  wire [M_COUNT-1:0]               m_axi_arready, // 所有 M 口 ARREADY 输入。
    input  wire [M_COUNT*M_ID_WIDTH-1:0]    m_axi_rid, // 所有 M 口拼接的 RID 输入。
    input  wire [M_COUNT*DATA_WIDTH-1:0]    m_axi_rdata, // 所有 M 口拼接的 RDATA 输入。
    input  wire [M_COUNT*2-1:0]             m_axi_rresp, // 所有 M 口拼接的 RRESP 输入。
    input  wire [M_COUNT-1:0]               m_axi_rlast, // 所有 M 口 RLAST 输入。
    input  wire [M_COUNT*RUSER_WIDTH-1:0]   m_axi_ruser, // 所有 M 口拼接的 RUSER 输入。
    input  wire [M_COUNT-1:0]               m_axi_rvalid, // 所有 M 口 RVALID 输入。
    output wire [M_COUNT-1:0]               m_axi_rready // 所有 M 口 RREADY 输出。
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

wire [S_COUNT*S_ID_WIDTH-1:0]    int_s_axi_arid; // S 侧寄存后内部 ARID 总线。
wire [S_COUNT*ADDR_WIDTH-1:0]    int_s_axi_araddr; // S 侧寄存后内部 ARADDR 总线。
wire [S_COUNT*8-1:0]             int_s_axi_arlen; // S 侧寄存后内部 ARLEN 总线。
wire [S_COUNT*3-1:0]             int_s_axi_arsize; // S 侧寄存后内部 ARSIZE 总线。
wire [S_COUNT*2-1:0]             int_s_axi_arburst; // S 侧寄存后内部 ARBURST 总线。
wire [S_COUNT-1:0]               int_s_axi_arlock; // S 侧寄存后内部 ARLOCK。
wire [S_COUNT*4-1:0]             int_s_axi_arcache; // S 侧寄存后内部 ARCACHE 总线。
wire [S_COUNT*3-1:0]             int_s_axi_arprot; // S 侧寄存后内部 ARPROT 总线。
wire [S_COUNT*4-1:0]             int_s_axi_arqos; // S 侧寄存后内部 ARQOS 总线。
wire [S_COUNT*4-1:0]             int_s_axi_arregion; // 地址解码后写入的 ARREGION 总线。
wire [S_COUNT*ARUSER_WIDTH-1:0]  int_s_axi_aruser; // S 侧寄存后内部 ARUSER 总线。
wire [S_COUNT-1:0]               int_s_axi_arvalid; // 各 S 口内部 ARVALID。
wire [S_COUNT-1:0]               int_s_axi_arready; // 各 S 口内部 ARREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axi_arvalid; // S->M 的 AR 路由有效矩阵。
wire [M_COUNT*S_COUNT-1:0]       int_axi_arready; // M->S 的 AR 路由 ready 矩阵。

wire [M_COUNT*M_ID_WIDTH-1:0]    int_m_axi_rid; // M 侧寄存后内部 RID 总线。
wire [M_COUNT*DATA_WIDTH-1:0]    int_m_axi_rdata; // M 侧寄存后内部 RDATA 总线。
wire [M_COUNT*2-1:0]             int_m_axi_rresp; // M 侧寄存后内部 RRESP 总线。
wire [M_COUNT-1:0]               int_m_axi_rlast; // M 侧寄存后内部 RLAST。
wire [M_COUNT*RUSER_WIDTH-1:0]   int_m_axi_ruser; // M 侧寄存后内部 RUSER 总线。
wire [M_COUNT-1:0]               int_m_axi_rvalid; // 各 M 口内部 RVALID。
wire [M_COUNT-1:0]               int_m_axi_rready; // 各 M 口内部 RREADY。

wire [M_COUNT*S_COUNT-1:0]       int_axi_rvalid; // M->S 的 R 路由有效矩阵。
wire [S_COUNT*M_COUNT-1:0]       int_axi_rready; // S->M 的 R 路由 ready 矩阵。

generate

    genvar m, n;

    for (m = 0; m < S_COUNT; m = m + 1) begin : s_ifaces
        // 地址解码与接纳控制
        wire [CL_M_COUNT-1:0] a_select; // 当前 S 口地址解码得到的目标 M 口。

        wire m_axi_avalid; // 地址输出命令有效。
        wire m_axi_aready; // 地址输出命令被目标侧接受。

        wire m_rc_decerr; // 返回命令是否为解码错误。
        wire m_rc_valid; // 返回命令有效。
        wire m_rc_ready; // 返回命令被本地 R 处理逻辑接收。

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
            .WC_OUTPUT(0)
        )
        addr_inst (
            .clk(clk),
            .rst(rst),

            /*
             * 地址输入
             */
            .s_axi_aid(int_s_axi_arid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_aaddr(int_s_axi_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axi_aprot(int_s_axi_arprot[m*3 +: 3]),
            .s_axi_aqos(int_s_axi_arqos[m*4 +: 4]),
            .s_axi_avalid(int_s_axi_arvalid[m]),
            .s_axi_aready(int_s_axi_arready[m]),

            /*
             * 地址输出
             */
            .m_axi_aregion(int_s_axi_arregion[m*4 +: 4]),
            .m_select(a_select),
            .m_axi_avalid(m_axi_avalid),
            .m_axi_aready(m_axi_aready),

            /*
             * 写命令输出
             */
            .m_wc_select(),
            .m_wc_decerr(),
            .m_wc_valid(),
            .m_wc_ready(1'b1),

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

        assign int_axi_arvalid[m*M_COUNT +: M_COUNT] = m_axi_avalid << a_select;
        assign m_axi_aready = int_axi_arready[a_select*S_COUNT+m];

        // 解码错误处理
        reg [S_ID_WIDTH-1:0]  decerr_m_axi_rid_reg = {S_ID_WIDTH{1'b0}}, decerr_m_axi_rid_next; // DECERR 虚拟读响应 RID。
        reg                   decerr_m_axi_rlast_reg = 1'b0, decerr_m_axi_rlast_next; // DECERR 虚拟读响应 RLAST。
        reg                   decerr_m_axi_rvalid_reg = 1'b0, decerr_m_axi_rvalid_next; // DECERR 虚拟读响应有效位。
        wire                  decerr_m_axi_rready; // DECERR 虚拟读响应被消费。

        reg [7:0] decerr_len_reg = 8'd0, decerr_len_next; // DECERR 剩余返回 beat 计数。

        assign m_rc_ready = !decerr_m_axi_rvalid_reg;

        always @* begin
            decerr_len_next = decerr_len_reg;
            decerr_m_axi_rid_next = decerr_m_axi_rid_reg;
            decerr_m_axi_rlast_next = decerr_m_axi_rlast_reg;
            decerr_m_axi_rvalid_next = decerr_m_axi_rvalid_reg;

            if (decerr_m_axi_rvalid_reg) begin
                if (decerr_m_axi_rready) begin
                    if (decerr_len_reg > 0) begin
                        decerr_len_next = decerr_len_reg-1;
                        decerr_m_axi_rlast_next = (decerr_len_next == 0);
                        decerr_m_axi_rvalid_next = 1'b1;
                    end else begin
                        decerr_m_axi_rvalid_next = 1'b0;
                    end
                end
            end else if (m_rc_valid && m_rc_ready) begin
                decerr_len_next = int_s_axi_arlen[m*8 +: 8];
                decerr_m_axi_rid_next = int_s_axi_arid[m*S_ID_WIDTH +: S_ID_WIDTH];
                decerr_m_axi_rlast_next = (decerr_len_next == 0);
                decerr_m_axi_rvalid_next = 1'b1;
            end
        end

        always @(posedge clk) begin
            if (rst) begin
                decerr_m_axi_rvalid_reg <= 1'b0;
            end else begin
                decerr_m_axi_rvalid_reg <= decerr_m_axi_rvalid_next;
            end

            decerr_m_axi_rid_reg <= decerr_m_axi_rid_next;
            decerr_m_axi_rlast_reg <= decerr_m_axi_rlast_next;
            decerr_len_reg <= decerr_len_next;
        end

        // 读响应仲裁
        wire [M_COUNT_P1-1:0] r_request; // R 仲裁请求(含一个 DECERR 虚拟端口)。
        wire [M_COUNT_P1-1:0] r_acknowledge; // R 仲裁完成应答。
        wire [M_COUNT_P1-1:0] r_grant; // R 仲裁授权 one-hot。
        wire r_grant_valid; // R 仲裁是否有有效授权。
        wire [CL_M_COUNT_P1-1:0] r_grant_encoded; // R 仲裁授权编码值。

        arbiter #(
            .PORTS(M_COUNT_P1),
            .ARB_TYPE_ROUND_ROBIN(1),
            .ARB_BLOCK(1),
            .ARB_BLOCK_ACK(1),
            .ARB_LSB_HIGH_PRIORITY(1)
        )
        r_arb_inst (
            .clk(clk),
            .rst(rst),
            .request(r_request),
            .acknowledge(r_acknowledge),
            .grant(r_grant),
            .grant_valid(r_grant_valid),
            .grant_encoded(r_grant_encoded)
        );

        // 读响应复用
        wire [S_ID_WIDTH-1:0]  m_axi_rid_mux    = {decerr_m_axi_rid_reg, int_m_axi_rid} >> r_grant_encoded*M_ID_WIDTH; // 复用后的 RID(剥离路由位后给 S 侧)。
        wire [DATA_WIDTH-1:0]  m_axi_rdata_mux  = {{DATA_WIDTH{1'b0}}, int_m_axi_rdata} >> r_grant_encoded*DATA_WIDTH; // 复用后的 RDATA(DECERR 路径为全 0)。
        wire [1:0]             m_axi_rresp_mux  = {2'b11, int_m_axi_rresp} >> r_grant_encoded*2; // 复用后的 RRESP(DECERR 或真实响应)。
        wire                   m_axi_rlast_mux  = {decerr_m_axi_rlast_reg, int_m_axi_rlast} >> r_grant_encoded; // 复用后的 RLAST。
        wire [RUSER_WIDTH-1:0] m_axi_ruser_mux  = {{RUSER_WIDTH{1'b0}}, int_m_axi_ruser} >> r_grant_encoded*RUSER_WIDTH; // 复用后的 RUSER。
        wire                   m_axi_rvalid_mux = ({decerr_m_axi_rvalid_reg, int_m_axi_rvalid} >> r_grant_encoded) & r_grant_valid; // 复用后的 RVALID。
        wire                   m_axi_rready_mux; // S 侧寄存器返回的 RREADY。

        assign int_axi_rready[m*M_COUNT +: M_COUNT] = (r_grant_valid && m_axi_rready_mux) << r_grant_encoded;
        assign decerr_m_axi_rready = (r_grant_valid && m_axi_rready_mux) && (r_grant_encoded == M_COUNT_P1-1);

        for (n = 0; n < M_COUNT; n = n + 1) begin
            assign r_request[n] = int_axi_rvalid[n*S_COUNT+m] && !r_grant[n];
            assign r_acknowledge[n] = r_grant[n] && int_axi_rvalid[n*S_COUNT+m] && m_axi_rlast_mux && m_axi_rready_mux;
        end

        assign r_request[M_COUNT_P1-1] = decerr_m_axi_rvalid_reg && !r_grant[M_COUNT_P1-1];
        assign r_acknowledge[M_COUNT_P1-1] = r_grant[M_COUNT_P1-1] && decerr_m_axi_rvalid_reg && decerr_m_axi_rlast_reg && m_axi_rready_mux;

        assign s_cpl_id = m_axi_rid_mux;
        assign s_cpl_valid = m_axi_rvalid_mux && m_axi_rready_mux && m_axi_rlast_mux;

        // S 侧寄存器切片
        axi_register_rd #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .ID_WIDTH(S_ID_WIDTH),
            .ARUSER_ENABLE(ARUSER_ENABLE),
            .ARUSER_WIDTH(ARUSER_WIDTH),
            .RUSER_ENABLE(RUSER_ENABLE),
            .RUSER_WIDTH(RUSER_WIDTH),
            .AR_REG_TYPE(S_AR_REG_TYPE[m*2 +: 2]),
            .R_REG_TYPE(S_R_REG_TYPE[m*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axi_arid(s_axi_arid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_araddr(s_axi_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axi_arlen(s_axi_arlen[m*8 +: 8]),
            .s_axi_arsize(s_axi_arsize[m*3 +: 3]),
            .s_axi_arburst(s_axi_arburst[m*2 +: 2]),
            .s_axi_arlock(s_axi_arlock[m]),
            .s_axi_arcache(s_axi_arcache[m*4 +: 4]),
            .s_axi_arprot(s_axi_arprot[m*3 +: 3]),
            .s_axi_arqos(s_axi_arqos[m*4 +: 4]),
            .s_axi_arregion(4'd0),
            .s_axi_aruser(s_axi_aruser[m*ARUSER_WIDTH +: ARUSER_WIDTH]),
            .s_axi_arvalid(s_axi_arvalid[m]),
            .s_axi_arready(s_axi_arready[m]),
            .s_axi_rid(s_axi_rid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .s_axi_rdata(s_axi_rdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .s_axi_rresp(s_axi_rresp[m*2 +: 2]),
            .s_axi_rlast(s_axi_rlast[m]),
            .s_axi_ruser(s_axi_ruser[m*RUSER_WIDTH +: RUSER_WIDTH]),
            .s_axi_rvalid(s_axi_rvalid[m]),
            .s_axi_rready(s_axi_rready[m]),
            .m_axi_arid(int_s_axi_arid[m*S_ID_WIDTH +: S_ID_WIDTH]),
            .m_axi_araddr(int_s_axi_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axi_arlen(int_s_axi_arlen[m*8 +: 8]),
            .m_axi_arsize(int_s_axi_arsize[m*3 +: 3]),
            .m_axi_arburst(int_s_axi_arburst[m*2 +: 2]),
            .m_axi_arlock(int_s_axi_arlock[m]),
            .m_axi_arcache(int_s_axi_arcache[m*4 +: 4]),
            .m_axi_arprot(int_s_axi_arprot[m*3 +: 3]),
            .m_axi_arqos(int_s_axi_arqos[m*4 +: 4]),
            .m_axi_arregion(),
            .m_axi_aruser(int_s_axi_aruser[m*ARUSER_WIDTH +: ARUSER_WIDTH]),
            .m_axi_arvalid(int_s_axi_arvalid[m]),
            .m_axi_arready(int_s_axi_arready[m]),
            .m_axi_rid(m_axi_rid_mux),
            .m_axi_rdata(m_axi_rdata_mux),
            .m_axi_rresp(m_axi_rresp_mux),
            .m_axi_rlast(m_axi_rlast_mux),
            .m_axi_ruser(m_axi_ruser_mux),
            .m_axi_rvalid(m_axi_rvalid_mux),
            .m_axi_rready(m_axi_rready_mux)
        );
    end // 从端接口循环

    for (n = 0; n < M_COUNT; n = n + 1) begin : m_ifaces
        // 在途事务计数
        wire trans_start; // 该 M 口本拍新启动一个读事务。
        wire trans_complete; // 该 M 口本拍完成一个读事务(RLAST 握手成功)。
        reg [$clog2(M_ISSUE[n*32 +: 32]+1)-1:0] trans_count_reg = 0; // 该 M 口在途读事务计数。

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
        wire [S_COUNT-1:0] a_request; // 该 M 口对各 S 口的 AR 仲裁请求。
        wire [S_COUNT-1:0] a_acknowledge; // 该 M 口 AR 仲裁应答。
        wire [S_COUNT-1:0] a_grant; // 该 M 口 AR 仲裁授权 one-hot。
        wire a_grant_valid; // 该 M 口 AR 仲裁是否有有效授权。
        wire [CL_S_COUNT-1:0] a_grant_encoded; // 该 M 口 AR 仲裁授权编码值。

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
        wire [M_ID_WIDTH-1:0]   s_axi_arid_mux     = int_s_axi_arid[a_grant_encoded*S_ID_WIDTH +: S_ID_WIDTH] | (a_grant_encoded << S_ID_WIDTH); // ARID 复用并拼接源口编号用于返回路由。
        wire [ADDR_WIDTH-1:0]   s_axi_araddr_mux   = int_s_axi_araddr[a_grant_encoded*ADDR_WIDTH +: ADDR_WIDTH]; // 复用后的 ARADDR。
        wire [7:0]              s_axi_arlen_mux    = int_s_axi_arlen[a_grant_encoded*8 +: 8]; // 复用后的 ARLEN。
        wire [2:0]              s_axi_arsize_mux   = int_s_axi_arsize[a_grant_encoded*3 +: 3]; // 复用后的 ARSIZE。
        wire [1:0]              s_axi_arburst_mux  = int_s_axi_arburst[a_grant_encoded*2 +: 2]; // 复用后的 ARBURST。
        wire                    s_axi_arlock_mux   = int_s_axi_arlock[a_grant_encoded]; // 复用后的 ARLOCK。
        wire [3:0]              s_axi_arcache_mux  = int_s_axi_arcache[a_grant_encoded*4 +: 4]; // 复用后的 ARCACHE。
        wire [2:0]              s_axi_arprot_mux   = int_s_axi_arprot[a_grant_encoded*3 +: 3]; // 复用后的 ARPROT。
        wire [3:0]              s_axi_arqos_mux    = int_s_axi_arqos[a_grant_encoded*4 +: 4]; // 复用后的 ARQOS。
        wire [3:0]              s_axi_arregion_mux = int_s_axi_arregion[a_grant_encoded*4 +: 4]; // 复用后的 ARREGION。
        wire [ARUSER_WIDTH-1:0] s_axi_aruser_mux   = int_s_axi_aruser[a_grant_encoded*ARUSER_WIDTH +: ARUSER_WIDTH]; // 复用后的 ARUSER。
        wire                    s_axi_arvalid_mux  = int_axi_arvalid[a_grant_encoded*M_COUNT+n] && a_grant_valid; // 复用后的 ARVALID。
        wire                    s_axi_arready_mux; // 目标 M 口寄存器返回的 ARREADY。

        assign int_axi_arready[n*S_COUNT +: S_COUNT] = (a_grant_valid && s_axi_arready_mux) << a_grant_encoded;

        for (m = 0; m < S_COUNT; m = m + 1) begin
            assign a_request[m] = int_axi_arvalid[m*M_COUNT+n] && !a_grant[m] && !trans_limit;
            assign a_acknowledge[m] = a_grant[m] && int_axi_arvalid[m*M_COUNT+n] && s_axi_arready_mux;
        end

        assign trans_start = s_axi_arvalid_mux && s_axi_arready_mux && a_grant_valid;

        // 读响应回传
        wire [CL_S_COUNT-1:0] r_select = m_axi_rid[n*M_ID_WIDTH +: M_ID_WIDTH] >> S_ID_WIDTH; // 从 RID 高位提取源 S 口索引。

        assign int_axi_rvalid[n*S_COUNT +: S_COUNT] = int_m_axi_rvalid[n] << r_select;
        assign int_m_axi_rready[n] = int_axi_rready[r_select*M_COUNT+n];

        assign trans_complete = int_m_axi_rvalid[n] && int_m_axi_rready[n] && int_m_axi_rlast[n];

        // M 侧寄存器切片
        axi_register_rd #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .ID_WIDTH(M_ID_WIDTH),
            .ARUSER_ENABLE(ARUSER_ENABLE),
            .ARUSER_WIDTH(ARUSER_WIDTH),
            .RUSER_ENABLE(RUSER_ENABLE),
            .RUSER_WIDTH(RUSER_WIDTH),
            .AR_REG_TYPE(M_AR_REG_TYPE[n*2 +: 2]),
            .R_REG_TYPE(M_R_REG_TYPE[n*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axi_arid(s_axi_arid_mux),
            .s_axi_araddr(s_axi_araddr_mux),
            .s_axi_arlen(s_axi_arlen_mux),
            .s_axi_arsize(s_axi_arsize_mux),
            .s_axi_arburst(s_axi_arburst_mux),
            .s_axi_arlock(s_axi_arlock_mux),
            .s_axi_arcache(s_axi_arcache_mux),
            .s_axi_arprot(s_axi_arprot_mux),
            .s_axi_arqos(s_axi_arqos_mux),
            .s_axi_arregion(s_axi_arregion_mux),
            .s_axi_aruser(s_axi_aruser_mux),
            .s_axi_arvalid(s_axi_arvalid_mux),
            .s_axi_arready(s_axi_arready_mux),
            .s_axi_rid(int_m_axi_rid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .s_axi_rdata(int_m_axi_rdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .s_axi_rresp(int_m_axi_rresp[n*2 +: 2]),
            .s_axi_rlast(int_m_axi_rlast[n]),
            .s_axi_ruser(int_m_axi_ruser[n*RUSER_WIDTH +: RUSER_WIDTH]),
            .s_axi_rvalid(int_m_axi_rvalid[n]),
            .s_axi_rready(int_m_axi_rready[n]),
            .m_axi_arid(m_axi_arid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .m_axi_araddr(m_axi_araddr[n*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axi_arlen(m_axi_arlen[n*8 +: 8]),
            .m_axi_arsize(m_axi_arsize[n*3 +: 3]),
            .m_axi_arburst(m_axi_arburst[n*2 +: 2]),
            .m_axi_arlock(m_axi_arlock[n]),
            .m_axi_arcache(m_axi_arcache[n*4 +: 4]),
            .m_axi_arprot(m_axi_arprot[n*3 +: 3]),
            .m_axi_arqos(m_axi_arqos[n*4 +: 4]),
            .m_axi_arregion(m_axi_arregion[n*4 +: 4]),
            .m_axi_aruser(m_axi_aruser[n*ARUSER_WIDTH +: ARUSER_WIDTH]),
            .m_axi_arvalid(m_axi_arvalid[n]),
            .m_axi_arready(m_axi_arready[n]),
            .m_axi_rid(m_axi_rid[n*M_ID_WIDTH +: M_ID_WIDTH]),
            .m_axi_rdata(m_axi_rdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .m_axi_rresp(m_axi_rresp[n*2 +: 2]),
            .m_axi_rlast(m_axi_rlast[n]),
            .m_axi_ruser(m_axi_ruser[n*RUSER_WIDTH +: RUSER_WIDTH]),
            .m_axi_rvalid(m_axi_rvalid[n]),
            .m_axi_rready(m_axi_rready[n])
        );
    end // 主端接口循环

endgenerate

endmodule

`resetall
