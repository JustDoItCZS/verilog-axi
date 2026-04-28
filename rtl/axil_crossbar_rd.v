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
 * AXI4-Lite 交叉开关（读通道）
 *
 * 模块目录
 * 1) 从端侧：
 *    - 对每个 AR 请求做地址解码并发往唯一目标主端
 *    - 将回程路由元数据（源从端 + decerr 标记）压入 FIFO
 * 2) 主端侧：
 *    - 对每个主端来自多个从端的 AR 请求进行仲裁
 *    - 将返回 R 通道路由到记录的源从端
 * 3) 通过 axil_register_rd 在从/主端两侧插入可选寄存器切片。
 */
module axil_crossbar_rd #
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
    // 每个从接口可并发处理事务数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_ACCEPT = {S_COUNT{32'd16}},
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
    parameter M_CONNECT = {M_COUNT{{S_COUNT{1'b1}}}},
    // 每个主接口可并发处理事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd16}},
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
    input  wire                             clk, // 读交叉开关时钟。
    input  wire                             rst, // 同步复位，清空仲裁与 FIFO 状态。

    /*
     * AXI-Lite 从接口
     */
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axil_araddr, // 从端 AR 地址向量；每个从端一个切片。
    input  wire [S_COUNT*3-1:0]             s_axil_arprot, // 从端 AR 保护属性向量。
    input  wire [S_COUNT-1:0]               s_axil_arvalid, // 从端 ARVALID 向量。
    output wire [S_COUNT-1:0]               s_axil_arready, // 从端 ARREADY 向量（由解码接纳链路返回）。
    output wire [S_COUNT*DATA_WIDTH-1:0]    s_axil_rdata, // 从端读数据向量。
    output wire [S_COUNT*2-1:0]             s_axil_rresp, // 从端读响应向量（含注入 DECERR）。
    output wire [S_COUNT-1:0]               s_axil_rvalid, // 从端 RVALID 向量。
    input  wire [S_COUNT-1:0]               s_axil_rready, // 从端 RREADY 向量（端口级反压）。

    /*
     * AXI-Lite 主接口
     */
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axil_araddr, // 主端 AR 地址向量（仲裁后输出）。
    output wire [M_COUNT*3-1:0]             m_axil_arprot, // 主端 AR 保护属性向量。
    output wire [M_COUNT-1:0]               m_axil_arvalid, // 主端 ARVALID 向量。
    input  wire [M_COUNT-1:0]               m_axil_arready, // 下游目标返回的主端 ARREADY 向量。
    input  wire [M_COUNT*DATA_WIDTH-1:0]    m_axil_rdata, // 目标返回的主端读数据向量。
    input  wire [M_COUNT*2-1:0]             m_axil_rresp, // 目标返回的主端读响应向量。
    input  wire [M_COUNT-1:0]               m_axil_rvalid, // 目标返回的主端 RVALID 向量。
    output wire [M_COUNT-1:0]               m_axil_rready // 由回程路由驱动的主端 RREADY 向量。
);

parameter CL_S_COUNT = $clog2(S_COUNT); // 编码从端索引所需位宽。
parameter CL_M_COUNT = $clog2(M_COUNT); // 编码主端索引所需位宽。
parameter M_COUNT_P1 = M_COUNT+1; // 辅助计数：主端数量加 1。
parameter CL_M_COUNT_P1 = $clog2(M_COUNT_P1); // M_COUNT_P1 对应编码位宽。

integer i; // 静态配置检查循环变量。

// 配置合法性检查
initial begin
    for (i = 0; i < M_COUNT*M_REGIONS; i = i + 1) begin
        if (M_ADDR_WIDTH[i*32 +: 32] && (M_ADDR_WIDTH[i*32 +: 32] < $clog2(STRB_WIDTH) || M_ADDR_WIDTH[i*32 +: 32] > ADDR_WIDTH)) begin
            $error("Error: value out of range (instance %m)");
            $finish;
        end
    end
end

wire [S_COUNT*ADDR_WIDTH-1:0]    int_s_axil_araddr; // 内部从端 AR 地址（经过可选 S 侧寄存器）。
wire [S_COUNT*3-1:0]             int_s_axil_arprot; // 内部从端 AR 保护属性。
wire [S_COUNT-1:0]               int_s_axil_arvalid; // 进入解码逻辑的内部 ARVALID。
wire [S_COUNT-1:0]               int_s_axil_arready; // 解码逻辑返回的内部 ARREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axil_arvalid; // 交叉点 ARVALID 矩阵 [slave][master]。
wire [M_COUNT*S_COUNT-1:0]       int_axil_arready; // 交叉点 ARREADY 矩阵 [master][slave]。

wire [M_COUNT*DATA_WIDTH-1:0]    int_m_axil_rdata; // 内部主端 RDATA 向量（经过 M 侧寄存器）。
wire [M_COUNT*2-1:0]             int_m_axil_rresp; // 内部主端 RRESP 向量。
wire [M_COUNT-1:0]               int_m_axil_rvalid; // 内部主端 RVALID。
wire [M_COUNT-1:0]               int_m_axil_rready; // 内部主端 RREADY。

wire [M_COUNT*S_COUNT-1:0]       int_axil_rvalid; // 交叉点 RVALID 矩阵 [master][slave]。
wire [S_COUNT*M_COUNT-1:0]       int_axil_rready; // 交叉点 RREADY 矩阵 [slave][master]。

generate

    genvar m, n;

    for (m = 0; m < S_COUNT; m = m + 1) begin : s_ifaces
        // 响应路由 FIFO
        localparam FIFO_ADDR_WIDTH = $clog2(S_ACCEPT[m*32 +: 32])+1; // 每个从端回程路由 FIFO 深度控制位宽。

        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_wr_ptr_reg = 0; // 写指针：解码元数据入队时递增。
        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_rd_ptr_reg = 0; // 读指针：从端消费 R 响应时递增。

        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg [CL_M_COUNT-1:0] fifo_select[(2**FIFO_ADDR_WIDTH)-1:0]; // 每个在途读事务记录目标主端索引。
        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg fifo_decerr[(2**FIFO_ADDR_WIDTH)-1:0]; // 记录解码错误标记，用于本地合成 DECERR 响应。

        wire [CL_M_COUNT-1:0] fifo_wr_select; // FIFO 写入内容：选中主端索引。
        wire fifo_wr_decerr; // FIFO 写入内容：解码错误标志。
        wire fifo_wr_en; // FIFO 写使能：响应命令元数据被接纳时拉高。

        reg [CL_M_COUNT-1:0] fifo_rd_select_reg = 0; // 锁存的路由选择，用于读响应复用。
        reg fifo_rd_decerr_reg = 0; // 锁存当前路由响应的解码错误状态。
        reg fifo_rd_valid_reg = 0; // 指示 fifo_rd_* 元数据有效。
        wire fifo_rd_en; // FIFO 出队：源从端 R 握手成功时触发。
        reg fifo_half_full_reg = 1'b0; // 高水位标志：节流新解码命令进入。

        wire fifo_empty = fifo_rd_ptr_reg == fifo_wr_ptr_reg; // FIFO 空标志，用于控制元数据预取。

        integer i; // 本地 FIFO RAM 初始化循环变量。

        initial begin
            for (i = 0; i < 2**FIFO_ADDR_WIDTH; i = i + 1) begin
                fifo_select[i] = 0;
                fifo_decerr[i] = 0;
            end
        end

        always @(posedge clk) begin
            if (fifo_wr_en) begin
                fifo_select[fifo_wr_ptr_reg[FIFO_ADDR_WIDTH-1:0]] <= fifo_wr_select;
                fifo_decerr[fifo_wr_ptr_reg[FIFO_ADDR_WIDTH-1:0]] <= fifo_wr_decerr;
                fifo_wr_ptr_reg <= fifo_wr_ptr_reg + 1;
            end

            fifo_rd_valid_reg <= fifo_rd_valid_reg && !fifo_rd_en;

            if ((fifo_rd_ptr_reg != fifo_wr_ptr_reg) && (!fifo_rd_valid_reg || fifo_rd_en)) begin
                fifo_rd_select_reg <= fifo_select[fifo_rd_ptr_reg[FIFO_ADDR_WIDTH-1:0]];
                fifo_rd_decerr_reg <= fifo_decerr[fifo_rd_ptr_reg[FIFO_ADDR_WIDTH-1:0]];
                fifo_rd_valid_reg <= 1'b1;
                fifo_rd_ptr_reg <= fifo_rd_ptr_reg + 1;
            end

            fifo_half_full_reg <= $unsigned(fifo_wr_ptr_reg - fifo_rd_ptr_reg) >= 2**(FIFO_ADDR_WIDTH-1);

            if (rst) begin
                fifo_wr_ptr_reg <= 0;
                fifo_rd_ptr_reg <= 0;
                fifo_rd_valid_reg <= 1'b0;
            end
        end

        // 地址解码与接纳控制
        wire [CL_M_COUNT-1:0] a_select; // 当前从端 AR 解码得到的目标主端索引。

        wire m_axil_avalid; // 解码后送往目标选择矩阵的 AR 有效信号。
        wire m_axil_aready; // 选中主端 AR 通路返回的 ready。

        wire [CL_M_COUNT-1:0] m_rc_select; // 压入 FIFO 的响应命令目标索引。
        wire m_rc_decerr; // 响应命令解码错误标记。
        wire m_rc_valid; // 解码器输出的响应命令有效信号。
        wire m_rc_ready; // 本地 FIFO 写端返回的响应命令就绪信号。

        axil_crossbar_addr #(
            .S(m),
            .S_COUNT(S_COUNT),
            .M_COUNT(M_COUNT),
            .ADDR_WIDTH(ADDR_WIDTH),
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
            .s_axil_aaddr(int_s_axil_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axil_aprot(int_s_axil_arprot[m*3 +: 3]),
            .s_axil_avalid(int_s_axil_arvalid[m]),
            .s_axil_aready(int_s_axil_arready[m]),

            /*
             * 地址输出
             */
            .m_select(a_select),
            .m_axil_avalid(m_axil_avalid),
            .m_axil_aready(m_axil_aready),

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
            .m_rc_select(m_rc_select),
            .m_rc_decerr(m_rc_decerr),
            .m_rc_valid(m_rc_valid),
            .m_rc_ready(m_rc_ready)
        );

        assign int_axil_arvalid[m*M_COUNT +: M_COUNT] = m_axil_avalid << a_select;
        assign m_axil_aready = int_axil_arready[a_select*S_COUNT+m];

        // 响应处理
        assign fifo_wr_select = m_rc_select;
        assign fifo_wr_decerr = m_rc_decerr;
        assign fifo_wr_en = m_rc_valid && !fifo_half_full_reg;
        assign m_rc_ready = !fifo_half_full_reg;

        // 读响应处理
        wire [CL_M_COUNT-1:0] r_select = M_COUNT > 1 ? fifo_rd_select_reg : 0; // 当前返回响应选中的主端索引。
        wire r_decerr = fifo_rd_decerr_reg; // 置位时该响应为本地合成 DECERR。
        wire r_valid = fifo_rd_valid_reg; // 响应元数据有效标记。

        // 读响应复用
        wire [DATA_WIDTH-1:0]  m_axil_rdata_mux  = r_decerr ? {DATA_WIDTH{1'b0}} : int_m_axil_rdata[r_select*DATA_WIDTH +: DATA_WIDTH]; // 路由或合成后的 RDATA，返回当前从端。
        wire [1:0]             m_axil_rresp_mux  = r_decerr ? 2'b11 : int_m_axil_rresp[r_select*2 +: 2]; // 路由或合成后的 RRESP。
        wire                   m_axil_rvalid_mux = (r_decerr ? 1'b1 : int_axil_rvalid[r_select*S_COUNT+m]) && r_valid; // 被选响应可用时拉高 RVALID。
        wire                   m_axil_rready_mux; // 从端寄存器切片返回的 RREADY。

        assign int_axil_rready[m*M_COUNT +: M_COUNT] = (r_valid && m_axil_rready_mux) << r_select;

        assign fifo_rd_en = m_axil_rvalid_mux && m_axil_rready_mux && r_valid;

        // S 侧寄存器切片
        axil_register_rd #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .AR_REG_TYPE(S_AR_REG_TYPE[m*2 +: 2]),
            .R_REG_TYPE(S_R_REG_TYPE[m*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axil_araddr(s_axil_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axil_arprot(s_axil_arprot[m*3 +: 3]),
            .s_axil_arvalid(s_axil_arvalid[m]),
            .s_axil_arready(s_axil_arready[m]),
            .s_axil_rdata(s_axil_rdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .s_axil_rresp(s_axil_rresp[m*2 +: 2]),
            .s_axil_rvalid(s_axil_rvalid[m]),
            .s_axil_rready(s_axil_rready[m]),
            .m_axil_araddr(int_s_axil_araddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axil_arprot(int_s_axil_arprot[m*3 +: 3]),
            .m_axil_arvalid(int_s_axil_arvalid[m]),
            .m_axil_arready(int_s_axil_arready[m]),
            .m_axil_rdata(m_axil_rdata_mux),
            .m_axil_rresp(m_axil_rresp_mux),
            .m_axil_rvalid(m_axil_rvalid_mux),
            .m_axil_rready(m_axil_rready_mux)
        );
    end // 从端接口循环

    for (n = 0; n < M_COUNT; n = n + 1) begin : m_ifaces
        // 响应路由 FIFO
        localparam FIFO_ADDR_WIDTH = $clog2(M_ISSUE[n*32 +: 32])+1; // 每个主端来源路由 FIFO 深度控制位宽。

        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_wr_ptr_reg = 0; // 写指针：AR 请求被接纳时递增。
        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_rd_ptr_reg = 0; // 读指针：对应 R 数据拍握手时递增。

        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg [CL_S_COUNT-1:0] fifo_select[(2**FIFO_ADDR_WIDTH)-1:0]; // 为发往该主端的每个在途请求记录源从端 ID。
        wire [CL_S_COUNT-1:0] fifo_wr_select; // FIFO 写入内容：仲裁授予的源从端 ID。
        wire fifo_wr_en; // FIFO 写使能：AR 接纳时拉高。
        wire fifo_rd_en; // FIFO 读使能：R 接纳时拉高。
        reg fifo_half_full_reg = 1'b0; // 高水位反压标志：用于仲裁节流。

        wire fifo_empty = fifo_rd_ptr_reg == fifo_wr_ptr_reg; // FIFO 空标志（该主端无待返回响应）。

        integer i; // 本地来源路由 FIFO RAM 初始化循环变量。

        initial begin
            for (i = 0; i < 2**FIFO_ADDR_WIDTH; i = i + 1) begin
                fifo_select[i] = 0;
            end
        end

        always @(posedge clk) begin
            if (fifo_wr_en) begin
                fifo_select[fifo_wr_ptr_reg[FIFO_ADDR_WIDTH-1:0]] <= fifo_wr_select;
                fifo_wr_ptr_reg <= fifo_wr_ptr_reg + 1;
            end
            if (fifo_rd_en) begin
                fifo_rd_ptr_reg <= fifo_rd_ptr_reg + 1;
            end

            fifo_half_full_reg <= $unsigned(fifo_wr_ptr_reg - fifo_rd_ptr_reg) >= 2**(FIFO_ADDR_WIDTH-1);

            if (rst) begin
                fifo_wr_ptr_reg <= 0;
                fifo_rd_ptr_reg <= 0;
            end
        end

        // 地址仲裁
        wire [S_COUNT-1:0] a_request; // 来自目标该主端从端请求向量。
        wire [S_COUNT-1:0] a_acknowledge; // AR 握手成功时仲裁确认位。
        wire [S_COUNT-1:0] a_grant; // 仲裁 one-hot 授予向量。
        wire a_grant_valid; // 仲裁器产生有效获胜者时置位。
        wire [CL_S_COUNT-1:0] a_grant_encoded; // 获胜从端编码索引。

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
        wire [ADDR_WIDTH-1:0]  s_axil_araddr_mux   = int_s_axil_araddr[a_grant_encoded*ADDR_WIDTH +: ADDR_WIDTH]; // 来自获胜源从端的 AR 地址。
        wire [2:0]             s_axil_arprot_mux   = int_s_axil_arprot[a_grant_encoded*3 +: 3]; // 来自获胜源从端的 AR 保护位。
        wire                   s_axil_arvalid_mux  = int_axil_arvalid[a_grant_encoded*M_COUNT+n] && a_grant_valid; // 送往该主端的复用 ARVALID。
        wire                   s_axil_arready_mux; // M 侧寄存器切片返回的 ARREADY。

        assign int_axil_arready[n*S_COUNT +: S_COUNT] = (a_grant_valid && s_axil_arready_mux) << a_grant_encoded;

        for (m = 0; m < S_COUNT; m = m + 1) begin
            assign a_request[m] = int_axil_arvalid[m*M_COUNT+n] && !a_grant[m] && !fifo_half_full_reg;
            assign a_acknowledge[m] = a_grant[m] && int_axil_arvalid[m*M_COUNT+n] && s_axil_arready_mux;
        end

        assign fifo_wr_select = a_grant_encoded;
        assign fifo_wr_en = s_axil_arvalid_mux && s_axil_arready_mux && a_grant_valid;

        // 读响应回传
        wire [CL_S_COUNT-1:0] r_select = S_COUNT > 1 ? fifo_select[fifo_rd_ptr_reg[FIFO_ADDR_WIDTH-1:0]] : 0; // 应接收当前返回 R 响应的源从端。

        assign int_axil_rvalid[n*S_COUNT +: S_COUNT] = int_m_axil_rvalid[n] << r_select;
        assign int_m_axil_rready[n] = int_axil_rready[r_select*M_COUNT+n];

        assign fifo_rd_en = int_m_axil_rvalid[n] && int_m_axil_rready[n];

        // M 侧寄存器切片
        axil_register_rd #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .AR_REG_TYPE(M_AR_REG_TYPE[n*2 +: 2]),
            .R_REG_TYPE(M_R_REG_TYPE[n*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axil_araddr(s_axil_araddr_mux),
            .s_axil_arprot(s_axil_arprot_mux),
            .s_axil_arvalid(s_axil_arvalid_mux),
            .s_axil_arready(s_axil_arready_mux),
            .s_axil_rdata(int_m_axil_rdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .s_axil_rresp(int_m_axil_rresp[n*2 +: 2]),
            .s_axil_rvalid(int_m_axil_rvalid[n]),
            .s_axil_rready(int_m_axil_rready[n]),
            .m_axil_araddr(m_axil_araddr[n*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axil_arprot(m_axil_arprot[n*3 +: 3]),
            .m_axil_arvalid(m_axil_arvalid[n]),
            .m_axil_arready(m_axil_arready[n]),
            .m_axil_rdata(m_axil_rdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .m_axil_rresp(m_axil_rresp[n*2 +: 2]),
            .m_axil_rvalid(m_axil_rvalid[n]),
            .m_axil_rready(m_axil_rready[n])
        );
    end // 主端接口循环

endgenerate

endmodule

`resetall
