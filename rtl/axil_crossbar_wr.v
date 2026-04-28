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
 * AXI4-Lite 交叉开关（写通道）
 *
 * 模块目录
 * 1) 从端侧：
 *    - 将每个 AW 地址解码到唯一目标主端口
 *    - 将对应 W 数据拍与解码结果配对转发
 *    - 按顺序把 B 响应返回给发起该请求的从端
 * 2) 主端侧：
 *    - 对每个主端口来自多个从端的 AW/W 请求进行仲裁
 *    - 维护每主端口响应路由 FIFO，把 B 返回到正确来源
 * 3) 通过 axil_register_wr 在各端口插入可选寄存器切片。
 */
module axil_crossbar_wr #
(
    // AXI 输入端口数量（从接口数量）
    parameter S_COUNT = 4,
    // AXI 输出端口数量（主接口数量）
    parameter M_COUNT = 4,
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节 lane 计算）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 每个从接口可并发处理的事务数量
    // 格式：S_COUNT 个 32 位字段拼接
    parameter S_ACCEPT = {S_COUNT{32'd16}},
    // 每个主接口的地址区域数量
    parameter M_REGIONS = 1,
    // 主接口基地址表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 ADDR_WIDTH 位字段
    // 置 0 时按 M_ADDR_WIDTH 自动生成默认地址划分
    parameter M_BASE_ADDR = 0,
    // 主接口地址宽度表
    // 格式：M_COUNT 组，每组含 M_REGIONS 个 32 位字段
    parameter M_ADDR_WIDTH = {M_COUNT{{M_REGIONS{32'd24}}}},
    // 接口间写通路连通矩阵
    // 格式：M_COUNT 组，每组 S_COUNT 位
    parameter M_CONNECT = {M_COUNT{{S_COUNT{1'b1}}}},
    // 每个主接口可并发发起的事务数量
    // 格式：M_COUNT 个 32 位字段拼接
    parameter M_ISSUE = {M_COUNT{32'd16}},
    // 安全主端口配置（按 awprot/arprot 拒绝非法访问）
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
    input  wire                             clk, // 写交叉开关时钟。
    input  wire                             rst, // 同步复位，清空仲裁与 FIFO 状态。

    /*
     * AXI-Lite 从接口
     */
    input  wire [S_COUNT*ADDR_WIDTH-1:0]    s_axil_awaddr, // 从端 AW 地址向量；每个从端占一个切片。
    input  wire [S_COUNT*3-1:0]             s_axil_awprot, // 从端 AW 保护属性。
    input  wire [S_COUNT-1:0]               s_axil_awvalid, // 从端 AWVALID 向量；各位随从端请求变化。
    output wire [S_COUNT-1:0]               s_axil_awready, // 从端 AWREADY 向量；地址解码并可接纳时返回。
    input  wire [S_COUNT*DATA_WIDTH-1:0]    s_axil_wdata, // 从端写数据向量。
    input  wire [S_COUNT*STRB_WIDTH-1:0]    s_axil_wstrb, // 从端字节写使能向量。
    input  wire [S_COUNT-1:0]               s_axil_wvalid, // 从端 WVALID 向量。
    output wire [S_COUNT-1:0]               s_axil_wready, // 从端 WREADY 向量；与目标主端写数据接纳能力关联。
    output wire [S_COUNT*2-1:0]             s_axil_bresp, // 从端 BRESP 向量，含本地合成 DECERR。
    output wire [S_COUNT-1:0]               s_axil_bvalid, // 从端 BVALID 向量；路由到响应时拉高。
    input  wire [S_COUNT-1:0]               s_axil_bready, // 从端 BREADY 向量；用于消费响应。

    /*
     * AXI-Lite 主接口
     */
    output wire [M_COUNT*ADDR_WIDTH-1:0]    m_axil_awaddr, // 主端 AW 地址向量（仲裁后输出）。
    output wire [M_COUNT*3-1:0]             m_axil_awprot, // 主端 AW 保护属性。
    output wire [M_COUNT-1:0]               m_axil_awvalid, // 主端 AWVALID 向量；每个主端一路请求流。
    input  wire [M_COUNT-1:0]               m_axil_awready, // 下游目标返回的主端 AWREADY 向量。
    output wire [M_COUNT*DATA_WIDTH-1:0]    m_axil_wdata, // 主端写数据向量。
    output wire [M_COUNT*STRB_WIDTH-1:0]    m_axil_wstrb, // 主端写字节使能向量。
    output wire [M_COUNT-1:0]               m_axil_wvalid, // 主端 WVALID 向量。
    input  wire [M_COUNT-1:0]               m_axil_wready, // 主端 WREADY 向量；用于释放已选源从端。
    input  wire [M_COUNT*2-1:0]             m_axil_bresp, // 目标返回的主端 BRESP 向量。
    input  wire [M_COUNT-1:0]               m_axil_bvalid, // 目标返回的主端 BVALID 向量。
    output wire [M_COUNT-1:0]               m_axil_bready // 发送给目标的主端 BREADY 向量。
);

parameter CL_S_COUNT = $clog2(S_COUNT); // 编码从端索引所需位宽。
parameter CL_M_COUNT = $clog2(M_COUNT); // 编码主端索引所需位宽。
parameter M_COUNT_P1 = M_COUNT+1; // 主端数量加 1（额外 DECERR 槽位）。
parameter CL_M_COUNT_P1 = $clog2(M_COUNT_P1); // M_COUNT_P1 路径的编码位宽。

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

wire [S_COUNT*ADDR_WIDTH-1:0]    int_s_axil_awaddr; // 内部从端 AW 地址总线（经过可选 S 侧寄存器）。
wire [S_COUNT*3-1:0]             int_s_axil_awprot; // 内部从端 AW 保护属性总线。
wire [S_COUNT-1:0]               int_s_axil_awvalid; // 进入地址解码模块的内部 AWVALID。
wire [S_COUNT-1:0]               int_s_axil_awready; // 地址解码模块返回的内部 AWREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axil_awvalid; // 交叉点 AWVALID 矩阵 [slave][master]。
wire [M_COUNT*S_COUNT-1:0]       int_axil_awready; // 交叉点 AWREADY 矩阵 [master][slave]。

wire [S_COUNT*DATA_WIDTH-1:0]    int_s_axil_wdata; // 内部从端写数据向量。
wire [S_COUNT*STRB_WIDTH-1:0]    int_s_axil_wstrb; // 内部从端写字节使能向量。
wire [S_COUNT-1:0]               int_s_axil_wvalid; // 内部从端 WVALID。
wire [S_COUNT-1:0]               int_s_axil_wready; // 目的端匹配后生成的内部从端 WREADY。

wire [S_COUNT*M_COUNT-1:0]       int_axil_wvalid; // 交叉点 WVALID 矩阵 [slave][master]。
wire [M_COUNT*S_COUNT-1:0]       int_axil_wready; // 交叉点 WREADY 矩阵 [master][slave]。

wire [M_COUNT*2-1:0]             int_m_axil_bresp; // 内部主端 BRESP 向量（经过 M 侧寄存器）。
wire [M_COUNT-1:0]               int_m_axil_bvalid; // 内部主端 BVALID。
wire [M_COUNT-1:0]               int_m_axil_bready; // 内部主端 BREADY。

wire [M_COUNT*S_COUNT-1:0]       int_axil_bvalid; // 回程路由用 BVALID 交叉矩阵 [master][slave]。
wire [S_COUNT*M_COUNT-1:0]       int_axil_bready; // BREADY 交叉矩阵 [slave][master]。

generate

    genvar m, n;

    for (m = 0; m < S_COUNT; m = m + 1) begin : s_ifaces
        // 响应路由 FIFO
        localparam FIFO_ADDR_WIDTH = $clog2(S_ACCEPT[m*32 +: 32])+1; // 每个从端响应路由 FIFO 的深度控制位宽。

        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_wr_ptr_reg = 0; // 写指针：解码请求入队时前进。
        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_rd_ptr_reg = 0; // 读指针：路由 B 响应被消费时前进。

        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg [CL_M_COUNT-1:0] fifo_select[(2**FIFO_ADDR_WIDTH)-1:0]; // 为每个在途请求记录目标主端口 ID。
        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg fifo_decerr[(2**FIFO_ADDR_WIDTH)-1:0]; // 记录地址解码错误位，用于本地生成 DECERR。

        wire [CL_M_COUNT-1:0] fifo_wr_select; // FIFO 写入内容：选中的目标主端口。
        wire fifo_wr_decerr; // FIFO 写入内容：当前请求的解码错误标记。
        wire fifo_wr_en; // FIFO 写使能：响应元数据被接纳时拉高。

        reg [CL_M_COUNT-1:0] fifo_rd_select_reg = 0; // 锁存的 FIFO 读出目标选择，用于 B 响应复用。
        reg fifo_rd_decerr_reg = 0; // 锁存的 FIFO 读出解码错误位。
        reg fifo_rd_valid_reg = 0; // 指示 fifo_rd_* 当前持有有效路由元数据。
        wire fifo_rd_en; // FIFO 出队使能：从端 B 握手完成时拉高。
        reg fifo_half_full_reg = 1'b0; // 半满阈值标志：占用过高时阻止新元数据进入。

        wire fifo_empty = fifo_rd_ptr_reg == fifo_wr_ptr_reg; // FIFO 空标志：用于控制是否预取元数据。

        integer i; // 从端 FIFO RAM 初始化循环变量。

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
        wire [CL_M_COUNT-1:0] a_select; // 当前从端 AW 请求解码得到的目标主端口。

        wire m_axil_avalid; // 解码后 AW 请求输出有效。
        wire m_axil_aready; // 选中主端 AW 交叉点返回的 ready。

        wire [CL_M_COUNT-1:0] m_wc_select; // 写命令选择的主端口（用于 W 通路配对）。
        wire m_wc_decerr; // 写命令解码错误标记（丢弃 W，并回送 DECERR B）。
        wire m_wc_valid; // 地址解码阶段输出的写命令元数据有效。
        wire m_wc_ready; // 本地 W 通路配对逻辑的就绪信号。

        wire [CL_M_COUNT-1:0] m_rc_select; // 响应命令选择的主端口（写入 B 路由队列）。
        wire m_rc_decerr; // 响应命令解码错误标记。
        wire m_rc_valid; // 地址解码阶段输出的响应命令元数据有效。
        wire m_rc_ready; // 本地响应路由 FIFO 写入端就绪。

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
            .WC_OUTPUT(1)
        )
        addr_inst (
            .clk(clk),
            .rst(rst),

            /*
             * 地址输入
             */
            .s_axil_aaddr(int_s_axil_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axil_aprot(int_s_axil_awprot[m*3 +: 3]),
            .s_axil_avalid(int_s_axil_awvalid[m]),
            .s_axil_aready(int_s_axil_awready[m]),

            /*
             * 地址输出
             */
            .m_select(a_select),
            .m_axil_avalid(m_axil_avalid),
            .m_axil_aready(m_axil_aready),

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
            .m_rc_select(m_rc_select),
            .m_rc_decerr(m_rc_decerr),
            .m_rc_valid(m_rc_valid),
            .m_rc_ready(m_rc_ready)
        );

        assign int_axil_awvalid[m*M_COUNT +: M_COUNT] = m_axil_avalid << a_select;
        assign m_axil_aready = int_axil_awready[a_select*S_COUNT+m];

        // 写命令处理
        reg [CL_M_COUNT-1:0] w_select_reg = 0, w_select_next; // 当前为进入 W 数据拍选择的目标主端口。
        reg w_drop_reg = 1'b0, w_drop_next; // 当前 W 数据流是否标记为解码错误（接收后丢弃）。
        reg w_select_valid_reg = 1'b0, w_select_valid_next; // 指示 w_select_reg 当前是否对应有效在途命令。

        assign m_wc_ready = !w_select_valid_reg;

        always @* begin
            w_select_next = w_select_reg;
            w_drop_next = w_drop_reg && !(int_s_axil_wvalid[m] && int_s_axil_wready[m]);
            w_select_valid_next = w_select_valid_reg && !(int_s_axil_wvalid[m] && int_s_axil_wready[m]);

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
        assign int_axil_wvalid[m*M_COUNT +: M_COUNT] = (int_s_axil_wvalid[m] && w_select_valid_reg && !w_drop_reg) << w_select_reg;
        assign int_s_axil_wready[m] = int_axil_wready[w_select_reg*S_COUNT+m] || w_drop_reg;

        // 响应处理
        assign fifo_wr_select = m_rc_select;
        assign fifo_wr_decerr = m_rc_decerr;
        assign fifo_wr_en = m_rc_valid && !fifo_half_full_reg;
        assign m_rc_ready = !fifo_half_full_reg;

        // 写响应处理
        wire [CL_M_COUNT-1:0] b_select = M_COUNT > 1 ? fifo_rd_select_reg : 0; // 当前从端应当接收响应的来源主端口。
        wire b_decerr = fifo_rd_decerr_reg; // 置位时本地合成 DECERR 响应。
        wire b_valid = fifo_rd_valid_reg; // B 响应复用所需元数据有效。

        // 写响应复用
        wire [1:0]  m_axil_bresp_mux  = b_decerr ? 2'b11 : int_m_axil_bresp[b_select*2 +: 2]; // 从选中主端口路由 BRESP，或强制 DECERR。
        wire        m_axil_bvalid_mux = (b_decerr ? 1'b1 : int_axil_bvalid[b_select*S_COUNT+m]) && b_valid; // 选中主端返回或注入 DECERR 时 BVALID 有效。
        wire        m_axil_bready_mux; // 当前从端 S 侧寄存器切片返回的 BREADY。

        assign int_axil_bready[m*M_COUNT +: M_COUNT] = (b_valid && m_axil_bready_mux) << b_select;

        assign fifo_rd_en = m_axil_bvalid_mux && m_axil_bready_mux && b_valid;

        // S 侧寄存器切片
        axil_register_wr #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .AW_REG_TYPE(S_AW_REG_TYPE[m*2 +: 2]),
            .W_REG_TYPE(S_W_REG_TYPE[m*2 +: 2]),
            .B_REG_TYPE(S_B_REG_TYPE[m*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axil_awaddr(s_axil_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .s_axil_awprot(s_axil_awprot[m*3 +: 3]),
            .s_axil_awvalid(s_axil_awvalid[m]),
            .s_axil_awready(s_axil_awready[m]),
            .s_axil_wdata(s_axil_wdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .s_axil_wstrb(s_axil_wstrb[m*STRB_WIDTH +: STRB_WIDTH]),
            .s_axil_wvalid(s_axil_wvalid[m]),
            .s_axil_wready(s_axil_wready[m]),
            .s_axil_bresp(s_axil_bresp[m*2 +: 2]),
            .s_axil_bvalid(s_axil_bvalid[m]),
            .s_axil_bready(s_axil_bready[m]),
            .m_axil_awaddr(int_s_axil_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axil_awprot(int_s_axil_awprot[m*3 +: 3]),
            .m_axil_awvalid(int_s_axil_awvalid[m]),
            .m_axil_awready(int_s_axil_awready[m]),
            .m_axil_wdata(int_s_axil_wdata[m*DATA_WIDTH +: DATA_WIDTH]),
            .m_axil_wstrb(int_s_axil_wstrb[m*STRB_WIDTH +: STRB_WIDTH]),
            .m_axil_wvalid(int_s_axil_wvalid[m]),
            .m_axil_wready(int_s_axil_wready[m]),
            .m_axil_bresp(m_axil_bresp_mux),
            .m_axil_bvalid(m_axil_bvalid_mux),
            .m_axil_bready(m_axil_bready_mux)
        );
    end // 从端接口循环

    for (n = 0; n < M_COUNT; n = n + 1) begin : m_ifaces
        // 响应路由 FIFO
        localparam FIFO_ADDR_WIDTH = $clog2(M_ISSUE[n*32 +: 32])+1; // 每个主端口来源跟踪 FIFO 的深度控制位宽。

        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_wr_ptr_reg = 0; // 写指针：某个从端 AW 被接纳时前进。
        reg [FIFO_ADDR_WIDTH+1-1:0] fifo_rd_ptr_reg = 0; // 读指针：主端 B 被接纳时前进。

        (* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
        reg [CL_S_COUNT-1:0] fifo_select[(2**FIFO_ADDR_WIDTH)-1:0]; // 为每个路由到该主端的在途写事务记录来源从端 ID。
        wire [CL_S_COUNT-1:0] fifo_wr_select; // FIFO 写入内容：本次仲裁授予的来源从端 ID。
        wire fifo_wr_en; // FIFO 写使能：AW 接纳成功时拉高。
        wire fifo_rd_en; // FIFO 读使能：B 接纳成功时拉高。
        reg fifo_half_full_reg = 1'b0; // 高水位标志：用于节流新授予请求。

        wire fifo_empty = fifo_rd_ptr_reg == fifo_wr_ptr_reg; // FIFO 空标志（无待返回响应路由条目）。

        integer i; // 每主端 FIFO RAM 初始化循环变量。

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
        reg [CL_S_COUNT-1:0] w_select_reg = 0, w_select_next = 0; // 当前向该主端提供 W 数据的来源从端。
        reg w_select_valid_reg = 1'b0, w_select_valid_next; // 指示 w_select_reg 是否指向有效数据流。
        reg w_select_new_reg = 1'b0, w_select_new_next; // 允许在地址授予后装载新的 W 来源。

        wire [S_COUNT-1:0] a_request; // 仲裁请求位：当前瞄准该主端口的从端。
        wire [S_COUNT-1:0] a_acknowledge; // AW 握手成功时的仲裁确认位。
        wire [S_COUNT-1:0] a_grant; // 每从端 one-hot 授予结果。
        wire a_grant_valid; // 仲裁器存在有效获胜者时置位。
        wire [CL_S_COUNT-1:0] a_grant_encoded; // 获胜从端的编码索引。

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
        wire [ADDR_WIDTH-1:0]  s_axil_awaddr_mux   = int_s_axil_awaddr[a_grant_encoded*ADDR_WIDTH +: ADDR_WIDTH]; // 来自获胜从端的 AW 地址。
        wire [2:0]             s_axil_awprot_mux   = int_s_axil_awprot[a_grant_encoded*3 +: 3]; // 来自获胜从端的 AW 保护属性。
        wire                   s_axil_awvalid_mux  = int_axil_awvalid[a_grant_encoded*M_COUNT+n] && a_grant_valid; // 送往该主端的复用 AWVALID。
        wire                   s_axil_awready_mux; // M 侧寄存器切片返回的 AWREADY。

        assign int_axil_awready[n*S_COUNT +: S_COUNT] = (a_grant_valid && s_axil_awready_mux) << a_grant_encoded;

        for (m = 0; m < S_COUNT; m = m + 1) begin
            assign a_request[m] = int_axil_awvalid[m*M_COUNT+n] && !a_grant[m] && !fifo_half_full_reg && !w_select_valid_next;
            assign a_acknowledge[m] = a_grant[m] && int_axil_awvalid[m*M_COUNT+n] && s_axil_awready_mux;
        end

        assign fifo_wr_select = a_grant_encoded;
        assign fifo_wr_en = s_axil_awvalid_mux && s_axil_awready_mux && a_grant_valid;

        // 写数据复用
        wire [DATA_WIDTH-1:0]  s_axil_wdata_mux   = int_s_axil_wdata[w_select_reg*DATA_WIDTH +: DATA_WIDTH]; // 来自选中来源从端的 W 数据。
        wire [STRB_WIDTH-1:0]  s_axil_wstrb_mux   = int_s_axil_wstrb[w_select_reg*STRB_WIDTH +: STRB_WIDTH]; // 来自选中来源从端的 WSTRB。
        wire                   s_axil_wvalid_mux  = int_axil_wvalid[w_select_reg*M_COUNT+n] && w_select_valid_reg; // 送往该主端的复用 WVALID。
        wire                   s_axil_wready_mux; // M 侧寄存器切片返回的 WREADY。

        assign int_axil_wready[n*S_COUNT +: S_COUNT] = (w_select_valid_reg && s_axil_wready_mux) << w_select_reg;

        // 写数据路由
        always @* begin
            w_select_next = w_select_reg;
            w_select_valid_next = w_select_valid_reg && !(s_axil_wvalid_mux && s_axil_wready_mux);
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
        wire [CL_S_COUNT-1:0] b_select = S_COUNT > 1 ? fifo_select[fifo_rd_ptr_reg[FIFO_ADDR_WIDTH-1:0]] : 0; // 应接收该主端下一拍 B 响应的来源从端 ID。

        assign int_axil_bvalid[n*S_COUNT +: S_COUNT] = int_m_axil_bvalid[n] << b_select;
        assign int_m_axil_bready[n] = int_axil_bready[b_select*M_COUNT+n];

        assign fifo_rd_en = int_m_axil_bvalid[n] && int_m_axil_bready[n];

        // M 侧寄存器切片
        axil_register_wr #(
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH),
            .STRB_WIDTH(STRB_WIDTH),
            .AW_REG_TYPE(M_AW_REG_TYPE[n*2 +: 2]),
            .W_REG_TYPE(M_W_REG_TYPE[n*2 +: 2]),
            .B_REG_TYPE(M_B_REG_TYPE[n*2 +: 2])
        )
        reg_inst (
            .clk(clk),
            .rst(rst),
            .s_axil_awaddr(s_axil_awaddr_mux),
            .s_axil_awprot(s_axil_awprot_mux),
            .s_axil_awvalid(s_axil_awvalid_mux),
            .s_axil_awready(s_axil_awready_mux),
            .s_axil_wdata(s_axil_wdata_mux),
            .s_axil_wstrb(s_axil_wstrb_mux),
            .s_axil_wvalid(s_axil_wvalid_mux),
            .s_axil_wready(s_axil_wready_mux),
            .s_axil_bresp(int_m_axil_bresp[n*2 +: 2]),
            .s_axil_bvalid(int_m_axil_bvalid[n]),
            .s_axil_bready(int_m_axil_bready[n]),
            .m_axil_awaddr(m_axil_awaddr[n*ADDR_WIDTH +: ADDR_WIDTH]),
            .m_axil_awprot(m_axil_awprot[n*3 +: 3]),
            .m_axil_awvalid(m_axil_awvalid[n]),
            .m_axil_awready(m_axil_awready[n]),
            .m_axil_wdata(m_axil_wdata[n*DATA_WIDTH +: DATA_WIDTH]),
            .m_axil_wstrb(m_axil_wstrb[n*STRB_WIDTH +: STRB_WIDTH]),
            .m_axil_wvalid(m_axil_wvalid[n]),
            .m_axil_wready(m_axil_wready[n]),
            .m_axil_bresp(m_axil_bresp[n*2 +: 2]),
            .m_axil_bvalid(m_axil_bvalid[n]),
            .m_axil_bready(m_axil_bready[n])
        );
    end // 主端接口循环

endgenerate

endmodule

`resetall
