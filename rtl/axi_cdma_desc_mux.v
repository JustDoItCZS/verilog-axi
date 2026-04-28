/*

Copyright (c) 2019 Alex Forencich

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
 * AXI CDMA 描述符多路复用模块
 *
 * 模块目录
 * 1) 多路 CDMA 描述符输入仲裁为一路输出给 CDMA 核心。
 * 2) 在输出 tag 高位拼接端口号，实现状态回送分发。
 * 3) 含输出级+临时级缓存，缓解下游 backpressure。
 */
module axi_cdma_desc_mux #
(
    // 端口数量
    parameter PORTS = 2,
    // AXI 地址位宽
    parameter AXI_ADDR_WIDTH = 16,
    // 长度字段位宽
    parameter LEN_WIDTH = 20,
    // 输入 tag 字段位宽
    parameter S_TAG_WIDTH = 8,
    // 输出 tag 字段位宽（发往 CDMA 核心）
    // 需要额外位用于状态回送路由
    parameter M_TAG_WIDTH = S_TAG_WIDTH+$clog2(PORTS),
    // 是否选择轮询仲裁
    parameter ARB_TYPE_ROUND_ROBIN = 1,
    // 最低位优先级选择
    parameter ARB_LSB_HIGH_PRIORITY = 1
)
(
    input  wire                            clk, // 模块时钟。
    input  wire                            rst, // 同步复位，高电平有效。

    /*
     * 描述符输出（到 AXI CDMA 核心）
     */
    output wire [AXI_ADDR_WIDTH-1:0]       m_axis_desc_read_addr, // 输出到 CDMA 核心的源地址。
    output wire [AXI_ADDR_WIDTH-1:0]       m_axis_desc_write_addr, // 输出到 CDMA 核心的目的地址。
    output wire [LEN_WIDTH-1:0]            m_axis_desc_len, // 输出到 CDMA 核心的搬运长度。
    output wire [M_TAG_WIDTH-1:0]          m_axis_desc_tag, // 输出到 CDMA 核心的扩展 tag(含源端口号)。
    output wire                            m_axis_desc_valid, // 输出描述符有效。
    input  wire                            m_axis_desc_ready, // CDMA 核心接收描述符 ready。

    /*
     * 描述符状态输入（来自 AXI CDMA 核心）
     */
    input  wire [M_TAG_WIDTH-1:0]          s_axis_desc_status_tag, // CDMA 核心返回的扩展 tag。
    input  wire [3:0]                      s_axis_desc_status_error, // CDMA 核心返回的错误码。
    input  wire                            s_axis_desc_status_valid, // CDMA 核心返回状态有效。

    /*
     * 描述符输入
     */
    input  wire [PORTS*AXI_ADDR_WIDTH-1:0] s_axis_desc_read_addr, // 各输入端口拼接的源地址。
    input  wire [PORTS*AXI_ADDR_WIDTH-1:0] s_axis_desc_write_addr, // 各输入端口拼接的目的地址。
    input  wire [PORTS*LEN_WIDTH-1:0]      s_axis_desc_len, // 各输入端口拼接的长度。
    input  wire [PORTS*S_TAG_WIDTH-1:0]    s_axis_desc_tag, // 各输入端口拼接的原始 tag。
    input  wire [PORTS-1:0]                s_axis_desc_valid, // 各输入端口描述符有效。
    output wire [PORTS-1:0]                s_axis_desc_ready, // 各输入端口描述符 ready。

    /*
     * 描述符状态输出
     */
    output wire [PORTS*S_TAG_WIDTH-1:0]    m_axis_desc_status_tag, // 广播到各端口的原始 tag(配合 valid 选通)。
    output wire [PORTS*4-1:0]              m_axis_desc_status_error, // 广播到各端口的错误码。
    output wire [PORTS-1:0]                m_axis_desc_status_valid // one-hot 状态有效，指示回送目标端口。
);

parameter CL_PORTS = $clog2(PORTS);

// 参数合法性检查
initial begin
    if (M_TAG_WIDTH < S_TAG_WIDTH+$clog2(PORTS)) begin
        $error("Error: M_TAG_WIDTH must be at least $clog2(PORTS) larger than S_TAG_WIDTH (instance %m)");
        $finish;
    end
end

// 描述符仲裁与复用
wire [PORTS-1:0] request; // 仲裁请求向量。
wire [PORTS-1:0] acknowledge; // 仲裁完成应答向量。
wire [PORTS-1:0] grant; // 仲裁授权 one-hot 向量。
wire grant_valid; // 是否存在有效授权端口。
wire [CL_PORTS-1:0] grant_encoded; // 仲裁授权编码值(端口号)。

// 内部数据通路
reg  [AXI_ADDR_WIDTH-1:0] m_axis_desc_read_addr_int; // 内部待输出读地址。
reg  [AXI_ADDR_WIDTH-1:0] m_axis_desc_write_addr_int; // 内部待输出写地址。
reg  [LEN_WIDTH-1:0]      m_axis_desc_len_int; // 内部待输出长度。
reg  [M_TAG_WIDTH-1:0]    m_axis_desc_tag_int; // 内部待输出扩展 tag。
reg                       m_axis_desc_valid_int; // 内部待输出有效。
reg                       m_axis_desc_ready_int_reg = 1'b0; // 内部输入端 ready 寄存器。
wire                      m_axis_desc_ready_int_early; // 内部输入端 ready 组合预测。

assign s_axis_desc_ready = (m_axis_desc_ready_int_reg && grant_valid) << grant_encoded;

// 输入描述符选择
wire [AXI_ADDR_WIDTH-1:0] current_s_desc_read_addr   = s_axis_desc_read_addr[grant_encoded*AXI_ADDR_WIDTH +: AXI_ADDR_WIDTH]; // 当前获授权端口的读地址。
wire [AXI_ADDR_WIDTH-1:0] current_s_desc_write_addr  = s_axis_desc_write_addr[grant_encoded*AXI_ADDR_WIDTH +: AXI_ADDR_WIDTH]; // 当前获授权端口的写地址。
wire [LEN_WIDTH-1:0]      current_s_desc_len         = s_axis_desc_len[grant_encoded*LEN_WIDTH +: LEN_WIDTH]; // 当前获授权端口的长度。
wire [S_TAG_WIDTH-1:0]    current_s_desc_tag         = s_axis_desc_tag[grant_encoded*S_TAG_WIDTH +: S_TAG_WIDTH]; // 当前获授权端口的原始 tag。
wire                      current_s_desc_valid       = s_axis_desc_valid[grant_encoded]; // 当前获授权端口 valid。
wire                      current_s_desc_ready       = s_axis_desc_ready[grant_encoded]; // 当前获授权端口 ready。

// 仲裁器实例
arbiter #(
    .PORTS(PORTS),
    .ARB_TYPE_ROUND_ROBIN(ARB_TYPE_ROUND_ROBIN),
    .ARB_BLOCK(1),
    .ARB_BLOCK_ACK(1),
    .ARB_LSB_HIGH_PRIORITY(ARB_LSB_HIGH_PRIORITY)
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

assign request = s_axis_desc_valid & ~grant;
assign acknowledge = grant & s_axis_desc_valid & s_axis_desc_ready;

always @* begin
    m_axis_desc_read_addr_int   = current_s_desc_read_addr;
    m_axis_desc_write_addr_int  = current_s_desc_write_addr;
    m_axis_desc_len_int         = current_s_desc_len;
    m_axis_desc_tag_int         = {grant_encoded, current_s_desc_tag};
    m_axis_desc_valid_int       = current_s_desc_valid && m_axis_desc_ready_int_reg && grant_valid;
end

// 输出数据通路逻辑
reg [AXI_ADDR_WIDTH-1:0]  m_axis_desc_read_addr_reg   = {AXI_ADDR_WIDTH{1'b0}}; // 主输出寄存器：读地址。
reg [AXI_ADDR_WIDTH-1:0]  m_axis_desc_write_addr_reg  = {AXI_ADDR_WIDTH{1'b0}}; // 主输出寄存器：写地址。
reg [LEN_WIDTH-1:0]       m_axis_desc_len_reg         = {LEN_WIDTH{1'b0}}; // 主输出寄存器：长度。
reg [M_TAG_WIDTH-1:0]     m_axis_desc_tag_reg         = {M_TAG_WIDTH{1'b0}}; // 主输出寄存器：扩展 tag。
reg                       m_axis_desc_valid_reg       = 1'b0, m_axis_desc_valid_next; // 主输出寄存器 valid。

reg [AXI_ADDR_WIDTH-1:0]  temp_m_axis_desc_read_addr_reg   = {AXI_ADDR_WIDTH{1'b0}}; // 临时寄存器：读地址。
reg [AXI_ADDR_WIDTH-1:0]  temp_m_axis_desc_write_addr_reg  = {AXI_ADDR_WIDTH{1'b0}}; // 临时寄存器：写地址。
reg [LEN_WIDTH-1:0]       temp_m_axis_desc_len_reg         = {LEN_WIDTH{1'b0}}; // 临时寄存器：长度。
reg [M_TAG_WIDTH-1:0]     temp_m_axis_desc_tag_reg         = {M_TAG_WIDTH{1'b0}}; // 临时寄存器：扩展 tag。
reg                       temp_m_axis_desc_valid_reg       = 1'b0, temp_m_axis_desc_valid_next; // 临时寄存器 valid。

// 数据通路控制
reg store_axis_int_to_output; // 将内部输入直接写入主输出寄存器。
reg store_axis_int_to_temp; // 将内部输入写入临时寄存器。
reg store_axis_temp_to_output; // 将临时寄存器回填主输出寄存器。

assign m_axis_desc_read_addr   = m_axis_desc_read_addr_reg;
assign m_axis_desc_write_addr  = m_axis_desc_write_addr_reg;
assign m_axis_desc_len         = m_axis_desc_len_reg;
assign m_axis_desc_tag         = m_axis_desc_tag_reg;
assign m_axis_desc_valid       = m_axis_desc_valid_reg;

// 下拍 ready 预判：输出可接收，或下拍临时寄存器不会被占用时拉高
assign m_axis_desc_ready_int_early = m_axis_desc_ready || (!temp_m_axis_desc_valid_reg && (!m_axis_desc_valid_reg || !m_axis_desc_valid_int));

always @* begin
    // 将下游就绪关系映射到上游
    m_axis_desc_valid_next = m_axis_desc_valid_reg;
    temp_m_axis_desc_valid_next = temp_m_axis_desc_valid_reg;

    store_axis_int_to_output = 1'b0;
    store_axis_int_to_temp = 1'b0;
    store_axis_temp_to_output = 1'b0;

    if (m_axis_desc_ready_int_reg) begin
        // 当前允许接收输入
        if (m_axis_desc_ready || !m_axis_desc_valid_reg) begin
            // 输出可接收，或当前输出无效：输入直写主输出
            m_axis_desc_valid_next = m_axis_desc_valid_int;
            store_axis_int_to_output = 1'b1;
        end else begin
            // 输出阻塞：输入写入临时寄存器
            temp_m_axis_desc_valid_next = m_axis_desc_valid_int;
            store_axis_int_to_temp = 1'b1;
        end
    end else if (m_axis_desc_ready) begin
        // 当前不接收新输入，但输出可接收：临时寄存器回放
        m_axis_desc_valid_next = temp_m_axis_desc_valid_reg;
        temp_m_axis_desc_valid_next = 1'b0;
        store_axis_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_axis_desc_valid_reg <= 1'b0;
        m_axis_desc_ready_int_reg <= 1'b0;
        temp_m_axis_desc_valid_reg <= 1'b0;
    end else begin
        m_axis_desc_valid_reg <= m_axis_desc_valid_next;
        m_axis_desc_ready_int_reg <= m_axis_desc_ready_int_early;
        temp_m_axis_desc_valid_reg <= temp_m_axis_desc_valid_next;
    end

    // 数据通路寄存
    if (store_axis_int_to_output) begin
        m_axis_desc_read_addr_reg <= m_axis_desc_read_addr_int;
        m_axis_desc_write_addr_reg <= m_axis_desc_write_addr_int;
        m_axis_desc_len_reg <= m_axis_desc_len_int;
        m_axis_desc_tag_reg <= m_axis_desc_tag_int;
    end else if (store_axis_temp_to_output) begin
        m_axis_desc_read_addr_reg <= temp_m_axis_desc_read_addr_reg;
        m_axis_desc_write_addr_reg <= temp_m_axis_desc_write_addr_reg;
        m_axis_desc_len_reg <= temp_m_axis_desc_len_reg;
        m_axis_desc_tag_reg <= temp_m_axis_desc_tag_reg;
    end

    if (store_axis_int_to_temp) begin
        temp_m_axis_desc_read_addr_reg <= m_axis_desc_read_addr_int;
        temp_m_axis_desc_write_addr_reg <= m_axis_desc_write_addr_int;
        temp_m_axis_desc_len_reg <= m_axis_desc_len_int;
        temp_m_axis_desc_tag_reg <= m_axis_desc_tag_int;
    end
end

// 描述符状态解复用
reg [S_TAG_WIDTH-1:0] m_axis_desc_status_tag_reg = {S_TAG_WIDTH{1'b0}}; // 最近一次完成状态原始 tag 缓存。
reg [3:0] m_axis_desc_status_error_reg = 4'd0; // 最近一次完成状态错误码缓存。
reg [PORTS-1:0] m_axis_desc_status_valid_reg = {PORTS{1'b0}}; // 完成状态 one-hot 有效位。

assign m_axis_desc_status_tag = {PORTS{m_axis_desc_status_tag_reg}};
assign m_axis_desc_status_error = {PORTS{m_axis_desc_status_error_reg}};
assign m_axis_desc_status_valid = m_axis_desc_status_valid_reg;

always @(posedge clk) begin
    if (rst) begin
        m_axis_desc_status_valid_reg <= {PORTS{1'b0}};
    end else begin
        m_axis_desc_status_valid_reg <= s_axis_desc_status_valid << (PORTS > 1 ? s_axis_desc_status_tag[S_TAG_WIDTH+CL_PORTS-1:S_TAG_WIDTH] : 0);
    end

    m_axis_desc_status_tag_reg <= s_axis_desc_status_tag;
    m_axis_desc_status_error_reg <= s_axis_desc_status_error;
end

endmodule

`resetall
