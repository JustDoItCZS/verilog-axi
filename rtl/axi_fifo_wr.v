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
 * AXI4 FIFO（写通道）
 *
 * 模块目录
 * 1) W 通道写入内部 FIFO，平衡上游写入节奏与下游写出节奏。
 * 2) 可选 `FIFO_DELAY` 模式下暂存 AW，等 W 数据达到条件后再放行地址。
 * 3) B 通道不缓存，直接从下游回传给上游。
 */
module axi_fifo_wr #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 AWUSER 信号
    parameter AWUSER_ENABLE = 0,
    // AWUSER 信号位宽
    parameter AWUSER_WIDTH = 1,
    // 是否透传 WUSER 信号
    parameter WUSER_ENABLE = 0,
    // WUSER 信号位宽
    parameter WUSER_WIDTH = 1,
    // 是否透传 BUSER 信号
    parameter BUSER_ENABLE = 0,
    // BUSER 信号位宽
    parameter BUSER_WIDTH = 1,
    // 写数据 FIFO 深度（拍）
    parameter FIFO_DEPTH = 32,
    // 尽可能等待写数据进入 FIFO 后再放行写地址
    parameter FIFO_DELAY = 0
)
(
    input  wire                     clk, // 时钟。
    input  wire                     rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // 上游写地址 ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // 上游写地址。
    input  wire [7:0]               s_axi_awlen, // 上游写突发长度。
    input  wire [2:0]               s_axi_awsize, // 上游写 beat 大小。
    input  wire [1:0]               s_axi_awburst, // 上游写突发类型。
    input  wire                     s_axi_awlock, // 上游写锁属性。
    input  wire [3:0]               s_axi_awcache, // 上游写 cache 属性。
    input  wire [2:0]               s_axi_awprot, // 上游写保护属性。
    input  wire [3:0]               s_axi_awqos, // 上游写 QoS。
    input  wire [3:0]               s_axi_awregion, // 上游写 region。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // 上游 AW user sideband。
    input  wire                     s_axi_awvalid, // 上游 AWVALID。
    output wire                     s_axi_awready, // 模块可接收上游写地址。
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata, // 上游写数据。
    input  wire [STRB_WIDTH-1:0]    s_axi_wstrb, // 上游写字节使能。
    input  wire                     s_axi_wlast, // 上游写突发最后一拍。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // 上游 W user sideband。
    input  wire                     s_axi_wvalid, // 上游 WVALID。
    output wire                     s_axi_wready, // FIFO 可接收写数据。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // 返回上游的写响应 ID。
    output wire [1:0]               s_axi_bresp, // 返回上游的写响应状态。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // 返回上游的 B user。
    output wire                     s_axi_bvalid, // 返回上游的 BVALID。
    input  wire                     s_axi_bready, // 上游 BREADY。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_awid, // 发送下游的写地址 ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr, // 发送下游的写地址。
    output wire [7:0]               m_axi_awlen, // 发送下游的写突发长度。
    output wire [2:0]               m_axi_awsize, // 发送下游的写 beat 大小。
    output wire [1:0]               m_axi_awburst, // 发送下游的写突发类型。
    output wire                     m_axi_awlock, // 发送下游的写锁属性。
    output wire [3:0]               m_axi_awcache, // 发送下游的写 cache 属性。
    output wire [2:0]               m_axi_awprot, // 发送下游的写保护属性。
    output wire [3:0]               m_axi_awqos, // 发送下游的写 QoS。
    output wire [3:0]               m_axi_awregion, // 发送下游的写 region。
    output wire [AWUSER_WIDTH-1:0]  m_axi_awuser, // 发送下游的 AW user。
    output wire                     m_axi_awvalid, // 发送下游的 AWVALID。
    input  wire                     m_axi_awready, // 下游 AWREADY。
    output wire [DATA_WIDTH-1:0]    m_axi_wdata, // 发送下游的写数据。
    output wire [STRB_WIDTH-1:0]    m_axi_wstrb, // 发送下游的写字节使能。
    output wire                     m_axi_wlast, // 发送下游的写突发最后一拍。
    output wire [WUSER_WIDTH-1:0]   m_axi_wuser, // 发送下游的 W user。
    output wire                     m_axi_wvalid, // 发送下游的 WVALID。
    input  wire                     m_axi_wready, // 下游 WREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_bid, // 下游写响应 ID。
    input  wire [1:0]               m_axi_bresp, // 下游写响应状态。
    input  wire [BUSER_WIDTH-1:0]   m_axi_buser, // 下游 B user。
    input  wire                     m_axi_bvalid, // 下游 BVALID。
    output wire                     m_axi_bready // 本模块对下游 B 通道 ready。
);

parameter STRB_OFFSET  = DATA_WIDTH; // 在 FIFO 打包总线中，wstrb 字段起始位。
parameter LAST_OFFSET  = STRB_OFFSET + STRB_WIDTH; // 在 FIFO 打包总线中，wlast 位位置。
parameter WUSER_OFFSET = LAST_OFFSET + 1; // 在 FIFO 打包总线中，wuser 字段起始位。
parameter WWIDTH       = WUSER_OFFSET + (WUSER_ENABLE ? WUSER_WIDTH : 0); // FIFO 每个条目的总位宽。

parameter FIFO_ADDR_WIDTH = $clog2(FIFO_DEPTH); // FIFO 地址位宽。

reg [FIFO_ADDR_WIDTH:0] wr_ptr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}, wr_ptr_next; // FIFO 写指针(含额外 MSB 用于满/空判断)。
reg [FIFO_ADDR_WIDTH:0] wr_addr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}; // 写 RAM 端口地址寄存器(与写指针对齐)。
reg [FIFO_ADDR_WIDTH:0] rd_ptr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}, rd_ptr_next; // FIFO 读指针(含额外 MSB 用于满/空判断)。
reg [FIFO_ADDR_WIDTH:0] rd_addr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}; // 读 RAM 端口地址寄存器(与读指针对齐)。

(* ramstyle = "no_rw_check" *)
reg [WWIDTH-1:0] mem[(2**FIFO_ADDR_WIDTH)-1:0]; // 写数据 FIFO 存储体。
reg [WWIDTH-1:0] mem_read_data_reg; // 从 FIFO RAM 读出的当前条目。
reg mem_read_data_valid_reg = 1'b0, mem_read_data_valid_next; // 读出条目有效标志。

wire [WWIDTH-1:0] s_axi_w; // 将 W 通道各字段打包后的 FIFO 写入数据。

reg [WWIDTH-1:0] m_axi_w_reg; // 输出给下游 W 通道的寄存器数据。
reg m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next; // 输出给下游 W 通道的 valid。

// FIFO 满条件：最高位不同且其余位相同
wire full = ((wr_ptr_reg[FIFO_ADDR_WIDTH] != rd_ptr_reg[FIFO_ADDR_WIDTH]) && // FIFO 满标志：高位不同且低位相同。
             (wr_ptr_reg[FIFO_ADDR_WIDTH-1:0] == rd_ptr_reg[FIFO_ADDR_WIDTH-1:0])); // FIFO 满标志：高位不同且低位相同。
// FIFO 空条件：读写指针完全一致
wire empty = wr_ptr_reg == rd_ptr_reg; // FIFO 空标志：读写指针完全相同。

wire hold; // 对 W 通道施加背压的保持信号(等待 AW 条件满足)。

// 控制信号
reg write; // FIFO RAM 写使能。
reg read; // FIFO RAM 读使能。
reg store_output; // 将读出条目装入输出寄存器的使能。

assign s_axi_wready = !full && !hold;

generate
    assign s_axi_w[DATA_WIDTH-1:0] = s_axi_wdata;
    assign s_axi_w[STRB_OFFSET +: STRB_WIDTH] = s_axi_wstrb;
    assign s_axi_w[LAST_OFFSET] = s_axi_wlast;
    if (WUSER_ENABLE) assign s_axi_w[WUSER_OFFSET +: WUSER_WIDTH] = s_axi_wuser;
endgenerate

generate

if (FIFO_DELAY) begin
    // 缓存 AW，直到对应 W 突发进入 FIFO（或 FIFO 达到阈值）

    localparam [1:0]
        STATE_IDLE = 2'd0, // 等待接收新的 AW。
        STATE_TRANSFER_IN = 2'd1, // 正在接收并缓存 W，尚未把 AW 发给下游。
        STATE_TRANSFER_OUT = 2'd2; // AW 已发，继续透传/缓存剩余 W 直到 WLAST。

    reg [1:0] state_reg = STATE_IDLE, state_next; // AW 延迟控制状态机。

    reg hold_reg = 1'b1, hold_next; // 对 W 通道的 hold 标志；1 表示先暂停写入 FIFO。
    reg [8:0] count_reg = 9'd0, count_next; // 当前 AW 对应已接收的 W beat 计数。

    reg [ID_WIDTH-1:0] m_axi_awid_reg = {ID_WIDTH{1'b0}}, m_axi_awid_next; // 暂存并输出到下游的 AWID。
    reg [ADDR_WIDTH-1:0] m_axi_awaddr_reg = {ADDR_WIDTH{1'b0}}, m_axi_awaddr_next; // 暂存并输出到下游的 AWADDR。
    reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next; // 暂存并输出到下游的 AWLEN。
    reg [2:0] m_axi_awsize_reg = 3'd0, m_axi_awsize_next; // 暂存并输出到下游的 AWSIZE。
    reg [1:0] m_axi_awburst_reg = 2'd0, m_axi_awburst_next; // 暂存并输出到下游的 AWBURST。
    reg m_axi_awlock_reg = 1'b0, m_axi_awlock_next; // 暂存并输出到下游的 AWLOCK。
    reg [3:0] m_axi_awcache_reg = 4'd0, m_axi_awcache_next; // 暂存并输出到下游的 AWCACHE。
    reg [2:0] m_axi_awprot_reg = 3'd0, m_axi_awprot_next; // 暂存并输出到下游的 AWPROT。
    reg [3:0] m_axi_awqos_reg = 4'd0, m_axi_awqos_next; // 暂存并输出到下游的 AWQOS。
    reg [3:0] m_axi_awregion_reg = 4'd0, m_axi_awregion_next; // 暂存并输出到下游的 AWREGION。
    reg [AWUSER_WIDTH-1:0] m_axi_awuser_reg = {AWUSER_WIDTH{1'b0}}, m_axi_awuser_next; // 暂存并输出到下游的 AWUSER。
    reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next; // 下游 AWVALID 寄存器。

    reg s_axi_awready_reg = 1'b0, s_axi_awready_next; // 上游 AWREADY 寄存器。

    assign m_axi_awid = m_axi_awid_reg;
    assign m_axi_awaddr = m_axi_awaddr_reg;
    assign m_axi_awlen = m_axi_awlen_reg;
    assign m_axi_awsize = m_axi_awsize_reg;
    assign m_axi_awburst = m_axi_awburst_reg;
    assign m_axi_awlock = m_axi_awlock_reg;
    assign m_axi_awcache = m_axi_awcache_reg;
    assign m_axi_awprot = m_axi_awprot_reg;
    assign m_axi_awqos = m_axi_awqos_reg;
    assign m_axi_awregion = m_axi_awregion_reg;
    assign m_axi_awuser = AWUSER_ENABLE ? m_axi_awuser_reg : {AWUSER_WIDTH{1'b0}};
    assign m_axi_awvalid = m_axi_awvalid_reg;

    assign s_axi_awready = s_axi_awready_reg;

    assign hold = hold_reg;

    always @* begin
        state_next = STATE_IDLE;

        hold_next = hold_reg;
        count_next = count_reg;

        m_axi_awid_next = m_axi_awid_reg;
        m_axi_awaddr_next = m_axi_awaddr_reg;
        m_axi_awlen_next = m_axi_awlen_reg;
        m_axi_awsize_next = m_axi_awsize_reg;
        m_axi_awburst_next = m_axi_awburst_reg;
        m_axi_awlock_next = m_axi_awlock_reg;
        m_axi_awcache_next = m_axi_awcache_reg;
        m_axi_awprot_next = m_axi_awprot_reg;
        m_axi_awqos_next = m_axi_awqos_reg;
        m_axi_awregion_next = m_axi_awregion_reg;
        m_axi_awuser_next = m_axi_awuser_reg;
        m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;
        s_axi_awready_next = s_axi_awready_reg;

        case (state_reg)
            STATE_IDLE: begin
                s_axi_awready_next = !m_axi_awvalid || m_axi_awready;
                hold_next = 1'b1;

                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awready_next = 1'b0;

                    m_axi_awid_next = s_axi_awid;
                    m_axi_awaddr_next = s_axi_awaddr;
                    m_axi_awlen_next = s_axi_awlen;
                    m_axi_awsize_next = s_axi_awsize;
                    m_axi_awburst_next = s_axi_awburst;
                    m_axi_awlock_next = s_axi_awlock;
                    m_axi_awcache_next = s_axi_awcache;
                    m_axi_awprot_next = s_axi_awprot;
                    m_axi_awqos_next = s_axi_awqos;
                    m_axi_awregion_next = s_axi_awregion;
                    m_axi_awuser_next = s_axi_awuser;

                    hold_next = 1'b0;
                    count_next = 0;
                    state_next = STATE_TRANSFER_IN;
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_TRANSFER_IN: begin
                s_axi_awready_next = 1'b0;
                hold_next = 1'b0;

                if (s_axi_wready && s_axi_wvalid) begin
                    count_next = count_reg + 1;
                    if (s_axi_wlast) begin
                        m_axi_awvalid_next = 1'b1;
                        hold_next = 1'b1;
                        state_next = STATE_IDLE;
                    end else if (FIFO_ADDR_WIDTH < 8 && count_next == 2**FIFO_ADDR_WIDTH) begin
                        m_axi_awvalid_next = 1'b1;
                        state_next = STATE_TRANSFER_OUT;
                    end else begin
                        state_next = STATE_TRANSFER_IN;
                    end
                end else begin
                    state_next = STATE_TRANSFER_IN;
                end
            end
            STATE_TRANSFER_OUT: begin
                s_axi_awready_next = 1'b0;
                hold_next = 1'b0;

                if (s_axi_wready && s_axi_wvalid) begin
                    if (s_axi_wlast) begin
                        hold_next = 1'b1;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_TRANSFER_OUT;
                    end
                end else begin
                    state_next = STATE_TRANSFER_OUT;
                end
            end
        endcase
    end

    always @(posedge clk) begin
        state_reg <= state_next;

        hold_reg <= hold_next;
        count_reg <= count_next;

        m_axi_awid_reg <= m_axi_awid_next;
        m_axi_awaddr_reg <= m_axi_awaddr_next;
        m_axi_awlen_reg <= m_axi_awlen_next;
        m_axi_awsize_reg <= m_axi_awsize_next;
        m_axi_awburst_reg <= m_axi_awburst_next;
        m_axi_awlock_reg <= m_axi_awlock_next;
        m_axi_awcache_reg <= m_axi_awcache_next;
        m_axi_awprot_reg <= m_axi_awprot_next;
        m_axi_awqos_reg <= m_axi_awqos_next;
        m_axi_awregion_reg <= m_axi_awregion_next;
        m_axi_awuser_reg <= m_axi_awuser_next;
        m_axi_awvalid_reg <= m_axi_awvalid_next;
        s_axi_awready_reg <= s_axi_awready_next;

        if (rst) begin
            state_reg <= STATE_IDLE;
            hold_reg <= 1'b1;
            m_axi_awvalid_reg <= 1'b0;
            s_axi_awready_reg <= 1'b0;
        end
    end
end else begin
    // AW 通道旁路
    assign m_axi_awid = s_axi_awid;
    assign m_axi_awaddr = s_axi_awaddr;
    assign m_axi_awlen = s_axi_awlen;
    assign m_axi_awsize = s_axi_awsize;
    assign m_axi_awburst = s_axi_awburst;
    assign m_axi_awlock = s_axi_awlock;
    assign m_axi_awcache = s_axi_awcache;
    assign m_axi_awprot = s_axi_awprot;
    assign m_axi_awqos = s_axi_awqos;
    assign m_axi_awregion = s_axi_awregion;
    assign m_axi_awuser = AWUSER_ENABLE ? s_axi_awuser : {AWUSER_WIDTH{1'b0}};
    assign m_axi_awvalid = s_axi_awvalid;
    assign s_axi_awready = m_axi_awready;

    assign hold = 1'b0;
end

endgenerate

// B 通道旁路
assign s_axi_bid = m_axi_bid;
assign s_axi_bresp = m_axi_bresp;
assign s_axi_buser = BUSER_ENABLE ? m_axi_buser : {BUSER_WIDTH{1'b0}};
assign s_axi_bvalid = m_axi_bvalid;
assign m_axi_bready = s_axi_bready;

assign m_axi_wvalid = m_axi_wvalid_reg;

assign m_axi_wdata = m_axi_w_reg[DATA_WIDTH-1:0];
assign m_axi_wstrb = m_axi_w_reg[STRB_OFFSET +: STRB_WIDTH];
assign m_axi_wlast = m_axi_w_reg[LAST_OFFSET];
assign m_axi_wuser = WUSER_ENABLE ? m_axi_w_reg[WUSER_OFFSET +: WUSER_WIDTH] : {WUSER_WIDTH{1'b0}};

// FIFO 写入逻辑
always @* begin
    write = 1'b0;

    wr_ptr_next = wr_ptr_reg;

    if (s_axi_wvalid) begin
        // 上游输入数据有效
        if (!full && !hold) begin
            // FIFO 未满，执行写入
            write = 1'b1;
            wr_ptr_next = wr_ptr_reg + 1;
        end
    end
end

always @(posedge clk) begin
    wr_ptr_reg <= wr_ptr_next;
    wr_addr_reg <= wr_ptr_next;

    if (write) begin
        mem[wr_addr_reg[FIFO_ADDR_WIDTH-1:0]] <= s_axi_w;
    end

    if (rst) begin
        wr_ptr_reg <= {FIFO_ADDR_WIDTH+1{1'b0}};
    end
end

// FIFO 读取逻辑
always @* begin
    read = 1'b0;

    rd_ptr_next = rd_ptr_reg;

    mem_read_data_valid_next = mem_read_data_valid_reg;

    if (store_output || !mem_read_data_valid_reg) begin
        // 输出无效或当前拍正在发生输出交接
        if (!empty) begin
            // FIFO 非空，执行读取
            read = 1'b1;
            mem_read_data_valid_next = 1'b1;
            rd_ptr_next = rd_ptr_reg + 1;
        end else begin
            // FIFO 为空，清空有效标记
            mem_read_data_valid_next = 1'b0;
        end
    end
end

always @(posedge clk) begin
    rd_ptr_reg <= rd_ptr_next;
    rd_addr_reg <= rd_ptr_next;

    mem_read_data_valid_reg <= mem_read_data_valid_next;

    if (read) begin
        mem_read_data_reg <= mem[rd_addr_reg[FIFO_ADDR_WIDTH-1:0]];
    end

    if (rst) begin
        rd_ptr_reg <= {FIFO_ADDR_WIDTH+1{1'b0}};
        mem_read_data_valid_reg <= 1'b0;
    end
end

// 输出寄存器逻辑
always @* begin
    store_output = 1'b0;

    m_axi_wvalid_next = m_axi_wvalid_reg;

    if (m_axi_wready || !m_axi_wvalid) begin
        store_output = 1'b1;
        m_axi_wvalid_next = mem_read_data_valid_reg;
    end
end

always @(posedge clk) begin
    m_axi_wvalid_reg <= m_axi_wvalid_next;

    if (store_output) begin
        m_axi_w_reg <= mem_read_data_reg;
    end

    if (rst) begin
        m_axi_wvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
