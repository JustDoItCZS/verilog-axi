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
 * AXI4 FIFO（读通道）
 *
 * 模块目录
 * 1) R 通道进入内部 FIFO，解耦下游返回节奏和上游取数节奏。
 * 2) 可选 `FIFO_DELAY` 模式下缓存 AR，确保 FIFO 有足够空间容纳突发返回数据。
 * 3) 读地址和读数据通过统一计数约束，避免 FIFO 溢出。
 */
module axi_fifo_rd #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 ARUSER 信号
    parameter ARUSER_ENABLE = 0,
    // ARUSER 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 RUSER 信号
    parameter RUSER_ENABLE = 0,
    // RUSER 信号位宽
    parameter RUSER_WIDTH = 1,
    // 读数据 FIFO 深度（拍）
    parameter FIFO_DEPTH = 32,
    // 尽可能等待 FIFO 有足够读返回空间后再放行读地址
    parameter FIFO_DELAY = 0
)
(
    input  wire                     clk, // 时钟。
    input  wire                     rst, // 同步复位，高电平有效。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // 上游读地址 ID。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // 上游读地址。
    input  wire [7:0]               s_axi_arlen, // 上游读突发长度。
    input  wire [2:0]               s_axi_arsize, // 上游读 beat 大小。
    input  wire [1:0]               s_axi_arburst, // 上游读突发类型。
    input  wire                     s_axi_arlock, // 上游读锁属性。
    input  wire [3:0]               s_axi_arcache, // 上游读 cache 属性。
    input  wire [2:0]               s_axi_arprot, // 上游读保护属性。
    input  wire [3:0]               s_axi_arqos, // 上游读 QoS。
    input  wire [3:0]               s_axi_arregion, // 上游读 region。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // 上游 AR user sideband。
    input  wire                     s_axi_arvalid, // 上游 ARVALID。
    output wire                     s_axi_arready, // 本模块可接收上游读地址。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // 返回上游的读数据 ID。
    output wire [DATA_WIDTH-1:0]    s_axi_rdata, // 返回上游的读数据。
    output wire [1:0]               s_axi_rresp, // 返回上游的读响应状态。
    output wire                     s_axi_rlast, // 返回上游的读突发最后一拍。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // 返回上游的 R user。
    output wire                     s_axi_rvalid, // 返回上游的 RVALID。
    input  wire                     s_axi_rready, // 上游 RREADY。

    /*
     * AXI 主接口
     */
    output wire [ID_WIDTH-1:0]      m_axi_arid, // 发送下游的读地址 ID。
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr, // 发送下游的读地址。
    output wire [7:0]               m_axi_arlen, // 发送下游的读突发长度。
    output wire [2:0]               m_axi_arsize, // 发送下游的读 beat 大小。
    output wire [1:0]               m_axi_arburst, // 发送下游的读突发类型。
    output wire                     m_axi_arlock, // 发送下游的读锁属性。
    output wire [3:0]               m_axi_arcache, // 发送下游的读 cache 属性。
    output wire [2:0]               m_axi_arprot, // 发送下游的读保护属性。
    output wire [3:0]               m_axi_arqos, // 发送下游的读 QoS。
    output wire [3:0]               m_axi_arregion, // 发送下游的读 region。
    output wire [ARUSER_WIDTH-1:0]  m_axi_aruser, // 发送下游的 AR user。
    output wire                     m_axi_arvalid, // 发送下游的 ARVALID。
    input  wire                     m_axi_arready, // 下游 ARREADY。
    input  wire [ID_WIDTH-1:0]      m_axi_rid, // 下游读数据 ID。
    input  wire [DATA_WIDTH-1:0]    m_axi_rdata, // 下游读数据。
    input  wire [1:0]               m_axi_rresp, // 下游读响应状态。
    input  wire                     m_axi_rlast, // 下游读突发最后一拍。
    input  wire [RUSER_WIDTH-1:0]   m_axi_ruser, // 下游 R user。
    input  wire                     m_axi_rvalid, // 下游 RVALID。
    output wire                     m_axi_rready // 本模块对下游 R 通道 ready。
);

parameter LAST_OFFSET  = DATA_WIDTH; // FIFO 打包总线中 rlast 位偏移。
parameter ID_OFFSET    = LAST_OFFSET + 1; // FIFO 打包总线中 rid 起始偏移。
parameter RESP_OFFSET  = ID_OFFSET + ID_WIDTH; // FIFO 打包总线中 rresp 起始偏移。
parameter RUSER_OFFSET = RESP_OFFSET + 2; // FIFO 打包总线中 ruser 起始偏移。
parameter RWIDTH       = RUSER_OFFSET + (RUSER_ENABLE ? RUSER_WIDTH : 0); // FIFO 每条读返回记录的总位宽。

parameter FIFO_ADDR_WIDTH = $clog2(FIFO_DEPTH); // FIFO 地址位宽。

reg [FIFO_ADDR_WIDTH:0] wr_ptr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}, wr_ptr_next; // FIFO 写指针(下游 R 进入 FIFO)。
reg [FIFO_ADDR_WIDTH:0] wr_addr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}; // 写 RAM 地址寄存器。
reg [FIFO_ADDR_WIDTH:0] rd_ptr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}, rd_ptr_next; // FIFO 读指针(上游 R 取出 FIFO)。
reg [FIFO_ADDR_WIDTH:0] rd_addr_reg = {FIFO_ADDR_WIDTH+1{1'b0}}; // 读 RAM 地址寄存器。

(* ramstyle = "no_rw_check" *)
reg [RWIDTH-1:0] mem[(2**FIFO_ADDR_WIDTH)-1:0]; // 读数据 FIFO 存储体。
reg [RWIDTH-1:0] mem_read_data_reg; // FIFO 读出的待输出数据。
reg mem_read_data_valid_reg = 1'b0, mem_read_data_valid_next; // 待输出数据有效标志。

wire [RWIDTH-1:0] m_axi_r; // 将下游 R 通道打包后的 FIFO 写入数据。

reg [RWIDTH-1:0] s_axi_r_reg; // 输出给上游的 R 通道寄存器数据。
reg s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next; // 输出给上游的 RVALID 寄存器。

// FIFO 满条件：最高位不同且其余位相同
wire full = ((wr_ptr_reg[FIFO_ADDR_WIDTH] != rd_ptr_reg[FIFO_ADDR_WIDTH]) && // FIFO 满标志：高位不同且低位相同。
             (wr_ptr_reg[FIFO_ADDR_WIDTH-1:0] == rd_ptr_reg[FIFO_ADDR_WIDTH-1:0])); // FIFO 满标志：高位不同且低位相同。
// FIFO 空条件：读写指针完全一致
wire empty = wr_ptr_reg == rd_ptr_reg; // FIFO 空标志：读写指针完全相同。

// 控制信号
reg write; // FIFO RAM 写使能(下游 R 入队)。
reg read; // FIFO RAM 读使能(上游 R 出队)。
reg store_output; // 将 FIFO 读出数据装入上游输出寄存器。

assign m_axi_rready = !full;

generate
    assign m_axi_r[DATA_WIDTH-1:0] = m_axi_rdata;
    assign m_axi_r[LAST_OFFSET] = m_axi_rlast;
    assign m_axi_r[ID_OFFSET +: ID_WIDTH] = m_axi_rid;
    assign m_axi_r[RESP_OFFSET +: 2] = m_axi_rresp;
    if (RUSER_ENABLE) assign m_axi_r[RUSER_OFFSET +: RUSER_WIDTH] = m_axi_ruser;
endgenerate

generate

if (FIFO_DELAY) begin
    // 缓存 AR，直到 FIFO 具备容纳该次读突发返回数据的空间（或当前为空）

    localparam COUNT_WIDTH = (FIFO_ADDR_WIDTH > 8 ? FIFO_ADDR_WIDTH : 8) + 1; // 计数器位宽，覆盖最大突发累积计数。

    localparam [1:0]
        STATE_IDLE = 1'd0, // 可接收新 AR，必要时立即下发。
        STATE_WAIT = 1'd1; // FIFO 空间不足，等待已缓存数据被上游消费。

    reg [1:0] state_reg = STATE_IDLE, state_next; // AR 延迟控制状态机。

    reg [COUNT_WIDTH-1:0] count_reg = 0, count_next; // 预留/已占用的 R beat 计数，用于空间管理。

    reg [ID_WIDTH-1:0] m_axi_arid_reg = {ID_WIDTH{1'b0}}, m_axi_arid_next; // 暂存并输出到下游的 ARID。
    reg [ADDR_WIDTH-1:0] m_axi_araddr_reg = {ADDR_WIDTH{1'b0}}, m_axi_araddr_next; // 暂存并输出到下游的 ARADDR。
    reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next; // 暂存并输出到下游的 ARLEN。
    reg [2:0] m_axi_arsize_reg = 3'd0, m_axi_arsize_next; // 暂存并输出到下游的 ARSIZE。
    reg [1:0] m_axi_arburst_reg = 2'd0, m_axi_arburst_next; // 暂存并输出到下游的 ARBURST。
    reg m_axi_arlock_reg = 1'b0, m_axi_arlock_next; // 暂存并输出到下游的 ARLOCK。
    reg [3:0] m_axi_arcache_reg = 4'd0, m_axi_arcache_next; // 暂存并输出到下游的 ARCACHE。
    reg [2:0] m_axi_arprot_reg = 3'd0, m_axi_arprot_next; // 暂存并输出到下游的 ARPROT。
    reg [3:0] m_axi_arqos_reg = 4'd0, m_axi_arqos_next; // 暂存并输出到下游的 ARQOS。
    reg [3:0] m_axi_arregion_reg = 4'd0, m_axi_arregion_next; // 暂存并输出到下游的 ARREGION。
    reg [ARUSER_WIDTH-1:0] m_axi_aruser_reg = {ARUSER_WIDTH{1'b0}}, m_axi_aruser_next; // 暂存并输出到下游的 ARUSER。
    reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next; // 下游 ARVALID 寄存器。

    reg s_axi_arready_reg = 1'b0, s_axi_arready_next; // 上游 ARREADY 寄存器。

    assign m_axi_arid = m_axi_arid_reg;
    assign m_axi_araddr = m_axi_araddr_reg;
    assign m_axi_arlen = m_axi_arlen_reg;
    assign m_axi_arsize = m_axi_arsize_reg;
    assign m_axi_arburst = m_axi_arburst_reg;
    assign m_axi_arlock = m_axi_arlock_reg;
    assign m_axi_arcache = m_axi_arcache_reg;
    assign m_axi_arprot = m_axi_arprot_reg;
    assign m_axi_arqos = m_axi_arqos_reg;
    assign m_axi_arregion = m_axi_arregion_reg;
    assign m_axi_aruser = ARUSER_ENABLE ? m_axi_aruser_reg : {ARUSER_WIDTH{1'b0}};
    assign m_axi_arvalid = m_axi_arvalid_reg;

    assign s_axi_arready = s_axi_arready_reg;

    always @* begin
        state_next = STATE_IDLE;

        count_next = count_reg;

        m_axi_arid_next = m_axi_arid_reg;
        m_axi_araddr_next = m_axi_araddr_reg;
        m_axi_arlen_next = m_axi_arlen_reg;
        m_axi_arsize_next = m_axi_arsize_reg;
        m_axi_arburst_next = m_axi_arburst_reg;
        m_axi_arlock_next = m_axi_arlock_reg;
        m_axi_arcache_next = m_axi_arcache_reg;
        m_axi_arprot_next = m_axi_arprot_reg;
        m_axi_arqos_next = m_axi_arqos_reg;
        m_axi_arregion_next = m_axi_arregion_reg;
        m_axi_aruser_next = m_axi_aruser_reg;
        m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;
        s_axi_arready_next = s_axi_arready_reg;

        case (state_reg)
            STATE_IDLE: begin
                s_axi_arready_next = !m_axi_arvalid || m_axi_arready;

                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arready_next = 1'b0;

                    m_axi_arid_next = s_axi_arid;
                    m_axi_araddr_next = s_axi_araddr;
                    m_axi_arlen_next = s_axi_arlen;
                    m_axi_arsize_next = s_axi_arsize;
                    m_axi_arburst_next = s_axi_arburst;
                    m_axi_arlock_next = s_axi_arlock;
                    m_axi_arcache_next = s_axi_arcache;
                    m_axi_arprot_next = s_axi_arprot;
                    m_axi_arqos_next = s_axi_arqos;
                    m_axi_arregion_next = s_axi_arregion;
                    m_axi_aruser_next = s_axi_aruser;

                    if (count_reg == 0 || count_reg + m_axi_arlen_next + 1 <= 2**FIFO_ADDR_WIDTH) begin
                        count_next = count_reg + m_axi_arlen_next + 1;
                        m_axi_arvalid_next = 1'b1;
                        s_axi_arready_next = 1'b0;
                        state_next = STATE_IDLE;
                    end else begin
                        s_axi_arready_next = 1'b0;
                        state_next = STATE_WAIT;
                    end
                end else begin
                    state_next = STATE_IDLE;
                end
            end
            STATE_WAIT: begin
                s_axi_arready_next = 1'b0;

                if (count_reg == 0 || count_reg + m_axi_arlen_reg + 1 <= 2**FIFO_ADDR_WIDTH) begin
                    count_next = count_reg + m_axi_arlen_reg + 1;
                    m_axi_arvalid_next = 1'b1;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_WAIT;
                end
            end
        endcase

        if (s_axi_rready && s_axi_rvalid) begin
            count_next = count_next - 1;
        end
    end

    always @(posedge clk) begin
        state_reg <= state_next;
        count_reg <= count_next;

        m_axi_arid_reg <= m_axi_arid_next;
        m_axi_araddr_reg <= m_axi_araddr_next;
        m_axi_arlen_reg <= m_axi_arlen_next;
        m_axi_arsize_reg <= m_axi_arsize_next;
        m_axi_arburst_reg <= m_axi_arburst_next;
        m_axi_arlock_reg <= m_axi_arlock_next;
        m_axi_arcache_reg <= m_axi_arcache_next;
        m_axi_arprot_reg <= m_axi_arprot_next;
        m_axi_arqos_reg <= m_axi_arqos_next;
        m_axi_arregion_reg <= m_axi_arregion_next;
        m_axi_aruser_reg <= m_axi_aruser_next;
        m_axi_arvalid_reg <= m_axi_arvalid_next;
        s_axi_arready_reg <= s_axi_arready_next;

        if (rst) begin
            state_reg <= STATE_IDLE;
            count_reg <= {COUNT_WIDTH{1'b0}};
            m_axi_arvalid_reg <= 1'b0;
            s_axi_arready_reg <= 1'b0;
        end
    end
end else begin
    // AR 通道旁路
    assign m_axi_arid = s_axi_arid;
    assign m_axi_araddr = s_axi_araddr;
    assign m_axi_arlen = s_axi_arlen;
    assign m_axi_arsize = s_axi_arsize;
    assign m_axi_arburst = s_axi_arburst;
    assign m_axi_arlock = s_axi_arlock;
    assign m_axi_arcache = s_axi_arcache;
    assign m_axi_arprot = s_axi_arprot;
    assign m_axi_arqos = s_axi_arqos;
    assign m_axi_arregion = s_axi_arregion;
    assign m_axi_aruser = ARUSER_ENABLE ? s_axi_aruser : {ARUSER_WIDTH{1'b0}};
    assign m_axi_arvalid = s_axi_arvalid;
    assign s_axi_arready = m_axi_arready;
end

endgenerate

assign s_axi_rvalid = s_axi_rvalid_reg;

assign s_axi_rdata = s_axi_r_reg[DATA_WIDTH-1:0];
assign s_axi_rlast = s_axi_r_reg[LAST_OFFSET];
assign s_axi_rid   = s_axi_r_reg[ID_OFFSET +: ID_WIDTH];
assign s_axi_rresp = s_axi_r_reg[RESP_OFFSET +: 2];
assign s_axi_ruser = RUSER_ENABLE ? s_axi_r_reg[RUSER_OFFSET +: RUSER_WIDTH] : {RUSER_WIDTH{1'b0}};

// FIFO 写入逻辑
always @* begin
    write = 1'b0;

    wr_ptr_next = wr_ptr_reg;

    if (m_axi_rvalid) begin
        // 下游输入数据有效
        if (!full) begin
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
        mem[wr_addr_reg[FIFO_ADDR_WIDTH-1:0]] <= m_axi_r;
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

    s_axi_rvalid_next = s_axi_rvalid_reg;

    if (s_axi_rready || !s_axi_rvalid) begin
        store_output = 1'b1;
        s_axi_rvalid_next = mem_read_data_valid_reg;
    end
end

always @(posedge clk) begin
    s_axi_rvalid_reg <= s_axi_rvalid_next;

    if (store_output) begin
        s_axi_r_reg <= mem_read_data_reg;
    end

    if (rst) begin
        s_axi_rvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
