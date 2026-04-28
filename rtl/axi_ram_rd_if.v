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
 * AXI4 RAM 读接口
 *
 * 模块目录
 * 1) 将 AXI AR 突发转换为逐拍 RAM 读命令。
 * 2) 将 RAM 读响应转发到 AXI R 通道。
 * 3) R 通道支持可选输出流水级以改善时序。
 */
module axi_ram_rd_if #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
    // 输出端额外流水寄存器开关
    parameter PIPELINE_OUTPUT = 0
)
(
    input  wire                     clk, // 读接口时钟。
    input  wire                     rst, // AR 命令 FSM 与输出流水同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_arid, // AXI AR ID（读地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr, // AXI AR 地址。
    input  wire [7:0]               s_axi_arlen, // AXI AR 突发长度。
    input  wire [2:0]               s_axi_arsize, // AXI AR 突发尺寸。
    input  wire [1:0]               s_axi_arburst, // AXI AR 突发类型。
    input  wire                     s_axi_arlock, // AXI AR 锁属性。
    input  wire [3:0]               s_axi_arcache, // AXI AR cache 属性。
    input  wire [2:0]               s_axi_arprot, // AXI AR 保护属性。
    input  wire [3:0]               s_axi_arqos, // AXI AR QoS（服务质量字段）。
    input  wire [3:0]               s_axi_arregion, // AXI AR region（区域属性字段）。
    input  wire [ARUSER_WIDTH-1:0]  s_axi_aruser, // AXI AR 用户旁带。
    input  wire                     s_axi_arvalid, // AXI ARVALID（读地址有效）。
    output wire                     s_axi_arready, // AXI ARREADY（读地址就绪）。
    output wire [ID_WIDTH-1:0]      s_axi_rid, // AXI RID（读数据通道标识）。
    output wire [DATA_WIDTH-1:0]    s_axi_rdata, // AXI RDATA（读数据载荷）。
    output wire [1:0]               s_axi_rresp, // AXI RRESP（固定 OKAY）。
    output wire                     s_axi_rlast, // AXI RLAST（最后一拍）。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // AXI RUSER（读用户旁带）。
    output wire                     s_axi_rvalid, // AXI RVALID（读数据有效）。
    input  wire                     s_axi_rready, // AXI RREADY（读数据就绪）。

    /*
     * RAM 接口
     */
    output wire [ID_WIDTH-1:0]      ram_rd_cmd_id, // RAM 读命令 ID（每拍对应一条）。
    output wire [ADDR_WIDTH-1:0]    ram_rd_cmd_addr, // RAM 读命令地址。
    output wire                     ram_rd_cmd_lock, // RAM 读命令锁属性透传。
    output wire [3:0]               ram_rd_cmd_cache, // RAM 读命令 cache 属性透传。
    output wire [2:0]               ram_rd_cmd_prot, // RAM 读命令保护属性透传。
    output wire [3:0]               ram_rd_cmd_qos, // RAM 读命令 QoS 属性透传。
    output wire [3:0]               ram_rd_cmd_region, // RAM 读命令 region 属性透传。
    output wire [ARUSER_WIDTH-1:0]  ram_rd_cmd_auser, // RAM 读命令 ARUSER 透传。
    output wire                     ram_rd_cmd_en, // RAM 读命令有效。
    output wire                     ram_rd_cmd_last, // 当前 AXI 突发最后一拍命令标记。
    input  wire                     ram_rd_cmd_ready, // RAM 读命令 ready。
    input  wire [ID_WIDTH-1:0]      ram_rd_resp_id, // RAM 读响应 ID。
    input  wire [DATA_WIDTH-1:0]    ram_rd_resp_data, // RAM 读响应数据。
    input  wire                     ram_rd_resp_last, // RAM 读响应最后一拍标记。
    input  wire [RUSER_WIDTH-1:0]   ram_rd_resp_user, // RAM 读响应用户旁带。
    input  wire                     ram_rd_resp_valid, // RAM 读响应有效。
    output wire                     ram_rd_resp_ready // RAM 读响应 ready。
);

parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH); // 去掉字节 lane 位后的字地址位宽。
parameter WORD_WIDTH = STRB_WIDTH; // 每个字包含的字节 lane 数量。
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH; // 每个 lane 对应位宽。

// 总线位宽断言检查
initial begin
    if (WORD_SIZE * STRB_WIDTH != DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisble (instance %m)");
        $finish;
    end

    if (2**$clog2(WORD_WIDTH) != WORD_WIDTH) begin
        $error("Error: AXI word width must be even power of two (instance %m)");
        $finish;
    end
end

localparam [0:0]
    STATE_IDLE = 1'd0, // 等待新的 AXI AR。
    STATE_BURST = 1'd1; // 为突发每一拍发出 RAM 读命令。

reg [0:0] state_reg = STATE_IDLE, state_next; // 读命令 FSM 状态。

reg [ID_WIDTH-1:0] read_id_reg = {ID_WIDTH{1'b0}}, read_id_next; // 当前突发 ID。
reg [ADDR_WIDTH-1:0] read_addr_reg = {ADDR_WIDTH{1'b0}}, read_addr_next; // 当前读地址指针。
reg read_lock_reg = 1'b0, read_lock_next; // 当前锁属性。
reg [3:0] read_cache_reg = 4'd0, read_cache_next; // 当前 cache 属性。
reg [2:0] read_prot_reg = 3'd0, read_prot_next; // 当前保护属性。
reg [3:0] read_qos_reg = 4'd0, read_qos_next; // 当前 QoS 属性。
reg [3:0] read_region_reg = 4'd0, read_region_next; // 当前 region 属性。
reg [ARUSER_WIDTH-1:0] read_aruser_reg = {ARUSER_WIDTH{1'b0}}, read_aruser_next; // 当前 ARUSER 属性。
reg read_addr_valid_reg = 1'b0, read_addr_valid_next; // RAM 命令有效状态。
reg read_last_reg = 1'b0, read_last_next; // RAM 命令最后一拍标记。
reg [7:0] read_count_reg = 8'd0, read_count_next; // 剩余拍计数。
reg [2:0] read_size_reg = 3'd0, read_size_next; // 地址步进尺寸。
reg [1:0] read_burst_reg = 2'd0, read_burst_next; // 突发类型。

reg s_axi_arready_reg = 1'b0, s_axi_arready_next; // AXI ARREADY 状态。
reg [ID_WIDTH-1:0] s_axi_rid_pipe_reg = {ID_WIDTH{1'b0}}; // 可选流水 RID 寄存器。
reg [DATA_WIDTH-1:0] s_axi_rdata_pipe_reg = {DATA_WIDTH{1'b0}}; // 可选流水 RDATA 寄存器。
reg s_axi_rlast_pipe_reg = 1'b0; // 可选流水 RLAST 寄存器。
reg [RUSER_WIDTH-1:0] s_axi_ruser_pipe_reg = {RUSER_WIDTH{1'b0}}; // 可选流水 RUSER 寄存器。
reg s_axi_rvalid_pipe_reg = 1'b0; // 可选流水 RVALID 寄存器。

assign s_axi_arready = s_axi_arready_reg;
assign s_axi_rid = PIPELINE_OUTPUT ? s_axi_rid_pipe_reg : ram_rd_resp_id;
assign s_axi_rdata = PIPELINE_OUTPUT ? s_axi_rdata_pipe_reg : ram_rd_resp_data;
assign s_axi_rresp = 2'b00;
assign s_axi_rlast = PIPELINE_OUTPUT ? s_axi_rlast_pipe_reg : ram_rd_resp_last;
assign s_axi_ruser = PIPELINE_OUTPUT ? s_axi_ruser_pipe_reg : ram_rd_resp_user;
assign s_axi_rvalid = PIPELINE_OUTPUT ? s_axi_rvalid_pipe_reg : ram_rd_resp_valid;

assign ram_rd_cmd_id = read_id_reg;
assign ram_rd_cmd_addr = read_addr_reg;
assign ram_rd_cmd_lock = read_lock_reg;
assign ram_rd_cmd_cache = read_cache_reg;
assign ram_rd_cmd_prot = read_prot_reg;
assign ram_rd_cmd_qos = read_qos_reg;
assign ram_rd_cmd_region = read_region_reg;
assign ram_rd_cmd_auser = ARUSER_ENABLE ? read_aruser_reg : {ARUSER_WIDTH{1'b0}};
assign ram_rd_cmd_en = read_addr_valid_reg;
assign ram_rd_cmd_last = read_last_reg;

assign ram_rd_resp_ready = s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg);

always @* begin
    state_next = STATE_IDLE;

    read_id_next = read_id_reg;
    read_addr_next = read_addr_reg;
    read_lock_next = read_lock_reg;
    read_cache_next = read_cache_reg;
    read_prot_next = read_prot_reg;
    read_qos_next = read_qos_reg;
    read_region_next = read_region_reg;
    read_aruser_next = read_aruser_reg;
    read_addr_valid_next = read_addr_valid_reg && !ram_rd_cmd_ready;
    read_last_next = read_last_reg;
    read_count_next = read_count_reg;
    read_size_next = read_size_reg;
    read_burst_next = read_burst_reg;

    s_axi_arready_next = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            s_axi_arready_next = 1'b1;

            if (s_axi_arready && s_axi_arvalid) begin
                read_id_next = s_axi_arid;
                read_addr_next = s_axi_araddr;
                read_lock_next = s_axi_arlock;
                read_cache_next = s_axi_arcache;
                read_prot_next = s_axi_arprot;
                read_qos_next = s_axi_arqos;
                read_region_next = s_axi_arregion;
                read_aruser_next = s_axi_aruser;
                read_count_next = s_axi_arlen;
                read_size_next = s_axi_arsize < $clog2(STRB_WIDTH) ? s_axi_arsize : $clog2(STRB_WIDTH);
                read_burst_next = s_axi_arburst;

                s_axi_arready_next = 1'b0;
                read_last_next = read_count_next == 0;
                read_addr_valid_next = 1'b1;
                state_next = STATE_BURST;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_BURST: begin
            if (ram_rd_cmd_ready && ram_rd_cmd_en) begin
                if (read_burst_reg != 2'b00) begin
                    read_addr_next = read_addr_reg + (1 << read_size_reg);
                end
                read_count_next = read_count_reg - 1;
                read_last_next = read_count_next == 0;
                if (read_count_reg > 0) begin
                    read_addr_valid_next = 1'b1;
                    state_next = STATE_BURST;
                end else begin
                    s_axi_arready_next = 1'b1;
                    state_next = STATE_IDLE;
                end
            end else begin
                state_next = STATE_BURST;
            end
        end
    endcase
end

always @(posedge clk) begin
    state_reg <= state_next;

    read_id_reg <= read_id_next;
    read_addr_reg <= read_addr_next;
    read_lock_reg <= read_lock_next;
    read_cache_reg <= read_cache_next;
    read_prot_reg <= read_prot_next;
    read_qos_reg <= read_qos_next;
    read_region_reg <= read_region_next;
    read_aruser_reg <= read_aruser_next;
    read_addr_valid_reg <= read_addr_valid_next;
    read_last_reg <= read_last_next;
    read_count_reg <= read_count_next;
    read_size_reg <= read_size_next;
    read_burst_reg <= read_burst_next;

    s_axi_arready_reg <= s_axi_arready_next;

    if (!s_axi_rvalid_pipe_reg || s_axi_rready) begin
        s_axi_rid_pipe_reg <= ram_rd_resp_id;
        s_axi_rdata_pipe_reg <= ram_rd_resp_data;
        s_axi_rlast_pipe_reg <= ram_rd_resp_last;
        s_axi_ruser_pipe_reg <= ram_rd_resp_user;
        s_axi_rvalid_pipe_reg <= ram_rd_resp_valid;
    end

    if (rst) begin
        state_reg <= STATE_IDLE;

        read_addr_valid_reg <= 1'b0;

        s_axi_arready_reg <= 1'b0;
        s_axi_rvalid_pipe_reg <= 1'b0;
    end
end

endmodule

`resetall
