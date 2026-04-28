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
 * AXI4 RAM 写接口
 *
 * 模块目录
 * 1) 将 AXI AW/W 突发转换为逐拍 RAM 写命令。
 * 2) 跟踪突发进度并更新地址步进。
 * 3) 最后一拍写数据被接纳后返回 AXI B 响应。
 */
module axi_ram_wr_if #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
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
    parameter BUSER_WIDTH = 1
)
(
    input  wire                     clk, // 写接口时钟。
    input  wire                     rst, // AW/W 命令 FSM 与 B 通道状态同步复位。

    /*
     * AXI 从接口
     */
    input  wire [ID_WIDTH-1:0]      s_axi_awid, // AXI AW ID（写地址通道标识）。
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr, // AXI AW 地址。
    input  wire [7:0]               s_axi_awlen, // AXI AW 突发长度。
    input  wire [2:0]               s_axi_awsize, // AXI AW 突发尺寸。
    input  wire [1:0]               s_axi_awburst, // AXI AW 突发类型。
    input  wire                     s_axi_awlock, // AXI AW 锁属性。
    input  wire [3:0]               s_axi_awcache, // AXI AW cache 属性。
    input  wire [2:0]               s_axi_awprot, // AXI AW 保护属性。
    input  wire [3:0]               s_axi_awqos, // AXI AW QoS（服务质量字段）。
    input  wire [3:0]               s_axi_awregion, // AXI AW region（区域属性字段）。
    input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser, // AXI AW 用户旁带。
    input  wire                     s_axi_awvalid, // AXI AWVALID（写地址有效）。
    output wire                     s_axi_awready, // AXI AWREADY（写地址就绪）。
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata, // AXI W 数据。
    input  wire [STRB_WIDTH-1:0]    s_axi_wstrb, // AXI W 字节使能。
    input  wire                     s_axi_wlast, // AXI WLAST（写突发最后一拍）。
    input  wire [WUSER_WIDTH-1:0]   s_axi_wuser, // AXI W 用户旁带。
    input  wire                     s_axi_wvalid, // AXI WVALID（写数据有效）。
    output wire                     s_axi_wready, // AXI WREADY（写数据就绪）。
    output wire [ID_WIDTH-1:0]      s_axi_bid, // AXI BID（写响应标识）。
    output wire [1:0]               s_axi_bresp, // AXI BRESP（固定 OKAY）。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // AXI BUSER（本模型固定为 0）。
    output wire                     s_axi_bvalid, // AXI BVALID（写响应有效）。
    input  wire                     s_axi_bready, // AXI BREADY（写响应就绪）。

    /*
     * RAM 接口
     */
    output wire [ID_WIDTH-1:0]      ram_wr_cmd_id, // RAM 写命令 ID。
    output wire [ADDR_WIDTH-1:0]    ram_wr_cmd_addr, // RAM 写命令地址。
    output wire                     ram_wr_cmd_lock, // RAM 写命令锁属性。
    output wire [3:0]               ram_wr_cmd_cache, // RAM 写命令 cache 属性。
    output wire [2:0]               ram_wr_cmd_prot, // RAM 写命令保护属性。
    output wire [3:0]               ram_wr_cmd_qos, // RAM 写命令 QoS 属性。
    output wire [3:0]               ram_wr_cmd_region, // RAM 写命令 region 属性。
    output wire [AWUSER_WIDTH-1:0]  ram_wr_cmd_auser, // RAM 写命令 AWUSER 属性。
    output wire [DATA_WIDTH-1:0]    ram_wr_cmd_data, // RAM 写命令数据。
    output wire [STRB_WIDTH-1:0]    ram_wr_cmd_strb, // RAM 写命令字节使能。
    output wire [WUSER_WIDTH-1:0]   ram_wr_cmd_user, // RAM 写命令 WUSER 属性。
    output wire                     ram_wr_cmd_en, // RAM 写命令有效信号。
    output wire                     ram_wr_cmd_last, // RAM 写命令最后一拍标记。
    input  wire                     ram_wr_cmd_ready // RAM 写命令 ready。
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

localparam [1:0]
    STATE_IDLE = 2'd0, // 等待 AXI AW 握手。
    STATE_BURST = 2'd1, // 将 W 数据拍连续输出为 RAM 写命令。
    STATE_RESP = 2'd2; // 发生反压时保持并发送 AXI B 响应。

reg [1:0] state_reg = STATE_IDLE, state_next; // 写命令 FSM 状态。

reg [ID_WIDTH-1:0] write_id_reg = {ID_WIDTH{1'b0}}, write_id_next; // 当前突发 ID。
reg [ADDR_WIDTH-1:0] write_addr_reg = {ADDR_WIDTH{1'b0}}, write_addr_next; // 当前写地址指针。
reg write_lock_reg = 1'b0, write_lock_next; // 当前锁属性。
reg [3:0] write_cache_reg = 4'd0, write_cache_next; // 当前 cache 属性。
reg [2:0] write_prot_reg = 3'd0, write_prot_next; // 当前保护属性。
reg [3:0] write_qos_reg = 4'd0, write_qos_next; // 当前 QoS 属性。
reg [3:0] write_region_reg = 4'd0, write_region_next; // 当前 region 属性。
reg [AWUSER_WIDTH-1:0] write_awuser_reg = {AWUSER_WIDTH{1'b0}}, write_awuser_next; // 当前 AWUSER 属性。
reg write_addr_valid_reg = 1'b0, write_addr_valid_next; // RAM 命令有效状态。
reg write_last_reg = 1'b0, write_last_next; // RAM 命令流最后一拍标记。
reg [7:0] write_count_reg = 8'd0, write_count_next; // 剩余拍计数。
reg [2:0] write_size_reg = 3'd0, write_size_next; // 地址步进尺寸。
reg [1:0] write_burst_reg = 2'd0, write_burst_next; // 突发类型。

reg s_axi_awready_reg = 1'b0, s_axi_awready_next; // AXI AWREADY 状态。
reg [ID_WIDTH-1:0] s_axi_bid_reg = {ID_WIDTH{1'b0}}, s_axi_bid_next; // AXI BID 输出寄存器。
reg s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next; // AXI BVALID 状态。

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = write_addr_valid_reg && ram_wr_cmd_ready;
assign s_axi_bid = s_axi_bid_reg;
assign s_axi_bresp = 2'b00;
assign s_axi_buser = {BUSER_WIDTH{1'b0}};
assign s_axi_bvalid = s_axi_bvalid_reg;

assign ram_wr_cmd_id = write_id_reg;
assign ram_wr_cmd_addr = write_addr_reg;
assign ram_wr_cmd_lock = write_lock_reg;
assign ram_wr_cmd_cache = write_cache_reg;
assign ram_wr_cmd_prot = write_prot_reg;
assign ram_wr_cmd_qos = write_qos_reg;
assign ram_wr_cmd_region = write_region_reg;
assign ram_wr_cmd_auser = AWUSER_ENABLE ? write_awuser_reg : {AWUSER_WIDTH{1'b0}};
assign ram_wr_cmd_data = s_axi_wdata;
assign ram_wr_cmd_strb = s_axi_wstrb;
assign ram_wr_cmd_user = WUSER_ENABLE ? s_axi_wuser : {WUSER_WIDTH{1'b0}};
assign ram_wr_cmd_en = write_addr_valid_reg && s_axi_wvalid;
assign ram_wr_cmd_last = write_last_reg;

always @* begin
    state_next = STATE_IDLE;

    write_id_next = write_id_reg;
    write_addr_next = write_addr_reg;
    write_lock_next = write_lock_reg;
    write_cache_next = write_cache_reg;
    write_prot_next = write_prot_reg;
    write_qos_next = write_qos_reg;
    write_region_next = write_region_reg;
    write_awuser_next = write_awuser_reg;
    write_addr_valid_next = write_addr_valid_reg;
    write_last_next = write_last_reg;
    write_count_next = write_count_reg;
    write_size_next = write_size_reg;
    write_burst_next = write_burst_reg;

    s_axi_awready_next = 1'b0;
    s_axi_bid_next = s_axi_bid_reg;
    s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;

    case (state_reg)
        STATE_IDLE: begin
            s_axi_awready_next = 1'b1;

            if (s_axi_awready && s_axi_awvalid) begin
                write_id_next = s_axi_awid;
                write_addr_next = s_axi_awaddr;
                write_lock_next = s_axi_awlock;
                write_cache_next = s_axi_awcache;
                write_prot_next = s_axi_awprot;
                write_qos_next = s_axi_awqos;
                write_region_next = s_axi_awregion;
                write_awuser_next = s_axi_awuser;
                write_count_next = s_axi_awlen;
                write_size_next = s_axi_awsize < $clog2(STRB_WIDTH) ? s_axi_awsize : $clog2(STRB_WIDTH);
                write_burst_next = s_axi_awburst;

                write_addr_valid_next = 1'b1;
                s_axi_awready_next = 1'b0;
                if (s_axi_awlen > 0) begin
                    write_last_next = 1'b0;
                end else begin
                    write_last_next = 1'b1;
                end
                state_next = STATE_BURST;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_BURST: begin
            if (s_axi_wready && s_axi_wvalid) begin
                if (write_burst_reg != 2'b00) begin
                    write_addr_next = write_addr_reg + (1 << write_size_reg);
                end
                write_count_next = write_count_reg - 1;
                write_last_next = write_count_next == 0;
                if (write_count_reg > 0) begin
                    write_addr_valid_next = 1'b1;
                    state_next = STATE_BURST;
                end else begin
                    write_addr_valid_next = 1'b0;
                    if (s_axi_bready || !s_axi_bvalid) begin
                        s_axi_bid_next = write_id_reg;
                        s_axi_bvalid_next = 1'b1;
                        s_axi_awready_next = 1'b1;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_RESP;
                    end
                end
            end else begin
                state_next = STATE_BURST;
            end
        end
        STATE_RESP: begin
            if (s_axi_bready || !s_axi_bvalid) begin
                s_axi_bid_next = write_id_reg;
                s_axi_bvalid_next = 1'b1;
                s_axi_awready_next = 1'b1;
                state_next = STATE_IDLE;
            end else begin
                state_next = STATE_RESP;
            end
        end
    endcase
end

always @(posedge clk) begin
    state_reg <= state_next;

    write_id_reg <= write_id_next;
    write_addr_reg <= write_addr_next;
    write_lock_reg <= write_lock_next;
    write_cache_reg <= write_cache_next;
    write_prot_reg <= write_prot_next;
    write_qos_reg <= write_qos_next;
    write_region_reg <= write_region_next;
    write_awuser_reg <= write_awuser_next;
    write_addr_valid_reg <= write_addr_valid_next;
    write_last_reg <= write_last_next;
    write_count_reg <= write_count_next;
    write_size_reg <= write_size_next;
    write_burst_reg <= write_burst_next;

    s_axi_awready_reg <= s_axi_awready_next;
    s_axi_bid_reg <= s_axi_bid_next;
    s_axi_bvalid_reg <= s_axi_bvalid_next;

    if (rst) begin
        state_reg <= STATE_IDLE;

        write_addr_valid_reg <= 1'b0;

        s_axi_awready_reg <= 1'b0;
        s_axi_bvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
