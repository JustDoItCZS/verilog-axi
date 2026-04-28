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
 * AXI4 RAM 读写接口
 *
 * 模块目录
 * 1) 实例化独立写命令生成器（`axi_ram_wr_if`）。
 * 2) 实例化独立读命令生成器（`axi_ram_rd_if`）。
 * 3) 在读写两路命令流间仲裁共享 RAM 命令端口。
 * 4) 通过可选交织策略控制长突发场景下的公平性。
 */
module axi_ram_wr_rd_if #
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
    parameter BUSER_WIDTH = 1,
    // 是否透传 aruser 信号
    parameter ARUSER_ENABLE = 0,
    // aruser 信号位宽
    parameter ARUSER_WIDTH = 1,
    // 是否透传 ruser 信号
    parameter RUSER_ENABLE = 0,
    // ruser 信号位宽
    parameter RUSER_WIDTH = 1,
    // auser 输出位宽
    parameter AUSER_WIDTH = (ARUSER_ENABLE && (!AWUSER_ENABLE || ARUSER_WIDTH > AWUSER_WIDTH)) ? ARUSER_WIDTH : AWUSER_WIDTH,
    // 输出端额外流水寄存器开关
    parameter PIPELINE_OUTPUT = 0,
    // 读写突发周期交织开关
    parameter INTERLEAVE = 0
)
(
    input  wire                     clk, // 读写子接口与仲裁逻辑共用时钟。
    input  wire                     rst, // 共用同步复位。

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
    output wire [1:0]               s_axi_bresp, // AXI BRESP（写响应码）。
    output wire [BUSER_WIDTH-1:0]   s_axi_buser, // AXI BUSER（写响应用户旁带）。
    output wire                     s_axi_bvalid, // AXI BVALID（写响应有效）。
    input  wire                     s_axi_bready, // AXI BREADY（写响应就绪）。
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
    output wire [1:0]               s_axi_rresp, // AXI RRESP（读响应码）。
    output wire                     s_axi_rlast, // AXI RLAST（读突发最后一拍）。
    output wire [RUSER_WIDTH-1:0]   s_axi_ruser, // AXI RUSER（读用户旁带）。
    output wire                     s_axi_rvalid, // AXI RVALID（读数据有效）。
    input  wire                     s_axi_rready, // AXI RREADY（读数据就绪）。

    /*
     * RAM 接口
     */
    output wire [ID_WIDTH-1:0]      ram_cmd_id, // 仲裁后的 RAM 命令 ID。
    output wire [ADDR_WIDTH-1:0]    ram_cmd_addr, // 仲裁后的 RAM 命令地址。
    output wire                     ram_cmd_lock, // 仲裁后的锁属性。
    output wire [3:0]               ram_cmd_cache, // 仲裁后的 cache 属性。
    output wire [2:0]               ram_cmd_prot, // 仲裁后的保护属性。
    output wire [3:0]               ram_cmd_qos, // 仲裁后的 QoS 属性。
    output wire [3:0]               ram_cmd_region, // 仲裁后的 region 属性。
    output wire [AUSER_WIDTH-1:0]   ram_cmd_auser, // 仲裁后的 AUSER 属性。
    output wire [DATA_WIDTH-1:0]    ram_cmd_wr_data, // RAM 命令端口写数据载荷。
    output wire [STRB_WIDTH-1:0]    ram_cmd_wr_strb, // RAM 命令端口写字节使能。
    output wire [WUSER_WIDTH-1:0]   ram_cmd_wr_user, // RAM 命令端口写用户旁带。
    output wire                     ram_cmd_wr_en, // 仲裁后的写命令有效。
    output wire                     ram_cmd_rd_en, // 仲裁后的读命令有效。
    output wire                     ram_cmd_last, // 当前选中命令流最后一拍标记。
    input  wire                     ram_cmd_ready, // RAM 命令 ready。
    input  wire [ID_WIDTH-1:0]      ram_rd_resp_id, // RAM 读响应 ID。
    input  wire [DATA_WIDTH-1:0]    ram_rd_resp_data, // RAM 读响应数据。
    input  wire                     ram_rd_resp_last, // RAM 读响应最后一拍标记。
    input  wire [RUSER_WIDTH-1:0]   ram_rd_resp_user, // RAM 读响应用户旁带。
    input  wire                     ram_rd_resp_valid, // RAM 读响应有效。
    output wire                     ram_rd_resp_ready // RAM 读响应 ready。
);


wire [ID_WIDTH-1:0]      ram_wr_cmd_id; // 写侧生成的命令 ID。
wire [ADDR_WIDTH-1:0]    ram_wr_cmd_addr; // 写侧生成的命令地址。
wire                     ram_wr_cmd_lock; // 写侧生成的锁属性。
wire [3:0]               ram_wr_cmd_cache; // 写侧生成的 cache 属性。
wire [2:0]               ram_wr_cmd_prot; // 写侧生成的保护属性。
wire [3:0]               ram_wr_cmd_qos; // 写侧生成的 QoS 属性。
wire [3:0]               ram_wr_cmd_region; // 写侧生成的 region 属性。
wire [AWUSER_WIDTH-1:0]  ram_wr_cmd_auser; // 写侧生成的 AUSER。
wire                     ram_wr_cmd_en; // 写侧命令有效。
wire                     ram_wr_cmd_last; // 写侧命令最后一拍标记。
wire                     ram_wr_cmd_ready; // 仲裁器返回的写侧命令 ready。

wire [ID_WIDTH-1:0]      ram_rd_cmd_id; // 读侧生成的命令 ID。
wire [ADDR_WIDTH-1:0]    ram_rd_cmd_addr; // 读侧生成的命令地址。
wire                     ram_rd_cmd_lock; // 读侧生成的锁属性。
wire [3:0]               ram_rd_cmd_cache; // 读侧生成的 cache 属性。
wire [2:0]               ram_rd_cmd_prot; // 读侧生成的保护属性。
wire [3:0]               ram_rd_cmd_qos; // 读侧生成的 QoS 属性。
wire [3:0]               ram_rd_cmd_region; // 读侧生成的 region 属性。
wire [AWUSER_WIDTH-1:0]  ram_rd_cmd_auser; // 读侧生成的 AUSER。
wire                     ram_rd_cmd_en; // 读侧命令有效。
wire                     ram_rd_cmd_last; // 读侧命令最后一拍标记。
wire                     ram_rd_cmd_ready; // 仲裁器返回的读侧命令 ready。

axi_ram_wr_if #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .AWUSER_ENABLE(AWUSER_ENABLE),
    .AWUSER_WIDTH(AWUSER_WIDTH),
    .WUSER_ENABLE(WUSER_ENABLE),
    .WUSER_WIDTH(WUSER_WIDTH),
    .BUSER_ENABLE(BUSER_ENABLE),
    .BUSER_WIDTH(BUSER_WIDTH)
)
axi_ram_wr_if_inst (
    .clk(clk),
    .rst(rst),
    .s_axi_awid(s_axi_awid),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awlen(s_axi_awlen),
    .s_axi_awsize(s_axi_awsize),
    .s_axi_awburst(s_axi_awburst),
    .s_axi_awlock(s_axi_awlock),
    .s_axi_awcache(s_axi_awcache),
    .s_axi_awprot(s_axi_awprot),
    .s_axi_awqos(s_axi_awqos),
    .s_axi_awregion(s_axi_awregion),
    .s_axi_awuser(s_axi_awuser),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wlast(s_axi_wlast),
    .s_axi_wuser(s_axi_wuser),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bid(s_axi_bid),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_buser(s_axi_buser),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .ram_wr_cmd_id(ram_wr_cmd_id),
    .ram_wr_cmd_addr(ram_wr_cmd_addr),
    .ram_wr_cmd_lock(ram_wr_cmd_lock),
    .ram_wr_cmd_cache(ram_wr_cmd_cache),
    .ram_wr_cmd_prot(ram_wr_cmd_prot),
    .ram_wr_cmd_qos(ram_wr_cmd_qos),
    .ram_wr_cmd_region(ram_wr_cmd_region),
    .ram_wr_cmd_auser(ram_wr_cmd_auser),
    .ram_wr_cmd_data(ram_cmd_wr_data),
    .ram_wr_cmd_strb(ram_cmd_wr_strb),
    .ram_wr_cmd_user(ram_cmd_wr_user),
    .ram_wr_cmd_en(ram_wr_cmd_en),
    .ram_wr_cmd_last(ram_wr_cmd_last),
    .ram_wr_cmd_ready(ram_wr_cmd_ready)
);

axi_ram_rd_if #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .ARUSER_ENABLE(ARUSER_ENABLE),
    .ARUSER_WIDTH(ARUSER_WIDTH),
    .RUSER_ENABLE(RUSER_ENABLE),
    .RUSER_WIDTH(RUSER_WIDTH),
    .PIPELINE_OUTPUT(PIPELINE_OUTPUT)
)
axi_ram_rd_if_inst (
    .clk(clk),
    .rst(rst),
    .s_axi_arid(s_axi_arid),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arlen(s_axi_arlen),
    .s_axi_arsize(s_axi_arsize),
    .s_axi_arburst(s_axi_arburst),
    .s_axi_arlock(s_axi_arlock),
    .s_axi_arcache(s_axi_arcache),
    .s_axi_arprot(s_axi_arprot),
    .s_axi_arqos(s_axi_arqos),
    .s_axi_arregion(s_axi_arregion),
    .s_axi_aruser(s_axi_aruser),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rid(s_axi_rid),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rlast(s_axi_rlast),
    .s_axi_ruser(s_axi_ruser),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),
    .ram_rd_cmd_id(ram_rd_cmd_id),
    .ram_rd_cmd_addr(ram_rd_cmd_addr),
    .ram_rd_cmd_lock(ram_rd_cmd_lock),
    .ram_rd_cmd_cache(ram_rd_cmd_cache),
    .ram_rd_cmd_prot(ram_rd_cmd_prot),
    .ram_rd_cmd_qos(ram_rd_cmd_qos),
    .ram_rd_cmd_region(ram_rd_cmd_region),
    .ram_rd_cmd_auser(ram_rd_cmd_auser),
    .ram_rd_cmd_en(ram_rd_cmd_en),
    .ram_rd_cmd_last(ram_rd_cmd_last),
    .ram_rd_cmd_ready(ram_rd_cmd_ready),
    .ram_rd_resp_id(ram_rd_resp_id),
    .ram_rd_resp_data(ram_rd_resp_data),
    .ram_rd_resp_last(ram_rd_resp_last),
    .ram_rd_resp_user(ram_rd_resp_user),
    .ram_rd_resp_valid(ram_rd_resp_valid),
    .ram_rd_resp_ready(ram_rd_resp_ready)
);

// 仲裁逻辑
reg read_eligible; // 本拍可接纳读命令时为真。
reg write_eligible; // 本拍可接纳写命令时为真。

reg write_en; // 仲裁决策：本拍授予写命令。
reg read_en; // 仲裁决策：本拍授予读命令。

reg last_read_reg = 1'b0, last_read_next; // 公平性提示：记录上次授予的命令类型。
reg transaction_reg = 1'b0, transaction_next; // 跟踪当前突发事务是否仍在进行。

assign ram_cmd_wr_en = write_en;
assign ram_cmd_rd_en = read_en;

assign ram_cmd_id     = ram_cmd_rd_en ? ram_rd_cmd_id     : ram_wr_cmd_id;
assign ram_cmd_addr   = ram_cmd_rd_en ? ram_rd_cmd_addr   : ram_wr_cmd_addr;
assign ram_cmd_lock   = ram_cmd_rd_en ? ram_rd_cmd_lock   : ram_wr_cmd_lock;
assign ram_cmd_cache  = ram_cmd_rd_en ? ram_rd_cmd_cache  : ram_wr_cmd_cache;
assign ram_cmd_prot   = ram_cmd_rd_en ? ram_rd_cmd_prot   : ram_wr_cmd_prot;
assign ram_cmd_qos    = ram_cmd_rd_en ? ram_rd_cmd_qos    : ram_wr_cmd_qos;
assign ram_cmd_region = ram_cmd_rd_en ? ram_rd_cmd_region : ram_wr_cmd_region;
assign ram_cmd_auser  = ram_cmd_rd_en ? ram_rd_cmd_auser  : ram_wr_cmd_auser;
assign ram_cmd_last   = ram_cmd_rd_en ? ram_rd_cmd_last   : ram_wr_cmd_last;

assign ram_wr_cmd_ready = ram_cmd_ready && write_en;
assign ram_rd_cmd_ready = ram_cmd_ready && read_en;

always @* begin
    write_en = 1'b0;
    read_en = 1'b0;

    last_read_next = last_read_reg;
    transaction_next = transaction_reg;

    write_eligible = ram_wr_cmd_en && ram_cmd_ready;
    read_eligible = ram_rd_cmd_en && ram_cmd_ready;

    if (write_eligible && (!read_eligible || last_read_reg || (!INTERLEAVE && transaction_reg)) && (INTERLEAVE || !transaction_reg || !last_read_reg)) begin
        last_read_next = 1'b0;
        transaction_next = !ram_wr_cmd_last;

        write_en = 1'b1;
    end else if (read_eligible && (INTERLEAVE || !transaction_reg || last_read_reg)) begin
        last_read_next = 1'b1;
        transaction_next = !ram_rd_cmd_last;

        read_en = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        last_read_reg <= 1'b0;
        transaction_reg <= 1'b0;
    end else begin
        last_read_reg <= last_read_next;
        transaction_reg <= transaction_next;
    end
end

endmodule

`resetall
