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
 * AXI4 双端口 RAM
 *
 * 模块目录
 * 1) A/B 两个端口在独立时钟下各自提供完整 AXI 从接口。
 * 2) 每个端口通过 `axi_ram_wr_rd_if` 生成抽象 RAM 命令与响应。
 * 3) 共享存储阵列并发处理两路命令流。
 */
module axi_dp_ram #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 16,
    // WSTRB 位宽（按字节 lane）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // ID 信号位宽
    parameter ID_WIDTH = 8,
    // A 端口输出额外流水寄存器开关
    parameter A_PIPELINE_OUTPUT = 0,
    // B 端口输出额外流水寄存器开关
    parameter B_PIPELINE_OUTPUT = 0,
    // A 端口读写突发周期交织开关
    parameter A_INTERLEAVE = 0,
    // B 端口读写突发周期交织开关
    parameter B_INTERLEAVE = 0
)
(
    input  wire                   a_clk, // A 端口时钟。
    input  wire                   a_rst, // A 端口同步复位。

    input  wire                   b_clk, // B 端口时钟。
    input  wire                   b_rst, // B 端口同步复位。

    input  wire [ID_WIDTH-1:0]    s_axi_a_awid, // A 端口 AW ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_a_awaddr, // A 端口 AW 地址。
    input  wire [7:0]             s_axi_a_awlen, // A 端口 AW 突发长度。
    input  wire [2:0]             s_axi_a_awsize, // A 端口 AW 突发尺寸。
    input  wire [1:0]             s_axi_a_awburst, // A 端口 AW 突发类型。
    input  wire                   s_axi_a_awlock, // A 端口 AW 锁属性。
    input  wire [3:0]             s_axi_a_awcache, // A 端口 AW cache 属性。
    input  wire [2:0]             s_axi_a_awprot, // A 端口 AW 保护属性。
    input  wire                   s_axi_a_awvalid, // A 端口 AWVALID。
    output wire                   s_axi_a_awready, // A 端口 AWREADY。
    input  wire [DATA_WIDTH-1:0]  s_axi_a_wdata, // A 端口 W 数据。
    input  wire [STRB_WIDTH-1:0]  s_axi_a_wstrb, // A 端口 W 字节使能。
    input  wire                   s_axi_a_wlast, // A 端口 WLAST。
    input  wire                   s_axi_a_wvalid, // A 端口 WVALID。
    output wire                   s_axi_a_wready, // A 端口 WREADY。
    output wire [ID_WIDTH-1:0]    s_axi_a_bid, // A 端口 BID。
    output wire [1:0]             s_axi_a_bresp, // A 端口 BRESP。
    output wire                   s_axi_a_bvalid, // A 端口 BVALID。
    input  wire                   s_axi_a_bready, // A 端口 BREADY。
    input  wire [ID_WIDTH-1:0]    s_axi_a_arid, // A 端口 AR ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_a_araddr, // A 端口 AR 地址。
    input  wire [7:0]             s_axi_a_arlen, // A 端口 AR 突发长度。
    input  wire [2:0]             s_axi_a_arsize, // A 端口 AR 突发尺寸。
    input  wire [1:0]             s_axi_a_arburst, // A 端口 AR 突发类型。
    input  wire                   s_axi_a_arlock, // A 端口 AR 锁属性。
    input  wire [3:0]             s_axi_a_arcache, // A 端口 AR cache 属性。
    input  wire [2:0]             s_axi_a_arprot, // A 端口 AR 保护属性。
    input  wire                   s_axi_a_arvalid, // A 端口 ARVALID。
    output wire                   s_axi_a_arready, // A 端口 ARREADY。
    output wire [ID_WIDTH-1:0]    s_axi_a_rid, // A 端口 RID。
    output wire [DATA_WIDTH-1:0]  s_axi_a_rdata, // A 端口 RDATA。
    output wire [1:0]             s_axi_a_rresp, // A 端口 RRESP。
    output wire                   s_axi_a_rlast, // A 端口 RLAST。
    output wire                   s_axi_a_rvalid, // A 端口 RVALID。
    input  wire                   s_axi_a_rready, // A 端口 RREADY。

    input  wire [ID_WIDTH-1:0]    s_axi_b_awid, // B 端口 AW ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_b_awaddr, // B 端口 AW 地址。
    input  wire [7:0]             s_axi_b_awlen, // B 端口 AW 突发长度。
    input  wire [2:0]             s_axi_b_awsize, // B 端口 AW 突发尺寸。
    input  wire [1:0]             s_axi_b_awburst, // B 端口 AW 突发类型。
    input  wire                   s_axi_b_awlock, // B 端口 AW 锁属性。
    input  wire [3:0]             s_axi_b_awcache, // B 端口 AW cache 属性。
    input  wire [2:0]             s_axi_b_awprot, // B 端口 AW 保护属性。
    input  wire                   s_axi_b_awvalid, // B 端口 AWVALID。
    output wire                   s_axi_b_awready, // B 端口 AWREADY。
    input  wire [DATA_WIDTH-1:0]  s_axi_b_wdata, // B 端口 W 数据。
    input  wire [STRB_WIDTH-1:0]  s_axi_b_wstrb, // B 端口 W 字节使能。
    input  wire                   s_axi_b_wlast, // B 端口 WLAST。
    input  wire                   s_axi_b_wvalid, // B 端口 WVALID。
    output wire                   s_axi_b_wready, // B 端口 WREADY。
    output wire [ID_WIDTH-1:0]    s_axi_b_bid, // B 端口 BID。
    output wire [1:0]             s_axi_b_bresp, // B 端口 BRESP。
    output wire                   s_axi_b_bvalid, // B 端口 BVALID。
    input  wire                   s_axi_b_bready, // B 端口 BREADY。
    input  wire [ID_WIDTH-1:0]    s_axi_b_arid, // B 端口 AR ID。
    input  wire [ADDR_WIDTH-1:0]  s_axi_b_araddr, // B 端口 AR 地址。
    input  wire [7:0]             s_axi_b_arlen, // B 端口 AR 突发长度。
    input  wire [2:0]             s_axi_b_arsize, // B 端口 AR 突发尺寸。
    input  wire [1:0]             s_axi_b_arburst, // B 端口 AR 突发类型。
    input  wire                   s_axi_b_arlock, // B 端口 AR 锁属性。
    input  wire [3:0]             s_axi_b_arcache, // B 端口 AR cache 属性。
    input  wire [2:0]             s_axi_b_arprot, // B 端口 AR 保护属性。
    input  wire                   s_axi_b_arvalid, // B 端口 ARVALID。
    output wire                   s_axi_b_arready, // B 端口 ARREADY。
    output wire [ID_WIDTH-1:0]    s_axi_b_rid, // B 端口 RID。
    output wire [DATA_WIDTH-1:0]  s_axi_b_rdata, // B 端口 RDATA。
    output wire [1:0]             s_axi_b_rresp, // B 端口 RRESP。
    output wire                   s_axi_b_rlast, // B 端口 RLAST。
    output wire                   s_axi_b_rvalid, // B 端口 RVALID。
    input  wire                   s_axi_b_rready // B 端口 RREADY。
);

parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH); // 有效存储地址位宽(按字节对齐后可寻址的深度位数)。
parameter WORD_WIDTH = STRB_WIDTH; // 每拍可按字节写掩码控制的分段数量。
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH; // 每个分段的位宽，用于按 wstrb 分段写入。

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

wire [ID_WIDTH-1:0]    ram_a_cmd_id; // Port A 当前 RAM 命令所属事务 ID(读响应时回传)。
wire [ADDR_WIDTH-1:0]  ram_a_cmd_addr; // Port A RAM 命令地址(字节地址)。
wire [DATA_WIDTH-1:0]  ram_a_cmd_wr_data; // Port A 写命令数据。
wire [STRB_WIDTH-1:0]  ram_a_cmd_wr_strb; // Port A 写命令字节使能；某位为 1 才更新对应 byte lane。
wire                   ram_a_cmd_wr_en; // Port A 写命令脉冲；写事务每个 beat 会拉高。
wire                   ram_a_cmd_rd_en; // Port A 读命令脉冲；读事务每个 beat 会拉高。
wire                   ram_a_cmd_last; // Port A 当前命令是否为 burst 最后一个 beat。
wire                   ram_a_cmd_ready; // Port A RAM 侧 ready；当读响应寄存器可接收新结果时为 1。
reg  [ID_WIDTH-1:0]    ram_a_rd_resp_id_reg = {ID_WIDTH{1'b0}}; // Port A 读响应 ID 寄存器；读命令被接受时更新。
reg  [DATA_WIDTH-1:0]  ram_a_rd_resp_data_reg = {DATA_WIDTH{1'b0}}; // Port A 读响应数据寄存器；读命令被接受时从 mem 取数。
reg                    ram_a_rd_resp_last_reg = 1'b0; // Port A 读响应 last 标记寄存器；跟随命令 last。
reg                    ram_a_rd_resp_valid_reg = 1'b0; // Port A 读响应有效位；响应未被消费期间保持为 1。
wire                   ram_a_rd_resp_ready; // Port A 读响应 ready(由接口模块回传)。

wire [ID_WIDTH-1:0]    ram_b_cmd_id; // Port B 当前 RAM 命令所属事务 ID(读响应时回传)。
wire [ADDR_WIDTH-1:0]  ram_b_cmd_addr; // Port B RAM 命令地址(字节地址)。
wire [DATA_WIDTH-1:0]  ram_b_cmd_wr_data; // Port B 写命令数据。
wire [STRB_WIDTH-1:0]  ram_b_cmd_wr_strb; // Port B 写命令字节使能；某位为 1 才更新对应 byte lane。
wire                   ram_b_cmd_wr_en; // Port B 写命令脉冲；写事务每个 beat 会拉高。
wire                   ram_b_cmd_rd_en; // Port B 读命令脉冲；读事务每个 beat 会拉高。
wire                   ram_b_cmd_last; // Port B 当前命令是否为 burst 最后一个 beat。
wire                   ram_b_cmd_ready; // Port B RAM 侧 ready；当读响应寄存器可接收新结果时为 1。
reg  [ID_WIDTH-1:0]    ram_b_rd_resp_id_reg = {ID_WIDTH{1'b0}}; // Port B 读响应 ID 寄存器；读命令被接受时更新。
reg  [DATA_WIDTH-1:0]  ram_b_rd_resp_data_reg = {DATA_WIDTH{1'b0}}; // Port B 读响应数据寄存器；读命令被接受时从 mem 取数。
reg                    ram_b_rd_resp_last_reg = 1'b0; // Port B 读响应 last 标记寄存器；跟随命令 last。
reg                    ram_b_rd_resp_valid_reg = 1'b0; // Port B 读响应有效位；响应未被消费期间保持为 1。
wire                   ram_b_rd_resp_ready; // Port B 读响应 ready(由接口模块回传)。

axi_ram_wr_rd_if #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .AWUSER_ENABLE(0),
    .WUSER_ENABLE(0),
    .BUSER_ENABLE(0),
    .ARUSER_ENABLE(0),
    .RUSER_ENABLE(0),
    .PIPELINE_OUTPUT(A_PIPELINE_OUTPUT),
    .INTERLEAVE(A_INTERLEAVE)
)
a_if (
    .clk(a_clk),
    .rst(a_rst),

    /*
     * AXI 从接口
     */
    .s_axi_awid(s_axi_a_awid),
    .s_axi_awaddr(s_axi_a_awaddr),
    .s_axi_awlen(s_axi_a_awlen),
    .s_axi_awsize(s_axi_a_awsize),
    .s_axi_awburst(s_axi_a_awburst),
    .s_axi_awlock(s_axi_a_awlock),
    .s_axi_awcache(s_axi_a_awcache),
    .s_axi_awprot(s_axi_a_awprot),
    .s_axi_awqos(4'd0),
    .s_axi_awregion(4'd0),
    .s_axi_awuser(0),
    .s_axi_awvalid(s_axi_a_awvalid),
    .s_axi_awready(s_axi_a_awready),
    .s_axi_wdata(s_axi_a_wdata),
    .s_axi_wstrb(s_axi_a_wstrb),
    .s_axi_wlast(s_axi_a_wlast),
    .s_axi_wuser(0),
    .s_axi_wvalid(s_axi_a_wvalid),
    .s_axi_wready(s_axi_a_wready),
    .s_axi_bid(s_axi_a_bid),
    .s_axi_bresp(s_axi_a_bresp),
    .s_axi_buser(),
    .s_axi_bvalid(s_axi_a_bvalid),
    .s_axi_bready(s_axi_a_bready),
    .s_axi_arid(s_axi_a_arid),
    .s_axi_araddr(s_axi_a_araddr),
    .s_axi_arlen(s_axi_a_arlen),
    .s_axi_arsize(s_axi_a_arsize),
    .s_axi_arburst(s_axi_a_arburst),
    .s_axi_arlock(s_axi_a_arlock),
    .s_axi_arcache(s_axi_a_arcache),
    .s_axi_arprot(s_axi_a_arprot),
    .s_axi_arqos(4'd0),
    .s_axi_arregion(4'd0),
    .s_axi_aruser(0),
    .s_axi_arvalid(s_axi_a_arvalid),
    .s_axi_arready(s_axi_a_arready),
    .s_axi_rid(s_axi_a_rid),
    .s_axi_rdata(s_axi_a_rdata),
    .s_axi_rresp(s_axi_a_rresp),
    .s_axi_rlast(s_axi_a_rlast),
    .s_axi_ruser(),
    .s_axi_rvalid(s_axi_a_rvalid),
    .s_axi_rready(s_axi_a_rready),

    /*
     * RAM 接口
     */
    .ram_cmd_id(ram_a_cmd_id),
    .ram_cmd_addr(ram_a_cmd_addr),
    .ram_cmd_lock(),
    .ram_cmd_cache(),
    .ram_cmd_prot(),
    .ram_cmd_qos(),
    .ram_cmd_region(),
    .ram_cmd_auser(),
    .ram_cmd_wr_data(ram_a_cmd_wr_data),
    .ram_cmd_wr_strb(ram_a_cmd_wr_strb),
    .ram_cmd_wr_user(),
    .ram_cmd_wr_en(ram_a_cmd_wr_en),
    .ram_cmd_rd_en(ram_a_cmd_rd_en),
    .ram_cmd_last(ram_a_cmd_last),
    .ram_cmd_ready(ram_a_cmd_ready),
    .ram_rd_resp_id(ram_a_rd_resp_id_reg),
    .ram_rd_resp_data(ram_a_rd_resp_data_reg),
    .ram_rd_resp_last(ram_a_rd_resp_last_reg),
    .ram_rd_resp_user(0),
    .ram_rd_resp_valid(ram_a_rd_resp_valid_reg),
    .ram_rd_resp_ready(ram_a_rd_resp_ready)
);

axi_ram_wr_rd_if #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .STRB_WIDTH(STRB_WIDTH),
    .ID_WIDTH(ID_WIDTH),
    .AWUSER_ENABLE(0),
    .WUSER_ENABLE(0),
    .BUSER_ENABLE(0),
    .ARUSER_ENABLE(0),
    .RUSER_ENABLE(0),
    .PIPELINE_OUTPUT(B_PIPELINE_OUTPUT),
    .INTERLEAVE(B_INTERLEAVE)
)
b_if (
    .clk(b_clk),
    .rst(b_rst),

    /*
     * AXI 从接口
     */
    .s_axi_awid(s_axi_b_awid),
    .s_axi_awaddr(s_axi_b_awaddr),
    .s_axi_awlen(s_axi_b_awlen),
    .s_axi_awsize(s_axi_b_awsize),
    .s_axi_awburst(s_axi_b_awburst),
    .s_axi_awlock(s_axi_b_awlock),
    .s_axi_awcache(s_axi_b_awcache),
    .s_axi_awprot(s_axi_b_awprot),
    .s_axi_awqos(4'd0),
    .s_axi_awregion(4'd0),
    .s_axi_awuser(0),
    .s_axi_awvalid(s_axi_b_awvalid),
    .s_axi_awready(s_axi_b_awready),
    .s_axi_wdata(s_axi_b_wdata),
    .s_axi_wstrb(s_axi_b_wstrb),
    .s_axi_wlast(s_axi_b_wlast),
    .s_axi_wuser(0),
    .s_axi_wvalid(s_axi_b_wvalid),
    .s_axi_wready(s_axi_b_wready),
    .s_axi_bid(s_axi_b_bid),
    .s_axi_bresp(s_axi_b_bresp),
    .s_axi_buser(),
    .s_axi_bvalid(s_axi_b_bvalid),
    .s_axi_bready(s_axi_b_bready),
    .s_axi_arid(s_axi_b_arid),
    .s_axi_araddr(s_axi_b_araddr),
    .s_axi_arlen(s_axi_b_arlen),
    .s_axi_arsize(s_axi_b_arsize),
    .s_axi_arburst(s_axi_b_arburst),
    .s_axi_arlock(s_axi_b_arlock),
    .s_axi_arcache(s_axi_b_arcache),
    .s_axi_arprot(s_axi_b_arprot),
    .s_axi_arqos(4'd0),
    .s_axi_arregion(4'd0),
    .s_axi_aruser(0),
    .s_axi_arvalid(s_axi_b_arvalid),
    .s_axi_arready(s_axi_b_arready),
    .s_axi_rid(s_axi_b_rid),
    .s_axi_rdata(s_axi_b_rdata),
    .s_axi_rresp(s_axi_b_rresp),
    .s_axi_rlast(s_axi_b_rlast),
    .s_axi_ruser(),
    .s_axi_rvalid(s_axi_b_rvalid),
    .s_axi_rready(s_axi_b_rready),

    /*
     * RAM 接口
     */
    .ram_cmd_id(ram_b_cmd_id),
    .ram_cmd_addr(ram_b_cmd_addr),
    .ram_cmd_lock(),
    .ram_cmd_cache(),
    .ram_cmd_prot(),
    .ram_cmd_qos(),
    .ram_cmd_region(),
    .ram_cmd_auser(),
    .ram_cmd_wr_data(ram_b_cmd_wr_data),
    .ram_cmd_wr_strb(ram_b_cmd_wr_strb),
    .ram_cmd_wr_user(),
    .ram_cmd_wr_en(ram_b_cmd_wr_en),
    .ram_cmd_rd_en(ram_b_cmd_rd_en),
    .ram_cmd_last(ram_b_cmd_last),
    .ram_cmd_ready(ram_b_cmd_ready),
    .ram_rd_resp_id(ram_b_rd_resp_id_reg),
    .ram_rd_resp_data(ram_b_rd_resp_data_reg),
    .ram_rd_resp_last(ram_b_rd_resp_last_reg),
    .ram_rd_resp_user(0),
    .ram_rd_resp_valid(ram_b_rd_resp_valid_reg),
    .ram_rd_resp_ready(ram_b_rd_resp_ready)
);

// RAM 风格属性示例：(* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[(2**VALID_ADDR_WIDTH)-1:0]; // 共享双口 RAM 存储体；两端口均可并发读写此数组。

wire [VALID_ADDR_WIDTH-1:0] addr_a_valid = ram_a_cmd_addr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // Port A 有效字地址(去掉低位字节偏移)。
wire [VALID_ADDR_WIDTH-1:0] addr_b_valid = ram_b_cmd_addr >> (ADDR_WIDTH - VALID_ADDR_WIDTH); // Port B 有效字地址(去掉低位字节偏移)。

integer i, j; // 初始化和按字节掩码写入时使用的循环变量。

initial begin
    // 使用两层嵌套循环，降低单层循环迭代次数
    // 规避综合器对大循环计数的告警
    for (i = 0; i < 2**VALID_ADDR_WIDTH; i = i + 2**(VALID_ADDR_WIDTH/2)) begin
        for (j = i; j < i + 2**(VALID_ADDR_WIDTH/2); j = j + 1) begin
            mem[j] = 0;
        end
    end
end

assign ram_a_cmd_ready = !ram_a_rd_resp_valid_reg || ram_a_rd_resp_ready;

always @(posedge a_clk) begin
    ram_a_rd_resp_valid_reg <= ram_a_rd_resp_valid_reg && !ram_a_rd_resp_ready;

    if (ram_a_cmd_rd_en && ram_a_cmd_ready) begin
        ram_a_rd_resp_id_reg <= ram_a_cmd_id;
        ram_a_rd_resp_data_reg <= mem[addr_a_valid];
        ram_a_rd_resp_last_reg <= ram_a_cmd_last;
        ram_a_rd_resp_valid_reg <= 1'b1;
    end else if (ram_a_cmd_wr_en && ram_a_cmd_ready) begin
        for (i = 0; i < WORD_WIDTH; i = i + 1) begin
            if (ram_a_cmd_wr_strb[i]) begin
                mem[addr_a_valid][WORD_SIZE*i +: WORD_SIZE] <= ram_a_cmd_wr_data[WORD_SIZE*i +: WORD_SIZE];
            end
        end
    end

    if (a_rst) begin
        ram_a_rd_resp_valid_reg <= 1'b0;
    end
end

assign ram_b_cmd_ready = !ram_b_rd_resp_valid_reg || ram_b_rd_resp_ready;

always @(posedge b_clk) begin
    ram_b_rd_resp_valid_reg <= ram_b_rd_resp_valid_reg && !ram_b_rd_resp_ready;

    if (ram_b_cmd_rd_en && ram_b_cmd_ready) begin
        ram_b_rd_resp_id_reg <= ram_b_cmd_id;
        ram_b_rd_resp_data_reg <= mem[addr_b_valid];
        ram_b_rd_resp_last_reg <= ram_b_cmd_last;
        ram_b_rd_resp_valid_reg <= 1'b1;
    end else if (ram_b_cmd_wr_en && ram_b_cmd_ready) begin
        for (i = 0; i < WORD_WIDTH; i = i + 1) begin
            if (ram_b_cmd_wr_strb[i]) begin
                mem[addr_b_valid][WORD_SIZE*i +: WORD_SIZE] <= ram_b_cmd_wr_data[WORD_SIZE*i +: WORD_SIZE];
            end
        end
    end

    if (b_rst) begin
        ram_b_rd_resp_valid_reg <= 1'b0;
    end
end

endmodule

`resetall
