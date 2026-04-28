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
 * AXI-Lite 寄存器接口模块（读通道）
 *
 * 模块目录
 * 1) 接收 AXI-Lite 的 AR 请求。
 * 2) 将 AR 转换为简单的寄存器读握手（reg_rd_*）。
 * 3) 寄存器侧应答或超时时返回包含 reg_rd_data 的 R 响应。
 */
module axil_reg_if_rd #
(
    // 数据总线位宽
    parameter DATA_WIDTH = 32,
    // 地址总线位宽
    parameter ADDR_WIDTH = 32,
    // WSTRB 位宽（按字节）
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    // 超时延迟（周期）
    parameter TIMEOUT = 4
)
(
    input  wire                   clk, // AXI 到寄存器读桥接时钟。
    input  wire                   rst, // 采样读请求/响应状态的同步复位。

    /*
     * AXI-Lite 从接口
     */
    input  wire [ADDR_WIDTH-1:0]  s_axil_araddr, // AXI-Lite 读地址；AR 暂存为空时采样。
    input  wire [2:0]             s_axil_arprot, // AXI-Lite 读保护属性（接收但内部不使用）。
    input  wire                   s_axil_arvalid, // 来自主机的 AXI-Lite ARVALID。
    output wire                   s_axil_arready, // AXI-Lite ARREADY；无 pending AR 时拉高。
    output wire [DATA_WIDTH-1:0]  s_axil_rdata, // 从寄存器侧返回的 AXI-Lite 读数据。
    output wire [1:0]             s_axil_rresp, // AXI-Lite RRESP；本桥接固定 OKAY。
    output wire                   s_axil_rvalid, // AXI-Lite RVALID；读完成/超时后置位。
    input  wire                   s_axil_rready, // 来自主机的 AXI-Lite RREADY。

    /*
     * 寄存器接口
     */
    output wire [ADDR_WIDTH-1:0]  reg_rd_addr, // 由采样 AR 导出的寄存器总线读地址。
    output wire                   reg_rd_en, // 等待完成期间寄存器总线读请求有效。
    input  wire [DATA_WIDTH-1:0]  reg_rd_data, // 完成/超时时采样的寄存器总线读数据。
    input  wire                   reg_rd_wait, // 寄存器总线背压；为高时暂停超时计数。
    input  wire                   reg_rd_ack // 寄存器总线完成脉冲。
);

parameter TIMEOUT_WIDTH = $clog2(TIMEOUT); // 表示超时周期所需计数器位宽。

reg [TIMEOUT_WIDTH-1:0] timeout_count_reg = 0, timeout_count_next; // 等待 reg_rd_ack 期间的倒计时。

reg [ADDR_WIDTH-1:0] s_axil_araddr_reg = {ADDR_WIDTH{1'b0}}, s_axil_araddr_next; // 采样后的 AR 地址，转发到寄存器总线。
reg s_axil_arvalid_reg = 1'b0, s_axil_arvalid_next; // 表示已采样 AR 请求处于 pending 状态。
reg [DATA_WIDTH-1:0] s_axil_rdata_reg = {DATA_WIDTH{1'b0}}, s_axil_rdata_next; // 返回给 AXI 主机的 RDATA 寄存器。
reg s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next; // RVALID 标志；响应就绪时置位，R 握手后清零。

reg reg_rd_en_reg = 1'b0, reg_rd_en_next; // 当前寄存器读请求有效标志。

assign s_axil_arready = !s_axil_arvalid_reg;
assign s_axil_rdata = s_axil_rdata_reg;
assign s_axil_rresp = 2'b00;
assign s_axil_rvalid = s_axil_rvalid_reg;

assign reg_rd_addr = s_axil_araddr_reg;
assign reg_rd_en = reg_rd_en_reg;

always @* begin
    timeout_count_next = timeout_count_reg;

    s_axil_araddr_next = s_axil_araddr_reg;
    s_axil_arvalid_next = s_axil_arvalid_reg;
    s_axil_rdata_next = s_axil_rdata_reg;
    s_axil_rvalid_next = s_axil_rvalid_reg && !s_axil_rready;

    if (reg_rd_en_reg && (reg_rd_ack || timeout_count_reg == 0)) begin
        s_axil_arvalid_next = 1'b0;
        s_axil_rdata_next = reg_rd_data;
        s_axil_rvalid_next = 1'b1;
    end

    if (!s_axil_arvalid_reg) begin
        s_axil_araddr_next = s_axil_araddr;
        s_axil_arvalid_next = s_axil_arvalid;
        timeout_count_next = TIMEOUT-1;
    end

    if (reg_rd_en && !reg_rd_wait && timeout_count_reg != 0)begin
        timeout_count_next = timeout_count_reg - 1;
    end

    reg_rd_en_next = s_axil_arvalid_next && !s_axil_rvalid_next;
end

always @(posedge clk) begin
    timeout_count_reg <= timeout_count_next;

    s_axil_araddr_reg <= s_axil_araddr_next;
    s_axil_arvalid_reg <= s_axil_arvalid_next;
    s_axil_rdata_reg <= s_axil_rdata_next;
    s_axil_rvalid_reg <= s_axil_rvalid_next;

    reg_rd_en_reg <= reg_rd_en_next;

    if (rst) begin
        s_axil_arvalid_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
        reg_rd_en_reg <= 1'b0;
    end
end

endmodule

`resetall
