/*

Copyright (c) 2023 Alex Forencich

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
 * AXI4 虚拟 FIFO（原始版）
 *
 * 模块目录：
 * 1) 写侧子模块 axi_vfifo_raw_wr：把分段输入数据缓存后，通过AXI AW/W/B写入外部存储
 * 2) 读侧子模块 axi_vfifo_raw_rd：从外部存储通过AXI AR/R读回，再输出分段数据
 * 3) 复位/使能控制：统一协调cfg_enable/cfg_reset与跨域复位请求
 * 4) FIFO指针联动：通过wr/rd start/finish指针构成环形虚拟FIFO
 */
module axi_vfifo_raw #
(
    // 输入分段位宽
    parameter SEG_WIDTH = 32,
    // 分段数量
    parameter SEG_CNT = 2,
    // AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = SEG_WIDTH*SEG_CNT,
    // AXI 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // AXI WSTRB 位宽（按字节）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 位宽
    parameter AXI_ID_WIDTH = 8,
    // 允许生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // 长度字段位宽
    parameter LEN_WIDTH = AXI_ADDR_WIDTH,
    // AXI 写数据输入 FIFO 深度（全宽字）
    parameter WRITE_FIFO_DEPTH = 64,
    // AXI 最大写突发长度
    parameter WRITE_MAX_BURST_LEN = WRITE_FIFO_DEPTH/4,
    // AXI 读数据输出 FIFO 深度（全宽字）
    parameter READ_FIFO_DEPTH = 128,
    // AXI 最大读突发长度
    parameter READ_MAX_BURST_LEN = WRITE_MAX_BURST_LEN,
    // 水位阈值
    parameter WATERMARK_LEVEL = WRITE_FIFO_DEPTH/2,
    // 是否启用控制输出
    parameter CTRL_OUT_EN = 0
)
(
    input  wire                          clk, // 核心控制时钟（写侧与读侧AXI控制共享）
    input  wire                          rst, // 核心控制复位

    /*
     * 分段数据输入（来自编码逻辑）
     */
    input  wire                          input_clk, // 分段输入时钟域
    input  wire                          input_rst, // 分段输入复位
    output wire                          input_rst_out, // 反馈给输入域上游的复位输出
    output wire                          input_watermark, // 输入水位告警（用于上游节流）
    input  wire [SEG_CNT*SEG_WIDTH-1:0]  input_data, // 分段输入数据
    input  wire [SEG_CNT-1:0]            input_valid, // 分段输入有效
    output wire [SEG_CNT-1:0]            input_ready, // 分段输入就绪

    /*
     * 分段数据输出（到解码逻辑）
     */
    input  wire                          output_clk, // 分段输出时钟域
    input  wire                          output_rst, // 分段输出复位
    output wire                          output_rst_out, // 反馈给输出域下游的复位输出
    output wire [SEG_CNT*SEG_WIDTH-1:0]  output_data, // 分段主数据输出
    output wire [SEG_CNT-1:0]            output_valid, // 分段主数据有效
    input  wire [SEG_CNT-1:0]            output_ready, // 分段主数据就绪
    output wire [SEG_CNT*SEG_WIDTH-1:0]  output_ctrl_data, // 分段控制数据输出（CTRL_OUT_EN启用）
    output wire [SEG_CNT-1:0]            output_ctrl_valid, // 分段控制数据有效
    input  wire [SEG_CNT-1:0]            output_ctrl_ready, // 分段控制数据就绪

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]       m_axi_awid, // AXI写地址ID
    output wire [AXI_ADDR_WIDTH-1:0]     m_axi_awaddr, // AXI写地址
    output wire [7:0]                    m_axi_awlen, // AXI 写突发长度（拍数减 1）
    output wire [2:0]                    m_axi_awsize, // AXI写每拍大小编码
    output wire [1:0]                    m_axi_awburst, // AXI写突发类型
    output wire                          m_axi_awlock, // AXI写锁信号
    output wire [3:0]                    m_axi_awcache, // AXI写缓存属性
    output wire [2:0]                    m_axi_awprot, // AXI写保护属性
    output wire                          m_axi_awvalid, // AXI写地址有效
    input  wire                          m_axi_awready, // AXI写地址就绪
    output wire [AXI_DATA_WIDTH-1:0]     m_axi_wdata, // AXI写数据
    output wire [AXI_STRB_WIDTH-1:0]     m_axi_wstrb, // AXI写字节使能
    output wire                          m_axi_wlast, // AXI写最后一拍
    output wire                          m_axi_wvalid, // AXI写数据有效
    input  wire                          m_axi_wready, // AXI写数据就绪
    input  wire [AXI_ID_WIDTH-1:0]       m_axi_bid, // AXI写响应ID
    input  wire [1:0]                    m_axi_bresp, // AXI写响应码
    input  wire                          m_axi_bvalid, // AXI写响应有效
    output wire                          m_axi_bready, // AXI写响应就绪
    output wire [AXI_ID_WIDTH-1:0]       m_axi_arid, // AXI读地址ID
    output wire [AXI_ADDR_WIDTH-1:0]     m_axi_araddr, // AXI读地址
    output wire [7:0]                    m_axi_arlen, // AXI 读突发长度（拍数减 1）
    output wire [2:0]                    m_axi_arsize, // AXI读每拍大小编码
    output wire [1:0]                    m_axi_arburst, // AXI读突发类型
    output wire                          m_axi_arlock, // AXI读锁信号
    output wire [3:0]                    m_axi_arcache, // AXI读缓存属性
    output wire [2:0]                    m_axi_arprot, // AXI读保护属性
    output wire                          m_axi_arvalid, // AXI读地址有效
    input  wire                          m_axi_arready, // AXI读地址就绪
    input  wire [AXI_ID_WIDTH-1:0]       m_axi_rid, // AXI读数据ID
    input  wire [AXI_DATA_WIDTH-1:0]     m_axi_rdata, // AXI读数据
    input  wire [1:0]                    m_axi_rresp, // AXI读响应码
    input  wire                          m_axi_rlast, // AXI读最后一拍
    input  wire                          m_axi_rvalid, // AXI读数据有效
    output wire                          m_axi_rready, // AXI读数据就绪

    /*
     * 复位同步
     */
    output wire                          rst_req_out, // 对外发布的复位请求（本模块任一域请求复位时拉高）
    input  wire                          rst_req_in, // 来自上层/相邻通道的复位请求输入

    /*
     * 配置
     */
    input  wire [AXI_ADDR_WIDTH-1:0]     cfg_fifo_base_addr, // FIFO映射到外部存储的基地址
    input  wire [LEN_WIDTH-1:0]          cfg_fifo_size_mask, // FIFO容量掩码（环形地址回绕）
    input  wire                          cfg_enable, // 使能虚拟FIFO读写流程
    input  wire                          cfg_reset, // 请求复位FIFO状态与指针

    /*
     * 状态
     */
    output wire [LEN_WIDTH+1-1:0]        sts_fifo_occupancy, // FIFO占用量（由写侧输出）
    output wire                          sts_fifo_empty, // FIFO空标志（由写侧输出）
    output wire                          sts_fifo_full, // FIFO满标志（由写侧输出）
    output wire                          sts_reset, // 当前处于复位/冲刷状态
    output wire                          sts_active, // 当前配置已生效且处于工作状态
    output wire                          sts_write_active, // 写侧活跃状态
    output wire                          sts_read_active // 读侧活跃状态
);

localparam ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH);

reg fifo_reset_reg = 1'b1, fifo_reset_next; // FIFO复位状态寄存器及其下一状态
reg fifo_enable_reg = 1'b0, fifo_enable_next; // FIFO使能状态寄存器及其下一状态
reg [AXI_ADDR_WIDTH-1:0] fifo_base_addr_reg = 0, fifo_base_addr_next; // 生效中的FIFO基地址配置
reg [LEN_WIDTH-1:0] fifo_size_mask_reg = 0, fifo_size_mask_next; // 生效中的FIFO尺寸掩码配置

assign sts_reset = fifo_reset_reg;
assign sts_active = fifo_enable_reg;

wire [LEN_WIDTH+1-1:0] wr_start_ptr; // 写侧已申请写出的起始指针（提供给读侧作边界）
wire [LEN_WIDTH+1-1:0] wr_finish_ptr; // 写侧已完成写入的指针（B响应确认）
wire [LEN_WIDTH+1-1:0] rd_start_ptr; // 读侧已申请读取的起始指针（提供给写侧作空间计算）
wire [LEN_WIDTH+1-1:0] rd_finish_ptr; // 读侧已完成读取的指针（R通道接收确认）

axi_vfifo_raw_wr #(
    .SEG_WIDTH(SEG_WIDTH),
    .SEG_CNT(SEG_CNT),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .LEN_WIDTH(LEN_WIDTH),
    .WRITE_FIFO_DEPTH(WRITE_FIFO_DEPTH),
    .WRITE_MAX_BURST_LEN(WRITE_MAX_BURST_LEN),
    .WATERMARK_LEVEL(WATERMARK_LEVEL)
)
axi_vfifo_raw_wr_inst (
    .clk(clk),
    .rst(rst),

    /*
     * 分段数据输入（来自编码逻辑）
     */
    .input_clk(input_clk),
    .input_rst(input_rst),
    .input_rst_out(input_rst_out),
    .input_watermark(input_watermark),
    .input_data(input_data),
    .input_valid(input_valid),
    .input_ready(input_ready),

    /*
     * AXI 主接口
     */
    .m_axi_awid(m_axi_awid),
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awlen(m_axi_awlen),
    .m_axi_awsize(m_axi_awsize),
    .m_axi_awburst(m_axi_awburst),
    .m_axi_awlock(m_axi_awlock),
    .m_axi_awcache(m_axi_awcache),
    .m_axi_awprot(m_axi_awprot),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),
    .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wlast(m_axi_wlast),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_wready(m_axi_wready),
    .m_axi_bid(m_axi_bid),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready),

    /*
     * FIFO 控制
     */
    .wr_start_ptr_out(wr_start_ptr),
    .wr_finish_ptr_out(wr_finish_ptr),
    .rd_start_ptr_in(rd_start_ptr),
    .rd_finish_ptr_in(rd_finish_ptr),

    /*
     * 配置
     */
    .cfg_fifo_base_addr(fifo_base_addr_reg),
    .cfg_fifo_size_mask(fifo_size_mask_reg),
    .cfg_enable(fifo_enable_reg),
    .cfg_reset(fifo_reset_reg),

    /*
     * 状态
     */
    .sts_fifo_occupancy(sts_fifo_occupancy),
    .sts_fifo_empty(sts_fifo_empty),
    .sts_fifo_full(sts_fifo_full),
    .sts_write_active(sts_write_active)
);

axi_vfifo_raw_rd #(
    .SEG_WIDTH(SEG_WIDTH),
    .SEG_CNT(SEG_CNT),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .LEN_WIDTH(LEN_WIDTH),
    .READ_FIFO_DEPTH(READ_FIFO_DEPTH),
    .READ_MAX_BURST_LEN(READ_MAX_BURST_LEN),
    .CTRL_OUT_EN(CTRL_OUT_EN)
)
axi_vfifo_raw_rd_inst (
    .clk(clk),
    .rst(rst),

    /*
     * 分段数据输出（到解码逻辑）
     */
    .output_clk(output_clk),
    .output_rst(output_rst),
    .output_rst_out(output_rst_out),
    .output_data(output_data),
    .output_valid(output_valid),
    .output_ready(output_ready),
    .output_ctrl_data(output_ctrl_data),
    .output_ctrl_valid(output_ctrl_valid),
    .output_ctrl_ready(output_ctrl_ready),

    /*
     * AXI 主接口
     */
    .m_axi_arid(m_axi_arid),
    .m_axi_araddr(m_axi_araddr),
    .m_axi_arlen(m_axi_arlen),
    .m_axi_arsize(m_axi_arsize),
    .m_axi_arburst(m_axi_arburst),
    .m_axi_arlock(m_axi_arlock),
    .m_axi_arcache(m_axi_arcache),
    .m_axi_arprot(m_axi_arprot),
    .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rid(m_axi_rid),
    .m_axi_rdata(m_axi_rdata),
    .m_axi_rresp(m_axi_rresp),
    .m_axi_rlast(m_axi_rlast),
    .m_axi_rvalid(m_axi_rvalid),
    .m_axi_rready(m_axi_rready),

    /*
     * FIFO 控制
     */
    .wr_start_ptr_in(wr_start_ptr),
    .wr_finish_ptr_in(wr_finish_ptr),
    .rd_start_ptr_out(rd_start_ptr),
    .rd_finish_ptr_out(rd_finish_ptr),

    /*
     * 配置
     */
    .cfg_fifo_base_addr(fifo_base_addr_reg),
    .cfg_fifo_size_mask(fifo_size_mask_reg),
    .cfg_enable(fifo_enable_reg),
    .cfg_reset(fifo_reset_reg),

    /*
     * 状态
     */
    .sts_read_active(sts_read_active)
);

// 复位同步
assign rst_req_out = rst | input_rst | output_rst | cfg_reset;

wire rst_req_int = rst_req_in | rst_req_out; // 本地复位请求总线（本模块请求与外部请求或并）

(* shreg_extract = "no" *)
reg rst_sync_1_reg = 1'b1,  rst_sync_2_reg = 1'b1, rst_sync_3_reg = 1'b1; // clk域三级复位同步链

always @(posedge clk or posedge rst_req_int) begin
    if (rst_req_int) begin
        rst_sync_1_reg <= 1'b1;
    end else begin
        rst_sync_1_reg <= 1'b0;
    end
end

always @(posedge clk) begin
    rst_sync_2_reg <= rst_sync_1_reg;
    rst_sync_3_reg <= rst_sync_2_reg;
end

// 复位与使能控制逻辑
always @* begin
    fifo_reset_next = 1'b0;
    fifo_enable_next = fifo_enable_reg;
    fifo_base_addr_next = fifo_base_addr_reg;
    fifo_size_mask_next = fifo_size_mask_reg;

    if (cfg_reset || rst_sync_3_reg) begin
        fifo_reset_next = 1'b1;
    end

    if (fifo_reset_reg) begin
        fifo_enable_next = 1'b0;
        // 保持复位，直到在途读写全部冲刷完成
        if (sts_write_active || sts_read_active) begin
            fifo_reset_next = 1'b1;
        end
    end else if (!fifo_enable_reg && cfg_enable) begin
        fifo_base_addr_next = cfg_fifo_base_addr & ADDR_MASK;
        fifo_size_mask_next = cfg_fifo_size_mask | ~ADDR_MASK;

        fifo_enable_next = 1'b1;
    end
end

always @(posedge clk) begin
    fifo_reset_reg <= fifo_reset_next;
    fifo_enable_reg <= fifo_enable_next;
    fifo_base_addr_reg <= fifo_base_addr_next;
    fifo_size_mask_reg <= fifo_size_mask_next;

    if (rst) begin
        fifo_reset_reg <= 1'b1;
        fifo_enable_reg <= 1'b0;
    end
end

endmodule

`resetall
