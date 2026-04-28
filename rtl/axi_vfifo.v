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
 * AXI4 虚拟 FIFO
 *
 * 模块目录：
 * 1) `axi_vfifo_enc`：把AXI-Stream输入封装成分段数据和控制头
 * 2) `axi_vfifo_raw` x AXI_CH：每个AXI通道一个原始虚拟FIFO（写外存+读外存）
 * 3) `axi_vfifo_dec`：把分段数据还原为AXI-Stream输出
 * 4) 配置/状态同步：负责跨时钟域同步cfg/sts以及全局复位请求
 */
module axi_vfifo #
(
    // AXI 通道数量
    parameter AXI_CH = 1,
    // AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = 32,
    // AXI 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // AXI WSTRB 位宽（按字节）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 位宽
    parameter AXI_ID_WIDTH = 8,
    // 允许生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // AXI-Stream 接口位宽
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH*AXI_CH/2,
    // 是否启用 AXI-Stream tkeep
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI-Stream tkeep 位宽（每拍字节数）
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // 是否启用 AXI-Stream tlast
    parameter AXIS_LAST_ENABLE = 1,
    // 是否透传 AXI-Stream tid
    parameter AXIS_ID_ENABLE = 0,
    // AXI-Stream tid 位宽
    parameter AXIS_ID_WIDTH = 8,
    // 是否透传 AXI-Stream tdest
    parameter AXIS_DEST_ENABLE = 0,
    // AXI-Stream tdest 位宽
    parameter AXIS_DEST_WIDTH = 8,
    // 是否透传 AXI-Stream tuser
    parameter AXIS_USER_ENABLE = 1,
    // AXI-Stream tuser 位宽
    parameter AXIS_USER_WIDTH = 1,
    // 长度字段位宽
    parameter LEN_WIDTH = AXI_ADDR_WIDTH,
    // 最大分段位宽
    parameter MAX_SEG_WIDTH = 256,
    // AXI 写数据输入 FIFO 深度（全宽字）
    parameter WRITE_FIFO_DEPTH = 64,
    // AXI 最大写突发长度
    parameter WRITE_MAX_BURST_LEN = WRITE_FIFO_DEPTH/4,
    // AXI 读数据输出 FIFO 深度（全宽字）
    parameter READ_FIFO_DEPTH = 128,
    // AXI 最大读突发长度
    parameter READ_MAX_BURST_LEN = WRITE_MAX_BURST_LEN
)
(
    input  wire                               clk, // 顶层配置时钟（用于配置寄存与状态采样）
    input  wire                               rst, // 顶层配置复位

    /*
     * AXI-Stream 数据输入
     */
    input  wire                               s_axis_clk, // AXI-Stream输入时钟
    input  wire                               s_axis_rst, // AXI-Stream输入复位
    output wire                               s_axis_rst_out, // 输入侧复位反馈（任一通道请求复位时拉高）
    input  wire [AXIS_DATA_WIDTH-1:0]         s_axis_tdata, // AXI-Stream输入数据
    input  wire [AXIS_KEEP_WIDTH-1:0]         s_axis_tkeep, // AXI-Stream输入字节有效掩码
    input  wire                               s_axis_tvalid, // AXI-Stream输入有效
    output wire                               s_axis_tready, // AXI-Stream输入就绪
    input  wire                               s_axis_tlast, // AXI-Stream输入帧尾
    input  wire [AXIS_ID_WIDTH-1:0]           s_axis_tid, // AXI-Stream输入ID
    input  wire [AXIS_DEST_WIDTH-1:0]         s_axis_tdest, // AXI-Stream输入目的地
    input  wire [AXIS_USER_WIDTH-1:0]         s_axis_tuser, // AXI-Stream输入用户侧带

    /*
     * AXI-Stream 数据输出
     */
    input  wire                               m_axis_clk, // AXI-Stream输出时钟
    input  wire                               m_axis_rst, // AXI-Stream输出复位
    output wire                               m_axis_rst_out, // 输出侧复位反馈（任一通道请求复位时拉高）
    output wire [AXIS_DATA_WIDTH-1:0]         m_axis_tdata, // AXI-Stream输出数据
    output wire [AXIS_KEEP_WIDTH-1:0]         m_axis_tkeep, // AXI-Stream输出字节有效掩码
    output wire                               m_axis_tvalid, // AXI-Stream输出有效
    input  wire                               m_axis_tready, // AXI-Stream输出就绪
    output wire                               m_axis_tlast, // AXI-Stream输出帧尾
    output wire [AXIS_ID_WIDTH-1:0]           m_axis_tid, // AXI-Stream输出ID
    output wire [AXIS_DEST_WIDTH-1:0]         m_axis_tdest, // AXI-Stream输出目的地
    output wire [AXIS_USER_WIDTH-1:0]         m_axis_tuser, // AXI-Stream输出用户侧带

    /*
     * AXI 主接口
     */
    input  wire [AXI_CH-1:0]                  m_axi_clk, // 各AXI通道时钟
    input  wire [AXI_CH-1:0]                  m_axi_rst, // 各AXI通道复位
    output wire [AXI_CH*AXI_ID_WIDTH-1:0]     m_axi_awid, // 各通道AXI AWID
    output wire [AXI_CH*AXI_ADDR_WIDTH-1:0]   m_axi_awaddr, // 各通道AXI AWADDR
    output wire [AXI_CH*8-1:0]                m_axi_awlen, // 各通道AXI AWLEN
    output wire [AXI_CH*3-1:0]                m_axi_awsize, // 各通道AXI AWSIZE
    output wire [AXI_CH*2-1:0]                m_axi_awburst, // 各通道AXI AWBURST
    output wire [AXI_CH-1:0]                  m_axi_awlock, // 各通道AXI AWLOCK
    output wire [AXI_CH*4-1:0]                m_axi_awcache, // 各通道AXI AWCACHE
    output wire [AXI_CH*3-1:0]                m_axi_awprot, // 各通道AXI AWPROT
    output wire [AXI_CH-1:0]                  m_axi_awvalid, // 各通道AXI AWVALID
    input  wire [AXI_CH-1:0]                  m_axi_awready, // 各通道AXI AWREADY
    output wire [AXI_CH*AXI_DATA_WIDTH-1:0]   m_axi_wdata, // 各通道AXI WDATA
    output wire [AXI_CH*AXI_STRB_WIDTH-1:0]   m_axi_wstrb, // 各通道AXI WSTRB
    output wire [AXI_CH-1:0]                  m_axi_wlast, // 各通道AXI WLAST
    output wire [AXI_CH-1:0]                  m_axi_wvalid, // 各通道AXI WVALID
    input  wire [AXI_CH-1:0]                  m_axi_wready, // 各通道AXI WREADY
    input  wire [AXI_CH*AXI_ID_WIDTH-1:0]     m_axi_bid, // 各通道AXI BID
    input  wire [AXI_CH*2-1:0]                m_axi_bresp, // 各通道AXI BRESP
    input  wire [AXI_CH-1:0]                  m_axi_bvalid, // 各通道AXI BVALID
    output wire [AXI_CH-1:0]                  m_axi_bready, // 各通道AXI BREADY
    output wire [AXI_CH*AXI_ID_WIDTH-1:0]     m_axi_arid, // 各通道AXI ARID
    output wire [AXI_CH*AXI_ADDR_WIDTH-1:0]   m_axi_araddr, // 各通道AXI ARADDR
    output wire [AXI_CH*8-1:0]                m_axi_arlen, // 各通道AXI ARLEN
    output wire [AXI_CH*3-1:0]                m_axi_arsize, // 各通道AXI ARSIZE
    output wire [AXI_CH*2-1:0]                m_axi_arburst, // 各通道AXI ARBURST
    output wire [AXI_CH-1:0]                  m_axi_arlock, // 各通道AXI ARLOCK
    output wire [AXI_CH*4-1:0]                m_axi_arcache, // 各通道AXI ARCACHE
    output wire [AXI_CH*3-1:0]                m_axi_arprot, // 各通道AXI ARPROT
    output wire [AXI_CH-1:0]                  m_axi_arvalid, // 各通道AXI ARVALID
    input  wire [AXI_CH-1:0]                  m_axi_arready, // 各通道AXI ARREADY
    input  wire [AXI_CH*AXI_ID_WIDTH-1:0]     m_axi_rid, // 各通道AXI RID
    input  wire [AXI_CH*AXI_DATA_WIDTH-1:0]   m_axi_rdata, // 各通道AXI RDATA
    input  wire [AXI_CH*2-1:0]                m_axi_rresp, // 各通道AXI RRESP
    input  wire [AXI_CH-1:0]                  m_axi_rlast, // 各通道AXI RLAST
    input  wire [AXI_CH-1:0]                  m_axi_rvalid, // 各通道AXI RVALID
    output wire [AXI_CH-1:0]                  m_axi_rready, // 各通道AXI RREADY

    /*
     * 配置
     */
    input  wire [AXI_CH*AXI_ADDR_WIDTH-1:0]   cfg_fifo_base_addr, // 每个通道虚拟FIFO基地址
    input  wire [LEN_WIDTH-1:0]               cfg_fifo_size_mask, // 所有通道共享的FIFO大小掩码
    input  wire                               cfg_enable, // 全局配置使能
    input  wire                               cfg_reset, // 全局配置复位

    /*
     * 状态
     */
    output wire [AXI_CH*(LEN_WIDTH+1)-1:0]    sts_fifo_occupancy, // 各通道FIFO占用量
    output wire [AXI_CH-1:0]                  sts_fifo_empty, // 各通道FIFO空标志
    output wire [AXI_CH-1:0]                  sts_fifo_full, // 各通道FIFO满标志
    output wire [AXI_CH-1:0]                  sts_reset, // 各通道复位状态
    output wire [AXI_CH-1:0]                  sts_active, // 各通道使能活动状态
    output wire                               sts_hdr_parity_err // 头部奇偶校验错误状态（同步到 m_axis_clk 域）
);

parameter CH_SEG_CNT = AXI_DATA_WIDTH > MAX_SEG_WIDTH ? AXI_DATA_WIDTH / MAX_SEG_WIDTH : 1;
parameter SEG_CNT = CH_SEG_CNT * AXI_CH;
parameter SEG_WIDTH = AXI_DATA_WIDTH / CH_SEG_CNT;

wire [AXI_CH-1:0]             ch_input_rst_out; // 各通道写侧对输入域发出的复位反馈
wire [AXI_CH-1:0]             ch_input_watermark; // 各通道写侧输入水位告警
wire [SEG_CNT*SEG_WIDTH-1:0]  ch_input_data; // 编码后分段输入数据总线（按通道拼接）
wire [SEG_CNT-1:0]            ch_input_valid; // 编码后分段输入有效
wire [SEG_CNT-1:0]            ch_input_ready; // 各通道对分段输入的就绪反馈

wire [AXI_CH-1:0]             ch_output_rst_out; // 各通道读侧对输出域发出的复位反馈
wire [SEG_CNT*SEG_WIDTH-1:0]  ch_output_data; // 各通道读出分段数据拼接总线
wire [SEG_CNT-1:0]            ch_output_valid; // 各通道读出分段数据有效
wire [SEG_CNT-1:0]            ch_output_ready; // 解码器对分段输出就绪
wire [SEG_CNT*SEG_WIDTH-1:0]  ch_output_ctrl_data; // 各通道读出分段控制数据
wire [SEG_CNT-1:0]            ch_output_ctrl_valid; // 各通道读出控制数据有效
wire [SEG_CNT-1:0]            ch_output_ctrl_ready; // 解码器对控制数据就绪

wire [AXI_CH-1:0] ch_rst_req; // 各通道复位请求总线（通道间互相可见）

// 配置管理
reg [AXI_CH*AXI_ADDR_WIDTH-1:0] cfg_fifo_base_addr_reg = 0; // 锁存后的每通道FIFO基地址配置
reg [LEN_WIDTH-1:0] cfg_fifo_size_mask_reg = 0; // 锁存后的FIFO大小掩码配置
reg cfg_enable_reg = 0; // 全局使能锁存位（一次使能后仅复位清除）
reg cfg_reset_reg = 0; // 全局复位锁存位（供各通道同步）

always @(posedge clk) begin
    if (cfg_enable_reg) begin
        if (cfg_reset) begin
            cfg_enable_reg <= 1'b0;
        end
    end else begin
        if (cfg_enable) begin
            cfg_enable_reg <= 1'b1;
        end
        cfg_fifo_base_addr_reg <= cfg_fifo_base_addr;
        cfg_fifo_size_mask_reg <= cfg_fifo_size_mask;
    end

    cfg_reset_reg <= cfg_reset;

    if (rst) begin
        cfg_enable_reg <= 0;
        cfg_reset_reg <= 0;
    end
end

// 状态同步
wire [AXI_CH*(LEN_WIDTH+1)-1:0] sts_fifo_occupancy_int; // 各通道原始占用量状态（通道时钟域采样回传）
wire [AXI_CH-1:0] sts_fifo_empty_int; // 各通道原始空状态
wire [AXI_CH-1:0] sts_fifo_full_int; // 各通道原始满状态
wire [AXI_CH-1:0] sts_reset_int; // 各通道原始复位状态
wire [AXI_CH-1:0] sts_active_int; // 各通道原始活动状态
wire sts_hdr_parity_err_int; // 解码器头部奇偶校验错误脉冲
reg [3:0] sts_hdr_parity_err_cnt_reg = 0; // 头部奇偶错误保持计数（拉宽脉冲便于跨域采样）
reg sts_hdr_parity_err_reg = 1'b0; // m_axis_clk 域错误状态寄存

reg [2:0] sts_sync_count_reg = 0; // 状态采样分频计数器
reg sts_sync_flag_reg = 1'b0; // 状态采样翻转标志（通道域检测边沿用于抓拍）

(* shreg_extract = "no" *)
reg [AXI_CH*(LEN_WIDTH+1)-1:0] sts_fifo_occupancy_sync_reg = 0; // 同步到 clk 域的占用量状态寄存
(* shreg_extract = "no" *)
reg [AXI_CH-1:0] sts_fifo_empty_sync_1_reg = 0, sts_fifo_empty_sync_2_reg = 0; // 空状态双级同步寄存
(* shreg_extract = "no" *)
reg [AXI_CH-1:0] sts_fifo_full_sync_1_reg = 0, sts_fifo_full_sync_2_reg = 0; // 满状态双级同步寄存
(* shreg_extract = "no" *)
reg [AXI_CH-1:0] sts_reset_sync_1_reg = 0, sts_reset_sync_2_reg = 0; // 复位状态双级同步寄存
(* shreg_extract = "no" *)
reg [AXI_CH-1:0] sts_active_sync_1_reg = 0, sts_active_sync_2_reg = 0; // 活动状态双级同步寄存
(* shreg_extract = "no" *)
reg sts_hdr_parity_err_sync_1_reg = 0, sts_hdr_parity_err_sync_2_reg = 0; // 头部奇偶错误状态双级同步寄存

assign sts_fifo_occupancy = sts_fifo_occupancy_sync_reg;
assign sts_fifo_empty = sts_fifo_empty_sync_2_reg;
assign sts_fifo_full = sts_fifo_full_sync_2_reg;
assign sts_reset = sts_reset_sync_2_reg;
assign sts_active = sts_active_sync_2_reg;
assign sts_hdr_parity_err = sts_hdr_parity_err_sync_2_reg;

always @(posedge m_axis_clk) begin
    sts_hdr_parity_err_reg <= 1'b0;

    if (sts_hdr_parity_err_cnt_reg) begin
        sts_hdr_parity_err_reg <= 1'b1;
        sts_hdr_parity_err_cnt_reg <= sts_hdr_parity_err_cnt_reg - 1;
    end

    if (sts_hdr_parity_err_int) begin
        sts_hdr_parity_err_cnt_reg <= 4'hf;
    end

    if (m_axis_rst) begin
        sts_hdr_parity_err_cnt_reg <= 4'h0;
        sts_hdr_parity_err_reg <= 1'b0;
    end
end

always @(posedge clk) begin
    sts_sync_count_reg <= sts_sync_count_reg + 1;

    if (sts_sync_count_reg == 0) begin
        sts_sync_flag_reg <= !sts_sync_flag_reg;
        sts_fifo_occupancy_sync_reg <= sts_fifo_occupancy_int;
    end

    sts_fifo_empty_sync_1_reg <= sts_fifo_empty_int;
    sts_fifo_empty_sync_2_reg <= sts_fifo_empty_sync_1_reg;
    sts_fifo_full_sync_1_reg <= sts_fifo_full_int;
    sts_fifo_full_sync_2_reg <= sts_fifo_full_sync_1_reg;
    sts_reset_sync_1_reg <= sts_reset_int;
    sts_reset_sync_2_reg <= sts_reset_sync_1_reg;
    sts_active_sync_1_reg <= sts_active_int;
    sts_active_sync_2_reg <= sts_active_sync_1_reg;
    sts_hdr_parity_err_sync_1_reg <= sts_hdr_parity_err_reg;
    sts_hdr_parity_err_sync_2_reg <= sts_hdr_parity_err_sync_1_reg;
end

assign s_axis_rst_out = |ch_input_rst_out;

axi_vfifo_enc #(
    .SEG_WIDTH(SEG_WIDTH),
    .SEG_CNT(SEG_CNT),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH)
)
axi_vfifo_enc_inst (
    .clk(s_axis_clk),
    .rst(s_axis_rst),

    /*
     * AXI-Stream 数据输入
     */
    .s_axis_tdata(s_axis_tdata),
    .s_axis_tkeep(s_axis_tkeep),
    .s_axis_tvalid(s_axis_tvalid),
    .s_axis_tready(s_axis_tready),
    .s_axis_tlast(s_axis_tlast),
    .s_axis_tid(s_axis_tid),
    .s_axis_tdest(s_axis_tdest),
    .s_axis_tuser(s_axis_tuser),

    /*
     * 分段数据输出（到虚拟 FIFO 通道）
     */
    .fifo_rst_in(s_axis_rst_out),
    .output_data(ch_input_data),
    .output_valid(ch_input_valid),
    .fifo_watermark_in(|ch_input_watermark)
);

generate

genvar  n;

for (n = 0; n < AXI_CH; n = n + 1) begin : axi_ch
    
    wire ch_clk = m_axi_clk[1*n +: 1]; // 第n个AXI通道时钟
    wire ch_rst = m_axi_rst[1*n +: 1]; // 第n个AXI通道复位

    wire [AXI_ID_WIDTH-1:0]    ch_axi_awid; // 当前通道AWID
    wire [AXI_ADDR_WIDTH-1:0]  ch_axi_awaddr; // 当前通道AWADDR
    wire [7:0]                 ch_axi_awlen; // 当前通道AWLEN
    wire [2:0]                 ch_axi_awsize; // 当前通道AWSIZE
    wire [1:0]                 ch_axi_awburst; // 当前通道AWBURST
    wire                       ch_axi_awlock; // 当前通道AWLOCK
    wire [3:0]                 ch_axi_awcache; // 当前通道AWCACHE
    wire [2:0]                 ch_axi_awprot; // 当前通道AWPROT
    wire                       ch_axi_awvalid; // 当前通道AWVALID
    wire                       ch_axi_awready; // 当前通道AWREADY
    wire [AXI_DATA_WIDTH-1:0]  ch_axi_wdata; // 当前通道WDATA
    wire [AXI_STRB_WIDTH-1:0]  ch_axi_wstrb; // 当前通道WSTRB
    wire                       ch_axi_wlast; // 当前通道WLAST
    wire                       ch_axi_wvalid; // 当前通道WVALID
    wire                       ch_axi_wready; // 当前通道WREADY
    wire [AXI_ID_WIDTH-1:0]    ch_axi_bid; // 当前通道BID
    wire [1:0]                 ch_axi_bresp; // 当前通道BRESP
    wire                       ch_axi_bvalid; // 当前通道BVALID
    wire                       ch_axi_bready; // 当前通道BREADY
    wire [AXI_ID_WIDTH-1:0]    ch_axi_arid; // 当前通道ARID
    wire [AXI_ADDR_WIDTH-1:0]  ch_axi_araddr; // 当前通道ARADDR
    wire [7:0]                 ch_axi_arlen; // 当前通道ARLEN
    wire [2:0]                 ch_axi_arsize; // 当前通道ARSIZE
    wire [1:0]                 ch_axi_arburst; // 当前通道ARBURST
    wire                       ch_axi_arlock; // 当前通道ARLOCK
    wire [3:0]                 ch_axi_arcache; // 当前通道ARCACHE
    wire [2:0]                 ch_axi_arprot; // 当前通道ARPROT
    wire                       ch_axi_arvalid; // 当前通道ARVALID
    wire                       ch_axi_arready; // 当前通道ARREADY
    wire [AXI_ID_WIDTH-1:0]    ch_axi_rid; // 当前通道RID
    wire [AXI_DATA_WIDTH-1:0]  ch_axi_rdata; // 当前通道RDATA
    wire [1:0]                 ch_axi_rresp; // 当前通道RRESP
    wire                       ch_axi_rlast; // 当前通道RLAST
    wire                       ch_axi_rvalid; // 当前通道RVALID
    wire                       ch_axi_rready; // 当前通道RREADY

    assign m_axi_awid[AXI_ID_WIDTH*n +: AXI_ID_WIDTH] = ch_axi_awid;
    assign m_axi_awaddr[AXI_ADDR_WIDTH*n +: AXI_ADDR_WIDTH] = ch_axi_awaddr;
    assign m_axi_awlen[8*n +: 8] = ch_axi_awlen;
    assign m_axi_awsize[3*n +: 3] = ch_axi_awsize;
    assign m_axi_awburst[2*n +: 2] = ch_axi_awburst;
    assign m_axi_awlock[1*n +: 1] = ch_axi_awlock;
    assign m_axi_awcache[4*n +: 4] = ch_axi_awcache;
    assign m_axi_awprot[3*n +: 3] = ch_axi_awprot;
    assign m_axi_awvalid[1*n +: 1] = ch_axi_awvalid;
    assign ch_axi_awready = m_axi_awready[1*n +: 1];
    assign m_axi_wdata[AXI_DATA_WIDTH*n +: AXI_DATA_WIDTH] = ch_axi_wdata;
    assign m_axi_wstrb[AXI_STRB_WIDTH*n +: AXI_STRB_WIDTH] = ch_axi_wstrb;
    assign m_axi_wlast[1*n +: 1] = ch_axi_wlast;
    assign m_axi_wvalid[1*n +: 1] = ch_axi_wvalid;
    assign ch_axi_wready = m_axi_wready[1*n +: 1];
    assign ch_axi_bid = m_axi_bid[AXI_ID_WIDTH*n +: AXI_ID_WIDTH];
    assign ch_axi_bresp = m_axi_bresp[2*n +: 2];
    assign ch_axi_bvalid = m_axi_bvalid[1*n +: 1];
    assign m_axi_bready[1*n +: 1] = ch_axi_bready;
    assign m_axi_arid[AXI_ID_WIDTH*n +: AXI_ID_WIDTH] = ch_axi_arid;
    assign m_axi_araddr[AXI_ADDR_WIDTH*n +: AXI_ADDR_WIDTH] = ch_axi_araddr;
    assign m_axi_arlen[8*n +: 8] = ch_axi_arlen;
    assign m_axi_arsize[3*n +: 3] = ch_axi_arsize;
    assign m_axi_arburst[2*n +: 2] = ch_axi_arburst;
    assign m_axi_arlock[1*n +: 1] = ch_axi_arlock;
    assign m_axi_arcache[4*n +: 4] = ch_axi_arcache;
    assign m_axi_arprot[3*n +: 3] = ch_axi_arprot;
    assign m_axi_arvalid[1*n +: 1] = ch_axi_arvalid;
    assign ch_axi_arready = m_axi_arready[1*n +: 1];
    assign ch_axi_rid = m_axi_rid[AXI_ID_WIDTH*n +: AXI_ID_WIDTH];
    assign ch_axi_rdata = m_axi_rdata[AXI_DATA_WIDTH*n +: AXI_DATA_WIDTH];
    assign ch_axi_rresp = m_axi_rresp[2*n +: 2];
    assign ch_axi_rlast = m_axi_rlast[1*n +: 1];
    assign ch_axi_rvalid = m_axi_rvalid[1*n +: 1];
    assign m_axi_rready[1*n +: 1] = ch_axi_rready;

    // 控制同步
    (* shreg_extract = "no" *)
    reg ch_cfg_enable_sync_1_reg = 1'b0,  ch_cfg_enable_sync_2_reg = 1'b0; // cfg_enable跨到当前通道时钟域的双级同步寄存
    (* shreg_extract = "no" *)
    reg ch_cfg_reset_sync_1_reg = 1'b0,  ch_cfg_reset_sync_2_reg = 1'b0; // cfg_reset跨到当前通道时钟域的双级同步寄存

    always @(posedge ch_clk) begin
        ch_cfg_enable_sync_1_reg <= cfg_enable_reg;
        ch_cfg_enable_sync_2_reg <= ch_cfg_enable_sync_1_reg;
        ch_cfg_reset_sync_1_reg <= cfg_reset_reg;
        ch_cfg_reset_sync_2_reg <= ch_cfg_reset_sync_1_reg;
    end

    // 状态同步
    wire [LEN_WIDTH+1-1:0] ch_sts_fifo_occupancy; // 当前通道原始FIFO占用量
    reg [LEN_WIDTH+1-1:0] ch_sts_fifo_occupancy_reg; // 当前通道采样后上报到顶层的占用量寄存

    (* shreg_extract = "no" *)
    reg ch_sts_flag_sync_1_reg = 1'b0,  ch_sts_flag_sync_2_reg = 1'b0,  ch_sts_flag_sync_3_reg = 1'b0; // 状态采样翻转标志跨域同步链（通道域检测边沿）

    assign sts_fifo_occupancy_int[(LEN_WIDTH+1)*n +: LEN_WIDTH+1] = ch_sts_fifo_occupancy_reg;

    always @(posedge ch_clk) begin
        ch_sts_flag_sync_1_reg <= sts_sync_flag_reg;
        ch_sts_flag_sync_2_reg <= ch_sts_flag_sync_1_reg;
        ch_sts_flag_sync_3_reg <= ch_sts_flag_sync_2_reg;

        if (ch_sts_flag_sync_3_reg ^ ch_sts_flag_sync_2_reg) begin
            ch_sts_fifo_occupancy_reg <= ch_sts_fifo_occupancy;
        end
    end

    axi_vfifo_raw #(
        .SEG_WIDTH(SEG_WIDTH),
        .SEG_CNT(CH_SEG_CNT),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
        .LEN_WIDTH(LEN_WIDTH),
        .WRITE_FIFO_DEPTH(WRITE_FIFO_DEPTH),
        .WRITE_MAX_BURST_LEN(WRITE_MAX_BURST_LEN),
        .READ_FIFO_DEPTH(READ_FIFO_DEPTH),
        .READ_MAX_BURST_LEN(READ_MAX_BURST_LEN),
        .WATERMARK_LEVEL(WRITE_FIFO_DEPTH-4),
        .CTRL_OUT_EN(1)
    )
    axi_vfifo_raw_inst (
        .clk(ch_clk),
        .rst(ch_rst),

        /*
         * 分段数据输入（来自编码逻辑）
         */
        .input_clk(s_axis_clk),
        .input_rst(s_axis_rst),
        .input_rst_out(ch_input_rst_out[n]),
        .input_watermark(ch_input_watermark[n]),
        .input_data(ch_input_data[SEG_WIDTH*CH_SEG_CNT*n +: SEG_WIDTH*CH_SEG_CNT]),
        .input_valid(ch_input_valid[CH_SEG_CNT*n +: CH_SEG_CNT]),
        .input_ready(ch_input_ready[CH_SEG_CNT*n +: CH_SEG_CNT]),

        /*
         * 分段数据输出（到解码逻辑）
         */
        .output_clk(m_axis_clk),
        .output_rst(m_axis_rst),
        .output_rst_out(ch_output_rst_out[n]),
        .output_data(ch_output_data[SEG_WIDTH*CH_SEG_CNT*n +: SEG_WIDTH*CH_SEG_CNT]),
        .output_valid(ch_output_valid[CH_SEG_CNT*n +: CH_SEG_CNT]),
        .output_ready(ch_output_ready[CH_SEG_CNT*n +: CH_SEG_CNT]),
        .output_ctrl_data(ch_output_ctrl_data[SEG_WIDTH*CH_SEG_CNT*n +: SEG_WIDTH*CH_SEG_CNT]),
        .output_ctrl_valid(ch_output_ctrl_valid[CH_SEG_CNT*n +: CH_SEG_CNT]),
        .output_ctrl_ready(ch_output_ctrl_ready[CH_SEG_CNT*n +: CH_SEG_CNT]),

        /*
         * AXI 主接口
         */
        .m_axi_awid(ch_axi_awid),
        .m_axi_awaddr(ch_axi_awaddr),
        .m_axi_awlen(ch_axi_awlen),
        .m_axi_awsize(ch_axi_awsize),
        .m_axi_awburst(ch_axi_awburst),
        .m_axi_awlock(ch_axi_awlock),
        .m_axi_awcache(ch_axi_awcache),
        .m_axi_awprot(ch_axi_awprot),
        .m_axi_awvalid(ch_axi_awvalid),
        .m_axi_awready(ch_axi_awready),
        .m_axi_wdata(ch_axi_wdata),
        .m_axi_wstrb(ch_axi_wstrb),
        .m_axi_wlast(ch_axi_wlast),
        .m_axi_wvalid(ch_axi_wvalid),
        .m_axi_wready(ch_axi_wready),
        .m_axi_bid(ch_axi_bid),
        .m_axi_bresp(ch_axi_bresp),
        .m_axi_bvalid(ch_axi_bvalid),
        .m_axi_bready(ch_axi_bready),
        .m_axi_arid(ch_axi_arid),
        .m_axi_araddr(ch_axi_araddr),
        .m_axi_arlen(ch_axi_arlen),
        .m_axi_arsize(ch_axi_arsize),
        .m_axi_arburst(ch_axi_arburst),
        .m_axi_arlock(ch_axi_arlock),
        .m_axi_arcache(ch_axi_arcache),
        .m_axi_arprot(ch_axi_arprot),
        .m_axi_arvalid(ch_axi_arvalid),
        .m_axi_arready(ch_axi_arready),
        .m_axi_rid(ch_axi_rid),
        .m_axi_rdata(ch_axi_rdata),
        .m_axi_rresp(ch_axi_rresp),
        .m_axi_rlast(ch_axi_rlast),
        .m_axi_rvalid(ch_axi_rvalid),
        .m_axi_rready(ch_axi_rready),

        /*
         * 复位同步
         */
        .rst_req_out(ch_rst_req[n]),
        .rst_req_in(|ch_rst_req),

        /*
         * 配置
         */
        .cfg_fifo_base_addr(cfg_fifo_base_addr_reg[AXI_ADDR_WIDTH*n +: AXI_ADDR_WIDTH]),
        .cfg_fifo_size_mask(cfg_fifo_size_mask_reg),
        .cfg_enable(ch_cfg_enable_sync_2_reg),
        .cfg_reset(ch_cfg_reset_sync_2_reg),

        /*
         * 状态
         */
        .sts_fifo_occupancy(ch_sts_fifo_occupancy),
        .sts_fifo_empty(sts_fifo_empty_int[n]),
        .sts_fifo_full(sts_fifo_full_int[n]),
        .sts_reset(sts_reset_int[n]),
        .sts_active(sts_active_int[n]),
        .sts_write_active(),
        .sts_read_active()
    );

end

endgenerate

assign m_axis_rst_out = |ch_output_rst_out;

axi_vfifo_dec #(
    .SEG_WIDTH(SEG_WIDTH),
    .SEG_CNT(SEG_CNT),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH)
)
axi_vfifo_dec_inst (
    .clk(m_axis_clk),
    .rst(m_axis_rst),

    /*
     * 分段数据输入（来自虚拟 FIFO 通道）
     */
    .fifo_rst_in(m_axis_rst_out),
    .input_data(ch_output_data),
    .input_valid(ch_output_valid),
    .input_ready(ch_output_ready),
    .input_ctrl_data(ch_output_ctrl_data),
    .input_ctrl_valid(ch_output_ctrl_valid),
    .input_ctrl_ready(ch_output_ctrl_ready),

    /*
     * AXI-Stream 数据输出
     */
    .m_axis_tdata(m_axis_tdata),
    .m_axis_tkeep(m_axis_tkeep),
    .m_axis_tvalid(m_axis_tvalid),
    .m_axis_tready(m_axis_tready),
    .m_axis_tlast(m_axis_tlast),
    .m_axis_tid(m_axis_tid),
    .m_axis_tdest(m_axis_tdest),
    .m_axis_tuser(m_axis_tuser),

    /*
     * 状态
     */
    .sts_hdr_parity_err(sts_hdr_parity_err_int)
);

endmodule

`resetall
