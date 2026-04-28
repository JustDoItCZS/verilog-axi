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
 * AXI4 DMA 顶层模块
 *
 * 模块目录
 * 1) 读路径由 `axi_dma_rd` 实现：AXI 内存 -> AXIS 流。
 * 2) 写路径由 `axi_dma_wr` 实现：AXIS 流 -> AXI 内存。
 * 3) 顶层负责读写两引擎拼接与共享配置分发。
 */
module axi_dma #
(
    // AXI 数据总线位宽
    parameter AXI_DATA_WIDTH = 32,
    // AXI 地址总线位宽
    parameter AXI_ADDR_WIDTH = 16,
    // AXI WSTRB 位宽（按字节）
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // AXI ID 信号位宽
    parameter AXI_ID_WIDTH = 8,
    // 生成的 AXI 最大突发长度
    parameter AXI_MAX_BURST_LEN = 16,
    // AXI-Stream 接口数据位宽
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
    // 是否使用 AXI-Stream TKEEP
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI-Stream TKEEP 位宽（每拍字节数）
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // 是否使用 AXI-Stream TLAST
    parameter AXIS_LAST_ENABLE = 1,
    // 是否透传 AXI-Stream TID
    parameter AXIS_ID_ENABLE = 0,
    // AXI-Stream TID 位宽
    parameter AXIS_ID_WIDTH = 8,
    // 是否透传 AXI-Stream TDEST
    parameter AXIS_DEST_ENABLE = 0,
    // AXI-Stream TDEST 位宽
    parameter AXIS_DEST_WIDTH = 8,
    // 是否透传 AXI-Stream TUSER
    parameter AXIS_USER_ENABLE = 1,
    // AXI-Stream TUSER 位宽
    parameter AXIS_USER_WIDTH = 1,
    // 长度字段位宽
    parameter LEN_WIDTH = 20,
    // tag 字段位宽
    parameter TAG_WIDTH = 8,
    // 是否启用散列/聚集 DMA 支持
    // （每个 AXI-Stream 帧可含多个描述符）
    parameter ENABLE_SG = 0,
    // 是否启用非对齐传输支持
    parameter ENABLE_UNALIGNED = 0
)
(
    input  wire                       clk, // DMA 顶层时钟。
    input  wire                       rst, // 同步复位，高电平有效。

    /*
     * AXI 读描述符输入
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_read_desc_addr, // 读描述符源地址。
    input  wire [LEN_WIDTH-1:0]       s_axis_read_desc_len, // 读描述符长度。
    input  wire [TAG_WIDTH-1:0]       s_axis_read_desc_tag, // 读描述符 tag。
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_read_desc_id, // 读描述符附带 AXIS ID。
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_read_desc_dest, // 读描述符附带 AXIS DEST。
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_read_desc_user, // 读描述符附带 AXIS USER。
    input  wire                       s_axis_read_desc_valid, // 读描述符有效。
    output wire                       s_axis_read_desc_ready, // 读引擎可接收描述符。

    /*
     * AXI 读描述符状态输出
     */
    output wire [TAG_WIDTH-1:0]       m_axis_read_desc_status_tag, // 读完成状态 tag。
    output wire [3:0]                 m_axis_read_desc_status_error, // 读完成状态错误码。
    output wire                       m_axis_read_desc_status_valid, // 读完成状态有效。

    /*
     * AXI-Stream 读数据输出
     */
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata, // 读数据流 tdata。
    output wire [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep, // 读数据流 tkeep。
    output wire                       m_axis_read_data_tvalid, // 读数据流 tvalid。
    input  wire                       m_axis_read_data_tready, // 读数据流 tready。
    output wire                       m_axis_read_data_tlast, // 读数据流 tlast。
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid, // 读数据流 tid。
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest, // 读数据流 tdest。
    output wire [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser, // 读数据流 tuser。

    /*
     * AXI 写描述符输入
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_write_desc_addr, // 写描述符目的地址。
    input  wire [LEN_WIDTH-1:0]       s_axis_write_desc_len, // 写描述符长度。
    input  wire [TAG_WIDTH-1:0]       s_axis_write_desc_tag, // 写描述符 tag。
    input  wire                       s_axis_write_desc_valid, // 写描述符有效。
    output wire                       s_axis_write_desc_ready, // 写引擎可接收描述符。

    /*
     * AXI 写描述符状态输出
     */
    output wire [LEN_WIDTH-1:0]       m_axis_write_desc_status_len, // 写完成状态实际写入长度。
    output wire [TAG_WIDTH-1:0]       m_axis_write_desc_status_tag, // 写完成状态 tag。
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_write_desc_status_id, // 写完成状态 tid。
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest, // 写完成状态 tdest。
    output wire [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user, // 写完成状态 tuser。
    output wire [3:0]                 m_axis_write_desc_status_error, // 写完成状态错误码。
    output wire                       m_axis_write_desc_status_valid, // 写完成状态有效。

    /*
     * AXI-Stream 写数据输入
     */
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_write_data_tdata, // 写数据流 tdata。
    input  wire [AXIS_KEEP_WIDTH-1:0] s_axis_write_data_tkeep, // 写数据流 tkeep。
    input  wire                       s_axis_write_data_tvalid, // 写数据流 tvalid。
    output wire                       s_axis_write_data_tready, // 写数据流 tready。
    input  wire                       s_axis_write_data_tlast, // 写数据流 tlast。
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_write_data_tid, // 写数据流 tid。
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_write_data_tdest, // 写数据流 tdest。
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_write_data_tuser, // 写数据流 tuser。

    /*
     * AXI 主接口
     */
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid, // AXI 写地址通道 ID。
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr, // AXI 写地址通道地址。
    output wire [7:0]                 m_axi_awlen, // AXI 写突发长度。
    output wire [2:0]                 m_axi_awsize, // AXI 写 beat 大小。
    output wire [1:0]                 m_axi_awburst, // AXI 写突发类型。
    output wire                       m_axi_awlock, // AXI 写锁属性。
    output wire [3:0]                 m_axi_awcache, // AXI 写 cache 属性。
    output wire [2:0]                 m_axi_awprot, // AXI 写保护属性。
    output wire                       m_axi_awvalid, // AXI 写地址有效。
    input  wire                       m_axi_awready, // AXI 写地址 ready。
    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata, // AXI 写数据。
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb, // AXI 写字节使能。
    output wire                       m_axi_wlast, // AXI 写最后一拍。
    output wire                       m_axi_wvalid, // AXI 写数据有效。
    input  wire                       m_axi_wready, // AXI 写数据 ready。
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid, // AXI 写响应 ID。
    input  wire [1:0]                 m_axi_bresp, // AXI 写响应状态。
    input  wire                       m_axi_bvalid, // AXI 写响应有效。
    output wire                       m_axi_bready, // AXI 写响应 ready。
    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid, // AXI 读地址通道 ID。
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr, // AXI 读地址通道地址。
    output wire [7:0]                 m_axi_arlen, // AXI 读突发长度。
    output wire [2:0]                 m_axi_arsize, // AXI 读 beat 大小。
    output wire [1:0]                 m_axi_arburst, // AXI 读突发类型。
    output wire                       m_axi_arlock, // AXI 读锁属性。
    output wire [3:0]                 m_axi_arcache, // AXI 读 cache 属性。
    output wire [2:0]                 m_axi_arprot, // AXI 读保护属性。
    output wire                       m_axi_arvalid, // AXI 读地址有效。
    input  wire                       m_axi_arready, // AXI 读地址 ready。
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid, // AXI 读响应 ID。
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata, // AXI 读响应数据。
    input  wire [1:0]                 m_axi_rresp, // AXI 读响应状态。
    input  wire                       m_axi_rlast, // AXI 读响应最后一拍。
    input  wire                       m_axi_rvalid, // AXI 读响应有效。
    output wire                       m_axi_rready, // AXI 读响应 ready。

    /*
     * 配置
     */
    input  wire                       read_enable, // 读 DMA 使能。
    input  wire                       write_enable, // 写 DMA 使能。
    input  wire                       write_abort // 写 DMA 中止请求。
);

axi_dma_rd #(
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
    .LEN_WIDTH(LEN_WIDTH),
    .TAG_WIDTH(TAG_WIDTH),
    .ENABLE_SG(ENABLE_SG),
    .ENABLE_UNALIGNED(ENABLE_UNALIGNED)
)
axi_dma_rd_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI 读描述符输入
     */
    .s_axis_read_desc_addr(s_axis_read_desc_addr),
    .s_axis_read_desc_len(s_axis_read_desc_len),
    .s_axis_read_desc_tag(s_axis_read_desc_tag),
    .s_axis_read_desc_id(s_axis_read_desc_id),
    .s_axis_read_desc_dest(s_axis_read_desc_dest),
    .s_axis_read_desc_user(s_axis_read_desc_user),
    .s_axis_read_desc_valid(s_axis_read_desc_valid),
    .s_axis_read_desc_ready(s_axis_read_desc_ready),

    /*
     * AXI 读描述符状态输出
     */
    .m_axis_read_desc_status_tag(m_axis_read_desc_status_tag),
    .m_axis_read_desc_status_error(m_axis_read_desc_status_error),
    .m_axis_read_desc_status_valid(m_axis_read_desc_status_valid),

    /*
     * AXI-Stream 读数据输出
     */
    .m_axis_read_data_tdata(m_axis_read_data_tdata),
    .m_axis_read_data_tkeep(m_axis_read_data_tkeep),
    .m_axis_read_data_tvalid(m_axis_read_data_tvalid),
    .m_axis_read_data_tready(m_axis_read_data_tready),
    .m_axis_read_data_tlast(m_axis_read_data_tlast),
    .m_axis_read_data_tid(m_axis_read_data_tid),
    .m_axis_read_data_tdest(m_axis_read_data_tdest),
    .m_axis_read_data_tuser(m_axis_read_data_tuser),

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
     * 配置
     */
    .enable(read_enable)
);

axi_dma_wr #(
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
    .LEN_WIDTH(LEN_WIDTH),
    .TAG_WIDTH(TAG_WIDTH),
    .ENABLE_SG(ENABLE_SG),
    .ENABLE_UNALIGNED(ENABLE_UNALIGNED)
)
axi_dma_wr_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI 写描述符输入
     */
    .s_axis_write_desc_addr(s_axis_write_desc_addr),
    .s_axis_write_desc_len(s_axis_write_desc_len),
    .s_axis_write_desc_tag(s_axis_write_desc_tag),
    .s_axis_write_desc_valid(s_axis_write_desc_valid),
    .s_axis_write_desc_ready(s_axis_write_desc_ready),

    /*
     * AXI 写描述符状态输出
     */
    .m_axis_write_desc_status_len(m_axis_write_desc_status_len),
    .m_axis_write_desc_status_tag(m_axis_write_desc_status_tag),
    .m_axis_write_desc_status_id(m_axis_write_desc_status_id),
    .m_axis_write_desc_status_dest(m_axis_write_desc_status_dest),
    .m_axis_write_desc_status_user(m_axis_write_desc_status_user),
    .m_axis_write_desc_status_error(m_axis_write_desc_status_error),
    .m_axis_write_desc_status_valid(m_axis_write_desc_status_valid),

    /*
     * AXI-Stream 写数据输入
     */
    .s_axis_write_data_tdata(s_axis_write_data_tdata),
    .s_axis_write_data_tkeep(s_axis_write_data_tkeep),
    .s_axis_write_data_tvalid(s_axis_write_data_tvalid),
    .s_axis_write_data_tready(s_axis_write_data_tready),
    .s_axis_write_data_tlast(s_axis_write_data_tlast),
    .s_axis_write_data_tid(s_axis_write_data_tid),
    .s_axis_write_data_tdest(s_axis_write_data_tdest),
    .s_axis_write_data_tuser(s_axis_write_data_tuser),

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
     * 配置
     */
    .enable(write_enable),
    .abort(write_abort)
);

endmodule

`resetall
