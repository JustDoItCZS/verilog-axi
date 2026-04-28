/*

Copyright (c) 2014-2021 Alex Forencich

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
 * 仲裁器模块
 *
 * 模块目录
 * 1) 接收多端口请求向量并输出 one-hot 授权。
 * 2) 支持固定优先级或轮询优先级策略。
 * 3) 可选阻塞模式：保持当前授权直到请求撤销或收到应答。
 */
module arbiter #
(
    parameter PORTS = 4,
    // 是否选择轮询仲裁
    parameter ARB_TYPE_ROUND_ROBIN = 0,
    // 是否启用阻塞仲裁
    parameter ARB_BLOCK = 0,
    // 阻塞条件：1 表示等待 acknowledge，0 表示等待 request 撤销
    parameter ARB_BLOCK_ACK = 1,
    // LSB 优先级方向选择
    parameter ARB_LSB_HIGH_PRIORITY = 0
)
(
    input  wire                     clk, // 仲裁状态时钟。
    input  wire                     rst, // 授权与掩码寄存器同步复位。

    input  wire [PORTS-1:0]         request, // 各端口请求位图。
    input  wire [PORTS-1:0]         acknowledge, // 在 ARB_BLOCK_ACK 模式下使用的应答位图。

    output wire [PORTS-1:0]         grant, // 被选端口的 one-hot 授权向量。
    output wire                     grant_valid, // 授权有效标志。
    output wire [$clog2(PORTS)-1:0] grant_encoded // 被授权端口的编码索引。
);

reg [PORTS-1:0] grant_reg = 0, grant_next; // 当前/下一拍 one-hot 授权寄存。
reg grant_valid_reg = 0, grant_valid_next; // 当前/下一拍授权有效标志。
reg [$clog2(PORTS)-1:0] grant_encoded_reg = 0, grant_encoded_next; // 当前/下一拍授权编码索引。

assign grant_valid = grant_valid_reg;
assign grant = grant_reg;
assign grant_encoded = grant_encoded_reg;

wire request_valid; // 表示至少存在一个有效请求。
wire [$clog2(PORTS)-1:0] request_index; // 完整请求向量中最高优先级请求的编码索引。
wire [PORTS-1:0] request_mask; // 完整请求向量中最高优先级请求的 one-hot 掩码。

priority_encoder #(
    .WIDTH(PORTS),
    .LSB_HIGH_PRIORITY(ARB_LSB_HIGH_PRIORITY)
)
priority_encoder_inst (
    .input_unencoded(request),
    .output_valid(request_valid),
    .output_encoded(request_index),
    .output_unencoded(request_mask)
);

reg [PORTS-1:0] mask_reg = 0, mask_next; // 轮询仲裁的旋转掩码状态。

wire masked_request_valid; // 当前轮询掩码下至少存在一个请求。
wire [$clog2(PORTS)-1:0] masked_request_index; // 掩码后请求集合中最高优先级请求的编码索引。
wire [PORTS-1:0] masked_request_mask; // 掩码后请求集合中最高优先级请求的 one-hot 掩码。

priority_encoder #(
    .WIDTH(PORTS),
    .LSB_HIGH_PRIORITY(ARB_LSB_HIGH_PRIORITY)
)
priority_encoder_masked (
    .input_unencoded(request & mask_reg),
    .output_valid(masked_request_valid),
    .output_encoded(masked_request_index),
    .output_unencoded(masked_request_mask)
);

always @* begin
    grant_next = 0;
    grant_valid_next = 0;
    grant_encoded_next = 0;
    mask_next = mask_reg;

    if (ARB_BLOCK && !ARB_BLOCK_ACK && grant_reg & request) begin
        // 已授权请求仍然保持：继续保持授权
        grant_valid_next = grant_valid_reg;
        grant_next = grant_reg;
        grant_encoded_next = grant_encoded_reg;
    end else if (ARB_BLOCK && ARB_BLOCK_ACK && grant_valid && !(grant_reg & acknowledge)) begin
        // 已授权请求尚未应答：继续保持授权
        grant_valid_next = grant_valid_reg;
        grant_next = grant_reg;
        grant_encoded_next = grant_encoded_reg;
    end else if (request_valid) begin
        if (ARB_TYPE_ROUND_ROBIN) begin
            if (masked_request_valid) begin
                grant_valid_next = 1;
                grant_next = masked_request_mask;
                grant_encoded_next = masked_request_index;
                if (ARB_LSB_HIGH_PRIORITY) begin
                    mask_next = {PORTS{1'b1}} << (masked_request_index + 1);
                end else begin
                    mask_next = {PORTS{1'b1}} >> (PORTS - masked_request_index);
                end
            end else begin
                grant_valid_next = 1;
                grant_next = request_mask;
                grant_encoded_next = request_index;
                if (ARB_LSB_HIGH_PRIORITY) begin
                    mask_next = {PORTS{1'b1}} << (request_index + 1);
                end else begin
                    mask_next = {PORTS{1'b1}} >> (PORTS - request_index);
                end
            end
        end else begin
            grant_valid_next = 1;
            grant_next = request_mask;
            grant_encoded_next = request_index;
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        grant_reg <= 0;
        grant_valid_reg <= 0;
        grant_encoded_reg <= 0;
        mask_reg <= 0;
    end else begin
        grant_reg <= grant_next;
        grant_valid_reg <= grant_valid_next;
        grant_encoded_reg <= grant_encoded_next;
        mask_reg <= mask_next;
    end
end

endmodule

`resetall
