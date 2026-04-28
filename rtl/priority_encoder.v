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
 * 优先级编码器模块
 *
 * 模块目录
 * 1) 把 one-hot/multi-hot 输入转换为有效位和编码索引。
 * 2) 支持参数化优先级方向（LSB 高优先或低优先）。
 * 3) 同时返回所选编码索引对应的 one-hot 输出。
 */
module priority_encoder #
(
    parameter WIDTH = 4,
    // LSB 优先级方向选择
    parameter LSB_HIGH_PRIORITY = 0
)
(
    input  wire [WIDTH-1:0]         input_unencoded, // 原始请求位图（可同时有多个位置 1）。
    output wire                     output_valid, // 任意输入位为 1 时拉高。
    output wire [$clog2(WIDTH)-1:0] output_encoded, // 选中优先级位的编码索引。
    output wire [WIDTH-1:0]         output_unencoded // 根据编码索引重建的 one-hot 输出。
);

parameter LEVELS = WIDTH > 2 ? $clog2(WIDTH) : 1; // 归约树层数。
parameter W = 2**LEVELS; // 内部使用的 2 的幂补齐位宽。

// 把输入补齐到 2 的幂位宽
wire [W-1:0] input_padded = {{W-WIDTH{1'b0}}, input_unencoded}; // 为平衡树逻辑补齐到 2 的幂位宽。

wire [W/2-1:0] stage_valid[LEVELS-1:0]; // 每级归约树的有效位。
wire [W/2-1:0] stage_enc[LEVELS-1:0]; // 每级归约树的部分编码结果。

generate
    genvar l, n;

    // 处理输入位：每两位生成一组有效位和编码位
    for (n = 0; n < W/2; n = n + 1) begin : loop_in
        assign stage_valid[0][n] = |input_padded[n*2+1:n*2];
        if (LSB_HIGH_PRIORITY) begin
            // bit 0 为最高优先级
            assign stage_enc[0][n] = !input_padded[n*2+0];
        end else begin
            // bit 0 为最低优先级
            assign stage_enc[0][n] = input_padded[n*2+1];
        end
    end

    // 逐级归约到单个有效位和完整编码总线
    for (l = 1; l < LEVELS; l = l + 1) begin : loop_levels
        for (n = 0; n < W/(2*2**l); n = n + 1) begin : loop_compress
            assign stage_valid[l][n] = |stage_valid[l-1][n*2+1:n*2];
            if (LSB_HIGH_PRIORITY) begin
                // bit 0 为最高优先级
                assign stage_enc[l][(n+1)*(l+1)-1:n*(l+1)] = stage_valid[l-1][n*2+0] ? {1'b0, stage_enc[l-1][(n*2+1)*l-1:(n*2+0)*l]} : {1'b1, stage_enc[l-1][(n*2+2)*l-1:(n*2+1)*l]};
            end else begin
                // bit 0 为最低优先级
                assign stage_enc[l][(n+1)*(l+1)-1:n*(l+1)] = stage_valid[l-1][n*2+1] ? {1'b1, stage_enc[l-1][(n*2+2)*l-1:(n*2+1)*l]} : {1'b0, stage_enc[l-1][(n*2+1)*l-1:(n*2+0)*l]};
            end
        end
    end
endgenerate

assign output_valid = stage_valid[LEVELS-1];
assign output_encoded = stage_enc[LEVELS-1];
assign output_unencoded = 1 << output_encoded;

endmodule

`resetall
