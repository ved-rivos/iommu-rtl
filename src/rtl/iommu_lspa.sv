// iommu_lspa.v
// IOMMU Load/store port arbiter
// Author - ved@rivosinc.com
// Copyright (c) 2022, Rivos inc.
// All rights reserved
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================
module rv_iommu_lspa
        (
        input wire      clk,
        input wire      rst_n,

        // Load/store from walker
        input wire [45:0]       w_ls_addr_i,
        input wire [1:0]        w_ls_op_i,
        output wire [$clog2(MAX_PW)-1:0] w_ls_tag_i,
        input wire [6:0]        w_ls_size_i,
        input wire              w_ls_req_irdy_i,
        output  wire            w_ls_req_trdy_o,

        // Load/AMO data return to walker
        output  wire [511:0]    w_ld_data_o,
        output  wire            w_ld_acc_fault_o,
        output  wire            w_ld_poison_o,
        output wire [$clog2(MAX_PW)-1:0] w_ld_tag_o,
        output  wire            w_ld_data_irdy_o,
        input wire              w_ld_data_trdy_i
    );
    `include "consts.vh"
    integer state;

    reg w_ls_req_trdy;
    assign w_ls_req_trdy_o = w_ls_req_trdy;

    reg [511:0] w_ld_data;
    reg w_ld_acc_fault;
    reg w_ld_poison;
    reg [$clog2(MAX_PW)-1:0] w_ls_tag;
    reg w_ld_data_irdy;

    assign w_ld_data_o = w_ld_data;
    assign w_ld_acc_fault_o = w_ld_acc_fault;
    assign w_ld_poison_o = w_ld_poison;
    assign w_ld_tag_o = w_ls_tag;
    assign w_ld_data_irdy_o = w_ld_data_irdy;


    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            state <= 0;
            w_ld_data <= 0;
            w_ld_acc_fault <= 0;
            w_ld_poison <= 0;
            w_ls_req_trdy <= 0;
            w_ld_data_irdy <= 0;
        end
        else begin
            case (state)
                0 : begin
                    if ( w_ls_req_irdy_i == 1 ) begin
                        w_ls_tag <= w_ls_tag_i;
                        state <= 1;
                        if ( w_ls_addr_i == 512'h1000 ) begin
                            w_ld_data <= 512'h2001;
                        end
                        if ( w_ls_addr_i == 512'h2020 ) begin
                            w_ld_data <= 512'h3001;
                        end
                        if ( w_ls_addr_i == 512'h3048 ) begin
                            w_ld_data <= 512'h4001;
                        end
                    end
                end
                1 : begin
                    w_ls_req_trdy <= 1;
                    if ( w_ls_req_irdy_i == 0 ) begin
                        if ( w_ls_op_i == LOAD ) begin
                            state <= 2;
                        end else begin
                            state <= 0;
                        end
                        w_ls_req_trdy <= 0;
                    end
                end
                2 : begin
                    w_ld_data_irdy <= 1;
                    if ( w_ld_data_trdy_i == 1 ) begin
                        state <= 3;
                    end
                end
                3 : begin
                    w_ld_data_irdy <= 0;
                    if ( w_ld_data_trdy_i == 0 ) begin
                        state <= 0;
                    end
                end
            endcase
        end
    end
endmodule
