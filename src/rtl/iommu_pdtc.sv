// iommu_pdtc.v
// IOMMU PDT cache
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
module rv_iommu_pdtc
        (
        input wire      clk,
        input wire      rst_n,

        // Lookup/fill port
        input  wire        pdtc_lookup_i,
        input  wire        pdtc_fill_i,
        input  wire [23:0] device_id_i,
        input  wire [19:0] process_id_i,

        // DDTC lookup result
        output wire        pdtc_lkup_fill_done_o,
        output wire        pdtc_hit_o,
        output wire        ens_o,
        output wire        sum_o,
        output wire [19:0] pscid_o,
        output wire [3:0]  fsc_mode_o,
        output wire [33:0] fsc_ppn_o,
        // DDTC fill data
        input wire        ens_i,
        input wire        sum_i,
        input wire [19:0] pscid_i,
        input wire [3:0]  fsc_mode_i,
        input wire [33:0] fsc_ppn_i,

        // flush port
        input  wire [23:0] flush_device_id_i,
        input  wire [19:0] flush_process_id_i,
        input  wire        pdtc_flush_i,
        output wire        pdtc_flush_done_o
    );
    reg pdtc_hit, pdtc_lkup_fill_done, pdtc_flush_done;
    reg [1:0] i, hit_row;

    reg [23:0] device_id_tag[4];
    reg [19:0] process_id_tag[4];
    reg        process_ctx_ens[4];
    reg        process_ctx_sum[4];
    reg [19:0] process_ctx_pscid[4];
    reg [3:0]  process_ctx_fsc_mode[4];
    reg [33:0] process_ctx_fsc_ppn[4];
    reg [3:0]  valid;
    reg [2:0]  lru;
    reg [1:0]  victim;

    assign pdtc_flush_done_o = pdtc_flush_done;

    assign pdtc_lkup_fill_done_o = pdtc_lkup_fill_done;
    assign pdtc_hit_o = pdtc_hit;
    assign ens_o = process_ctx_ens[hit_row];
    assign sum_o = process_ctx_sum[hit_row];
    assign pscid_o = process_ctx_pscid[hit_row];
    assign fsc_mode_o = process_ctx_fsc_mode[hit_row];
    assign fsc_ppn_o = process_ctx_fsc_ppn[hit_row];

    // pick invalid as victim or the lru if all valid
    function [1:0] make_victim();
        reg [1:0] row;
        casez (valid)
            4'b???0 : row = 0;
            4'b0000 : row = 0;
            4'b??0? : row = 1;
            4'b?0?? : row = 2;
            4'b0??? : row = 3;
            4'b1111 : 
                casez (lru)
                    3'b00?: row = 0;
                    3'b10?: row = 1;
                    3'b?10: row = 2;
                    3'b?11: row = 3;
                endcase
        endcase
        return row;
    endfunction

    // Make entry MRU and update the LRU
    function void make_mru(reg [1:0] row);
        case (row)
            0: lru = {2'b11, lru[0]};
            1: lru = {2'b01, lru[0]};
            2: lru = {lru[2], 2'b01};
            3: lru = {lru[2], 2'b00};
        endcase
    endfunction

    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            valid <= 4'b0000;
            lru = 3'b000;
            pdtc_lkup_fill_done <= 0;
            pdtc_hit <= 0;
            pdtc_flush_done <= 0;
        end
        else begin
            // Flush logic
            if ( pdtc_flush_i && pdtc_flush_done == 0 ) begin
                for ( i = 0; i <= 3; i++) begin
                    if ( (process_id_tag[i] == flush_process_id_i) && 
                         (device_id_tag[i] == flush_device_id_i) ) begin
                        valid[i] <= 0;
                    end
                end
                pdtc_flush_done <= 1;
            end
            if ( pdtc_flush_i == 0 && pdtc_flush_done == 1) begin
                pdtc_flush_done <= 0;
            end
            // Fill logic - invoked if no lookup or flush being done
            if ( pdtc_fill_i && !pdtc_lookup_i && !pdtc_flush_i && pdtc_lkup_fill_done == 0) begin
                victim <= make_victim();
                device_id_tag[victim] <= device_id_i;
                process_id_tag[victim] <= process_id_i;
                process_ctx_ens[victim] <= ens_i;
                process_ctx_sum[victim] <= sum_i;
                process_ctx_pscid[victim] <= pscid_i;
                process_ctx_fsc_mode[victim] <= fsc_mode_i;
                process_ctx_fsc_ppn[victim] <= fsc_ppn_i;
                pdtc_lkup_fill_done <= 1;
            end
            // Lookup logic - invoked if no fill or flush being done
            if ( pdtc_lookup_i && !pdtc_fill_i && !pdtc_flush_i && pdtc_lkup_fill_done == 0) begin
                for ( i = 0; i <= 3; i++) begin
                    if ( (device_id_tag[i] == device_id_i) && valid[i] && 
                         (process_id_tag[i] == process_id_i) ) begin
                        hit_row <= i;
                        pdtc_hit <= 1;
                        make_mru(i);
                    end
                end
                pdtc_lkup_fill_done <= 1;
            end
            if ( pdtc_lkup_fill_done == 1 && pdtc_lookup_i == 0 && pdtc_fill_i == 0) begin
                pdtc_lkup_fill_done <= 0;
                pdtc_hit <= 0;
            end
        end
    end
endmodule
