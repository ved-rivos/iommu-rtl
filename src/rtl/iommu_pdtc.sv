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
`include "iommu.svh"
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
        output wire pc_t   pc_o,
        // DDTC fill data
        input  wire pc_t   pc_i,

        // flush port
        input  wire [23:0] flush_device_id_i,
        input  wire [19:0] flush_process_id_i,
        input  wire        pdtc_flush_i,
        output wire        pdtc_flush_done_o
    );
    reg pdtc_hit, pdtc_lkup_fill_done, pdtc_flush_done;
    integer i, hit_row, j;

    reg [23:0] device_id_tag[4];
    reg [19:0] process_id_tag[4];
    reg pc_t   process_ctx[4];
    reg        valid[4];
    reg [1:0]  lru[4];

    reg        invalid_found;
    reg [1:0]  invalid_entry;
    reg [1:0]  lru_entry, lru_val;
    reg [1:0]  victim;

    assign pdtc_lkup_fill_done_o = pdtc_lkup_fill_done;
    assign pdtc_hit_o = pdtc_hit;
    assign pc_o = process_ctx[hit_row];

    assign pdtc_flush_done_o = pdtc_flush_done;

    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            for ( i = 0; i < 4; i++) begin
                valid[i] <= 0;
            end
            pdtc_lkup_fill_done <= 0;
            pdtc_hit <= 0;
            pdtc_flush_done <= 0;
        end
        else begin
            // Flush logic
            if ( pdtc_flush_i && pdtc_flush_done == 0 ) begin
                for ( i = 0; i < 4; i++) begin
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
                invalid_found = 0;
                lru_entry = 0;
                lru_val = 3;
                // Find victim - invalid or LRU
                for ( i = 0; i < 4; i++ ) begin
                    if ( valid[i] == 0 ) begin
                        invalid_found <= 1;
                        invalid_entry <= i;
                    end
                    if ( lru[i] <= lru_val ) begin
                        lru_val <= lru[i];
                        lru_entry <= i;
                    end
                end
                victim = (invalid_found) ? invalid_entry : lru_entry;
                device_id_tag[victim] <= device_id_i;
                process_id_tag[victim] <= process_id_i;
                process_ctx[victim] <= pc_i;
                pdtc_lkup_fill_done <= 1;
            end
            // Lookup logic - invoked if no fill or flush being done
            if ( pdtc_lookup_i && !pdtc_fill_i && !pdtc_flush_i && pdtc_lkup_fill_done == 0) begin
                for ( i = 0; i < 4; i++) begin
                    if ( (device_id_tag[i] == device_id_i) && valid[i] && 
                         (process_id_tag[i] == process_id_i) ) begin
                        hit_row <= i;
                        pdtc_hit <= 1;
                        // age older entries and make entry MRU
                        for ( j = 0; j < 4; j++ ) begin
                            if ( lru[j] > lru[i] ) begin
                                lru[j] <= lru[j] - 1;
                            end
                        end
                        lru[i] <= 3;
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
