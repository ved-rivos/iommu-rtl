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
       #(parameter MAX_PPN=34,
         parameter MAX_PA=46,
         parameter MAX_PDTC_ENTRIES=4)
        (
        input wire                clk,
        input wire                rst_n,

        // Lookup/fill port
        input  wire               pdtc_lookup_i,
        input  wire               pdtc_fill_i,
        input  wire [23:0]        device_id_i,
        input  wire [19:0]        process_id_i,

        // PDTC lookup result
        output wire               pdtc_lkup_fill_done_o,
        output wire               pdtc_hit_o,
        output wire               ens_o,
        output wire               sum_o,
        output wire [19:0]        pscid_o,
        output wire [3:0]         fsc_mode_o,
        output wire [MAX_PPN-1:0] fsc_ppn_o,
        // PDTC fill data
        input wire                ens_i,
        input wire                sum_i,
        input wire [19:0]         pscid_i,
        input wire [3:0]          fsc_mode_i,
        input wire [MAX_PPN-1:0]  fsc_ppn_i,

        // invalidate port
        input  wire [23:0]        inval_device_id_i,
        input  wire [19:0]        inval_process_id_i,
        input  wire               pdtc_inval_i,
        output wire               pdtc_inval_done_o
    );
    reg                                pdtc_hit;
    reg [$clog2(MAX_PDTC_ENTRIES)-1:0] hit_row;

    reg [23:0]                         pdtc_device_id_tag[MAX_PDTC_ENTRIES];
    reg [19:0]                         pdtc_process_id_tag[MAX_PDTC_ENTRIES];
    reg                                pdtc_ens[MAX_PDTC_ENTRIES];
    reg                                pdtc_sum[MAX_PDTC_ENTRIES];
    reg [19:0]                         pdtc_pscid[MAX_PDTC_ENTRIES];
    reg [3:0]                          pdtc_fsc_mode[MAX_PDTC_ENTRIES];
    reg [MAX_PPN-1:0]                  pdtc_fsc_ppn[MAX_PDTC_ENTRIES];
    reg [$clog2(MAX_PDTC_ENTRIES):0]   lru, lru_q;
    reg [$clog2(MAX_PDTC_ENTRIES)-1:0] victim, victim_q;
    reg [MAX_PDTC_ENTRIES-1:0]         pdtc_valid;


    assign pdtc_inval_done_o = 
               pdtc_inval_i & !pdtc_lookup_i & !pdtc_fill_i;
    assign pdtc_lkup_fill_done_o = 
               (!pdtc_inval_i & (pdtc_lookup_i | pdtc_fill_i));
    assign pdtc_hit_o = pdtc_hit;

    assign ens_o = pdtc_ens[hit_row];
    assign sum_o = pdtc_sum[hit_row];
    assign pscid_o = pdtc_pscid[hit_row];
    assign fsc_mode_o = pdtc_fsc_mode[hit_row];
    assign fsc_ppn_o = pdtc_fsc_ppn[hit_row];

    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            pdtc_valid <= 0;
            victim_q <= 0;
            lru_q <= 0;
        end else begin
            victim_q <= victim;
            lru_q <= lru;
        end
    end
    // pick a invalid entry as victim; if all valid then pick
    // LRU entry as victim
    always_comb begin
        victim = victim_q;
        casez (pdtc_valid)
            4'b???0: victim = 0;
            4'b??01: victim = 1;
            4'b?011: victim = 2;
            4'b0111: victim = 3;
            4'b1111:
                casez (lru)
                    3'b00?: victim = 0;
                    3'b10?: victim = 1;
                    3'b?10: victim = 2;
                    3'b?11: victim = 3;
                endcase
        endcase
    end
    // DDT LRU update logic; make hit entry as MRU
    always_comb begin
        lru = lru_q;
        case ({pdtc_hit, hit_row})
            3'b100: lru = (lru & 3'b001) | 3'b110;
            3'b101: lru = (lru & 3'b001) | 3'b010;
            3'b110: lru = (lru & 3'b100) | 3'b001;
            3'b111: lru = (lru & 3'b100) | 3'b000;
        endcase
    end
    // DDT cache fill logic
    always_comb begin
        int f;
        for ( f = 0; f < MAX_PDTC_ENTRIES; f++) begin
            // replace a victim entry with the new tag and data
            if ( (f == victim_q) && 
                 !pdtc_inval_i && !pdtc_lookup_i && pdtc_fill_i ) begin
                pdtc_device_id_tag[f] = device_id_i;
                pdtc_process_id_tag[f] = process_id_i;
                pdtc_ens[f] = ens_i;
                pdtc_sum[f] = sum_i;
                pdtc_pscid[f] = pscid_i;
                pdtc_fsc_mode[f] = fsc_mode_i;
                pdtc_fsc_ppn[f] = fsc_ppn_i;
                pdtc_valid = pdtc_valid | (1 << f);
            end
        end   
    end
    // DDT cache lookup logic
    always_comb begin
        int t;
        pdtc_hit = 0;
        hit_row = 0;
        for ( t = 0; t < MAX_PDTC_ENTRIES; t++) begin
            // Signal hit if a valid tag matches. If invalidation in progress 
            // then hold off signaling hit
            if ( (pdtc_device_id_tag[t] == device_id_i) && 
                 (pdtc_process_id_tag[t] == process_id_i) && pdtc_valid[t] && 
                 !pdtc_inval_i && pdtc_lookup_i && !pdtc_fill_i ) begin
                pdtc_hit = 1;
                hit_row = t;
            end
        end
    end 
    // DDT cache invalidation logic
    always_comb begin
        int i;
        for ( i = 0; i < MAX_PDTC_ENTRIES; i++) begin
            // match valid entries with matching device and process id invalid
            // Do not invalidate entries when a fill or lookup is in progress
            if ( (pdtc_process_id_tag[i] == inval_process_id_i) && 
                 (pdtc_device_id_tag[i] == inval_device_id_i) && pdtc_valid[i] &&
                  pdtc_inval_i && !pdtc_lookup_i && !pdtc_fill_i ) begin
                pdtc_valid[i] = 0;
            end
        end
    end
endmodule
