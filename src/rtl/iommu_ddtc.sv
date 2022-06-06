// iommu_ddtc.v
// IOMMU DDT cache
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
module rv_iommu_ddtc
       #(parameter MAX_PPN=34,
         parameter MAX_PA=46,
         parameter MAX_DDTC_ENTRIES=4)
        (
        input wire                clk,
        input wire                rst_n,

        // Lookup/fill port
        input  wire               ddtc_lookup_i,
        input  wire               ddtc_fill_i,
        input  wire [23:0]        device_id_i,

        // DDTC lookup result
        output wire               ddtc_lkup_fill_done_o,
        output wire               ddtc_hit_o,
        output wire               en_ats_o,
        output wire               en_pri_o,
        output wire               t2gpa_o,
        output wire               dtf_o,
        output wire               pdtv_o,
        output wire               prpr_o,
        output wire [3:0]         iohgatp_mode_o,
        output wire [15:0]        gscid_o,
        output wire [MAX_PPN-1:0] iohgatp_ppn_o,
        output wire [3:0]         fsc_mode_o,
        output wire [MAX_PPN-1:0] fsc_ppn_o,
        output wire [19:0]        dc_pscid_o,
        output wire [3:0]         msiptp_mode_o,
        output wire [MAX_PPN-1:0] msiptp_ppn_o,
        output wire [51:0]        msi_addr_mask_o,
        output wire [51:0]        msi_addr_pat_o,

        // DDTC fill data
        input wire               en_ats_i,
        input wire               en_pri_i,
        input wire               t2gpa_i,
        input wire               dtf_i,
        input wire               pdtv_i,
        input wire               prpr_i,
        input wire [3:0]         iohgatp_mode_i,
        input wire [15:0]        gscid_i,
        input wire [MAX_PPN-1:0] iohgatp_ppn_i,
        input wire [3:0]         fsc_mode_i,
        input wire [MAX_PPN-1:0] fsc_ppn_i,
        input wire [19:0]        dc_pscid_i,
        input wire [3:0]         msiptp_mode_i,
        input wire [MAX_PPN-1:0] msiptp_ppn_i,
        input wire [51:0]        msi_addr_mask_i,
        input wire [51:0]        msi_addr_pat_i,

        // invalidate port
        input  wire [23:0]       inval_device_id_i,
        input  wire              inval_device_id_valid_i,
        input  wire              ddtc_inval_i,
        output wire              ddtc_inval_done_o
    );
    reg                                ddtc_hit;
    reg [$clog2(MAX_DDTC_ENTRIES)-1:0] hit_row;

    // Device context cache state
    reg                                ddtc_en_ats[MAX_DDTC_ENTRIES];
    reg                                ddtc_en_pri[MAX_DDTC_ENTRIES];
    reg                                ddtc_t2gpa[MAX_DDTC_ENTRIES];
    reg                                ddtc_dtf[MAX_DDTC_ENTRIES];
    reg                                ddtc_pdtv[MAX_DDTC_ENTRIES];
    reg                                ddtc_prpr[MAX_DDTC_ENTRIES];
    reg [3:0]                          ddtc_iohgatp_mode[MAX_DDTC_ENTRIES];
    reg [15:0]                         ddtc_gscid[MAX_DDTC_ENTRIES];
    reg [MAX_PPN-1:0]                  ddtc_iohgatp_ppn[MAX_DDTC_ENTRIES];
    reg [3:0]                          ddtc_fsc_mode[MAX_DDTC_ENTRIES];
    reg [MAX_PPN-1:0]                  ddtc_fsc_ppn[MAX_DDTC_ENTRIES];
    reg [19:0]                         ddtc_dc_pscid[MAX_DDTC_ENTRIES];
    reg [3:0]                          ddtc_msiptp_mode[MAX_DDTC_ENTRIES];
    reg [MAX_PPN-1:0]                  ddtc_msiptp_ppn[MAX_DDTC_ENTRIES];
    reg [51:0]                         ddtc_msi_addr_mask[MAX_DDTC_ENTRIES];
    reg [51:0]                         ddtc_msi_addr_pat[MAX_DDTC_ENTRIES];
    reg [23:0]                         ddtc_device_id_tag[MAX_DDTC_ENTRIES];
    reg [MAX_DDTC_ENTRIES-1:0]         ddtc_valid;
    reg [$clog2(MAX_DDTC_ENTRIES):0]   lru, lru_q;
    reg [$clog2(MAX_DDTC_ENTRIES)-1:0] victim, victim_q;

    // Device context cache output signals
    assign ddtc_inval_done_o = 
                ddtc_inval_i & !ddtc_lookup_i & !ddtc_fill_i;
    assign ddtc_lkup_fill_done_o = 
                (!ddtc_inval_i & (ddtc_lookup_i | ddtc_fill_i));
    assign ddtc_hit_o = ddtc_hit;
    assign en_ats_o = ddtc_en_ats[hit_row];
    assign en_pri_o = ddtc_en_pri[hit_row];
    assign t2gpa_o = ddtc_t2gpa[hit_row];
    assign dtf_o = ddtc_dtf[hit_row];
    assign pdtv_o = ddtc_pdtv[hit_row];
    assign prpr_o = ddtc_prpr[hit_row];
    assign iohgatp_mode_o = ddtc_iohgatp_mode[hit_row];
    assign gscid_o = ddtc_gscid[hit_row];
    assign iohgatp_ppn_o = ddtc_iohgatp_ppn[hit_row];
    assign fsc_mode_o = ddtc_fsc_mode[hit_row];
    assign fsc_ppn_o = ddtc_fsc_ppn[hit_row];
    assign dc_pscid_o = ddtc_dc_pscid[hit_row];
    assign msiptp_mode_o = ddtc_msiptp_mode[hit_row];
    assign msiptp_ppn_o = ddtc_msiptp_ppn[hit_row];
    assign msi_addr_mask_o = ddtc_msi_addr_mask[hit_row];
    assign msi_addr_pat_o = ddtc_msi_addr_pat[hit_row];

    // Device context cache process
    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            ddtc_valid <= 4'b0000;
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
        casez (ddtc_valid)
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
        case ({ddtc_hit, hit_row})
            3'b100: lru = (lru & 3'b001) | 3'b110;
            3'b101: lru = (lru & 3'b001) | 3'b010;
            3'b110: lru = (lru & 3'b100) | 3'b001;
            3'b111: lru = (lru & 3'b100) | 3'b000;
        endcase
    end
    // DDT cache fill logic
    always_comb begin
        int f;
        for ( f = 0; f < MAX_DDTC_ENTRIES; f++) begin
            // replace a victim entry with the new tag and data
            if ( (f == victim_q) && 
                 !ddtc_inval_i && !ddtc_lookup_i && ddtc_fill_i ) begin
                ddtc_device_id_tag[f] = device_id_i;
                ddtc_en_ats[f] = en_ats_i;
                ddtc_en_pri[f] = en_pri_i;
                ddtc_t2gpa[f] = t2gpa_i;
                ddtc_dtf[f] = dtf_i;
                ddtc_pdtv[f] = pdtv_i;
                ddtc_prpr[f] = prpr_i;
                ddtc_iohgatp_mode[f] = iohgatp_mode_i;
                ddtc_gscid[f] = gscid_i;
                ddtc_iohgatp_ppn[f] = iohgatp_ppn_i;
                ddtc_fsc_mode[f] = fsc_mode_i;
                ddtc_fsc_ppn[f] = fsc_ppn_i;
                ddtc_dc_pscid[f] = dc_pscid_i;
                ddtc_msiptp_mode[f] = msiptp_mode_i;
                ddtc_msiptp_ppn[f] = msiptp_ppn_i;
                ddtc_msi_addr_mask[f] = msi_addr_mask_i;
                ddtc_msi_addr_pat[f] = msi_addr_pat_i;
                ddtc_valid = ddtc_valid | (1 << f);
            end
        end   
    end
    // DDT cache lookup logic
    always_comb begin
        int t;
        ddtc_hit = 0;
        hit_row = 0;
        for ( t = 0; t < MAX_DDTC_ENTRIES; t++) begin
            // Signal hit if a valid tag matches. If invalidation in 
            // progress then hold off signaling hit
            if ( (ddtc_device_id_tag[t] == device_id_i) && ddtc_valid[t] && 
                 !ddtc_inval_i && ddtc_lookup_i && !ddtc_fill_i ) begin
                ddtc_hit = 1;
                hit_row = t;
            end
        end
    end 
    // DDT cache invalidation logic
    always_comb begin
        int i;
        for ( i = 0; i < MAX_DDTC_ENTRIES; i++) begin
            // If device ID specified then mark that entry invalid else mark all
            // entries invalid Do not invalidate entries when a fill or lookup 
            // is in progress
            if ( (((inval_device_id_valid_i == 1) && 
                   (ddtc_device_id_tag[i] == inval_device_id_i) && ddtc_valid[i]) |
                  (inval_device_id_valid_i == 0)) && 
                 ddtc_inval_i && !ddtc_lookup_i && !ddtc_fill_i ) begin
                ddtc_valid[i] = 0;
            end
        end
    end
endmodule
