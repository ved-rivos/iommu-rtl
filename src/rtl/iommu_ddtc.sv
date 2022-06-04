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
        (
        input wire      clk,
        input wire      rst_n,

        // Lookup/fill port
        input  wire        ddtc_lookup_i,
        input  wire        ddtc_fill_i,
        input  wire [23:0] device_id_i,

        // DDTC lookup result
        output wire        ddtc_lkup_fill_done_o,
        output wire        ddtc_hit_o,
        output wire        en_ats_o,
        output wire        en_pri_o,
        output wire        t2gpa_o,
        output wire        dtf_o,
        output wire        pdtv_o,
        output wire        prpr_o,
        output wire [3:0]  iohgatp_mode_o,
        output wire [15:0] gscid_o,
        output wire [33:0] iohgatp_ppn_o,
        output wire [3:0]  fsc_mode_o,
        output wire [33:0] fsc_ppn_o,
        output wire [19:0] dc_pscid_o,
        output wire [3:0]  msiptp_mode_o,
        output wire [43:0] msiptp_ppn_o,
        output wire [51:0] msi_addr_mask_o,
        output wire [51:0] msi_addr_pat_o,

        // DDTC fill data
        input wire        en_ats_i,
        input wire        en_pri_i,
        input wire        t2gpa_i,
        input wire        dtf_i,
        input wire        pdtv_i,
        input wire        prpr_i,
        input wire [3:0]  iohgatp_mode_i,
        input wire [15:0] gscid_i,
        input wire [33:0] iohgatp_ppn_i,
        input wire [3:0]  fsc_mode_i,
        input wire [33:0] fsc_ppn_i,
        input wire [19:0] dc_pscid_i,
        input wire [3:0]  msiptp_mode_i,
        input wire [43:0] msiptp_ppn_i,
        input wire [51:0] msi_addr_mask_i,
        input wire [51:0] msi_addr_pat_i,

        // flush port
        input  wire [23:0] flush_device_id_i,
        input  wire        ddtc_flush_i,
        input  wire        flush_device_id_valid_i,
        output wire        ddtc_flush_done_o
    );
    reg ddtc_lkup_fill_done, ddtc_flush_done;
    reg ddtc_hit[4];
    reg [1:0] hit_row;
    reg [2:0] i;

    reg        dcc_en_ats[4];
    reg        dcc_en_pri[4];
    reg        dcc_t2gpa[4];
    reg        dcc_dtf[4];
    reg        dcc_pdtv[4];
    reg        dcc_prpr[4];
    reg [3:0]  dcc_iohgatp_mode[4];
    reg [15:0] dcc_gscid[4];
    reg [33:0] dcc_iohgatp_ppn[4];
    reg [3:0]  dcc_fsc_mode[4];
    reg [33:0] dcc_fsc_ppn[4];
    reg [19:0] dcc_dc_pscid[4];
    reg [3:0]  dcc_msiptp_mode[4];
    reg [43:0] dcc_msiptp_ppn[4];
    reg [51:0] dcc_msi_addr_mask[4];
    reg [51:0] dcc_msi_addr_pat[4];
    reg [23:0] device_id_tag[4];
    reg        valid[4];
    reg [2:0]  lru;
    reg [2:0]  victim;

    assign ddtc_flush_done_o = ddtc_flush_i & !ddtc_lookup_i & !ddtc_fill_i;
    assign ddtc_lkup_fill_done_o = !ddtc_flush_i & (ddtc_lookup_i | !ddtc_fill_i);
    assign ddtc_hit_o = ddtc_hit[0] | ddtc_hit[1] | ddtc_hit[2] | ddtc_hit[3];

    assign en_ats_o = dcc_en_ats[hit_row];
    assign en_pri_o = dcc_en_pri[hit_row];
    assign t2gpa_o = dcc_t2gpa[hit_row];
    assign dtf_o = dcc_dtf[hit_row];
    assign pdtv_o = dcc_pdtv[hit_row];
    assign prpr_o = dcc_prpr[hit_row];
    assign iohgatp_mode_o = dcc_iohgatp_mode[hit_row];
    assign gscid_o = dcc_gscid[hit_row];
    assign iohgatp_ppn_o = dcc_iohgatp_ppn[hit_row];
    assign fsc_mode_o = dcc_fsc_mode[hit_row];
    assign fsc_ppn_o = dcc_fsc_ppn[hit_row];
    assign dc_pscid_o = dcc_dc_pscid[hit_row];
    assign msiptp_mode_o = dcc_msiptp_mode[hit_row];
    assign msiptp_ppn_o = dcc_msiptp_ppn[hit_row];
    assign msi_addr_mask_o = dcc_msi_addr_mask[hit_row];
    assign msi_addr_pat_o = dcc_msi_addr_pat[hit_row];

    // pick invalid as victim or the lru if all valid
    //always_comb begin
    always @(negedge rst_n or negedge ddtc_fill_i) begin
        if ( rst_n == 0 ) begin
            valid[0] = 0;
            valid[1] = 0;
            valid[2] = 0;
            valid[3] = 0;
            lru = 3'b000;
            ddtc_lkup_fill_done <= 0;
            ddtc_hit[0] = 0;
            ddtc_hit[1] = 0;
            ddtc_hit[2] = 0;
            ddtc_hit[3] = 0;
            ddtc_flush_done <= 0;
        end
        if ( valid[0] & valid[1] & valid[2] & valid[3] ) begin
                casez (lru)
                    3'b00?: victim = 0;
                    3'b10?: victim = 1;
                    3'b?10: victim = 2;
                    3'b?11: victim = 3;
                endcase
        end else begin
            victim = valid[0] == 0 ? 0 :
                     valid[1] == 0 ? 1 :
                     valid[2] == 0 ? 2 : 3;
        end
    end
    genvar k;
    generate
        for ( k = 0; k < 4; k++) begin
            always_comb begin
                if ( k == victim && !ddtc_flush_i && !ddtc_lookup_i && ddtc_fill_i ) begin
                    device_id_tag[k] = device_id_i;
                    dcc_en_ats[k] = en_ats_i;
                    dcc_en_pri[k] = en_pri_i;
                    dcc_t2gpa[k] = t2gpa_i;
                    dcc_dtf[k] = dtf_i;
                    dcc_pdtv[k] = pdtv_i;
                    dcc_prpr[k] = prpr_i;
                    dcc_iohgatp_mode[k] = iohgatp_mode_i;
                    dcc_gscid[k] = gscid_i;
                    dcc_iohgatp_ppn[k] = iohgatp_ppn_i;
                    dcc_fsc_mode[k] = fsc_mode_i;
                    dcc_fsc_ppn[k] = fsc_ppn_i;
                    dcc_dc_pscid[k] = dc_pscid_i;
                    dcc_msiptp_mode[k] = msiptp_mode_i;
                    dcc_msiptp_ppn[k] = msiptp_ppn_i;
                    dcc_msi_addr_mask[k] = msi_addr_mask_i;
                    dcc_msi_addr_pat[k] = msi_addr_pat_i;
                    valid[k] = 1;
                end
            end
        end
    endgenerate

    generate
    for ( k = 0; k < 4; k++) begin
        always_comb begin
            if ( (device_id_tag[k] == device_id_i) && valid[k] && !ddtc_flush_i && ddtc_lookup_i && !ddtc_fill_i ) begin
                ddtc_hit[k] = 1;
                hit_row = k;
                case (k)
                    0: lru = (lru & 3'b001) | 3'b110;
                    1: lru = (lru & 3'b001) | 3'b010;
                    2: lru = (lru & 3'b100) | 3'b001;
                    3: lru = (lru & 3'b100) | 3'b000;
                endcase
            end
        end
    end
    endgenerate

    generate
    for ( k = 0; k < 4; k++) begin
        always_comb begin
            if ( (((flush_device_id_valid_i == 1) && (device_id_tag[k] == flush_device_id_i)) |
                  (flush_device_id_valid_i == 0)) && ddtc_flush_i && !ddtc_lookup_i && !ddtc_fill_i ) begin
                valid[k] = 0;
            end
        end
    end
    endgenerate


//    always @(negedge rst_n or posedge ddtc_flush_i or posedge ddtc_lookup_i or posedge ddtc_fill_i or 
//                              negedge ddtc_flush_i or negedge ddtc_lookup_i or negedge ddtc_fill_i ) begin
//        if ( rst_n == 0 ) begin
//            valid <= 4'b0000;
//            lru = 3'b000;
//            ddtc_lkup_fill_done <= 0;
//            ddtc_hit <= 0;
//            ddtc_flush_done <= 0;
//        end
//        else begin
//            // Flush logic
//            if ( ddtc_flush_i && !ddtc_fill_i && !ddtc_lookup_i && ddtc_flush_done == 0 ) begin
//                do_flush <= 1;
//                ddtc_flush_done <= 1;
//            end
//            if ( ddtc_flush_i == 0 && ddtc_flush_done == 1) begin
//                ddtc_flush_done <= 0;
//            end
//            // Fill logic - invoked if no lookup or flush being done
//            if ( ddtc_fill_i && !ddtc_lookup_i && !ddtc_flush_i && ddtc_lkup_fill_done == 0) begin
//                //victim <= make_victim();
//                device_id_tag[victim] <= device_id_i;
//                dcc_en_ats[victim] <= en_ats_i;
//                dcc_en_pri[victim] <= en_pri_i;
//                dcc_t2gpa[victim] <= t2gpa_i;
//                dcc_dtf[victim] <= dtf_i;
//                dcc_pdtv[victim] <= pdtv_i;
//                dcc_prpr[victim] <= prpr_i;
//                dcc_iohgatp_mode[victim] <= iohgatp_mode_i;
//                dcc_gscid[victim] <= gscid_i;
//                dcc_iohgatp_ppn[victim] <= iohgatp_ppn_i;
//                dcc_fsc_mode[victim] <= fsc_mode_i;
//                dcc_fsc_ppn[victim] <= fsc_ppn_i;
//                dcc_dc_pscid[victim] <= dc_pscid_i;
//                dcc_msiptp_mode[victim] <= msiptp_mode_i;
//                dcc_msiptp_ppn[victim] <= msiptp_ppn_i;
//                dcc_msi_addr_mask[victim] <= msi_addr_mask_i;
//                dcc_msi_addr_pat[victim] <= msi_addr_pat_i;
//                valid[victim] <= 1;
//                ddtc_lkup_fill_done <= 1;
//            end
//            // Lookup logic - invoked if no fill or flush being done
//            if ( ddtc_lookup_i && !ddtc_fill_i && !ddtc_flush_i && ddtc_lkup_fill_done == 0) begin
//                for ( i = 0; i < 4; i++) begin
//                    if ( (device_id_tag[i[1:0]] == device_id_i) && valid[i[1:0]] ) begin
//                        hit_row <= i[1:0];
//                        ddtc_hit <= 1;
//                    end
//                end
//                ddtc_lkup_fill_done <= 1;
//            end
//            if ( ddtc_lkup_fill_done == 1 && ddtc_lookup_i == 0 && ddtc_fill_i == 0) begin
//                ddtc_lkup_fill_done <= 0;
//                ddtc_hit <= 0;
//            end
//        end
//    end
endmodule
