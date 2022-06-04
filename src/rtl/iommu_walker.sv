// iommu_mmio.v
// IOMMU table walker
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
module rv_iommu_walker
        (
        input wire      clk,
        input wire      rst_n,

        // translation request interface
        input wire        atr_irdy_i,
        input wire [51:0] atr_iova_i,
        input wire [23:0] atr_device_id_i,
        input wire [19:0] atr_process_id_i,
        input wire [1:0]  atr_addr_type_i,
        input wire        atr_read_write_i,
        input wire        atr_pid_valid_i,
        input wire        atr_no_write_i,
        input wire        atr_exec_req_i,
        input wire        atr_priv_req_i,
        input wire        atr_tee_req_i,
        input wire [7:0]  atr_tag_i,
        output wire       atr_trdy_o,


        // translation response interface
        output wire        atc_irdy_o,
        output wire [2:0]  atc_status_o,
        output wire [33:0] atc_resp_pa_o,
        output wire [7:0]  atc_tag_o,
        output wire        atc_size_o,
        output wire        atc_no_snoop_o,
        output wire        atc_cxl_io_o,
        output wire        atc_g_o,
        output wire        atc_priv_o,
        output wire        atc_exe_o,
        output wire        atc_u_o,
        output wire        atc_r_o,
        output wire        atc_w_o,
        input  wire        atc_trdy_i,

        input  wire [3:0]  ddtp_iommu_mode_i,
        input  wire [33:0] ddtp_ppn_i,

        // Control signals
        input wire      ddtp_pgwk_stall_req_i,
        output wire     ddtp_pgwk_idle_o,

        // DDTC port control signals
        output wire        ddtc_lookup_o,
        output wire        ddtc_fill_o,
        input  wire        ddtc_lkup_fill_done_i,
        output wire [23:0] device_id_o,

        // DDTC lookup result
        input  wire        ddtc_hit_i,
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
        // DDTC fill data
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

        // PDTC port control signals
        output wire        pdtc_lookup_o,
        output wire        pdtc_fill_o,
        input  wire        pdtc_lkup_fill_done_i,
        output wire [19:0] process_id_o,

        // PDTC lookup result
        input  wire        pdtc_hit_i,
        input  wire ens_i,
        input  wire sum_i,
        input  wire [19:0] pc_pscid_i,
        input  wire [3:0]  pc_fsc_mode_i,
        input  wire [33:0] pc_fsc_ppn_i,

        // PDTC fill data
        output  wire ens_o,
        output  wire sum_o,
        output  wire [19:0] pc_pscid_o,
        output  wire [3:0]  pc_fsc_mode_o,
        output  wire [33:0] pc_fsc_ppn_o,

        // Load/store unit port
        output wire [45:0]       ls_addr_o,
        output wire [1:0]        ls_op_o,
        output wire [$clog2(MAX_PW)-1:0] ls_tag_o,
        output wire [6:0]        ls_size_o,
        output wire              ls_req_irdy_o,
        input  wire              ls_req_trdy_i,

        // Load/AMO data return port
        input  wire [511:0]      ld_data_i,
        input  wire              ld_acc_fault_i,
        input  wire              ld_poison_i,
        output wire [$clog2(MAX_PW)-1:0] ld_tag_i,
        input  wire              ld_data_irdy_i,
        output wire              ld_data_trdy_o
    );
    `include "consts.vh"

    // Page walk tracker

    // Input request in page walk tracker
    reg [51:0]  pwt_iova[MAX_PW];
    reg [23:0]  pwt_device_id[MAX_PW];
    reg [19:0]  pwt_process_id[MAX_PW];
    reg [1:0]   pwt_addr_type[MAX_PW];
    reg         pwt_read_write[MAX_PW];
    reg         pwt_pid_valid[MAX_PW];
    reg         pwt_no_write[MAX_PW];
    reg         pwt_exec_req[MAX_PW];
    reg         pwt_priv_req[MAX_PW];
    reg         pwt_tee_req[MAX_PW];
    reg [7:0]   pwt_req_tag[MAX_PW];

    // Output response from page walk tracker
    reg [2:0]   pwt_status[MAX_PW];
    reg [33:0]  pwt_resp_pa[MAX_PW];
    reg [7:0]   pwt_rsp_tag[MAX_PW];
    reg         pwt_size[MAX_PW];
    reg         pwt_no_snoop[MAX_PW];
    reg         pwt_cxl_io[MAX_PW];
    reg         pwt_g[MAX_PW];
    reg         pwt_priv[MAX_PW];
    reg         pwt_exe[MAX_PW];
    reg         pwt_u[MAX_PW];
    reg         pwt_r[MAX_PW];
    reg         pwt_w[MAX_PW];

    // Page walk tracker state
    reg [4:0]   pw_state;
    reg [4:0]   pwt_next_state[MAX_PW];
    reg [4:0]   pwt_sub_state[MAX_PW];
    reg         pwt_pcvalid[MAX_PW];
    reg         pwt_pc_ens[MAX_PW];
    reg         pwt_pc_sum[MAX_PW];
    reg [19:0]  pwt_pc_pscid[MAX_PW];
    reg [3:0]   pwt_pc_fsc_mode[MAX_PW];
    reg [33:0]  pwt_pc_fsc_ppn[MAX_PW];
    reg [11:0]  pwt_cause[MAX_PW];
    reg [63:0]  pwt_iotval2[MAX_PW];
    reg [511:0] pwt_ld_data[MAX_PW];
    reg         pwt_acc_fault[MAX_PW];
    reg         pwt_poison[MAX_PW];
    reg         pwt_en_ats[MAX_PW];
    reg         pwt_en_pri[MAX_PW];
    reg         pwt_t2gpa[MAX_PW];
    reg         pwt_dtf[MAX_PW];
    reg         pwt_pdtv[MAX_PW];
    reg         pwt_prpr[MAX_PW];
    reg [3:0]   pwt_iohgatp_mode[MAX_PW];
    reg [15:0]  pwt_gscid[MAX_PW];
    reg [33:0]  pwt_iohgatp_ppn[MAX_PW];
    reg [3:0]   pwt_fsc_mode[MAX_PW];
    reg [33:0]  pwt_fsc_ppn[MAX_PW];
    reg [19:0]  pwt_dc_pscid[MAX_PW];
    reg [3:0]   pwt_msiptp_mode[MAX_PW];
    reg [43:0]  pwt_msiptp_ppn[MAX_PW];
    reg [51:0]  pwt_msi_addr_mask[MAX_PW];
    reg [51:0]  pwt_msi_addr_pat[MAX_PW];


    // page walk tracker allocator controls
    reg [MAX_PW-1 : 0]     pwt_free_bitmap;
    reg [MAX_PW-1 : 0]     pwt_ready_bitmap;
    reg [MAX_PW-1 : 0]     pwt_done_bitmap;
    reg [$clog2(MAX_PW)-1:0] next_free;
    reg [$clog2(MAX_PW)-1:0] next_ready;
    reg [$clog2(MAX_PW)-1:0] next_done;
    reg [$clog2(MAX_PW)-1:0] pgwk_in_flight;
    wire                   is_any_tracker_free;
    wire                   is_any_tracker_ready;
    wire                   is_any_tracker_done;
    assign                 is_any_tracker_free = |pwt_free_bitmap;
    assign                 is_any_tracker_done = |pwt_done_bitmap;
    assign                 is_any_tracker_ready = |pwt_ready_bitmap;
    assign                 ddtp_pgwk_idle_o = (pgwk_in_flight == 0) ? 1 : 0;

    localparam IDLE = 0;
    localparam REQ_READY = 1;
    localparam RSP_READY = 2;

    // Request/response bus controls
    reg [1:0] req_rsp_st;
    assign    atr_trdy_o = (req_rsp_st == REQ_READY) ? 1 : 0;
    assign    atc_irdy_o = (req_rsp_st == RSP_READY) ? 1 : 0;
    assign    atc_status_o = pwt_status[next_ready];
    assign    atc_resp_pa_o = pwt_resp_pa[next_ready];
    assign    atc_tag_o = pwt_rsp_tag[next_ready];
    assign    atc_size_o = pwt_size[next_ready];
    assign    atc_no_snoop_o = pwt_no_snoop[next_ready];
    assign    atc_cxl_io_o = pwt_cxl_io[next_ready];
    assign    atc_g_o = pwt_g[next_ready];
    assign    atc_priv_o = pwt_priv[next_ready];
    assign    atc_exe_o = pwt_exe[next_ready];
    assign    atc_u_o = pwt_u[next_ready];
    assign    atc_r_o = pwt_r[next_ready];
    assign    atc_w_o = pwt_w[next_ready];

    reg               ddtc_fill;
    reg               pdtc_fill;

    assign device_id_o = pwt_device_id[next_ready];
    assign process_id_o = pwt_process_id[next_ready];
    assign ddtc_lookup_o = (pw_state == WALK_SM_DDTC_PDTC_LOOKUP ) ? 1 : 0;
    assign ddtc_fill_o = ddtc_fill;
    assign en_ats_o = pwt_en_ats[next_ready];
    assign en_pri_o = pwt_en_pri[next_ready];
    assign t2gpa_o = pwt_t2gpa[next_ready];
    assign dtf_o = pwt_dtf[next_ready];
    assign pdtv_o = pwt_pdtv[next_ready];
    assign prpr_o = pwt_prpr[next_ready];
    assign iohgatp_mode_o = pwt_iohgatp_mode[next_ready];
    assign gscid_o = pwt_gscid[next_ready];
    assign iohgatp_ppn_o = pwt_iohgatp_ppn[next_ready];
    assign fsc_mode_o = pwt_fsc_mode[next_ready];
    assign fsc_ppn_o = pwt_fsc_ppn[next_ready];
    assign dc_pscid_o = pwt_dc_pscid[next_ready];
    assign msiptp_mode_o = pwt_msiptp_mode[next_ready];
    assign msiptp_ppn_o = pwt_msiptp_ppn[next_ready];
    assign msi_addr_mask_o = pwt_msi_addr_mask[next_ready];
    assign msi_addr_pat_o = pwt_msi_addr_pat[next_ready];

    assign            pdtc_lookup_o = (ddtc_lookup_o && pwt_pid_valid[next_ready]);
    assign            pdtc_fill_o = pdtc_fill;

    reg [45:0]        ls_addr;
    reg [1:0]         ls_op;
    reg [6:0]         ls_size;
    reg               ls_req_irdy;

    assign            ls_addr_o = ls_addr;
    assign            ls_op_o = ls_op;
    assign            ls_tag_o = next_ready;
    assign            ls_size_o = ls_size;
    assign            ls_req_irdy_o = ls_req_irdy;


    reg               ld_data_trdy;
    assign            ld_data_trdy_o = ld_data_trdy;


    // Task to receive page walk requests, allocate a page walk
    // tracker entry and place the request into the page walk tracker
    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            integer i;
            for ( i = 0; i < MAX_PW; i = i + 1) begin
                pwt_free_bitmap[i] <= 1;
                pwt_done_bitmap[i] <= 0;
                pwt_ready_bitmap[i] <= 0;
            end
            req_rsp_st <= IDLE;
            pgwk_in_flight <= 0;
            next_free <= 0;
            next_done <= 0;
            ddtc_fill <= 0;
            pdtc_fill <= 0;
            ld_data_trdy <= 0;
            ls_req_irdy <= 0;
        end
        else begin
            case (req_rsp_st)
                IDLE: begin
                    if ( is_any_tracker_done ) begin
                        while ( pwt_done_bitmap[next_done] == 0 ) begin
                            next_done <= next_done + 1;
                            next_done <= next_done & ($clog2(MAX_PW)-1);
                        end
                        req_rsp_st <= RSP_READY;
                    end else 
                    if ( atr_irdy_i && is_any_tracker_free && ddtp_pgwk_stall_req_i == 0 ) begin
                        while ( pwt_free_bitmap[next_free] == 0 ) begin
                            next_free <= next_free + 1;
                            next_free <= next_free & (MAX_PW - 1);
                        end
                        req_rsp_st <= REQ_READY;
                    end
                end
                REQ_READY: begin
                    // Initialize the page walk tracker
                    pwt_free_bitmap[next_free] <= 0;
                    pwt_iova[next_free] <= atr_iova_i;
                    pwt_device_id[next_free] <= atr_device_id_i;
                    pwt_process_id[next_free] <= atr_process_id_i;
                    pwt_addr_type[next_free] <= atr_addr_type_i;
                    pwt_read_write[next_free] <= atr_read_write_i;
                    pwt_pid_valid[next_free] <= atr_pid_valid_i;
                    pwt_no_write[next_free] <= atr_no_write_i;
                    pwt_exec_req[next_free] <= atr_exec_req_i;
                    pwt_priv_req[next_free] <= atr_priv_req_i;
                    pwt_tee_req[next_free] <= atr_tee_req_i;
                    pwt_req_tag[next_free] <= atr_tag_i;
                    pwt_next_state[next_free] <= WALK_SM_DDTC_PDTC_LOOKUP;
                    pwt_sub_state[next_free] <= 0;
                    pwt_pcvalid[next_free] <= 0;
                    pwt_ready_bitmap[next_free] <= 1;
                    if ( atr_irdy_i == 0 ) begin
                        pgwk_in_flight <= pgwk_in_flight + 1;
                        req_rsp_st <= IDLE;
                    end
                end
                RSP_READY: begin
                    // wait for trdy
                    if ( atc_trdy_i ) begin
                        pwt_done_bitmap[next_done] <= 0;
                        pgwk_in_flight <= pgwk_in_flight - 1;
                        pwt_free_bitmap[next_done] <= 1;
                        req_rsp_st <= IDLE;
                    end
                end
            endcase
        end
    end
    // page walk state machine
    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            pw_state <= PICK;
            next_ready <= 0;
        end
        else begin
            case ( pw_state ) 
                PICK: begin
                    if ( is_any_tracker_ready ) begin
                        // Pick next ready request from PWT
                        while ( pwt_ready_bitmap[next_ready] == 0 ) begin
                            next_ready <= next_ready + 1;
                            next_ready <= next_ready & (MAX_PW - 1);
                        end
                        pw_state <= pwt_next_state[next_ready];
                    end
                end
                WALK_SM_DDTC_PDTC_LOOKUP: begin
                    if ( ddtp_iommu_mode_i == IOMMU_OFF ) begin
                        pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                        pwt_cause[next_ready] <= ALL_IB_TRANS_DISALLOWED;
                        pwt_iotval2[next_ready] <= 0;
                        pw_state <= WALK_SM_FAULT_REPORT;
                    end else if ( ddtp_iommu_mode_i == DDT_BARE ) begin
                        if ( pwt_addr_type[next_ready] != ADDR_TYPE_UNTRANSLATED ) begin
                            pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                            pwt_cause[next_ready] <= TRANS_TYPE_DISALLOWED;
                            pwt_iotval2[next_ready] <= 0;
                            pw_state <= WALK_SM_FAULT_REPORT;
                        end
                    end else if ( (ddtp_iommu_mode_i == DDT_TWO_LEVEL && 
                                   device_id_o[23:15] != 0) || 
                                  (ddtp_iommu_mode_i == DDT_ONE_LEVEL && 
                                   device_id_o[23:6] != 0) ) begin
                        // device ID too wide for mode
                        pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                        pwt_cause[next_ready] <= TRANS_TYPE_DISALLOWED;
                        pwt_iotval2[next_ready] <= 0;
                        pw_state <= WALK_SM_FAULT_REPORT;
                    end else begin
                        // Wait for the DDTC and PDTC (if looked up) to completed lookup
                        if ( (ddtc_lkup_fill_done_i == 1) && 
                             (pdtc_lkup_fill_done_i || ~pdtc_lookup_o) ) begin
                            pwt_pc_ens[next_ready] <= ens_i;
                            pwt_pc_sum[next_ready] <= sum_i;
                            pwt_pc_pscid[next_ready] <= pc_pscid_i;
                            pwt_pc_fsc_mode[next_ready] <= pc_fsc_mode_i;
                            pwt_pc_fsc_ppn[next_ready] <= pc_fsc_ppn_i;
                            pwt_pcvalid[next_ready] <= pdtc_hit_i & pdtc_lookup_o;
                            pwt_next_state[next_ready] <= WALK_SM_DDT_WALK;
                            if ( ddtc_hit_i == 1 ) begin 
                                pwt_en_ats[next_ready] <= en_ats_i;
                                pwt_en_pri[next_ready] <= en_pri_i;
                                pwt_t2gpa[next_ready] <= t2gpa_i;
                                pwt_dtf[next_ready] <= dtf_i;
                                pwt_pdtv[next_ready] <= pdtv_i;
                                pwt_prpr[next_ready] <= prpr_i;
                                pwt_iohgatp_mode[next_ready] <= iohgatp_mode_i;
                                pwt_gscid[next_ready] <= gscid_i;
                                pwt_iohgatp_ppn[next_ready] <= iohgatp_ppn_i;
                                pwt_fsc_mode[next_ready] <= fsc_mode_i;
                                pwt_fsc_ppn[next_ready] <= fsc_ppn_i;
                                pwt_dc_pscid[next_ready] <= dc_pscid_i;
                                pwt_msiptp_mode[next_ready] <= msiptp_mode_i;
                                pwt_msiptp_ppn[next_ready] <= msiptp_ppn_i;
                                pwt_msi_addr_mask[next_ready] <= msi_addr_mask_i;
                                pwt_msi_addr_pat[next_ready] <= msi_addr_pat_i;
                                $display("%x", msi_addr_pat_i);
                                pwt_sub_state[next_ready] <= DDT_CHECK_L0;
                            end else begin
                                if (ddtp_iommu_mode_i == DDT_ONE_LEVEL) begin
                                    ls_addr <= ((ddtp_ppn_i * 4096) | (device_id_o[5:0] * 8));
                                    ls_size <= 64;
                                    pwt_sub_state[next_ready] <= DDT_LOAD_L0;
                                end
                                if (ddtp_iommu_mode_i == DDT_TWO_LEVEL) begin
                                    ls_addr <= ((ddtp_ppn_i * 4096) | (device_id_o[14:6] * 8));
                                    ls_size <= 8;
                                    pwt_sub_state[next_ready] <= DDT_LOAD_L1;
                                end
                                if (ddtp_iommu_mode_i == DDT_THREE_LEVEL) begin
                                    ls_addr <= ((ddtp_ppn_i * 4096) | (device_id_o[23:15] * 8));
                                    ls_size <= 8;
                                    pwt_sub_state[next_ready] <= DDT_LOAD_L2;
                                end
                                ls_op <= LOAD;
                                ls_req_irdy <= 1;
                            end
                            pw_state <= WALK_SM_DDT_WALK;
                        end
                    end
                end 

                WALK_SM_DDT_WALK: begin
                    case ( pwt_sub_state[next_ready] ) 
                        DDT_LOAD_L0, DDT_LOAD_L1, DDT_LOAD_L2: begin
                            // Wait for load store unit to accept the request
                            if ( ls_req_trdy_i == 1 ) begin
                                // Suspend walk till load data returns
                                if ( pwt_sub_state[next_ready] == DDT_LOAD_L0 ) begin
                                    pwt_sub_state[next_ready] <= DDT_CHECK_L0;
                                end else if ( pwt_sub_state[next_ready] == DDT_LOAD_L1 ) begin
                                    pwt_sub_state[next_ready] <= DDT_CHECK_L1;
                                end else if ( pwt_sub_state[next_ready] == DDT_LOAD_L2 ) begin
                                    pwt_sub_state[next_ready] <= DDT_CHECK_L2;
                                end
                                pwt_ready_bitmap[next_ready] <= 0;
                                ls_req_irdy <= 0;
                                pw_state <= PICK;
                            end
                        end

                        DDT_CHECK_L1, DDT_CHECK_L2: begin
                            if ( pwt_acc_fault[next_ready] == 1 || pwt_poison[next_ready] == 1 ) begin
                                pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                pwt_cause[next_ready] <= (pwt_poison[next_ready] == 1) ?
                                                          DDT_DATA_CORRUPTION : DDT_LOAD_ACC_FAULT;
                                pwt_iotval2[next_ready] <= 0;
                                pw_state <= WALK_SM_FAULT_REPORT;
                            end else if ( pwt_ld_data[next_ready][0] == 0 ) begin
                                pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                pwt_cause[next_ready] <= DDT_ENTRY_NOT_VALID;
                                pwt_iotval2[next_ready] <= 0;
                                pw_state <= WALK_SM_FAULT_REPORT;
                            end else if ( pwt_ld_data[next_ready][11:1] != 0  ||
                                          pwt_ld_data[next_ready][63:46] != 0 ) begin
                                pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                pwt_cause[next_ready] <= DDT_ENTRY_MISCONFIGURED;
                                pwt_iotval2[next_ready] <= 0;
                                pw_state <= WALK_SM_FAULT_REPORT;
                            end else begin
                                if ( pwt_sub_state[next_ready] == DDT_CHECK_L2) begin
                                    ls_addr <= ((pwt_ld_data[next_ready][45:12] * 4096 ) | 
                                                (device_id_o[14:6] * 8));
                                    pwt_sub_state[next_ready] <= DDT_LOAD_L1;
                                    ls_size <= 8;
                                end else if ( pwt_sub_state[next_ready] == DDT_CHECK_L1) begin
                                    ls_addr <= ((pwt_ld_data[next_ready][45:12] * 4096) | 
                                                (device_id_o[5:0] * 8));
                                    pwt_sub_state[next_ready] <= DDT_LOAD_L0;
                                    ls_size <= 64;
                                end
                                ls_op <= LOAD;
                                ls_req_irdy <= 1;
                            end
                        end
                        DDT_CHECK_L0: begin
                            if ( pwt_ld_data[next_ready][0] == 0 ) begin
                                pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                pwt_cause[next_ready] <= DDT_ENTRY_NOT_VALID;
                                pwt_iotval2[next_ready] <= 0;
                                pw_state <= WALK_SM_FAULT_REPORT;
                            end else begin
                                pwt_en_ats[next_ready] <= pwt_ld_data[next_ready][1];
                                pwt_en_pri[next_ready] <= pwt_ld_data[next_ready][2];
                                pwt_t2gpa[next_ready] <= pwt_ld_data[next_ready][3];
                                pwt_dtf[next_ready] <= pwt_ld_data[next_ready][4];
                                pwt_pdtv[next_ready] <= pwt_ld_data[next_ready][5];
                                pwt_prpr[next_ready] <= pwt_ld_data[next_ready][6];
                                pwt_iohgatp_mode[next_ready] <= pwt_ld_data[next_ready][127:124];
                                pwt_gscid[next_ready] <= pwt_ld_data[next_ready][123:108];
                                pwt_iohgatp_ppn[next_ready] <= pwt_ld_data[next_ready][97:64];
                                pwt_fsc_mode[next_ready] <= pwt_ld_data[next_ready][191:188];
                                pwt_fsc_ppn[next_ready] <= pwt_ld_data[next_ready][161:128];
                                pwt_dc_pscid[next_ready] <= pwt_ld_data[next_ready][255:236];
                                pwt_msiptp_mode[next_ready] <= pwt_ld_data[next_ready][319:316];
                                pwt_msiptp_ppn[next_ready] <= pwt_ld_data[next_ready][289:256];
                                pwt_msi_addr_mask[next_ready] <= pwt_ld_data[next_ready][371:320];
                                pwt_msi_addr_pat[next_ready] <= pwt_ld_data[next_ready][435:384];
                                if ( |(pwt_ld_data[next_ready] & DC_RSVD_BIT_MAP) ||
                                     (pwt_iohgatp_mode[next_ready] != IOHGATP_BARE && 
                                      pwt_iohgatp_mode[next_ready] != IOHGATP_SV48X4 ) ||
                                     ((pwt_pdtv[next_ready] == 0) && 
                                      (pwt_fsc_mode[next_ready] != SATP_BARE && 
                                       pwt_fsc_mode[next_ready] != SATP_SV48)) ||
                                     ((pwt_pdtv[next_ready] == 1) && 
                                      (pwt_fsc_mode[next_ready] > PDTP_PD8)) ||
                                     (pwt_msiptp_mode[next_ready] > MSIPTP_FLAT) ) begin
                                    pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                    pwt_cause[next_ready] <= DDT_ENTRY_MISCONFIGURED;
                                    pwt_iotval2[next_ready] <= 0;
                                    pw_state <= WALK_SM_FAULT_REPORT;
                                end else begin
                                    pw_state <= WALK_SM_CACHE_DDTE;
                                    ddtc_fill <= 1;
                                end
                            end
                        end
                    endcase
                end
                WALK_SM_CACHE_DDTE: begin
                    if ( ddtc_lkup_fill_done_i == 1 && ddtc_fill == 1) begin
                        $display("Reached cache ddt ");
                        ddtc_fill <= 0;
                    end
                end
            endcase
        end
    end

                                //if ( (pwt_req[next_read].addr_type != ADDR_TYPE_UNTRANSLATED) &&
                                //     (pwt_en_ats[next_ready] == 0) ) begin
                                //    pwt_next_state[next_ready] <= WALK_SM_FAULT_REPORT;
                                //    pwt_cause[next_ready] <= TRANS_TYPE_DISALLOWED;
                                //    pwt_iotval2[next_ready] <= 0;
                                //    pw_state <= WALK_SM_FAULT_REPORT;
                                //end
    // Task to wait for load data completion and make 
    // waiting trackers ready
    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
        end
        else begin
            if ( ld_data_irdy_i == 1 ) begin
                ld_data_trdy <= 1;
                pwt_ld_data[ld_tag_i] <= ld_data_i;
                pwt_poison[ld_tag_i] <= ld_poison_i;
                pwt_acc_fault[ld_tag_i] <= ld_acc_fault_i;
                pwt_ready_bitmap[ld_tag_i] <= 1;
            end else begin
                ld_data_trdy <= 0;
            end
        end
    end
endmodule
