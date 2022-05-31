// iommu.v
// Top level wrapper for a RISC-V IOMMU
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
module iommu 
    (
        input wire clk,
        input wire rst_n,

        input wire mmio_ctrl_t mmio_ctl,
        input wire [63:0] mmio_pwdata,
        output wire [63:0] mmio_prdata,

        // Address Translation Request bus
        input  wire pw_req_t pwreq,
        input  wire pgwkr_irdy,
        output wire pgwkr_trdy,

        // Address translation Completion bus
        output wire pw_rsp_t pwrsp,
        output wire pgwkc_irdy,
        input  wire pgwkc_trdy
    );
    `include "consts.vh"

    wire ddtp_pgwk_idle, ddtp_pgwk_stall_req;
    wire [3:0] ddtp_iommu_mode;
    wire [33:0] ddtp_ppn;
    wire ddtc_lookup, ddtc_fill, ddtc_lkup_fill_done;
    wire [23:0] device_id;
    wire ddtc_hit;

    wire        l_en_ats;
    wire        l_en_pri;
    wire        l_t2gpa;
    wire        l_dtf;
    wire        l_pdtv;
    wire        l_prpr;
    wire [3:0]  l_iohgatp_mode;
    wire [15:0] l_gscid;
    wire [33:0] l_iohgatp_ppn;
    wire [3:0]  l_fsc_mode;
    wire [33:0] l_fsc_ppn;
    wire [19:0] l_dc_pscid;
    wire [3:0]  l_msiptp_mode;
    wire [43:0] l_msiptp_ppn;
    wire [51:0] l_msi_addr_mask;
    wire [51:0] l_msi_addr_pat;

    wire        f_en_ats;
    wire        f_en_pri;
    wire        f_t2gpa;
    wire        f_dtf;
    wire        f_pdtv;
    wire        f_prpr;
    wire [3:0]  f_iohgatp_mode;
    wire [15:0] f_gscid;
    wire [33:0] f_iohgatp_ppn;
    wire [3:0]  f_fsc_mode;
    wire [33:0] f_fsc_ppn;
    wire [19:0] f_dc_pscid;
    wire [3:0]  f_msiptp_mode;
    wire [43:0] f_msiptp_ppn;
    wire [51:0] f_msi_addr_mask;
    wire [51:0] f_msi_addr_pat;

    wire [23:0] ddtc_flush_device_id;
    wire ddtc_flush;
    wire ddtc_flush_device_id_valid;
    wire ddtc_flush_done;

    wire pdtc_lookup, pdtc_fill, pdtc_lkup_fill_done; 
    wire [19:0] process_id;
    wire pdtc_hit; 
    wire pc_t pc_from_pdtc; 
    wire pc_t pc_to_pdtc;
    wire [23:0] pdtc_flush_device_id;
    wire [19:0] pdtc_flush_process_id;
    wire pdtc_flush;
    wire pdtc_flush_done;

    // Page walker Load/Store request port to arbiter
    wire [45:0]       ls_addr_w;
    wire [1:0]        ls_op_w;
    wire [MAX_PW-1:0] ls_tag_w;
    wire [6:0]        ls_size_w;
    wire              ls_req_irdy_w;
    wire              ls_req_trdy_w;

    // Page walker Load/AMO data return port from arbiter
    wire [511:0]      ld_data_w;
    wire              ld_acc_fault_w;
    wire              ld_poison_w;
    wire [MAX_PW-1:0] ld_tag_w;
    wire              ld_data_irdy_w;
    wire              ld_data_trdy_w;

    rv_iommu_mmio mmio(
        .clk(clk),
        .rst_n(rst_n),
        .mmio_ctl(mmio_ctl),
        .pwdata(mmio_pwdata),
        .prdata(mmio_prdata),
        .ddtp_pgwk_stall_req_o(ddtp_pgwk_stall_req),
        .ddtp_pgwk_idle_i(ddtp_pgwk_idle),

        .ddtp_iommu_mode_o(ddtp_iommu_mode),
        .ddtp_ppn_o(ddtp_ppn)
    );

    rv_iommu_walker walker(
        .clk(clk),
        .rst_n(rst_n),

        // translation request interface
        .atr_pwreq(pwreq),
        .atr_irdy(pgwkr_irdy),
        .atr_trdy(pgwkr_trdy),


        // translation response interface
        .atc_pwrsp(pwrsp), 
        .atc_irdy(pgwkc_irdy),
        .atc_trdy(pgwkc_trdy),

        // Control signals - input
        .ddtp_pgwk_stall_req_i(ddtp_pgwk_stall_req),

        // Control signals - output
        .ddtp_pgwk_idle_o(ddtp_pgwk_idle),

        .ddtp_iommu_mode_i(ddtp_iommu_mode),
        .ddtp_ppn_i(ddtp_ppn),

        // DDTC port control signals
        .ddtc_lookup_o(ddtc_lookup),
        .ddtc_fill_o(ddtc_fill),
        .ddtc_lkup_fill_done_i(ddtc_lkup_fill_done),
        .device_id_o(device_id),

        // DDTC lookup result
        .ddtc_hit_i(ddtc_hit),
        .en_ats_i(l_en_ats),
        .en_pri_i(l_en_pri),
        .t2gpa_i(l_t2gpa),
        .dtf_i(l_dtf),
        .pdtv_i(l_pdtv),
        .prpr_i(l_prpr),
        .iohgatp_mode_i(l_iohgatp_mode),
        .gscid_i(l_gscid),
        .iohgatp_ppn_i(l_iohgatp_ppn),
        .fsc_mode_i(l_fsc_mode),
        .fsc_ppn_i(l_fsc_ppn),
        .dc_pscid_i(l_dc_pscid),
        .msiptp_mode_i(l_msiptp_mode),
        .msiptp_ppn_i(l_msiptp_ppn),
        .msi_addr_mask_i(l_msi_addr_mask),
        .msi_addr_pat_i(l_msi_addr_pat),
        // DDTC fill data
        .en_ats_o(f_en_ats),
        .en_pri_o(f_en_pri),
        .t2gpa_o(f_t2gpa),
        .dtf_o(f_dtf),
        .pdtv_o(f_pdtv),
        .prpr_o(f_prpr),
        .iohgatp_mode_o(f_iohgatp_mode),
        .gscid_o(f_gscid),
        .iohgatp_ppn_o(f_iohgatp_ppn),
        .fsc_mode_o(f_fsc_mode),
        .fsc_ppn_o(f_fsc_ppn),
        .dc_pscid_o(f_dc_pscid),
        .msiptp_mode_o(f_msiptp_mode),
        .msiptp_ppn_o(f_msiptp_ppn),
        .msi_addr_mask_o(f_msi_addr_mask),
        .msi_addr_pat_o(f_msi_addr_pat),

        // PDTC port control signals
        .pdtc_lookup_o(pdtc_lookup),
        .pdtc_fill_o(pdtc_fill),
        .pdtc_lkup_fill_done_i(pdtc_lkup_fill_done),
        .process_id_o(process_id),

        // PDTC lookup result
        .pdtc_hit_i(pdtc_hit),
        .pc_i(pc_from_pdtc),

        // PDTC fill data
        .pc_o(pc_to_pdtc),

        // Load/store unit port
        .ls_addr_o(ls_addr_w),
        .ls_op_o(ls_op_w),
        .ls_tag_o(ls_tag_w),
        .ls_size_o(ls_size_w),
        .ls_req_irdy_o(ls_req_irdy_w),
        .ls_req_trdy_i(ls_req_trdy_w),

        // Load/AMO data return port
        .ld_data_i(ld_data_w),
        .ld_acc_fault_i(ld_acc_fault_w),
        .ld_poison_i(ld_poison_w),
        .ld_tag_i(ld_tag_w),
        .ld_data_irdy_i(ld_data_irdy_w),
        .ld_data_trdy_o(ld_data_trdy_w)
    );

    rv_iommu_ddtc ddtc(
        .clk(clk),
        .rst_n(rst_n),

        // DDTC port control signals
        .ddtc_lookup_i(ddtc_lookup),
        .ddtc_fill_i(ddtc_fill),
        .ddtc_lkup_fill_done_o(ddtc_lkup_fill_done),
        .device_id_i(device_id),

        // DDTC lookup result
        .ddtc_hit_o(ddtc_hit),
        .en_ats_o(l_en_ats),
        .en_pri_o(l_en_pri),
        .t2gpa_o(l_t2gpa),
        .dtf_o(l_dtf),
        .pdtv_o(l_pdtv),
        .prpr_o(l_prpr),
        .iohgatp_mode_o(l_iohgatp_mode),
        .gscid_o(l_gscid),
        .iohgatp_ppn_o(l_iohgatp_ppn),
        .fsc_mode_o(l_fsc_mode),
        .fsc_ppn_o(l_fsc_ppn),
        .dc_pscid_o(l_dc_pscid),
        .msiptp_mode_o(l_msiptp_mode),
        .msiptp_ppn_o(l_msiptp_ppn),
        .msi_addr_mask_o(l_msi_addr_mask),
        .msi_addr_pat_o(l_msi_addr_pat),

        // DDTC fill data
        .en_ats_i(f_en_ats),
        .en_pri_i(f_en_pri),
        .t2gpa_i(f_t2gpa),
        .dtf_i(f_dtf),
        .pdtv_i(f_pdtv),
        .prpr_i(f_prpr),
        .iohgatp_mode_i(f_iohgatp_mode),
        .gscid_i(f_gscid),
        .iohgatp_ppn_i(f_iohgatp_ppn),
        .fsc_mode_i(f_fsc_mode),
        .fsc_ppn_i(f_fsc_ppn),
        .dc_pscid_i(f_dc_pscid),
        .msiptp_mode_i(f_msiptp_mode),
        .msiptp_ppn_i(f_msiptp_ppn),
        .msi_addr_mask_i(f_msi_addr_mask),
        .msi_addr_pat_i(f_msi_addr_pat),

        // DDTC flush interface
        .flush_device_id_i(ddtc_flush_device_id),
        .ddtc_flush_i(1'b0),
        .flush_device_id_valid_i(ddtc_flush_device_id_valid),
        .ddtc_flush_done_o(ddtc_flush_done)
    );
    rv_iommu_pdtc pdtc(
        .clk(clk),
        .rst_n(rst_n),

        // PDTC port control signals
        .pdtc_lookup_i(pdtc_lookup),
        .pdtc_fill_i(pdtc_fill),
        .pdtc_lkup_fill_done_o(pdtc_lkup_fill_done),
        .device_id_i(device_id),
        .process_id_i(process_id),

        // PDTC lookup result
        .pdtc_hit_o(pdtc_hit),
        .pc_o(pc_from_pdtc),
        // PDTC fill data
        .pc_i(pc_to_pdtc),

        // PDTC flush interface
        .flush_device_id_i(pdtc_flush_device_id),
        .flush_process_id_i(pdtc_flush_process_id),
        .pdtc_flush_i(1'b0),
        .pdtc_flush_done_o(pdtc_flush_done)
    );
    rv_iommu_lspa lspa (
        .clk(clk),
        .rst_n(rst_n),
        // Load/store from walker, CQ, FQ, PQ
        .w_ls_addr_i(ls_addr_w),
        .w_ls_op_i(ls_op_w),
        .w_ls_tag_i(ls_tag_w),
        .w_ls_size_i(ls_size_w),
        .w_ls_req_irdy_i(ls_req_irdy_w),
        .w_ls_req_trdy_o(ls_req_trdy_w),

        // Load/AMO data return to walker
        .w_ld_data_o(ld_data_w),
        .w_ld_acc_fault_o(ld_acc_fault_w),
        .w_ld_poison_o(ld_poison_w),
        .w_ld_tag_o(ld_tag_w),
        .w_ld_data_irdy_o(ld_data_irdy_w),
        .w_ld_data_trdy_i(ld_data_trdy_w)
    );
endmodule
