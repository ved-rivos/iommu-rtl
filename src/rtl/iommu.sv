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
module iommu 
    (
        input wire clk,
        input wire rst_n,

        input wire [11:0] paddr_i,
        input wire        pwrite_i,
        input wire        psel_i,
        input wire        penable_i,
        input wire [63:0] pwdata_i,
        output wire [63:0] prdata_o,

        // Address Translation Request bus
        input wire [51:0]  atr_iova_i,
        input wire [23:0]  atr_device_id_i,
        input wire [19:0]  atr_process_id_i,
        input wire [1:0]   atr_addr_type_i,
        input wire         atr_read_write_i,
        input wire         atr_pid_valid_i,
        input wire         atr_no_write_i,
        input wire         atr_exec_req_i,
        input wire         atr_priv_req_i,
        input wire         atr_tee_req_i,
        input wire [7:0]   atr_tag_i,
        input  wire        atr_irdy_i,
        output wire        atr_trdy_o,

        // Address translation Completion bus
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
        output wire        atc_irdy_o,
        input  wire        atc_trdy_i
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
    wire [33:0] l_msiptp_ppn;
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
    wire [33:0] f_msiptp_ppn;
    wire [51:0] f_msi_addr_mask;
    wire [51:0] f_msi_addr_pat;

    wire [23:0] ddtc_inval_device_id;
    wire ddtc_inval;
    wire ddtc_inval_device_id_valid;
    wire ddtc_inval_done;

    wire pdtc_lookup, pdtc_fill, pdtc_lkup_fill_done; 
    wire [19:0] process_id;
    wire pdtc_hit; 
    wire        l_ens;
    wire        l_sum;
    wire [19:0] l_pscid;
    wire [3:0]  l_pc_fsc_mode;
    wire [33:0] l_pc_fsc_ppn;
    wire        f_ens;
    wire        f_sum;
    wire [19:0] f_pscid;
    wire [3:0]  f_pc_fsc_mode;
    wire [33:0] f_pc_fsc_ppn;
    wire [23:0] pdtc_inval_device_id;
    wire [19:0] pdtc_inval_process_id;
    wire pdtc_inval;
    wire pdtc_inval_done;

    // Page walker Load/Store request port to arbiter
    wire [45:0]       ls_addr_w;
    wire [1:0]        ls_op_w;
    wire [$clog2(MAX_PW)-1:0] ls_tag_w;
    wire [6:0]        ls_size_w;
    wire              ls_req_irdy_w;
    wire              ls_req_trdy_w;

    // Page walker Load/AMO data return port from arbiter
    wire [511:0]      ld_data_w;
    wire              ld_acc_fault_w;
    wire              ld_poison_w;
    wire [$clog2(MAX_PW)-1:0] ld_tag_w;
    wire              ld_data_irdy_w;
    wire              ld_data_trdy_w;

    rv_iommu_mmio mmio(
        .clk(clk),
        .rst_n(rst_n),
        .paddr_i(paddr_i),
        .pwrite_i(pwrite_i),
        .psel_i(psel_i),
        .penable_i(penable_i),
        .pwdata_i(pwdata_i),
        .prdata_o(prdata_o),
        .ddtp_pgwk_stall_req_o(ddtp_pgwk_stall_req),
        .ddtp_pgwk_idle_i(ddtp_pgwk_idle),

        .ddtp_iommu_mode_o(ddtp_iommu_mode),
        .ddtp_ppn_o(ddtp_ppn)
    );

    rv_iommu_walker walker(
        .clk(clk),
        .rst_n(rst_n),

        // translation request interface
        .atr_iova_i(atr_iova_i),
        .atr_device_id_i(atr_device_id_i),
        .atr_process_id_i(atr_process_id_i),
        .atr_addr_type_i(atr_addr_type_i),
        .atr_read_write_i(atr_read_write_i),
        .atr_pid_valid_i(atr_pid_valid_i),
        .atr_no_write_i(atr_no_write_i),
        .atr_exec_req_i(atr_exec_req_i),
        .atr_priv_req_i(atr_priv_req_i),
        .atr_tee_req_i(atr_tee_req_i),
        .atr_tag_i(atr_tag_i),
        .atr_irdy_i(atr_irdy_i),
        .atr_trdy_o(atr_trdy_o),


        // translation response interface
        .atc_status_o(atc_status_o),
        .atc_resp_pa_o(atc_resp_pa_o),
        .atc_tag_o(atc_tag_o),
        .atc_size_o(atc_size_o),
        .atc_no_snoop_o(atc_no_snoop_o),
        .atc_cxl_io_o(atc_cxl_io_o),
        .atc_g_o(atc_g_o),
        .atc_priv_o(atc_priv_o),
        .atc_exe_o(atc_exe_o),
        .atc_u_o(atc_u_o),
        .atc_r_o(atc_r_o),
        .atc_w_o(atc_w_o),
        .atc_irdy_o(atc_irdy_o),
        .atc_trdy_i(atc_trdy_i),

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
        .ens_i(l_ens),
        .sum_i(l_sum),
        .pc_pscid_i(l_pscid),
        .pc_fsc_mode_i(l_pc_fsc_mode),
        .pc_fsc_ppn_i(l_pc_fsc_ppn),

        // PDTC fill data
        .ens_o(f_ens),
        .sum_o(f_sum),
        .pc_pscid_o(f_pscid),
        .pc_fsc_mode_o(f_pc_fsc_mode),
        .pc_fsc_ppn_o(f_pc_fsc_ppn),

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

        // DDTC inval interface - TODO: inval is tied off
        .inval_device_id_i(ddtc_inval_device_id),
        .ddtc_inval_i(1'b0),
        .inval_device_id_valid_i(ddtc_inval_device_id_valid),
        .ddtc_inval_done_o(ddtc_inval_done)
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
        .ens_o(l_ens),
        .sum_o(l_sum),
        .pscid_o(l_pscid),
        .fsc_mode_o(l_fsc_mode),
        .fsc_ppn_o(l_fsc_ppn),
        // PDTC fill data
        .ens_i(f_ens),
        .sum_i(f_sum),
        .pscid_i(f_pscid),
        .fsc_mode_i(f_fsc_mode),
        .fsc_ppn_i(f_fsc_ppn),

        // PDTC inval interface
        .inval_device_id_i(pdtc_inval_device_id),
        .inval_process_id_i(pdtc_inval_process_id),
        .pdtc_inval_i(1'b0),
        .pdtc_inval_done_o(pdtc_inval_done)
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
