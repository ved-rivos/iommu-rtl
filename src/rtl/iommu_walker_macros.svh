// iommu_mmio.vh
// IOMMU table walker macros
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
`define report_fault(__CAUSE__)\
    pwt_cause[next_ready] <= __CAUSE__;\
    pw_state <= WALK_SM_FAULT_REPORT;

`define store_pdtc_info_in_pwt(void)\
    pwt_pc_ens[next_ready] <= ens_i;\
    pwt_pc_sum[next_ready] <= sum_i;\
    pwt_pc_pscid[next_ready] <= pc_pscid_i;\
    pwt_pc_fsc_mode[next_ready] <= pc_fsc_mode_i;\
    pwt_pc_fsc_ppn[next_ready] <= pc_fsc_ppn_i;\
    pwt_pcvalid[next_ready] <= pdtc_hit_i & pdtc_lookup_o;

`define store_pdtc_info_in_pwt(void)\
    pwt_pc_ens[next_ready] <= ens_i;\
    pwt_pc_sum[next_ready] <= sum_i;\
    pwt_pc_pscid[next_ready] <= pc_pscid_i;\
    pwt_pc_fsc_mode[next_ready] <= pc_fsc_mode_i;\
    pwt_pc_fsc_ppn[next_ready] <= pc_fsc_ppn_i;\
    pwt_pcvalid[next_ready] <= pdtc_hit_i & pdtc_lookup_o;

`define store_ddtc_info_in_pwt(void)\
    pwt_en_ats[next_ready] <= en_ats_i;\
    pwt_en_pri[next_ready] <= en_pri_i;\
    pwt_t2gpa[next_ready] <= t2gpa_i;\
    pwt_dtf[next_ready] <= dtf_i;\
    pwt_pdtv[next_ready] <= pdtv_i;\
    pwt_prpr[next_ready] <= prpr_i;\
    pwt_iohgatp_mode[next_ready] <= iohgatp_mode_i;\
    pwt_gscid[next_ready] <= gscid_i;\
    pwt_iohgatp_ppn[next_ready] <= iohgatp_ppn_i;\
    pwt_fsc_mode[next_ready] <= fsc_mode_i;\
    pwt_fsc_ppn[next_ready] <= fsc_ppn_i;\
    pwt_dc_pscid[next_ready] <= dc_pscid_i;\
    pwt_msiptp_mode[next_ready] <= msiptp_mode_i;\
    pwt_msiptp_ppn[next_ready] <= msiptp_ppn_i;\
    pwt_msi_addr_mask[next_ready] <= msi_addr_mask_i;\
    pwt_msi_addr_pat[next_ready] <= msi_addr_pat_i;

`define store_ddt_entry_in_pwt(void)\
    pwt_en_ats[next_ready] <= pwt_ld_data[next_ready][1];\
    pwt_en_pri[next_ready] <= pwt_ld_data[next_ready][2];\
    pwt_t2gpa[next_ready] <= pwt_ld_data[next_ready][3];\
    pwt_dtf[next_ready] <= pwt_ld_data[next_ready][4];\
    pwt_pdtv[next_ready] <= pwt_ld_data[next_ready][5];\
    pwt_prpr[next_ready] <= pwt_ld_data[next_ready][6];\
    pwt_iohgatp_mode[next_ready] <= pwt_ld_data[next_ready][127:124];\
    pwt_gscid[next_ready] <= pwt_ld_data[next_ready][123:108];\
    pwt_iohgatp_ppn[next_ready] <= pwt_ld_data[next_ready][97:64];\
    pwt_fsc_mode[next_ready] <= pwt_ld_data[next_ready][191:188];\
    pwt_fsc_ppn[next_ready] <= pwt_ld_data[next_ready][161:128];\
    pwt_dc_pscid[next_ready] <= pwt_ld_data[next_ready][255:236];\
    pwt_msiptp_mode[next_ready] <= pwt_ld_data[next_ready][319:316];\
    pwt_msiptp_ppn[next_ready] <= pwt_ld_data[next_ready][289:256];   \
    pwt_msi_addr_mask[next_ready] <= pwt_ld_data[next_ready][371:320];\
    pwt_msi_addr_pat[next_ready] <= pwt_ld_data[next_ready][435:384];

`define is_ddt_entry_malformed(void)\
    ( |(pwt_ld_data[next_ready] & DC_RSVD_BIT_MAP) ||\
      (pwt_iohgatp_mode[next_ready] != IOHGATP_BARE &&\
       pwt_iohgatp_mode[next_ready] != IOHGATP_SV48X4 ) ||\
      ((pwt_pdtv[next_ready] == 0) &&\
       (pwt_fsc_mode[next_ready] != SATP_BARE &&\
        pwt_fsc_mode[next_ready] != SATP_SV48)) ||\
      ((pwt_pdtv[next_ready] == 1) &&\
       (pwt_fsc_mode[next_ready] > PDTP_PD8)) ||\
      (pwt_msiptp_mode[next_ready] > MSIPTP_FLAT) )
