// iommu.svh
// Top level headed file  for a RISC-V IOMMU
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
`ifndef _iommu_svh_
`define _iommu_svh_
typedef struct packed {
    logic [11:0] paddr;
    logic        pwrite;
    logic        psel;
    logic        penable;
} mmio_ctrl_t;

typedef struct packed {
    logic [51:0] iova;
    logic [23:0] device_id;
    logic [19:0] process_id;
    logic [1:0]  addr_type;
    logic        read_write;
    logic        pid_valid;
    logic        no_write;
    logic        exec_req;
    logic        priv_req;
    logic        tee_req;
    logic [7:0]  tag;
} pw_req_t;

typedef struct packed {
    logic [2:0]  status;
    logic [33:0] resp_pa;
    logic [7:0]  tag;
    logic        size;
    logic        no_snoop;
    logic        cxl_io;
    logic        g;
    logic        priv;
    logic        exe;
    logic        u;
    logic        r;
    logic        w;
} pw_rsp_t;

typedef struct packed {
    logic        en_ats;
    logic        en_pri;
    logic        t2gpa;
    logic        dtf;
    logic        pdtv;
    logic        prpr;
    logic [15:0] gscid;
    logic        iohgatp_mode;
    logic [34:0] iohgatp_ppn;
    logic [19:0] dc_pscid;
    logic [3:0]  dc_fsc_mode;
    logic [34:0] dc_fsc_ppn;
    logic        msiptp_mode;
    logic [34:0] msiptp_ppn;
    logic [51:0] msi_addr_mask;
    logic [51:0] msi_addr_pat;
} dc_t;
typedef struct packed {
    logic        ens;
    logic        sum;
    logic [19:0] pc_pscid;
    logic [3:0]  pc_fsc_mode;
    logic [34:0] pc_fsc_ppn;
} pc_t;
typedef struct packed {
    logic [4:0]  next_state;
    logic [2:0]  sub_state;
    logic        dcvalid;
    logic        en_ats;
    logic        en_pri;
    logic        t2gpa;
    logic        dtf;
    logic        pdtv;
    logic        prpr;
    logic [15:0] gscid;
    logic        iohgatp_is_bare;
    logic [34:0] iohgatp_ppn;
    logic [19:0] dc_pscid;
    logic [3:0]  dc_fsc_mode;
    logic [34:0] dc_fsc_ppn;
    logic        msiptp_is_bare;
    logic [34:0] msiptp_ppn;
    logic [37:0] msi_addr_mask;
    logic [37:0] msi_addr_mask;
    logic        pcvalid;
    logic        ens;
    logic        sum;
    logic [19:0] pc_pscid;
    logic [3:0]  pc_fsc_mode;
    logic [34:0] pc_fsc_ppn;
    logic [11:0] cause;
    logic [63:0] iotval2;
} pw_trk_t;
`endif
