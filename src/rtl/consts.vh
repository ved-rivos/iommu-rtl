// RISC-V IOMMU constants
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
//`ifndef _PARAMS_VH_
//`define _PARAMS_VH_

localparam MAX_PA  = 46;
// MMIO register address
localparam CAPABILITIES_ADDR   = 0;
localparam CAPABILITIES_H_ADDR = 4;
localparam FCTRL_ADDR          = 8;
localparam DDTP_ADDR           = 16;

// MMIO register offsets
localparam CAPABILITIES   = CAPABILITIES_ADDR / 4;
localparam CAPABILITIES_H = CAPABILITIES_H_ADDR / 4;
localparam FCTRL          = FCTRL_ADDR / 4;
localparam DDTP           = DDTP_ADDR / 4;

// DDTP defines
localparam MODE_SB  = 63;
localparam MODE_EB  = 60;
localparam BUSY_BIT = 59;
localparam RSVD_SB  = 58;
localparam RSVD_EB  = MAX_PA;
localparam IOMMU_OFF = 0;
localparam DDT_BARE = 1;
localparam DDT_ONE_LEVEL   = 2;
localparam DDT_TWO_LEVEL   = 3;
localparam DDT_THREE_LEVEL = 4;

// CAPABILITIES register params
localparam IOMMU_VERSION = 1;
localparam CAP_SV32      = (0 << 8);
localparam CAP_SV39      = (1 << 9);
localparam CAP_SV48      = (1 << 10);
localparam CAP_SV57      = (0 << 11);
localparam CAP_SVNAPOT   = (0 << 14);
localparam CAP_SVPBMT    = (0 << 15);
localparam CAP_SV32x4    = (0 << 16);
localparam CAP_SV39x4    = (1 << 17);
localparam CAP_SV48x4    = (1 << 18);
localparam CAP_SV57x4    = (0 << 19);
localparam MSI_FLAT      = (1 << 22);
localparam MSI_MRIF      = (1 << 23);
localparam AMO           = (1 << 24);
localparam ATS           = (1 << 25);
localparam T2GPA         = (1 << 26);
localparam END           = (0 << 27);
localparam IGS           = (0 << 28);
localparam PMON          = (1 << 30);
localparam PAS           = (46 << 32);
localparam IOMMU_CAPS    = IOMMU_VERSION | CAP_SV32 |
                           CAP_SV39 | CAP_SV39 | CAP_SV48 | CAP_SV57 |
                           CAP_SVNAPOT | CAP_SVPBMT |
                           CAP_SV39x4 | CAP_SV39x4 | CAP_SV48x4 | CAP_SV57x4 |
                           MSI_FLAT | MSI_MRIF | AMO | ATS |
                           T2GPA | END | IGS | PMON | PAS;
localparam IOMMU_CAPS_H  = (IOMMU_CAPS >> 32);


localparam MAX_PW = 2;

localparam PICK                     = 0;
localparam WALK_SM_DDTC_PDTC_LOOKUP = 1;
localparam WALK_SM_DDT_WALK         = 2;
localparam WALK_SM_CACHE_DDTE       = 3;
localparam WALK_SM_PDT_WALK         = 4;
localparam WALK_SM_DDT_PDT_DONE     = 5;

localparam WALK_SM_FAULT_REPORT     = 16;


localparam DDT_CHECK_L0             = 0;
localparam DDT_LOAD_L0              = 1;
localparam DDT_CHECK_L1             = 2;
localparam DDT_LOAD_L1              = 3;
localparam DDT_CHECK_L2             = 4;
localparam DDT_LOAD_L2              = 5;

// Fault causes
localparam ALL_IB_TRANS_DISALLOWED = 256;
localparam DDT_LOAD_ACC_FAULT = 257;
localparam DDT_ENTRY_NOT_VALID = 258;
localparam DDT_ENTRY_MISCONFIGURED = 259;
localparam TRANS_TYPE_DISALLOWED = 260;
localparam DDT_DATA_CORRUPTION = 268;

// Load bus opcodes
localparam LOAD = 1;
localparam STORE = 2;

// DC
localparam DC_RSVD_BIT_MAP        = 0; // FIXME
localparam IOHGATP_BARE = 0;
localparam IOHGATP_SV48X4 = 9;
localparam SATP_BARE = 0;
localparam SATP_SV48 = 9;
localparam MSIPTP_BARE = 0;
localparam MSIPTP_FLAT = 1;
localparam PDTP_BARE = 0;
localparam PDTP_PD20 = 1;
localparam PDTP_PD17 = 2;
localparam PDTP_PD8  = 3;


// Address type
localparam ADDR_TYPE_UNTRANSLATED = 0;
localparam ADDR_TYPE_TRANSLATED   = 1;
localparam ADDR_TYPE_TRANS_REQ    = 2;
localparam ADDR_TYPE_PAGE_REQUEST = 3;

//`endif //_PARAMS_VH_
