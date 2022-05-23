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
localparam OFF         = 0;
localparam BARE        = 1;
localparam ONE_LEVEL   = 2;
localparam TWO_LEVEL   = 3;
localparam THREE_LEVEL = 4;

// CAPABILITIES register params
localparam IOMMU_VERSION = 1;
localparam CAP_SV32      = (0 << 8);
localparam CAP_SV39      = (0 << 9);
localparam CAP_SV48      = (1 << 10);
localparam CAP_SV57      = (0 << 11);
localparam CAP_SVNAPOT   = (0 << 14);
localparam CAP_SVPBMT    = (0 << 15);
localparam CAP_SV32x4    = (0 << 16);
localparam CAP_SV39x4    = (0 << 17);
localparam CAP_SV48x4    = (1 << 18);
localparam CAP_SV57x4    = (0 << 19);
localparam MSI_FLAT      = (0 << 22);
localparam MSI_MRIF      = (0 << 23);
localparam AMO           = (0 << 24);
localparam ATS           = (0 << 25);
localparam T2GPA         = (0 << 26);
localparam END           = (0 << 27);
localparam IGS           = (0 << 28);
localparam PMON          = (0 << 30);
localparam PAS           = (46 << 32);
localparam IOMMU_CAPS    = IOMMU_VERSION | CAP_SV32 |
                           CAP_SV39 | CAP_SV39 | CAP_SV48 | CAP_SV57 |
                           CAP_SVNAPOT | CAP_SVPBMT |
                           CAP_SV39x4 | CAP_SV39x4 | CAP_SV48x4 | CAP_SV57x4 |
                           MSI_FLAT | MSI_MRIF | AMO | ATS |
                           T2GPA | END | IGS | PMON | PAS;
localparam IOMMU_CAPS_H  = (IOMMU_CAPS >> 32);


localparam MAX_PW = 2;

localparam WALK_SM_IDLE        = 0;
localparam WALK_SM_DDTC_LOOKUP = 1;

//`endif //_PARAMS_VH_
