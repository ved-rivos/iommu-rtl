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
       #(
        parameter MAX_PA  = 46,
        parameter MAX_PAB = 45,
        parameter MAX_PPNB = 33
        )
        (
        input wire clk,
        input wire rst_n,

        // MMIO bus
        input  wire [11:0] mmio_paddr,
        input  wire        mmio_pwrite,
        input  wire        mmio_psel,
        input  wire        mmio_penable,
        input  wire [63:0] mmio_pwdata,
        output wire [63:0] mmio_prdata,

        // Address Translation Request bus
        input wire [51:0] pgwkr_iova,
        input wire [23:0] pgwkr_device_id,
        input wire [19:0] pgwkr_process_id,
        input wire [1:0]  pgwkr_addr_type,
        input wire        pgwkr_pid_valid,
        input wire [7:0]  pgwkr_tag,
        input wire        pgwkr_no_write,
        input wire        pgwkr_exec_req,
        input wire        pgwkr_priv_req,
        input wire        pgwkr_irdy,
        output wire       pgwkr_trdy,

        // Address translation Completion bus
        output wire [2:0]        pgwkc_status, 
        output wire [MAX_PPNB:0] pgwkc_resp_pa, 
        output wire [7:0]        pgwkc_tag, 
        output wire              pgwkc_size,
        output wire              pgwkc_no_snoop,
        output wire              pgwkc_cxl_io,
        output wire              pgwkc_global,
        output wire              pgwkc_priv,
        output wire              pgwkc_exe,
        output wire              pgwkc_u,
        output wire              pgwkc_r,
        output wire              pgwkc_w,
        output wire              pgwkc_irdy,
        input  wire              pgwkc_trdy
    );
    `include "consts.vh"

    wire ddtp_pgwk_idle, ddtp_pgwk_stall_req;

    rv_iommu_mmio mmio(
        .clk(clk),
        .rst_n(rst_n),
        .paddr(mmio_paddr),
        .pwrite(mmio_pwrite),
        .psel(mmio_psel),
        .penable(mmio_penable),
        .pwdata(mmio_pwdata),
        .prdata(mmio_prdata),
        .ddtp_pgwk_stall_req_o(ddtp_pgwk_stall_req),
        .ddtp_pgwk_idle_i(ddtp_pgwk_idle)
    );

    rv_iommu_walker walker(
        .clk(clk),
        .rst_n(rst_n),

        // translation request interface
        .atr_iova(pgwkr_iova),
        .atr_device_id(pgwkr_device_id),
        .atr_process_id(pgwkr_process_id),
        .atr_addr_type(pgwkr_addr_type),
        .atr_pid_valid(pgwkr_pid_valid),
        .atr_tag(pgwkr_tag),
        .atr_no_write(pgwkr_no_write),
        .atr_exec_req(pgwkr_exec_req),
        .atr_priv_req(pgwkr_priv_req),
        .atr_irdy(pgwkr_irdy),
        .atr_trdy(pgwkr_trdy),


        // translation response interface
        .atc_status(pgwkc_status), 
        .atc_resp_pa(pgwkc_resp_pa), 
        .atc_tag(pgwkc_tag), 
        .atc_size(pgwkc_size),
        .atc_no_snoop(pgwkc_no_snoop),
        .atc_cxl_io(pgwkc_cxl_io),
        .atc_global(pgwkc_global),
        .atc_priv(pgwkc_priv),
        .atc_exe(pgwkc_exe),
        .atc_u(pgwkc_u),
        .atc_r(pgwkc_r),
        .atc_w(pgwkc_w),
        .atc_irdy(pgwkc_irdy),
        .atc_trdy(pgwkc_trdy),

        // Control signals - input
        .ddtp_pgwk_stall_req_i(ddtp_pgwk_stall_req),

        // Control signals - output
        .ddtp_pgwk_idle_o(ddtp_pgwk_idle)
    );

endmodule
