// iommu_mmio.v
// MMIO interface module for IOMMU
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

module rv_iommu_mmio
       #(
        parameter MAX_PA  = 46,
        parameter MAX_PAB = 45,
        parameter MAX_PPNB = 33
        )
        (
        input  wire        clk,
        input  wire        rst_n,

        // MMIO input-output bus
        input  wire [11:0] paddr,
        input  wire        pwrite,
        input  wire        psel,
        input  wire        penable,
        input  wire [63:0] pwdata,
        output wire [63:0] prdata,

        // MMIO state 
        output wire [3:0]        ddtp_iommu_mode_o,
        output wire [MAX_PAB:0]  ddtp_ppn_o,

        // Control signals - output
        output wire              ddtp_pgwk_stall_req_o,

        // Control signals - input
        input wire               ddtp_pgwk_idle_i
    );
    `include "consts.vh"
    localparam SETUP = 0;
    localparam W_ENABLE = 1;
    localparam R_ENABLE = 2;

    reg [1:0]  apb_st;
    reg [9:0]  reg_ofst;
    reg [63:0] read_data;

    // DDTP - MMIO state
    reg [MODE_SB:MODE_EB] ddtp_iommu_mode;
    reg [MAX_PAB:0]       ddtp_ppn;
    reg                   ddtp_busy;
    // DDTP - Shadow state for page walks
    reg [MODE_SB:MODE_EB] pgwk_ddtp_iommu_mode;
    reg [MAX_PAB:0]       pgwk_ddtp_ppn;

    // Wire connections
    assign prdata = read_data;
    assign ddtp_iommu_mode_o = pgwk_ddtp_iommu_mode;
    assign ddtp_ppn_o = pgwk_ddtp_ppn;
    assign ddtp_pgwk_stall_req_o = ddtp_busy;

    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
                apb_st <= 0;
                read_data <= 0;
                ddtp_busy <= 0;
                pgwk_ddtp_iommu_mode <= 0;
        end
        else begin
            case (apb_st)
                SETUP : begin
                    read_data <= 0;
                    if ( psel && !penable) begin
                        reg_ofst <= paddr[11:2];
                        if ( pwrite ) begin
                            apb_st <= W_ENABLE;
                        end
                        else begin
                            apb_st <= R_ENABLE;
                        end
                    end
                end
                R_ENABLE : begin
                    if ( psel && penable && !pwrite ) begin
                        case ( reg_ofst )
                            CAPABILITIES : begin
                                read_data <= IOMMU_CAPS;
                            end
                            CAPABILITIES_H : begin
                                read_data <= IOMMU_CAPS_H;
                            end
                            // No writeable feature controls
                            // MSI and little-endian by default
                            FCTRL : begin
                                read_data <= 0;
                            end
                            DDTP : begin
                                read_data[MAX_PAB:0] <= ddtp_ppn;
                                read_data[MODE_SB:MODE_EB] <= ddtp_iommu_mode;
                                read_data[BUSY_BIT] <= ddtp_busy;
                                read_data[RSVD_SB:RSVD_EB] <= 0;
                            end
                        endcase
                    end
                    apb_st <= SETUP;
                end
                W_ENABLE : begin
                    if ( psel && penable && pwrite ) begin
                        case ( reg_ofst )
                            DDTP : begin
                                if ( (pwdata[MODE_SB:MODE_EB] <= THREE_LEVEL) && (ddtp_busy == 0)) begin
                                    // Set ddtp to busy to trigger a page walk
                                    // stall, IOATC flush, and fencing outstanding requests
                                    ddtp_busy <= 1;
                                    ddtp_ppn <= pwdata[MAX_PAB:0];
                                    ddtp_iommu_mode <= pwdata[MODE_SB:MODE_EB];
                                end
                            end
                        endcase
                    end
                    apb_st <= SETUP;
                end
            endcase
        end
    end
    always @(negedge rst_n or posedge clk) begin
        if ( ddtp_pgwk_idle_i && ddtp_busy ) begin
            pgwk_ddtp_iommu_mode <= ddtp_iommu_mode;
            pgwk_ddtp_ppn <= ddtp_ppn;
            ddtp_busy = 0;
        end
    end
endmodule
