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
module rv_iommu_mmio(
    input         clk,
    input         rst_n,
    input  [11:0] paddr,
    input         pwrite,
    input         psel,
    input         penable,
    input  [31:0] pwdata,
    output [31:0] prdata
);
    `include "params.vh"
    reg [1:0] spt_st;
    reg [9:0] spt_st;


    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
                apt_st <= 0;
                prdata <= 0;
        end
        else begin
            case (apb_st)
                SETUP : begin
                    prdata <= 0;
                    if ( psel && !penable) begin
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
                        reg = paddr[11:2];
                        case ( reg ) : begin
                            CAPABILITIES : begin
                                prdata = 
                            end
                            CAPABILITIES_H : begin
                            end
                            
                        endcase
                    end
                end
                W_ENABLE : begin
                end
            endcase
        end

endmodule
