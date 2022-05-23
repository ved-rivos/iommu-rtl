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
       #(
        parameter MAX_PA  = 46,
        parameter MAX_PAB = 45,
        parameter MAX_PPNB = 33
        )
        (
        input wire        clk,
        input wire        rst_n,

        // translation request interface
        input wire [51:0] atr_iova,
        input wire [23:0] atr_device_id,
        input wire [19:0] atr_process_id,
        input wire [1:0]  atr_addr_type,
        input wire        atr_pid_valid,
        input wire [7:0]  atr_tag,
        input wire        atr_no_write,
        input wire        atr_exec_req,
        input wire        atr_priv_req,
        input wire        atr_irdy,
        output wire       atr_trdy,


        // translation response interface
        output wire [2:0]        atc_status, 
        output wire [MAX_PPNB:0] atc_resp_pa, 
        output wire [7:0]        atc_tag, 
        output wire              atc_size,
        output wire              atc_no_snoop,
        output wire              atc_cxl_io,
        output wire              atc_global,
        output wire              atc_priv,
        output wire              atc_exe,
        output wire              atc_u,
        output wire              atc_r,
        output wire              atc_w,
        output wire              atc_irdy,
        input  wire              atc_trdy,

        // Control signals - input
        input wire               ddtp_pgwk_stall_req_i,

        // Control signals - output
        output wire               ddtp_pgwk_idle_o
    );
    `include "consts.vh"
    // Page walk tracker
    reg [51:0] pwt_iova       [0 : MAX_PW-1];
    reg [23:0] pwt_device_id  [0 : MAX_PW-1];
    reg [19:0] pwt_process_id [0 : MAX_PW-1];
    reg [1:0]  pwt_addr_type  [0 : MAX_PW-1];
    reg        pwt_pid_valid  [0 : MAX_PW-1];
    reg [7:0]  pwt_tag        [0 : MAX_PW-1];
    reg        pwt_no_write   [0 : MAX_PW-1];
    reg        pwt_exec_req   [0 : MAX_PW-1];
    reg        pwt_priv_req   [0 : MAX_PW-1];
    reg        pwt_next_free  [0 : MAX_PW-1];
    reg        pwt_state      [0 : MAX_PW-1];
    reg [MAX_PW-1 : 0] pwt_free_bitmap;
    integer    pgwk_in_flight;
    integer    i; 

    wire   is_any_tracker_free;
    wire   next_free;
    assign is_any_tracker_free = |pwt_free_bitmap;
    

    reg  req_st;
    localparam IDLE = 0;
    localparam READY = 1;
    assign atr_trdy = req_st;
    assign ddtp_pgwk_idle_o = (pgwk_in_flight == 0) ? 1 : 0;

    always @(pwt_free_bitmap) begin
        i = 0;
        while ( pwt_free_bitmap[i] == 0 && i < MAX_PW ) i = i + 1;
    end
    assign next_free= i;

    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
            for ( i = 0; i < MAX_PW; i = i + 1) pwt_free_bitmap[i] <= 1;
            req_st = IDLE;
            pgwk_in_flight = 0;
        end
        else begin
            case (req_st)
                IDLE: begin
                    if ( atr_irdy && is_any_tracker_free && ddtp_pgwk_stall_req_i != 1 ) begin
                        req_st <= READY;
                    end
                end
                READY: begin
                    pwt_free_bitmap[next_free] <= 0;
                    pwt_iova[next_free] <= atr_iova;
                    pwt_device_id[next_free] <= atr_device_id;
                    pwt_process_id[next_free] <= atr_process_id;
                    pwt_addr_type[next_free] <= atr_addr_type;
                    pwt_pid_valid[next_free] <= atr_pid_valid;
                    pwt_tag[next_free] <= atr_tag;
                    pwt_no_write[next_free] <= atr_no_write;
                    pwt_exec_req[next_free] <= atr_exec_req;
                    pwt_priv_req[next_free] <= atr_priv_req;
                    pwt_state[next_free] <= WALK_SM_DDTC_LOOKUP;
                    pgwk_in_flight = pgwk_in_flight + 1;
                    req_st <= IDLE;
                end
            endcase
        end
    end
    always @(negedge rst_n or posedge clk) begin
        if ( rst_n == 0 ) begin
        end
        else begin
        end
    end
endmodule
