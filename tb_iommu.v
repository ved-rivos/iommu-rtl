//======================================================================
//
// tb_iommu.v
// -----------
// Testbench for the iommu top level wrapper
//
//
// ved@rivosinc.com
// Copyright (c) 2022, Rivos Inc.
// All rights reserved.
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

module tb_iommu();
   localparam MAX_PA = 46; 
   localparam MAX_PAB = 45; 
   localparam MAX_PPNB = 33;
  `include "consts.vh"
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;

  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg           tb_clk;
  reg           tb_reset_n;
  reg  [11 : 0] tb_mmio_paddr;
  reg           tb_mmio_pwrite;
  reg           tb_mmio_psel;
  reg           tb_mmio_penable;
  reg  [63 : 0] tb_mmio_pwdata;
  wire [63 : 0] tb_mmio_prdata;

  reg  [63 : 0] read_data;

  reg [51:0] tb_atr_iova;
  reg [23:0] tb_atr_device_id;
  reg [19:0] tb_atr_process_id;
  reg [1:0]  tb_atr_addr_type;
  reg        tb_atr_pid_valid;
  reg [7:0]  tb_atr_tag;
  reg        tb_atr_no_write;
  reg        tb_atr_exec_req;
  reg        tb_atr_priv_req;
  reg        tb_atr_irdy;
  wire       tb_atr_trdy;


  wire [2:0]        tb_atc_status;
  wire [MAX_PPNB:0] tb_atc_resp_pa;
  wire [7:0]        tb_atc_tag;
  wire              tb_atc_size;
  wire              tb_atc_no_snoop;
  wire              tb_atc_cxl_io;
  wire              tb_atc_global;
  wire              tb_atc_priv;
  wire              tb_atc_exe;
  wire              tb_atc_u;
  wire              tb_atc_r;
  wire              tb_atc_w;
  wire              tb_atc_irdy;
  reg               tb_atc_trdy;



  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  iommu dut(
        .clk(tb_clk),
        .rst_n(tb_reset_n),

        // MMIO bus
        .mmio_paddr(tb_mmio_paddr),
        .mmio_pwrite(tb_mmio_pwrite),
        .mmio_psel(tb_mmio_psel),
        .mmio_penable(tb_mmio_penable),
        .mmio_pwdata(tb_mmio_pwdata),
        .mmio_prdata(tb_mmio_prdata),

         // translation request interface
        .pgwkr_iova(tb_atr_iova),
        .pgwkr_device_id(tb_atr_device_id),
        .pgwkr_process_id(tb_atr_process_id),
        .pgwkr_addr_type(tb_atr_addr_type),
        .pgwkr_pid_valid(tb_atr_pid_valid),
        .pgwkr_tag(tb_atr_tag),
        .pgwkr_no_write(tb_atr_no_write),
        .pgwkr_exec_req(tb_atr_exec_req),
        .pgwkr_priv_req(tb_atr_priv_req),
        .pgwkr_irdy(tb_atr_irdy),
        .pgwkr_trdy(tb_atr_trdy),


        // translation response interface
        .pgwkc_status(tb_atc_status),
        .pgwkc_resp_pa(tb_atc_resp_pa),
        .pgwkc_tag(tb_atc_tag),
        .pgwkc_size(tb_atc_size),
        .pgwkc_no_snoop(tb_atc_no_snoop),
        .pgwkc_cxl_io(tb_atc_cxl_io),
        .pgwkc_global(tb_atc_global),
        .pgwkc_priv(tb_atc_priv),
        .pgwkc_exe(tb_atc_exe),
        .pgwkc_u(tb_atc_u),
        .pgwkc_r(tb_atc_r),
        .pgwkc_w(tb_atc_w),
        .pgwkc_irdy(tb_atc_irdy),
        .pgwkc_trdy(tb_atc_trdy)
    );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      cycle_ctr = cycle_ctr + 1;
      #(CLK_PERIOD);
      if (tb_monitor)
        begin
          dump_dut_state();
        end
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("State of DUT");
      $display("------------");
      $display("Cycle: %08d", cycle_ctr);
      $display("");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      $display("*** Toggle reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result;
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_result


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      tb_monitor = 0;

      tb_clk        = 1'h0;
      tb_reset_n    = 1'h1;

      tb_mmio_paddr = 12'hFF0;
      tb_mmio_pwrite = 1'h0;
      tb_mmio_psel = 1'h0;
      tb_mmio_penable = 1'h0;
      tb_mmio_pwdata = 64'h0;
      tb_atr_irdy = 0;
      tb_atc_trdy = 0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // write()
  //
  // Write the given word to the DUT using the DUT interface.
  //----------------------------------------------------------------
  task write(input [11 : 0] address,
             input [63 : 0] data);
    begin
      if (DEBUG)
        begin
          $display("*** Writing 0x%16x to 0x%02x.", data, address);
          $display("");
        end

      // Select the bus and setup address and data
      @(negedge tb_clk);
      tb_mmio_psel   = 1;
      tb_mmio_paddr  = address;
      tb_mmio_pwdata = data;
      tb_mmio_pwrite = 1;
      tb_mmio_penable = 0;

      // enable the write
      @(negedge tb_clk);
      tb_mmio_penable = 1;

      // move back to idle
      @(negedge tb_clk);
      tb_mmio_psel = 0;
      tb_mmio_penable = 0;
      tb_mmio_pwrite = 0;
    end
  endtask // write


  //----------------------------------------------------------------
  // read_word()
  //
  // Read a data word from the given address in the DUT.
  // the word read will be available in the global variable
  // read_data.
  //----------------------------------------------------------------
  task read(input [11 : 0]  address);
    begin

      @(negedge tb_clk);
      tb_mmio_psel   = 1;
      tb_mmio_paddr  = address;
      tb_mmio_pwrite = 0;
      tb_mmio_penable = 0;

      @(negedge tb_clk);
      tb_mmio_penable = 1;

      @(negedge tb_clk);
      read_data = tb_mmio_prdata;

      // make idle
      @(negedge tb_clk);
      tb_mmio_psel = 0;
      tb_mmio_penable = 0;

      if (DEBUG)
        begin
          $display("*** Reading 0x%16x from 0x%02x.", read_data, address);
          $display("");
        end
    end
  endtask // read_word

  //----------------------------------------------------------------
  // read_cap_and_write_ddtp()
  //----------------------------------------------------------------
  task read_cap_and_write_ddtp(input integer testcase);
    begin : test
      $display("");
      $display("*** TC%02d started.", testcase);
      
      read(CAPABILITIES_ADDR);
      $display("*** CAPABILITIES = 0x%08x%08x.", read_data[63:32], read_data[31:0]);
      read_data <= 0;
      write(DDTP_ADDR, ((1 << 60) | (1234)));
      read(DDTP_ADDR);
      while ( read_data[59] == 1 ) begin
          $display("*** DDTP-busy-waiting = 0x%08x%08x.", read_data[63:32], read_data[31:0]);
          read(DDTP_ADDR);
      end
      $display("*** DDTP = 0x%08x%08x.", read_data[63:32], read_data[31:0]);

      $display("*** TC%02d completed.", testcase);
      $display("");
    end
  endtask // test

  //----------------------------------------------------------------
  // send_trans_request()
  //----------------------------------------------------------------
  task send_trans_request(input integer testcase);
    begin : test
      $display("");
      $display("*** TC%02d started.", testcase);

      @(negedge tb_clk);
      tb_atr_iova = 1;
      tb_atr_device_id = 1;
      tb_atr_process_id = 0;
      tb_atr_addr_type = 0;
      tb_atr_pid_valid = 0;
      tb_atr_tag = 1;
      tb_atr_no_write = 0;
      tb_atr_exec_req = 0;
      tb_atr_priv_req = 0;
      tb_atr_irdy = 1;

      //while ( tb_atr_trdy == 0 ) begin
      @(posedge tb_atr_trdy);
      @(negedge tb_atr_trdy);
      tb_atr_irdy = 0;
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      
      $display("*** TC%02d completed.", testcase);
      $display("");
    end
  endtask // test


  //----------------------------------------------------------------
  // iommu_test
  //----------------------------------------------------------------
  initial
    begin : iommu_test
      $dumpfile("test.vcd");
      $dumpvars(0, dut);
      $display("   -= Testbench for iommu started =-");
      $display("     ============================");
      $display("");

      init_sim();
      reset_dut();

      read_cap_and_write_ddtp(1);
      send_trans_request(2);

      display_test_result();
      $display("");
      $display("*** iommu simulation done. ***");
      $finish;
    end // iommu_test
endmodule // tb_iommu

//======================================================================
// EOF tb_iommu.v
//======================================================================
