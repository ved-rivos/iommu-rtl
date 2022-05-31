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
`include "../rtl/iommu.svh"
module tb_iommu();
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
  reg mmio_ctrl_t tb_mmio_ctl;
  reg  [63 : 0] tb_mmio_pwdata;
  wire [63 : 0] tb_mmio_prdata;

  reg  [63 : 0] read_data;

  reg pw_req_t tb_pwreq;
  reg          tb_atr_irdy;
  input wire   tb_atr_trdy;


  wire pw_rsp_t tb_pwrsp;
  wire          tb_atc_irdy;
  reg           tb_atc_trdy;



  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  iommu dut(
        .clk(tb_clk),
        .rst_n(tb_reset_n),

        // MMIO bus
        .mmio_ctl(tb_mmio_ctl),
        .mmio_pwdata(tb_mmio_pwdata),
        .mmio_prdata(tb_mmio_prdata),

         // translation request interface
        .pwreq(tb_pwreq),
        .pgwkr_irdy(tb_atr_irdy),
        .pgwkr_trdy(tb_atr_trdy),


        // translation response interface
        .pwrsp(tb_pwrsp),
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

      tb_mmio_ctl.paddr = 12'hFF0;
      tb_mmio_ctl.pwrite = 1'h0;
      tb_mmio_ctl.psel = 1'h0;
      tb_mmio_ctl.penable = 1'h0;
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
      tb_mmio_ctl.psel   = 1;
      tb_mmio_ctl.paddr  = address;
      tb_mmio_ctl.pwrite = 1;
      tb_mmio_ctl.penable = 0;
      tb_mmio_pwdata = data;

      // enable the write
      @(negedge tb_clk);
      tb_mmio_ctl.penable = 1;

      // move back to idle
      @(negedge tb_clk);
      tb_mmio_ctl.psel = 0;
      tb_mmio_ctl.penable = 0;
      tb_mmio_ctl.pwrite = 0;
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
      tb_mmio_ctl.psel   = 1;
      tb_mmio_ctl.paddr  = address;
      tb_mmio_ctl.pwrite = 0;
      tb_mmio_ctl.penable = 0;

      @(negedge tb_clk);
      tb_mmio_ctl.penable = 1;

      @(negedge tb_clk);
      read_data = tb_mmio_prdata;

      // make idle
      @(negedge tb_clk);
      tb_mmio_ctl.psel = 0;
      tb_mmio_ctl.penable = 0;

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
      write(DDTP_ADDR, ((4 << 60) | (1)));
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

      @(posedge tb_clk);
      tb_pwreq.iova = 1;
      tb_pwreq.device_id = 265;
      tb_pwreq.process_id = 0;
      tb_pwreq.addr_type = 0;
      tb_pwreq.pid_valid = 0;
      tb_pwreq.tag = 1;
      tb_pwreq.no_write = 0;
      tb_pwreq.exec_req = 0;
      tb_pwreq.priv_req = 0;
      tb_atr_irdy = 1;

      //while ( tb_atr_trdy == 0 );
      $display("*** WAIT START ", tb_atr_trdy);
      wait (tb_atr_trdy == 1);
      $display("*** WAIT END %d", tb_atr_trdy);
      @(negedge tb_clk);
      tb_atr_irdy = 0;
      @(posedge tb_clk);
      @(posedge tb_clk);
      $display("*** WAIT END %d", tb_atr_trdy);

      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      tb_pwreq.iova = 1;
      tb_pwreq.device_id = 4;
      tb_pwreq.process_id = 0;
      tb_pwreq.addr_type = 0;
      tb_pwreq.pid_valid = 0;
      tb_pwreq.tag = 2;
      tb_pwreq.no_write = 0;
      tb_pwreq.exec_req = 0;
      tb_pwreq.priv_req = 0;
      //tb_atr_irdy <= 1;
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
      @(posedge tb_clk);
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
      $display("   -= Testbench for iommu started =-");
      $display("     ============================");
      $display("");

      init_sim();
      reset_dut();

      read_cap_and_write_ddtp(1);
      $dumpfile("test.vcd");
      $dumpvars(0, dut);
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
