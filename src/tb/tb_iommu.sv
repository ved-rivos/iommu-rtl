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
  reg  [11:0]   tb_mmio_ctl_paddr;
  reg           tb_mmio_ctl_pwrite;
  reg           tb_mmio_ctl_psel;
  reg           tb_mmio_ctl_penable;
  reg  [63 : 0] tb_mmio_pwdata;
  wire [63 : 0] tb_mmio_prdata;

  reg  [63 : 0] read_data;

  reg [51:0] tb_atr_iova;
  reg [23:0] tb_atr_device_id;
  reg [19:0] tb_atr_process_id;
  reg [1:0]  tb_atr_addr_type;
  reg        tb_atr_read_write;
  reg        tb_atr_pid_valid;
  reg        tb_atr_no_write;
  reg        tb_atr_exec_req;
  reg        tb_atr_priv_req;
  reg        tb_atr_tee_req;
  reg [7:0]  tb_atr_tag;
  reg        tb_atr_irdy;
  input wire tb_atr_trdy;


  wire [2:0]    tb_atc_status_o;
  wire [33:0]   tb_atc_resp_pa_o;
  wire [7:0]    tb_atc_tag_o;
  wire          tb_atc_size_o;
  wire          tb_atc_no_snoop_o;
  wire          tb_atc_cxl_io_o;
  wire          tb_atc_g_o;
  wire          tb_atc_priv_o;
  wire          tb_atc_exe_o;
  wire          tb_atc_u_o;
  wire          tb_atc_r_o;
  wire          tb_atc_w_o;
  wire          tb_atc_irdy;
  reg           tb_atc_trdy;



  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  iommu dut(
        .clk(tb_clk),
        .rst_n(tb_reset_n),

        // MMIO bus
        .paddr_i(tb_mmio_ctl_paddr),
        .pwrite_i(tb_mmio_ctl_pwrite),
        .psel_i(tb_mmio_ctl_psel),
        .penable_i(tb_mmio_ctl_penable),
        .pwdata_i(tb_mmio_pwdata),
        .prdata_o(tb_mmio_prdata),

         // translation request interface
        .atr_iova_i(tb_atr_iova),
        .atr_device_id_i(tb_atr_device_id),
        .atr_process_id_i(tb_atr_process_id),
        .atr_addr_type_i(tb_atr_addr_type),
        .atr_read_write_i(tb_atr_read_write),
        .atr_pid_valid_i(tb_atr_pid_valid),
        .atr_no_write_i(tb_atr_no_write),
        .atr_exec_req_i(tb_atr_exec_req),
        .atr_priv_req_i(tb_atr_priv_req),
        .atr_tee_req_i(tb_atr_tee_req),
        .atr_tag_i(tb_atr_tag),
        .atr_irdy_i(tb_atr_irdy),
        .atr_trdy_o(tb_atr_trdy),


        // translation response interface
        .atc_status_o(tb_atc_status_o),
        .atc_resp_pa_o(tb_atc_resp_pa_o),
        .atc_tag_o(tb_atc_tag_o),
        .atc_size_o(tb_atc_size_o),
        .atc_no_snoop_o(tb_atc_no_snoop_o),
        .atc_cxl_io_o(tb_atc_cxl_io_o),
        .atc_g_o(tb_atc_g_o),
        .atc_priv_o(tb_atc_priv_o),
        .atc_exe_o(tb_atc_exe_o),
        .atc_u_o(tb_atc_u_o),
        .atc_r_o(tb_atc_r_o),
        .atc_w_o(tb_atc_w_o),
        .atc_irdy_o(tb_atc_irdy),
        .atc_trdy_i(tb_atc_trdy)
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

      tb_mmio_ctl_paddr = 12'hFF0;
      tb_mmio_ctl_pwrite = 1'h0;
      tb_mmio_ctl_psel = 1'h0;
      tb_mmio_ctl_penable = 1'h0;
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
      tb_mmio_ctl_psel   = 1;
      tb_mmio_ctl_paddr  = address;
      tb_mmio_ctl_pwrite = 1;
      tb_mmio_ctl_penable = 0;
      tb_mmio_pwdata = data;

      // enable the write
      @(negedge tb_clk);
      tb_mmio_ctl_penable = 1;

      // move back to idle
      @(negedge tb_clk);
      tb_mmio_ctl_psel = 0;
      tb_mmio_ctl_penable = 0;
      tb_mmio_ctl_pwrite = 0;
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
      tb_mmio_ctl_psel   = 1;
      tb_mmio_ctl_paddr  = address;
      tb_mmio_ctl_pwrite = 0;
      tb_mmio_ctl_penable = 0;

      @(negedge tb_clk);
      tb_mmio_ctl_penable = 1;

      @(negedge tb_clk);
      read_data = tb_mmio_prdata;

      // make idle
      @(negedge tb_clk);
      tb_mmio_ctl_psel = 0;
      tb_mmio_ctl_penable = 0;

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
      tb_atr_iova = 1;
      tb_atr_device_id = 265;
      tb_atr_process_id = 0;
      tb_atr_addr_type = 0;
      tb_atr_pid_valid = 0;
      tb_atr_tag = 1;
      tb_atr_no_write = 0;
      tb_atr_exec_req = 0;
      tb_atr_priv_req = 0;
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
      tb_atr_iova = 1;
      tb_atr_device_id = 4;
      tb_atr_process_id = 0;
      tb_atr_addr_type = 0;
      tb_atr_pid_valid = 0;
      tb_atr_tag = 2;
      tb_atr_no_write = 0;
      tb_atr_exec_req = 0;
      tb_atr_priv_req = 0;
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
