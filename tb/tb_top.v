////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText:  2021 , Dinesh Annayya
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0
// SPDX-FileContributor: Modified by Dinesh Annayya <dinesha@opencores.org>
//////////////////////////////////////////////////////////////////////
////                                                              ////
////  Standalone User validation Test bench                       ////
////                                                              ////
////  This file is part of the YIFive cores project               ////
////  https://github.com/dineshannayya/yifive_r0.git              ////
////  http://www.opencores.org/cores/yifive/                      ////
////                                                              ////
////  Description                                                 ////
////   This is a standalone test bench to validate the            ////
////   Digital core flash access through External WB i/F.         ////
////   1.  Check SPI Read Identification                          ////
////   2.  Check the Direct Memory Read (Qual/Single/Quad)        ////        
////   3.  Direct SPI Memory Prefetch - 3DW                       ////
////   4.  Direct SPI Memory Prefetch - 2DW                       ////
////   5.  Direct SPI Memory Prefetch - 1DW                       ////
////   6.  Direct SPI Memory Prefetch - 7DW                       ////
////   7.  1DW  Indirect Read                                     ////
////   8.  2DW  Indirect Read                                     ////
////   9.  3DW  Indirect Read                                     ////
////   10. 4DW  Indirect Read                                     ////
////   11. 5DW  Indirect Read                                     ////
////   12. 8DW  Indirect Read                                     ////
////   13. Sector Erase command + Page Write & Read Back          ////
////                                                              ////
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision :                                                  ////
////    0.1 - 16th Feb 2021, Dinesh A                             ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

`default_nettype none

`timescale 1 ns / 1 ps

//`include "spiflash.v"
`include "s25fl256s.sv"
`include "cy15b104qs.v"
`include "spiram.v"
`ifdef GL
     //`define USE_POWER_PINS
     `define UNIT_DELAY #0.1
     `include "libs.ref/sky130_fd_sc_hd/verilog/primitives.v"
     `include "libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v"
     `include "libs.ref/sky130_fd_sc_hvl/verilog/primitives.v"
     `include "qspim_top.nl.v"
 `else
     `define USE_POWER_PINS
     `define UNIT_DELAY #0.1
     `include "qspim_ctrl.sv"
     `include "qspim_tx.sv"
     `include "qspim_rx.sv"
     `include "qspim_clkgen.sv"
     `include "qspim_regs.sv"
     `include "qspim_if.sv"
     `include "qspim_fifo.sv"
     `include "qspim_top.sv"
     `include "clk_skew_adjust.gv"
     `include "ctech_cells.sv"
     `include "reset_sync.sv"

     `include "libs.ref/sky130_fd_sc_hd/verilog/primitives.v"
     `include "libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v"
     `include "libs.ref/sky130_fd_sc_hvl/verilog/primitives.v"
     `include "libs.ref/sky130_fd_sc_hvl/verilog/sky130_fd_sc_hvl.v"

 `endif

 // REGISTER MAP
 `define QSPIM_GLBL_CTRL           32'h10000000
 `define QSPIM_DMEM_G0_RD_CTRL    32'h10000004
 `define QSPIM_DMEM_G0_WR_CTRL    32'h10000008
 `define QSPIM_DMEM_G1_RD_CTRL    32'h1000000C
 `define QSPIM_DMEM_G1_WR_CTRL    32'h10000010

 `define QSPIM_DMEM_CS_AMAP        32'h10000014
 `define QSPIM_DMEM_CA_AMASK       32'h10000018

 `define QSPIM_IMEM_CTRL1          32'h1000001C
 `define QSPIM_IMEM_CTRL2          32'h10000020
 `define QSPIM_IMEM_ADDR           32'h10000024
 `define QSPIM_IMEM_WDATA          32'h10000028
 `define QSPIM_IMEM_RDATA          32'h1000002C
 `define QSPIM_SPI_STATUS          32'h10000030

module tb_top;
/*************************************************************
*  SPI FSM State Control
*
*   OPERATION   COMMAND                   SEQUENCE 
*
*    ERASE       P4E(0x20)           ->  COMMAND + ADDRESS
*    ERASE       P8E(0x40)           ->  COMMAND + ADDRESS
*    ERASE       SE(0xD8)            ->  COMMAND + ADDRESS
*    ERASE       BE(0x60)            ->  COMMAND + ADDRESS
*    ERASE       BE(0xC7)            ->  COMMAND 
*    PROGRAM     PP(0x02)            ->  COMMAND + ADDRESS + Write DATA
*    PROGRAM     QPP(0x32)           ->  COMMAND + ADDRESS + Write DATA
*    READ        READ(0x3)           ->  COMMAND + ADDRESS + READ DATA
*    READ        FAST_READ(0xB)      ->  COMMAND + ADDRESS + DUMMY + READ DATA
*    READ        DOR (0x3B)          ->  COMMAND + ADDRESS + DUMMY + READ DATA
*    READ        QOR (0x6B)          ->  COMMAND + ADDRESS + DUMMY + READ DATA
*    READ        DIOR (0xBB)         ->  COMMAND + ADDRESS + MODE  + READ DATA
*    READ        QIOR (0xEB)         ->  COMMAND + ADDRESS + MODE  + DUMMY + READ DATA
*    READ        RDID (0x9F)         ->  COMMAND + READ DATA
*    READ        READ_ID (0x90)      ->  COMMAND + ADDRESS + READ DATA
*    WRITE       WREN(0x6)           ->  COMMAND
*    WRITE       WRDI                ->  COMMAND
*    STATUS      RDSR(0x05)          ->  COMMAND + READ DATA
*    STATUS      RCR(0x35)           ->  COMMAND + READ DATA
*    CONFIG      WRR(0x01)           ->  COMMAND + WRITE DATA
*    CONFIG      CLSR(0x30)          ->  COMMAND
*    Power Saving DP(0xB9)           ->  COMMAND
*    Power Saving RES(0xAB)          ->  COMMAND + READ DATA
*    OTP          OTPP(0x42)         ->  COMMAND + ADDR+ WRITE DATA
*    OTP          OTPR(0x4B)         ->  COMMAND + ADDR + DUMMY + READ DATA
*    ********************************************************************/
parameter P_FSM_C      = 4'b0000; // Command Phase Only
parameter P_FSM_CW     = 4'b0001; // Command + Write DATA Phase Only
parameter P_FSM_CA     = 4'b0010; // Command -> Address Phase Only

parameter P_FSM_CAR    = 4'b0011; // Command -> Address -> Read Data
parameter P_FSM_CADR   = 4'b0100; // Command -> Address -> Dummy -> Read Data
parameter P_FSM_CAMR   = 4'b0101; // Command -> Address -> Mode -> Read Data
parameter P_FSM_CAMDR  = 4'b0110; // Command -> Address -> Mode -> Dummy -> Read Data

parameter P_FSM_CAW    = 4'b0111; // Command -> Address ->Write Data
parameter P_FSM_CADW   = 4'b1000; // Command -> Address -> DUMMY + Write Data
parameter P_FSM_CAMW   = 4'b1001; // Command -> Address -> MODE + Write Data

parameter P_FSM_CDR    = 4'b1010; // COMMAND -> DUMMY -> READ
parameter P_FSM_CDW    = 4'b1011; // COMMAND -> DUMMY -> WRITE
parameter P_FSM_CR     = 4'b1100;  // COMMAND -> READ

parameter P_MODE_SWITCH_IDLE     = 2'b00;
parameter P_MODE_SWITCH_AT_ADDR  = 2'b01;
parameter P_MODE_SWITCH_AT_DATA  = 2'b10;

parameter P_SINGLE = 2'b00;
parameter P_DOUBLE = 2'b01;
parameter P_QUAD   = 2'b10;
parameter P_QDDR   = 2'b11;

parameter S_SINGLE = 1'b0; // SRAM Single
parameter S_QUAD   = 1'b1; // SRAM Quad

parameter P_FLASH_READ    = 8'h03;  // Normal Read
parameter P_FLASH_FREAD   = 8'h0B;  // Fast Read
parameter P_FLASH_DOR     = 8'h3B;  // Dual Read
parameter P_FLASH_QOR     = 8'h6B;  // Quad Read
parameter P_FLASH_DIOR    = 8'hBB;  // Dual I/O Read
parameter P_FLASH_QIOR    = 8'hEB;  // Quad IO Read
parameter P_FLASH_DDRFR   = 8'h0D;  // DDR Read
parameter P_FLASH_DDRDIOR = 8'hBD;  // DDR Dual I/O Read
parameter P_FLASH_DDRQIOR = 8'hED;  // DDR Quad I/O Read

	reg clock;
	reg wb_rst_i;

        reg        wbd_ext_cyc_i;  // strobe/request
        reg        wbd_ext_stb_i;  // strobe/request
        reg [31:0] wbd_ext_adr_i;  // address
        reg        wbd_ext_we_i;   // write
        reg        wbd_ext_bry_i;  // Busrt Ready
        reg [9:0]  wbd_ext_bl_i;   // Burst Length
        reg [31:0] wbd_ext_dat_i;  // data output
        reg [3:0]  wbd_ext_sel_i;  // byte enable

        wire [31:0] wbd_ext_dat_o;  // data input
        wire        wbd_ext_ack_o;  // acknowlegement
        wire        wbd_ext_lack_o;  // acknowlegement
        wire        wbd_ext_err_o;  // error

	reg [31:0]  data_array [0 : 511]; // Data Array
	reg         test_fail;
	reg [31:0] read_data;

    reg  [1:0]  strap_flash;
    reg         strap_sram;
    reg         strap_pre_sram;

        wire flash_clk;
        wire [3:0] spi_csb;
        wire [3:0] spi_sdo;
        wire [3:0] spi_oeb;
        wire [3:0] io_oeb;
        tri  flash_io0;
        tri  flash_io1;
        tri  flash_io2;
        tri  flash_io3;
        wire [3:0] spi_sdi ;

	reg [7:0] cnt;

	// External clock is used by default.  Make this artificially fast for the
	// simulation.  Normally this would be a slow clock and the digital PLL
	// would be the fast clock.

	always #12.5ns clock <= (clock === 1'b0);

	initial begin
		clock = 0;
                wbd_ext_cyc_i ='h0;  // strobe/request
                wbd_ext_stb_i ='h0;  // strobe/request
                wbd_ext_adr_i ='h0;  // address
                wbd_ext_we_i  ='h0;  // write
                wbd_ext_bry_i = 1'b1;
                wbd_ext_bl_i  = 10'h1;
                wbd_ext_dat_i ='h0;  // data output
                wbd_ext_sel_i ='h0;  // byte enable
	end

	`ifdef WFDUMP
	   initial begin
	   	$dumpfile("simx.vcd");
	   	$dumpvars(0, tb_top);

	   end
       `endif

	initial begin
        strap_flash = 2'b11;
        strap_pre_sram  = 1'b0;
        strap_sram  = 1'b1;
		test_fail = 0;
		wb_rst_i = 1'b1;
		wait(u_spi_flash0_256mb.PoweredUp==1);
		$display("Monitor: SPI MEMORY POWER UP Sequence Completed");
		#100;
		wb_rst_i = 1'b0;	    	// Release reset

		$display("############################################################");
		$display("  Flash System Strap Testing....Step-1: strap_flash:P_SINGLE ");
		$display("############################################################");
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_SINGLE;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
         `ifndef GL
         if(u_top.cfg_m0_g0_rd_cmd_reg != P_FLASH_FREAD) begin
            $display("ERROR: Wrong Command Found Exp: %x Rxd:%x", P_FLASH_FREAD,u_top.cfg_m0_g0_rd_cmd_reg);
            test_fail = 1;
         end
         `endif
		    wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		    wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		    wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		    wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		    wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		    wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		    wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		    wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		    wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		    wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		    wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		    wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		    wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		    wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		    wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		    wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);
       
		$display("############################################################");
		$display("  Flash System Strap Testing....Step-2: strap_flash:P_DOUBLE ");
		$display("############################################################");
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_DOUBLE;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
         `ifndef GL
         if(u_top.cfg_m0_g0_rd_cmd_reg != P_FLASH_DOR) begin
            $display("ERROR: Wrong Command Found Exp: %x Rxd:%x", P_FLASH_DOR,u_top.cfg_m0_g0_rd_cmd_reg);
            test_fail = 1;
         end
         `endif
		    wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		    wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		    wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		    wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		    wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		    wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		    wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		    wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		    wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		    wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		    wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		    wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		    wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		    wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		    wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		    wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("############################################################");
		$display("  Flash System Strap Testing....Step-3: strap_flash:P_QUAD ");
		$display("############################################################");
	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_QUAD;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
         `ifndef GL
         if(u_top.cfg_m0_g0_rd_cmd_reg != P_FLASH_QIOR) begin
            $display("ERROR: Wrong Command Found Exp: %x Rxd:%x", P_FLASH_QIOR,u_top.cfg_m0_g0_rd_cmd_reg);
            test_fail = 1;
         end
         `endif
		    wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		    wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		    wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		    wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		    wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		    wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		    wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		    wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		    wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		    wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		    wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		    wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		    wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		    wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		    wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		    wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);
		
        $display("############################################################");
		$display("  Flash System Strap Testing....Step-4: strap_flash:P_QDDR ");
		$display("############################################################");
	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_QDDR;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
         `ifndef GL
         if(u_top.cfg_m0_g0_rd_cmd_reg != P_FLASH_DDRQIOR) begin
            $display("ERROR: Wrong Command Found Exp: %x Rxd:%x", P_FLASH_DDRQIOR,u_top.cfg_m0_g0_rd_cmd_reg);
            test_fail = 1;
         end
         `endif
		    wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		    wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		    wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		    wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		    wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		    wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		    wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		    wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		    wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		    wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		    wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		    wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		    wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		    wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		    wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		    wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

        $display("############################################################");
		$display("  SRAM System Strap Testing....Step-1: strap_flash:S_SINGLE ");
		$display("############################################################");
	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_QDDR;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
		data_array[10'h0]    = 32'h00112233;
		data_array[10'h1]    = 32'h44556677;
		data_array[10'h2]    = 32'h8899AABB;
		data_array[10'h3]    = 32'hCCDDEEFF;
		data_array[10'h4]    = 32'h00001111;
		data_array[10'h5]    = 32'h22223333;
		data_array[10'h6]    = 32'h44445555;
		data_array[10'h7]    = 32'h66667777;
		wb_user_core_bwrite(32'h08002000,10'h8);
		wb_user_core_bread_check(32'h08002000,10'h8);

        $display("############################################################");
		$display("  SRAM System Strap Testing....Step-2: strap_sram:S_QUAD ");
		$display("############################################################");
	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash     = P_QDDR;
              strap_pre_sram  = S_SINGLE;
              strap_sram      = S_QUAD;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
		data_array[10'h0]    = 32'h00112233;
		data_array[10'h1]    = 32'h44556677;
		data_array[10'h2]    = 32'h8899AABB;
		data_array[10'h3]    = 32'hCCDDEEFF;
		data_array[10'h4]    = 32'h00001111;
		data_array[10'h5]    = 32'h22223333;
		data_array[10'h6]    = 32'h44445555;
		data_array[10'h7]    = 32'h66667777;
		wb_user_core_bwrite(32'h08003000,10'h8);
		wb_user_core_bread_check(32'h08003000,10'h8);

        $display("############################################################");
		$display("  SRAM System Strap Testing....Step-3: strap_sram:S_SINGLE ");
		$display("############################################################");
	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash     = P_QDDR;
              strap_pre_sram  = S_QUAD;
              strap_sram      = S_SINGLE;

        #1000 wb_rst_i = 0;
	     repeat (200) @(posedge clock);
		data_array[10'h0]    = 32'h00112233;
		data_array[10'h1]    = 32'h44556677;
		data_array[10'h2]    = 32'h8899AABB;
		data_array[10'h3]    = 32'hCCDDEEFF;
		data_array[10'h4]    = 32'h00001111;
		data_array[10'h5]    = 32'h22223333;
		data_array[10'h6]    = 32'h44445555;
		data_array[10'h7]    = 32'h66667777;
		wb_user_core_bwrite(32'h08004000,10'h8);
		wb_user_core_bread_check(32'h08004000,10'h8);

        $display("############################################################");
		$display("  End of Strap Testing                                     ");
		$display("############################################################");


       //------------------ End of Strap Testing ------------------------------------------

	     repeat (200) @(posedge clock);
        #1000 wb_rst_i = 1;
        #1000 strap_flash = P_QDDR;
              strap_pre_sram  = S_SINGLE;
              strap_sram  = S_SINGLE;

        #1000 wb_rst_i = 0;

		// CS#2 Switch From QSPI to SSPI Mode
		//wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0100});
		//wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'hFF});
		//wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);

        $display("############################################################");
		$display("  CS#2 SRAM INDIRECT READ ACCESS in SSPI Mode              ");
		$display("############################################################");
	        repeat (200) @(posedge clock);
		// CS#2 SSPI Indirect RAM READ ACCESS-
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b10,P_FSM_CADR,8'h00,8'h03});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h03020100);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000004);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h07060504);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000008);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0b0a0908);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000000C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0f0e0d0c);

		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h11111111);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000204);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h22222222);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000208);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h33333333);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000020C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h44444444);

		// CS#2 SSPI Indiect Write DATA
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h10,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00112233);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h44556677);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h8899AABB);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'hCCDDEEFF);
		
		// CS#2 SSPI Indirect READ DATA
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h10,2'b00,2'b10,P_FSM_CADR,8'h00,8'h03});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00112233);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h44556677);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h8899AABB);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'hCCDDEEFF);


        $display("############################################################");
		$display("  CS#2 SRAM INDIRECT READ ACCESS in QSPI Mode              ");
		$display("############################################################");
		// CS#2 Switch to QSPI Mode
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'h38});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);


		// CS#2 QUAD Indirect Write DATA
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h10,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h01234557);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h89ABCDEF);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h12345678);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h9ABCDEF0);


		// CS#2 QUAD Indirect READ DATA
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h10,2'b00,2'b10,P_FSM_CADR,8'h00,8'h03});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h01234557);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h89ABCDEF);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h12345678);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9ABCDEF0);

		// CS#2 Switch From QSPI to SSPI Mode
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'hFF});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
		///////////////////// End of CS#2 Indirect SRAM Memory Access Testing ///////////////////////////////////
		$display("#############################################");
		$display("  CS#2: Page Read through Direct Burst Access in SSPI Mode       ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G1_WR_CTRL,{P_FSM_CAW, 4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h02});
		wb_user_core_write(`QSPIM_DMEM_G1_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h03});

		data_array[10'h0]    = 32'h01234557;
		data_array[10'h1]    = 32'h89ABCDEF;
		data_array[10'h2]    = 32'h12345678;
		data_array[10'h3]    = 32'h9ABCDEF0;
		wb_user_core_bread_check(32'h08000000,10'h4);

		data_array[10'h0]    = 32'h00112233;
		data_array[10'h1]    = 32'h44556677;
		data_array[10'h2]    = 32'h8899AABB;
		data_array[10'h3]    = 32'hCCDDEEFF;
		data_array[10'h4]    = 32'h00001111;
		data_array[10'h5]    = 32'h22223333;
		data_array[10'h6]    = 32'h44445555;
		data_array[10'h7]    = 32'h66667777;
		wb_user_core_bwrite(32'h08000000,10'h8);
		wb_user_core_bread_check(32'h08000000,10'h8);

		$display("#############################################");
		$display("  Page Read through Direct Burst Access in QSPI Mode       ");
		$display("#############################################");
		// CS#1 Switch to QSPI Mode
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'h38});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
		wb_user_core_write(`QSPIM_DMEM_G1_WR_CTRL,{P_FSM_CAW,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,8'h00,8'h02});
		wb_user_core_write(`QSPIM_DMEM_G1_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,8'h00,8'h03});

		data_array[10'h0]    = 32'h00112233;
		data_array[10'h1]    = 32'h44556677;
		data_array[10'h2]    = 32'h8899AABB;
		data_array[10'h3]    = 32'hCCDDEEFF;
		data_array[10'h4]    = 32'h00001111;
		data_array[10'h5]    = 32'h22223333;
		data_array[10'h6]    = 32'h44445555;
		data_array[10'h7]    = 32'h66667777;
		wb_user_core_bwrite(32'h08000000,10'h8);
		wb_user_core_bread_check(32'h08000000,10'h8);
		
		data_array[10'h000]    = 32'h01234557;
		data_array[10'h001]    = 32'h89ABCDEF;
		data_array[10'h002]    = 32'h12345678;
		data_array[10'h003]    = 32'h9ABCDEF0;
		data_array[10'h004]    = 32'h23456789;
		data_array[10'h005]    = 32'hABCDEF01;
		data_array[10'h006]    = 32'h3456789A;
		data_array[10'h007]    = 32'hBCDEF012;
		wb_user_core_bwrite(32'h08000200,10'h8);
		wb_user_core_bread_check(32'h08000200,10'h8);

		// CS#1 Switch From QSPI to SSPI Mode
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0100});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'hFF});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);


		///////////////////// End of CS#1 Testing ///////////////////////////////////


		$display("#############################################");
		$display("  Read Identification (RDID:0x9F)            ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,2'b00,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b00,P_FSM_CR,8'h00,8'h9F});
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00190201);
		$display("#############################################");
		$display("Testing Direct SPI Memory Read              ");
		$display(" SPI Mode: QDDR (Dual 4 bit)                ");
		$display("Prefetch : 1DW, OPCODE:READ(0xED)           ");
		$display("SEQ: Command -> Address -> Read Data        ");
		$display("#############################################");
		// QDDR Config
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0100,2'b10,P_MODE_SWITCH_AT_ADDR,P_QDDR,P_SINGLE,8'h00,8'hED});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);
		$display("#############################################");
		$display("Testing Direct SPI Memory Read              ");
		$display(" SPI Mode: Normal/Single Bit                ");
		$display("Prefetch : 1DW, OPCODE:READ(0x3)            ");
		$display("SEQ: Command -> Address -> Read Data        ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h03});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);
		$display("#############################################");
		$display("Testing Direct SPI Memory Read              ");
		$display(" SPI Mode: Normal/Single Bit                ");
		$display("Prefetch : 1DW, OPCODE:FASTREAD(0xB)        ");
		$display("SEQ: Command -> Address -> Dummy -> Read Data");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h0B});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("Testing Direct SPI Memory Read              ");
		$display(" SPI Mode: Dual Mode                        ");
		$display("Prefetch : 1DW, OPCODE:DOR(0x3B)        ");
		$display("SEQ: Command -> Address -> Dummy -> Read Data");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_AT_DATA,P_DOUBLE,P_SINGLE,8'h00,8'h3B});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("Testing Direct SPI Memory Read with Prefetch");
		$display(" SPI Mode: Quad                             ");
		$display("Prefetch : 8DW, OPCODE:URAD READ(0xEB)      ");
		$display("SEQ: Command -> Address -> Dummy -> Read Data");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0001,2'b10,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,8'h00,8'hEB});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("Testing Direct SPI Memory Read with Prefetch:3DW");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0001,2'b10,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,8'h00,8'hEB});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("Testing Direct SPI Memory Read with Prefetch:2DW");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0001,2'b10,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,8'h00,8'hEB});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);


		$display("#############################################");
		$display("Testing Direct SPI Memory Read with Prefetch:1DW");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0001,2'b10,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,8'h00,8'hEB});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("Testing Direct SPI Memory Read with Prefetch:7DW");
		$display("#############################################");
		wb_user_core_write(`QSPIM_DMEM_G0_RD_CTRL,{P_FSM_CAMDR,4'b0001,2'b10,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,8'h00,8'hEB});
		wb_user_core_read_check(32'h00000200,read_data,32'h00000093);
		wb_user_core_read_check(32'h00000204,read_data,32'h00000113);
		wb_user_core_read_check(32'h00000208,read_data,32'h00000193);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00000213);
		wb_user_core_read_check(32'h00000210,read_data,32'h00000293);
		wb_user_core_read_check(32'h00000214,read_data,32'h00000313);
		wb_user_core_read_check(32'h00000218,read_data,32'h00000393);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00000413);
		wb_user_core_read_check(32'h00000300,read_data,32'h0005A023);
		wb_user_core_read_check(32'h00000304,read_data,32'h9DE30591);
		wb_user_core_read_check(32'h00000308,read_data,32'h02B7FEE5);
		wb_user_core_read_check(32'h0000030C,read_data,32'h43050049);
		wb_user_core_read_check(32'h00000310,read_data,32'h0062A023);
		wb_user_core_read_check(32'h00000314,read_data,32'h004902B7);
		wb_user_core_read_check(32'h00000318,read_data,32'h03130291);
		wb_user_core_read_check(32'h0000031C,read_data,32'ha0230630);

		$display("#############################################");
		$display("  Testing Single Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000204);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000208);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000020C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000210);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000214);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000313);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000218);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000393);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000021C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000413);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000304);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000308);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000030C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000310);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000314);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h004902B7);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000318);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h03130291);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000031C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'ha0230630);
		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Testing Two Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h8,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000208);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000210);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000313);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000218);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000393);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000413);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000308);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000310);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h004902B7);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000318);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h03130291);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'ha0230630);
		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Testing Three Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'hC,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000020C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000313);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h0000030C);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h004902B7);
		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Testing Four Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h10,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000210);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000313);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000393);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000413);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000310);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h004902B7);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h03130291);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'ha0230630);
		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Testing Five Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h14,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		$display("#############################################");
		$display("  Testing Eight Word Indirect SPI Memory Read");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h20,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000093);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000113);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000193);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000213);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000293);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000313);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000393);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00000413);
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000300);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0005A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h9DE30591);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h02B7FEE5);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h43050049);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h0062A023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h004902B7);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h03130291);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'ha0230630);

		$display("#############################################");
		$display("  Sector Erase Command            ");
		$display("#############################################");
		// WEN COMMAND
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
                // Sector Erase
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b10,P_FSM_CA,8'h00,8'hD8});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);

		// RDSR
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b00,P_FSM_CR,8'h00,8'h05});
		read_data = 32'hFFFF_FFFF;
		while (read_data[1:0] == 2'b11) begin
		    wb_user_core_read(`QSPIM_IMEM_RDATA,read_data);
		    repeat (10) @(posedge clock);
		end

		$display("#############################################");
		$display("  Page Write Command Address: 0x00          ");
		$display("#############################################");
		// WEN COMMAND
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
		 // Page Programing
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'hF0,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010001);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010002);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010003);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010004);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010005);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010006);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010007);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010008);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010009);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010010);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010011);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010012);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010013);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010014);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010015);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010016);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010017);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010018);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010019);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010020);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010021);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010022);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010023);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010024);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010025);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010026);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010027);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010028);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010029);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010030);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010031);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010032);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010033);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010034);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010035);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010036);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010037);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010038);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010039);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010040);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010041);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010042);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010043);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010044);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010045);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010046);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010047);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010048);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010049);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010050);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010051);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010052);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010053);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010054);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010055);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010056);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010057);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010058);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00010059);

		// RDSR
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b00,P_FSM_CR,8'h00,8'h05});
		read_data = 32'hFFFF_FFFF;
		while (read_data[1:0] == 2'b11) begin
		    wb_user_core_read(`QSPIM_IMEM_RDATA,read_data);
		    repeat (10) @(posedge clock);
		 end

		$display("#############################################");
		$display("  Page Read through Direct Access            ");
		$display("#############################################");

		wb_user_core_read_check(32'h00000000,read_data,32'h00010000);
		wb_user_core_read_check(32'h00000004,read_data,32'h00010001);
		wb_user_core_read_check(32'h00000008,read_data,32'h00010002);
		wb_user_core_read_check(32'h0000000C,read_data,32'h00010003);
		wb_user_core_read_check(32'h00000010,read_data,32'h00010004);
		wb_user_core_read_check(32'h00000014,read_data,32'h00010005);
		wb_user_core_read_check(32'h00000018,read_data,32'h00010006);
		wb_user_core_read_check(32'h0000001C,read_data,32'h00010007);
		wb_user_core_read_check(32'h00000020,read_data,32'h00010008);
		wb_user_core_read_check(32'h00000024,read_data,32'h00010009);
		wb_user_core_read_check(32'h00000028,read_data,32'h00010010);
		wb_user_core_read_check(32'h0000002C,read_data,32'h00010011);
		wb_user_core_read_check(32'h00000030,read_data,32'h00010012);
		wb_user_core_read_check(32'h00000034,read_data,32'h00010013);
		wb_user_core_read_check(32'h00000038,read_data,32'h00010014);
		wb_user_core_read_check(32'h0000003C,read_data,32'h00010015);
		wb_user_core_read_check(32'h00000040,read_data,32'h00010016);
		wb_user_core_read_check(32'h00000044,read_data,32'h00010017);
		wb_user_core_read_check(32'h00000048,read_data,32'h00010018);
		wb_user_core_read_check(32'h0000004C,read_data,32'h00010019);
		wb_user_core_read_check(32'h00000050,read_data,32'h00010020);
		wb_user_core_read_check(32'h00000054,read_data,32'h00010021);
		wb_user_core_read_check(32'h00000058,read_data,32'h00010022);
		wb_user_core_read_check(32'h0000005C,read_data,32'h00010023);
		wb_user_core_read_check(32'h00000060,read_data,32'h00010024);
		wb_user_core_read_check(32'h00000064,read_data,32'h00010025);
		wb_user_core_read_check(32'h00000068,read_data,32'h00010026);
		wb_user_core_read_check(32'h0000006C,read_data,32'h00010027);
		wb_user_core_read_check(32'h00000070,read_data,32'h00010028);
		wb_user_core_read_check(32'h00000074,read_data,32'h00010029);
		wb_user_core_read_check(32'h00000078,read_data,32'h00010030);
		wb_user_core_read_check(32'h0000007C,read_data,32'h00010031);
		wb_user_core_read_check(32'h00000080,read_data,32'h00010032);
		wb_user_core_read_check(32'h00000084,read_data,32'h00010033);
		wb_user_core_read_check(32'h00000088,read_data,32'h00010034);
		wb_user_core_read_check(32'h0000008C,read_data,32'h00010035);
		wb_user_core_read_check(32'h00000090,read_data,32'h00010036);
		wb_user_core_read_check(32'h00000094,read_data,32'h00010037);
		wb_user_core_read_check(32'h00000098,read_data,32'h00010038);
		wb_user_core_read_check(32'h0000009C,read_data,32'h00010039);
		wb_user_core_read_check(32'h000000A0,read_data,32'h00010040);
		wb_user_core_read_check(32'h000000A4,read_data,32'h00010041);
		wb_user_core_read_check(32'h000000A8,read_data,32'h00010042);
		wb_user_core_read_check(32'h000000AC,read_data,32'h00010043);
		wb_user_core_read_check(32'h000000B0,read_data,32'h00010044);
		wb_user_core_read_check(32'h000000B4,read_data,32'h00010045);
		wb_user_core_read_check(32'h000000B8,read_data,32'h00010046);
		wb_user_core_read_check(32'h000000BC,read_data,32'h00010047);
		wb_user_core_read_check(32'h000000C0,read_data,32'h00010048);
		wb_user_core_read_check(32'h000000C4,read_data,32'h00010049);
		wb_user_core_read_check(32'h000000C8,read_data,32'h00010050);
		wb_user_core_read_check(32'h000000CC,read_data,32'h00010051);
		wb_user_core_read_check(32'h000000D0,read_data,32'h00010052);
		wb_user_core_read_check(32'h000000D4,read_data,32'h00010053);
		wb_user_core_read_check(32'h000000D8,read_data,32'h00010054);
		wb_user_core_read_check(32'h000000DC,read_data,32'h00010055);
		wb_user_core_read_check(32'h000000E0,read_data,32'h00010056);
		wb_user_core_read_check(32'h000000E4,read_data,32'h00010057);
		wb_user_core_read_check(32'h000000E8,read_data,32'h00010058);
		wb_user_core_read_check(32'h000000EC,read_data,32'h00010059);

		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Page Read through Indirect Access           ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'hF0,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000000);

		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010000);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010001);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010002);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010003);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010004);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010005);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010006);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010007);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010008);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010009);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010010);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010011);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010012);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010013);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010014);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010015);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010016);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010017);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010018);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010019);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010020);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010021);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010022);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010024);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010025);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010026);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010027);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010028);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010029);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010030);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010031);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010032);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010033);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010034);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010035);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010036);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010037);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010038);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010039);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010040);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010041);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010042);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010043);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010044);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010045);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010046);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010047);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010048);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010049);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010050);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010051);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010052);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010053);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010054);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010055);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010056);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010057);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010058);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00010059);

		repeat (100) @(posedge clock);
		$display("#############################################");
		$display("  Page Write Command Address: 0x200          ");
		$display("#############################################");
		// WEN COMMAND
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h0,2'b00,2'b00,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
		 // Page Programing
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'hF0,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020000);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020001);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020002);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020003);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020004);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020005);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020006);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020007);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020008);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020009);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020010);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020011);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020012);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020013);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020014);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020015);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020016);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020017);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020018);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020019);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020020);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020021);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020022);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020023);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020024);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020025);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020026);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020027);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020028);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020029);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020030);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020031);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020032);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020033);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020034);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020035);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020036);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020037);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020038);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020039);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020040);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020041);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020042);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020043);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020044);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020045);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020046);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020047);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020048);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020049);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020050);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020051);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020052);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020053);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020054);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020055);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020056);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020057);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020058);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h00020059);

		// RDSR
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h4,2'b00,2'b00,P_FSM_CR,8'h00,8'h05});
		read_data = 32'hFFFF_FFFF;
		while (read_data[1:0] == 2'b11) begin
		    wb_user_core_read(`QSPIM_IMEM_RDATA,read_data);
		    repeat (10) @(posedge clock);
		 end

		$display("#############################################");
		$display("  Page Read through Direct Access            ");
		$display("#############################################");

		wb_user_core_read_check(32'h00000200,read_data,32'h00020000);
		wb_user_core_read_check(32'h00000204,read_data,32'h00020001);
		wb_user_core_read_check(32'h00000208,read_data,32'h00020002);
		wb_user_core_read_check(32'h0000020C,read_data,32'h00020003);
		wb_user_core_read_check(32'h00000210,read_data,32'h00020004);
		wb_user_core_read_check(32'h00000214,read_data,32'h00020005);
		wb_user_core_read_check(32'h00000218,read_data,32'h00020006);
		wb_user_core_read_check(32'h0000021C,read_data,32'h00020007);
		wb_user_core_read_check(32'h00000220,read_data,32'h00020008);
		wb_user_core_read_check(32'h00000224,read_data,32'h00020009);
		wb_user_core_read_check(32'h00000228,read_data,32'h00020010);
		wb_user_core_read_check(32'h0000022C,read_data,32'h00020011);
		wb_user_core_read_check(32'h00000230,read_data,32'h00020012);
		wb_user_core_read_check(32'h00000234,read_data,32'h00020013);
		wb_user_core_read_check(32'h00000238,read_data,32'h00020014);
		wb_user_core_read_check(32'h0000023C,read_data,32'h00020015);
		wb_user_core_read_check(32'h00000240,read_data,32'h00020016);
		wb_user_core_read_check(32'h00000244,read_data,32'h00020017);
		wb_user_core_read_check(32'h00000248,read_data,32'h00020018);
		wb_user_core_read_check(32'h0000024C,read_data,32'h00020019);
		wb_user_core_read_check(32'h00000250,read_data,32'h00020020);
		wb_user_core_read_check(32'h00000254,read_data,32'h00020021);
		wb_user_core_read_check(32'h00000258,read_data,32'h00020022);
		wb_user_core_read_check(32'h0000025C,read_data,32'h00020023);
		wb_user_core_read_check(32'h00000260,read_data,32'h00020024);
		wb_user_core_read_check(32'h00000264,read_data,32'h00020025);
		wb_user_core_read_check(32'h00000268,read_data,32'h00020026);
		wb_user_core_read_check(32'h0000026C,read_data,32'h00020027);
		wb_user_core_read_check(32'h00000270,read_data,32'h00020028);
		wb_user_core_read_check(32'h00000274,read_data,32'h00020029);
		wb_user_core_read_check(32'h00000278,read_data,32'h00020030);
		wb_user_core_read_check(32'h0000027C,read_data,32'h00020031);
		wb_user_core_read_check(32'h00000280,read_data,32'h00020032);
		wb_user_core_read_check(32'h00000284,read_data,32'h00020033);
		wb_user_core_read_check(32'h00000288,read_data,32'h00020034);
		wb_user_core_read_check(32'h0000028C,read_data,32'h00020035);
		wb_user_core_read_check(32'h00000290,read_data,32'h00020036);
		wb_user_core_read_check(32'h00000294,read_data,32'h00020037);
		wb_user_core_read_check(32'h00000298,read_data,32'h00020038);
		wb_user_core_read_check(32'h0000029C,read_data,32'h00020039);
		wb_user_core_read_check(32'h000002A0,read_data,32'h00020040);
		wb_user_core_read_check(32'h000002A4,read_data,32'h00020041);
		wb_user_core_read_check(32'h000002A8,read_data,32'h00020042);
		wb_user_core_read_check(32'h000002AC,read_data,32'h00020043);
		wb_user_core_read_check(32'h000002B0,read_data,32'h00020044);
		wb_user_core_read_check(32'h000002B4,read_data,32'h00020045);
		wb_user_core_read_check(32'h000002B8,read_data,32'h00020046);
		wb_user_core_read_check(32'h000002BC,read_data,32'h00020047);
		wb_user_core_read_check(32'h000002C0,read_data,32'h00020048);
		wb_user_core_read_check(32'h000002C4,read_data,32'h00020049);
		wb_user_core_read_check(32'h000002C8,read_data,32'h00020050);
		wb_user_core_read_check(32'h000002CC,read_data,32'h00020051);
		wb_user_core_read_check(32'h000002D0,read_data,32'h00020052);
		wb_user_core_read_check(32'h000002D4,read_data,32'h00020053);
		wb_user_core_read_check(32'h000002D8,read_data,32'h00020054);
		wb_user_core_read_check(32'h000002DC,read_data,32'h00020055);
		wb_user_core_read_check(32'h000002E0,read_data,32'h00020056);
		wb_user_core_read_check(32'h000002E4,read_data,32'h00020057);
		wb_user_core_read_check(32'h000002E8,read_data,32'h00020058);
		wb_user_core_read_check(32'h000002EC,read_data,32'h00020059);

		repeat (10) @(posedge clock);
		$display("#############################################");
		$display("  Page Read through Indirect Access           ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0001,P_MODE_SWITCH_AT_ADDR,P_QUAD,P_SINGLE,4'b0001});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'hF0,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000200);

		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020000);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020001);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020002);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020003);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020004);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020005);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020006);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020007);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020008);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020009);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020010);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020011);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020012);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020013);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020014);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020015);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020016);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020017);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020018);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020019);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020020);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020021);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020022);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020023);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020024);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020025);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020026);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020027);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020028);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020029);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020030);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020031);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020032);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020033);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020034);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020035);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020036);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020037);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020038);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020039);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020040);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020041);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020042);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020043);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020044);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020045);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020046);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020047);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020048);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020049);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020050);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020051);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020052);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020053);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020054);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020055);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020056);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020057);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020058);
		wb_user_core_read_check(`QSPIM_IMEM_RDATA,read_data,32'h00020059);

		$display("#############################################");
		$display("  Page Read through Direct Burst Access       ");
		$display("#############################################");

		data_array[10'h0]    = 32'h00010000;
		data_array[10'h1]    = 32'h00010001;
		data_array[10'h2]    = 32'h00010002;
		data_array[10'h3]    = 32'h00010003;
		data_array[10'h4]    = 32'h00010004;
		data_array[10'h5]    = 32'h00010005;
		data_array[10'h6]    = 32'h00010006;
		data_array[10'h7]    = 32'h00010007;
		data_array[10'h8]    = 32'h00010008;
		data_array[10'h9]    = 32'h00010009;
		data_array[10'hA]    = 32'h00010010;
		data_array[10'hB]    = 32'h00010011;
		data_array[10'hC]    = 32'h00010012;
		data_array[10'hD]    = 32'h00010013;
		data_array[10'hE]    = 32'h00010014;
		data_array[10'hF]    = 32'h00010015;
		data_array[10'h10]   = 32'h00010016;
		data_array[10'h11]   = 32'h00010017;
		data_array[10'h12]   = 32'h00010018;
		data_array[10'h13]   = 32'h00010019;
		data_array[10'h14]   = 32'h00010020;
		data_array[10'h15]   = 32'h00010021;
		data_array[10'h16]   = 32'h00010022;
		data_array[10'h17]   = 32'h00010023;
		data_array[10'h18]   = 32'h00010024;
		data_array[10'h19]   = 32'h00010025;
		data_array[10'h1A]   = 32'h00010026;
		data_array[10'h1B]   = 32'h00010027;
		data_array[10'h1C]   = 32'h00010028;
		data_array[10'h1D]   = 32'h00010029;
		data_array[10'h1E]   = 32'h00010030;
		data_array[10'h1F]   = 32'h00010031;
		wb_user_core_bread_check(32'h00000000,10'h20);
		data_array[10'h0]    = 32'h00010032;
		data_array[10'h1]    = 32'h00010033;
		data_array[10'h2]    = 32'h00010034;
		data_array[10'h3]    = 32'h00010035;
		data_array[10'h4]    = 32'h00010036;
		data_array[10'h5]    = 32'h00010037;
		data_array[10'h6]    = 32'h00010038;
		data_array[10'h7]    = 32'h00010039;
		data_array[10'h8]    = 32'h00010040;
		data_array[10'h9]    = 32'h00010041;
		data_array[10'hA]    = 32'h00010042;
		data_array[10'hB]    = 32'h00010043;
		data_array[10'hC]    = 32'h00010044;
		data_array[10'hD]    = 32'h00010045;
		data_array[10'hE]    = 32'h00010046;
		data_array[10'hF]    = 32'h00010047;
		data_array[10'h10]   = 32'h00010048;
		data_array[10'h11]   = 32'h00010049;
		data_array[10'h12]   = 32'h00010050;
		data_array[10'h13]   = 32'h00010051;
		data_array[10'h14]   = 32'h00010052;
		data_array[10'h15]   = 32'h00010053;
		data_array[10'h16]   = 32'h00010054;
		data_array[10'h17]   = 32'h00010055;
		data_array[10'h18]   = 32'h00010056;
		data_array[10'h19]   = 32'h00010057;
		data_array[10'h1A]   = 32'h00010058;
		data_array[10'h1B]   = 32'h00010059;
		wb_user_core_bread_check(32'h00000080,10'h1C);

		data_array[10'h0]    = 32'h00020000;
		data_array[10'h1]    = 32'h00020001;
		data_array[10'h2]    = 32'h00020002;
		data_array[10'h3]    = 32'h00020003;
		data_array[10'h4]    = 32'h00020004;
		data_array[10'h5]    = 32'h00020005;
		data_array[10'h6]    = 32'h00020006;
		data_array[10'h7]    = 32'h00020007;
		data_array[10'h8]    = 32'h00020008;
		data_array[10'h9]    = 32'h00020009;
		data_array[10'hA]    = 32'h00020010;
		data_array[10'hB]    = 32'h00020011;
		data_array[10'hC]    = 32'h00020012;
		data_array[10'hD]    = 32'h00020013;
		data_array[10'hE]    = 32'h00020014;
		data_array[10'hF]    = 32'h00020015;
		data_array[10'h10]   = 32'h00020016;
		data_array[10'h11]   = 32'h00020017;
		data_array[10'h12]   = 32'h00020018;
		data_array[10'h13]   = 32'h00020019;
		data_array[10'h14]   = 32'h00020020;
		data_array[10'h15]   = 32'h00020021;
		data_array[10'h16]   = 32'h00020022;
		data_array[10'h17]   = 32'h00020023;
		data_array[10'h18]   = 32'h00020024;
		data_array[10'h19]   = 32'h00020025;
		data_array[10'h1A]   = 32'h00020026;
		data_array[10'h1B]   = 32'h00020027;
		data_array[10'h1C]   = 32'h00020028;
		data_array[10'h1D]   = 32'h00020029;
		data_array[10'h1E]   = 32'h00020030;
		data_array[10'h1F]   = 32'h00020031;
		wb_user_core_bread_check(32'h00000200,10'h20);
		data_array[10'h0]   = 32'h00020032;
		data_array[10'h1]   = 32'h00020033;
		data_array[10'h2]   = 32'h00020034;
		data_array[10'h3]   = 32'h00020035;
		data_array[10'h4]   = 32'h00020036;
		data_array[10'h5]   = 32'h00020037;
		data_array[10'h6]   = 32'h00020038;
		data_array[10'h7]   = 32'h00020039;
		data_array[10'h8]   = 32'h00020040;
		data_array[10'h9]   = 32'h00020041;
		data_array[10'hA]   = 32'h00020042;
		data_array[10'hB]   = 32'h00020043;
		data_array[10'hC]   = 32'h00020044;
		data_array[10'hD]   = 32'h00020045;
		data_array[10'hE]   = 32'h00020046;
		data_array[10'hF]   = 32'h00020047;
		data_array[10'h10]   = 32'h00020048;
		data_array[10'h11]   = 32'h00020049;
		data_array[10'h12]   = 32'h00020050;
		data_array[10'h13]   = 32'h00020051;
		data_array[10'h14]   = 32'h00020052;
		data_array[10'h15]   = 32'h00020053;
		data_array[10'h16]   = 32'h00020054;
		data_array[10'h17]   = 32'h00020055;
		data_array[10'h18]   = 32'h00020056;
		data_array[10'h19]   = 32'h00020057;
		data_array[10'h1A]   = 32'h00020058;
		data_array[10'h1B]   = 32'h00020059;
		wb_user_core_bread_check(32'h00000280,10'h1C);

		$dumpon;
		$display("#############################################");
		$display("  clock config validation in SSPI Mode       ");
		$display("#############################################");
		for(cnt = 4; cnt < 16; cnt = cnt+2) begin
		    $display("SPI Testing with Clock Div value: %x ",cnt);
		    wb_user_core_write(`QSPIM_GLBL_CTRL,{16'h0, cnt[7:0],4'h4,2'b01,2'b01});

		    wb_user_core_write(`QSPIM_DMEM_G1_WR_CTRL,{P_FSM_CAW, 4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h02});
		    wb_user_core_write(`QSPIM_DMEM_G1_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h03});

		    data_array[10'h0]    = 32'h00112233;
		    data_array[10'h1]    = 32'h44556677;
		    data_array[10'h2]    = 32'h8899AABB;
		    data_array[10'h3]    = 32'hCCDDEEFF;
		    data_array[10'h4]    = 32'h00001111;
		    data_array[10'h5]    = 32'h22223333;
		    data_array[10'h6]    = 32'h44445555;
		    data_array[10'h7]    = 32'h66667777;
		    wb_user_core_bwrite(32'h08000000,10'h8);
		    wb_user_core_bread_check(32'h08000000,10'h8);
		 end
		$display("#############################################");
		$display("  Testing SRAM Byte enable Write Access       ");
		$display("#############################################");
		    wb_user_core_write(`QSPIM_GLBL_CTRL,{16'h0, 8'h4,4'h4,2'b01,2'b01});
		    wb_user_core_write(`QSPIM_DMEM_G1_WR_CTRL,{P_FSM_CAW, 4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h02});
		    wb_user_core_write(`QSPIM_DMEM_G1_RD_CTRL,{P_FSM_CADR,4'b0000,2'b10,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,8'h00,8'h03});

		    wb_user_core_be_write(32'h08000100,32'h01234567,4'hF);
		    wb_user_core_be_write(32'h08000200,32'h00112233,4'h1);
		    wb_user_core_be_write(32'h08000200,32'h00112233,4'h2);
		    wb_user_core_be_write(32'h08000200,32'h00112233,4'h4);
		    wb_user_core_be_write(32'h08000200,32'h00112233,4'h8);
		    wb_user_core_be_write(32'h08000204,32'h44556677,4'h3);
		    wb_user_core_be_write(32'h08000204,32'h44556677,4'hC);
		    wb_user_core_be_write(32'h08000208,32'h8899AABB,4'h7);
		    wb_user_core_be_write(32'h08000208,32'h8899AABB,4'h8);
		    wb_user_core_be_write(32'h0800020C,32'hCCDDEEFF,4'h1);
		    wb_user_core_be_write(32'h0800020C,32'hCCDDEEFF,4'hE);
		    wb_user_core_be_write(32'h08000210,32'h01234567,4'h1);
		    wb_user_core_be_write(32'h08000210,32'h01234567,4'h6);
		    wb_user_core_be_write(32'h08000210,32'h01234567,4'h8);
		    wb_user_core_be_write(32'h08000104,32'h89ABCDEF,4'hF);

		    wb_user_core_read_check(32'h08000100,read_data,32'h01234567);
		    wb_user_core_read_check(32'h08000200,read_data,32'h00112233);
		    wb_user_core_read_check(32'h08000204,read_data,32'h44556677);
		    wb_user_core_read_check(32'h08000208,read_data,32'h8899AABB);
		    wb_user_core_read_check(32'h0800020C,read_data,32'hCCDDEEFF);
		    wb_user_core_read_check(32'h08000210,read_data,32'h01234567);
		    wb_user_core_read_check(32'h08000104,read_data,32'h89ABCDEF);


	        /**
		//-----------------------------------------------------------------
		//  FRAM TESTING SEQUENCE START HERE
		//
		// Important: Config the FRAM to QURD MODE to make the SRAM
		// RESET pin to behave like to data pin. Other wise
		// Toggling on Reset pin afect the other memory trasaction
		// in the bus
		// Set QUAD bit CR2_NV[6]=1 
		$display("#############################################");
		$display("  CONFIG CS#1 FRAM to QURD MODE              ");
		$display("#############################################");
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_SINGLE,P_SINGLE,4'b0010});
		// SEND WREN Command to enable write access 
		// <COMMAND PHASE> ONLY
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);

	        // CR1:<WRAR:0x71> <Address:0x000002> <Data:0x40> to Dummy=2
		// Dummy=2 means 2 half SCK cycles
	
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,P_FSM_CAW,8'h00,8'h71});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000002);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h22);

		// <COMMAND PHASE> ONLY
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,4'b0000,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);
	        // CR2: <WRAR:0x71> <Address:0x000003> <Data:0x40> to enable qurd mode
		// Setting QPI  = CR2_NV[6];
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,P_FSM_CAW,8'h00,8'h71});
		wb_user_core_write(`QSPIM_IMEM_ADDR,32'h00000003);
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h40);

		$display("#############################################");
		$display("Testing CS[1]                               ");
		$display("Testing Direct SPI Memory Read              ");
		$display(" SPI Mode: QIOR (Qard 4 bit)                ");
		$display("Prefetch : 1DW, OPCODE:READ(0xEB)           ");
		$display("SEQ: Command->Address-> MODE->Dummy->Read Data");
		$display("#############################################");
		// Reading Data store from flash1 hex
		// QDDR Config
		wb_user_core_write(`QSPIM_DMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0001});
		wb_user_core_write(`QSPIM_DMEM_CTRL2,{8'h04,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		data_array[0]   = 32'h03020100;
		data_array[1]   = 32'h07060504;
		data_array[2]   = 32'h0b0a0908;
		data_array[3]   = 32'h0f0e0d0c;
		data_array[4]   = 32'h13121110;
		data_array[5]   = 32'h17161514;
		data_array[6]   = 32'h1b1a1918;
		data_array[7]   = 32'h1f1e1d1c;
		data_array[8]   = 32'h23222120;
		data_array[9]   = 32'h27262524;
		data_array[10]  = 32'h2b2a2928;
		data_array[11]  = 32'h2f2e2d2c;
		data_array[12]  = 32'h33323130;
		data_array[13]  = 32'h37363534;
		data_array[14]  = 32'h3b3a3938;
		data_array[15]  = 32'h3f3e3d3c;

		wb_user_core_bread_check(32'h01000000,10'h10);
	        	
		$display("#############################################");
		$display("Testing CS[1]                               ");
		$display("Testing Direct SPI Memory Write              ");
		$display(" SPI Mode: QIOW (Qard 4 bit)                ");
		$display("Prefetch : 1DW, OPCODE:READ(0xD2)           ");
		$display("SEQ: Command->Address-> MODE->Write Data");
		$display("#############################################");
		
		// SEND WREN Command to enable write access 
		// <COMMAND PHASE> ONLY
		// Note: Since due to previous sequence device is already in
		// QUAD mode, So we need to do WREN in QUAD MODE
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0010});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);

		wb_user_core_write(`QSPIM_DMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0001});
		wb_user_core_write(`QSPIM_DMEM_CTRL2,{8'h04,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});
		// WRITE
		data_array[0]   = 32'h00112233;
		data_array[1]   = 32'h11223344;
		data_array[2]   = 32'h22334455;
		data_array[3]   = 32'h33445566;
		data_array[4]   = 32'h44556677;
		data_array[5]   = 32'h55667788;
		data_array[6]   = 32'h66778899;
		data_array[7]   = 32'h778899AA;
		data_array[8]   = 32'h8899AABB;
		data_array[9]   = 32'h99AABBCC;
		data_array[10]  = 32'hAABBCCDD;
		data_array[11]  = 32'hBBCCDDEE;
		data_array[12]  = 32'hCCDDEEFF;
		data_array[13]  = 32'hDDEEFF00;
		data_array[14]  = 32'hEEFF0011;
		data_array[15]  = 32'hFF001122;
		wb_user_core_bwrite(32'h01000000,10'h10);
	
		wb_user_core_write(`QSPIM_IMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0010});
		wb_user_core_write(`QSPIM_IMEM_CTRL2,{8'h1,2'b00,2'b10,P_FSM_C,8'h00,8'h06});
		wb_user_core_write(`QSPIM_IMEM_WDATA,32'h0);

		wb_user_core_write(`QSPIM_DMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0001});
		wb_user_core_write(`QSPIM_DMEM_CTRL2,{8'h04,2'b00,2'b10,P_FSM_CAW,8'h00,8'h02});

		data_array[0]   = 32'h12345678;
		data_array[1]   = 32'h23456789;
		data_array[2]   = 32'h3456789A;
		data_array[3]   = 32'h456789AB;
		data_array[4]   = 32'h56789ABC;
		data_array[5]   = 32'h6789ABCD;
		data_array[6]   = 32'h789ABCDE;
		data_array[7]   = 32'h89ABCDEF;
		data_array[8]   = 32'h9ABCDEF0;
		data_array[9]   = 32'hABCDEF01;
		data_array[10]  = 32'hBCDEF012;
		data_array[11]  = 32'hCDEF0123;
		data_array[12]  = 32'hDEF01234;
		data_array[13]  = 32'hEF012345;
		data_array[14]  = 32'hF0123456;
		data_array[15]  = 32'h01234567;
		wb_user_core_bwrite(32'h01000200,10'h10);
		// READ BACK
		wb_user_core_write(`QSPIM_DMEM_CTRL1,{16'h0,1'b0,1'b0,4'b0000,P_MODE_SWITCH_IDLE,P_QUAD,P_QUAD,4'b0001});
		wb_user_core_write(`QSPIM_DMEM_CTRL2,{8'h04,2'b00,2'b10,P_FSM_CAMDR,8'h00,8'hEB});
		data_array[0]   = 32'h00112233;
		data_array[1]   = 32'h11223344;
		data_array[2]   = 32'h22334455;
		data_array[3]   = 32'h33445566;
		data_array[4]   = 32'h44556677;
		data_array[5]   = 32'h55667788;
		data_array[6]   = 32'h66778899;
		data_array[7]   = 32'h778899AA;
		data_array[8]   = 32'h8899AABB;
		data_array[9]   = 32'h99AABBCC;
		data_array[10]  = 32'hAABBCCDD;
		data_array[11]  = 32'hBBCCDDEE;
		data_array[12]  = 32'hCCDDEEFF;
		data_array[13]  = 32'hDDEEFF00;
		data_array[14]  = 32'hEEFF0011;
		data_array[15]  = 32'hFF001122;

		wb_user_core_bread_check(32'h01000000,10'h10);
	
		data_array[0]   = 32'h12345678;
		data_array[1]   = 32'h23456789;
		data_array[2]   = 32'h3456789A;
		data_array[3]   = 32'h456789AB;
		data_array[4]   = 32'h56789ABC;
		data_array[5]   = 32'h6789ABCD;
		data_array[6]   = 32'h789ABCDE;
		data_array[7]   = 32'h89ABCDEF;
		data_array[8]   = 32'h9ABCDEF0;
		data_array[9]   = 32'hABCDEF01;
		data_array[10]  = 32'hBCDEF012;
		data_array[11]  = 32'hCDEF0123;
		data_array[12]  = 32'hDEF01234;
		data_array[13]  = 32'hEF012345;
		data_array[14]  = 32'hF0123456;
		data_array[15]  = 32'h01234567;
		
		wb_user_core_bread_check(32'h01000200,10'h10);
		**/

		repeat (1000) @(posedge clock);
			// $display("+1000 cycles");

          	if(test_fail == 0) begin
		   `ifdef GL
	    	       $display("Monitor: SPI Master Mode (GL) Passed");
		   `else
		       $display("Monitor: SPI Master Mode (RTL) Passed");
		   `endif
	        end else begin
		    `ifdef GL
	    	        $display("Monitor: SPI Master Mode (GL) Failed");
		    `else
		        $display("Monitor: SPI Master Mode (RTL) Failed");
		    `endif
		 end
	    	$display("###################################################");
	    $finish;
	end
wire USER_VDD1V8 = 1'b1;
wire VSS = 1'b0;

qspim_top u_top(
   `ifdef USE_POWER_PINS
    .vccd1       (1'b1),
    .vssd1       (1'b0),
    `endif
    .mclk        (clock),  // System clock
    .rst_n       (!wb_rst_i),  // Regular Reset signal
    .cfg_fast_sim (1'b1),

    .cfg_init_bypass (1'b0),
    .strap_flash (strap_flash),
    .strap_pre_sram  (strap_pre_sram),
    .strap_sram  (strap_sram),

    .cfg_cska_sp_co  ('h0), // spi clock skew adjust
    .cfg_cska_spi    ('h0),
    .wbd_clk_int     (1'b0),
    .wbd_clk_spi     (),
    .spi_debug       (),


    .wbd_stb_i   (wbd_ext_stb_i),  // strobe/request
    .wbd_adr_i   (wbd_ext_adr_i),  // address
    .wbd_we_i    (wbd_ext_we_i),  // write
    .wbd_dat_i   (wbd_ext_dat_i),  // data output
    .wbd_sel_i   (wbd_ext_sel_i),  // byte enable
    .wbd_bl_i    (wbd_ext_bl_i),            // Busrt Count
    .wbd_bry_i   (wbd_ext_bry_i),

    .wbd_dat_o   (wbd_ext_dat_o),  // data input
    .wbd_ack_o   (wbd_ext_ack_o),  // acknowlegement
    .wbd_lack_o  (wbd_ext_lack_o),  // Last Ack

 
    // IOs
    .spi_sdi     (spi_sdi),
    .spi_clk     (flash_clk),
    .spi_csn     (spi_csb),
    .spi_sdo     (spi_sdo),
    .spi_oen     (spi_oeb)


);

//------------------------------------------------------
//  Integrate the Serial flash with qurd support to
//  user core using the gpio pads
//  ----------------------------------------------------

   // Modeling the Pad Delay
   assign #1 io_oeb = spi_oeb;
   wire #1 io_d0 = spi_sdo[0];
   wire #1 io_d1 = spi_sdo[1];
   wire #1 io_d2 = spi_sdo[2];
   wire #1 io_d3 = spi_sdo[3];
   assign flash_io0 = (io_oeb[0]== 1'b0) ? io_d0 : 1'bz;
   assign flash_io1 = (io_oeb[1]== 1'b0) ? io_d1 : 1'bz;
   assign flash_io2 = (io_oeb[2]== 1'b0) ? io_d2 : 1'bz;
   assign flash_io3 = (io_oeb[3]== 1'b0) ? io_d3 : 1'bz;

   assign #1 spi_sdi[0] = flash_io0;
   assign #1 spi_sdi[1] = flash_io1;
   assign #1 spi_sdi[2] = flash_io2;
   assign #1 spi_sdi[3] = flash_io3;

   /***

   // Quard flash
	spiflash #(
		.FILENAME("input.hex")
	) u_user_spiflash (
		.csb(flash_csb),
		.clk(flash_clk),
		.io0(flash_io0),
		.io1(flash_io1),
		.io2(flash_io2),
		.io3(flash_io3)	
	);

	**/
       s25fl256s #(.mem_file_name("flash0.hex"),
	           .otp_file_name("none"),
	           .TimingModel("S25FL512SAGMFI010_F_30pF")) u_spi_flash0_256mb
           (
               // Data Inputs/Outputs
           .SI      (flash_io0),
           .SO      (flash_io1),
           // Controls
           .SCK     (flash_clk),
           .CSNeg   (spi_csb[0]),
           .WPNeg   (flash_io2),
           .HOLDNeg (flash_io3),
           .RSTNeg  (!wb_rst_i)
       
       );

   spiram #(.mem_file_name("flash1.hex"))
	u_sfram (
         // Data Inputs/Outputs
           .io0     (flash_io0),
           .io1     (flash_io1),
           // Controls
           .clk    (flash_clk),
           .csb    (spi_csb[2]),
           .io2    (flash_io2),
           .io3    (flash_io3)
    );





task wb_user_core_write;
input [31:0] address;
input [31:0] data;
begin
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h1;  // write
  wbd_ext_dat_i =data;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_bl_i  = 'h1;
  wbd_ext_bry_i = 1'b1;
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  $display("STATUS: WB USER ACCESS WRITE Address : 0x%x, Data : 0x%x",address,data);
  repeat (2) @(posedge clock);
end
endtask



task MapData;
input [31:0] DataIn;
input [3:0]  Sel;
output [31:0] DataOut;
begin
   case(Sel)
   4'b0001 : DataOut = {24'hxxxxxx,DataIn[7:0]};
   4'b0010 : DataOut = {16'hxxxx,DataIn[15:8],8'hxx};
   4'b0100 : DataOut = {8'hxx,DataIn[23:16],16'hxxxx};
   4'b1000 : DataOut = {DataIn[31:24],24'hxxxxxx};
   4'b0011 : DataOut = {16'hxxxx,DataIn[15:0]};
   4'b0110 : DataOut = {8'hxx,DataIn[23:8],8'hxx};
   4'b1100 : DataOut = {DataIn[31:16],16'hxxxx};
   4'b0111 : DataOut = {8'hxx,DataIn[23:0]};
   4'b1110 : DataOut = {DataIn[31:8],8'hxx};
   4'b1111 : DataOut = {DataIn[31:0]};
   default : DataOut = 32'hxxxxxxxx;
   endcase

end
endtask


// Write with Byte enable
task wb_user_core_be_write;
input [31:0] address;
input [31:0] data;
input [3:0]  sel;
begin
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_adr_i = address;  // address
  wbd_ext_we_i  = 'h1;  // write
  MapData(data,sel,wbd_ext_dat_i);  // data output
  wbd_ext_sel_i = sel;  // byte enable
  wbd_ext_bl_i  = 'h1;
  wbd_ext_bry_i =  1'b1;
  wbd_ext_cyc_i = 'h1;  // strobe/request
  wbd_ext_stb_i = 'h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  $display("STATUS: WB USER ACCESS WRITE Address : 0x%x, Data : 0x%x Byte Enable: 0x%x",wbd_ext_adr_i,wbd_ext_dat_i,wbd_ext_sel_i);
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  repeat (2) @(posedge clock);
end
endtask

// Burst Write
task wb_user_core_bwrite;
input [31:0] address;
input [11:0] burst;
reg   [11:0] cnt;
begin
   cnt = 0;
   while(burst != cnt) begin
      repeat (1) @(posedge clock);
      #1;
      wbd_ext_adr_i =address;  // address
      wbd_ext_we_i  ='h1;      // write
      wbd_ext_bl_i  =burst;    // Burst Count
      wbd_ext_bry_i = 1'b1;
      wbd_ext_dat_i =data_array[cnt];  // data output
      wbd_ext_sel_i ='hF;  // byte enable
      wbd_ext_cyc_i ='h1;  // strobe/request
      wbd_ext_stb_i ='h1;  // strobe/request
      wait(wbd_ext_ack_o == 1);
      $display("STATUS: WB USER ACCESS WRITE Address : 0x%x, Data : 0x%x",address+(cnt*4),data_array[cnt]);
      cnt = cnt + 1;
  end
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  repeat (2) @(posedge clock);
end
endtask

task  wb_user_core_read;
input [31:0] address;
output [31:0] data;
reg    [31:0] data;
begin
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='0;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_bl_i  = 'h1;
  wbd_ext_bry_i = 1'b1;
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  data  = wbd_ext_dat_o;  
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  $display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address,data);
  repeat (2) @(posedge clock);
end
endtask

task  wb_user_core_read_check;
input [31:0] address;
output [31:0] data;
input [31:0] cmp_data;
reg    [31:0] data;
begin
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='0;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_bl_i  = 'h1;
  wbd_ext_bry_i = 1'b1;
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  data  = wbd_ext_dat_o;  
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  if(data !== cmp_data) begin
     $display("ERROR : WB USER ACCESS READ  Address : 0x%x, Exd: 0x%x Rxd: 0x%x ",address,cmp_data,data);
     tb_top.test_fail = 1;
  end else begin
     $display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address,data);
  end
  repeat (2) @(posedge clock);
end
endtask

// Burst Read Check
task  wb_user_core_bread_check;
input [31:0] address;
input [9:0]  burst;
reg   [9:0]  cnt;
reg    [31:0] data;
begin
  cnt = 0;
  while(cnt != burst) begin
      repeat (1) @(posedge clock);
      #1;
      wbd_ext_adr_i =address;  // address
      wbd_ext_we_i  ='h0;  // write
      wbd_ext_dat_i ='0;  // data output
      wbd_ext_sel_i ='hF;  // byte enable
      wbd_ext_bl_i  =burst;    // Burst Count
      wbd_ext_bry_i = 1'b1;
      wbd_ext_cyc_i ='h1;  // strobe/request
      wbd_ext_stb_i ='h1;  // strobe/request
      wait(wbd_ext_ack_o == 1);
      data  = wbd_ext_dat_o;  
      if(data !== data_array[cnt]) begin
         $display("ERROR : WB USER ACCESS READ  Address : 0x%x, Exd: 0x%x Rxd: 0x%x ",address+(cnt*4),data_array[cnt],data);
         tb_top.test_fail = 1;
      end else begin
         $display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address+(cnt*4),data);
      end
      cnt = cnt + 1;
  end
  repeat (1) @(posedge clock);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  repeat (2) @(posedge clock);
end
endtask


endmodule
`default_nettype wire
