//////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText: 2021 , Dinesh Annayya                          
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
// SPDX-FileContributor: Created by Dinesh Annayya <dinesh.annayya@gmail.com>
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                                   ////
////  SPI Master Top Module                                                                            ////
////                                                                                                   ////
////  This file is part of the YIFive cores project                                                    ////
////  https://github.com/dineshannayya/yifive_r0.git                                                   ////
////  http://www.opencores.org/cores/yifive/                                                           ////
////                                                                                                   ////
////  Description                                                                                      ////
////     SPI Master Top module                                                                         ////
////     There are two seperate Data path managed here                                                 ////
////     with seperate command and response memory                                                     ////
////     Master-0 : This is targetted for CORE IMEM request                                            ////
////                and expect only Read access                                                        ////
////     Master-1: This is targetted to CORE DMEM or                                                   ////
////               Indirect Memory access, Both Write and Read                                         ////
////               accesss are supported.                                                              ////
////               Upto 255 Byte Read/Write Burst supported                                            ////
////    Limitation:                                                                                    ////
////       1.  Write/Read FIFO Abort case not managed M1 port,                                         ////
////           expect user to clearly close the busrt request                                          ////
////       2.  Wishbone Request abort not yet supported.                                               ////
////       3.  Write access through M0 Port not supported                                              ////
////       4.  When Pre fetch feature used and both port m0 and                                        ////
////           m1 used, user need to make sure that data pre fetch                                     ////
////           count is withing 8DW, less Read path can hang due                                       ////
////           to response FIFO full from one master port                                              ////
////                                                                                                   ////
////      Assumed Default Direct Address CS Mapping                                                    ////
////      0x0000_0000 to 0x03FF_FFFF - CS-0 (64MB)                                                     ////
////      0x0400_0000 to 0x07FF_FFFF - CS-1 (64MB)                                                     ////
////      0x0800_0000 to 0x0BFF_FFFF - CS-2 (64MB)                                                     ////
////      0x0C00_0000 to 0x0FFF_FFFF - CS-3 (64MB)                                                     ////
////  To Do:                                                                                           ////
////    1. Add support for WishBone request timout                                                     ////
////    2. Add Pre-fetch feature for M0 Port                                                           ////
////                                                                                                   ////
////  Author(s):                                                                                       ////
////      - Dinesh Annayya, dinesh.annayya@gmail.com                                                   ////
////                                                                                                   ////
////  Revision :                                                                                       ////
////     0.0  -  June 8, 2021                                                                          //// 
////     0.1  - June 25, 2021                                                                          ////
////            Pad logic is brought inside the block to avoid                                         ////
////            logic at digital core level for caravel project                                        ////
////     0.2  - July 6, 2021                                                                           ////
////            Added Hold fix cell for SPI data out signal to                                         ////
////            met interface hold                                                                     ////
////     0.3  - July 13, 2021                                                                          ////
////            Data Prefetch feature added in M0 port, If Only                                        ////
////            M0 Read used, then Prefetch read can be 255 Byte,                                      ////
////            But if the Both M0 and M1 read access enabled,                                         ////
////            then user need to make sure that M0 Prefetch is                                        ////
////            with in 8DW or 32 Byte, else there is chance                                           ////
////            data path can hang due to response FIFO full due                                       ////
////            to partial reading of data                                                             ////
////     0.4  -  July 26, 2021                                                                         ////
////             QDDR (0xED) supported is added                                                        ////
////     0.5  -  Nov 6, 2021                                                                           ////
////             Clock Skew Moves inside the block                                                     ////
////     0.6  -  Jan 13, 2022                                                                          ////
////             All CS# brougt out from block                                                         ////
////     0.7  -  Jan 14, 2022                                                                          ////
////             1. Changed Dummy to 2 to 4 bit                                                        ////
////             2. CS Address Map and Mask Reg added for Direct                                       ////
////                access mode                                                                        ////
////             3. Seperated spi init and spi final mode config                                       ////
////                cfg_m*_spi_imode & cfg_m*_spi_fmode                                                ////
////                *_imode loaded at init place of CS Assertion                                       ////
////                & *_fmode loaded on switching place                                                //// 
////     1.0  - Jan 25, 2022, Dinesh A                                                                 ////
////             Direct memory Burst Mode support is added                                             ////
////     1.1  - Feb 7, 2022, Dinesh A                                                                  ////
////             CS0/CS1 will have Config support for FLASH SPI                                        ////
////             CS2/CS3 will have config support SRAM SPI                                             ////
////     1.2  - Feb 19, 2022, Dinesh A                                                                 ////
////        A. Bug fix in spi rise and fall pulse relation w.r.t                                       ////
////           spi_clk. Note: Previous version work only with                                          ////
////           spi clock config = 0x2                                                                  ////
////        B. spi_oen generation fix for different spi mode                                           ////
////        C. spi_csn de-assertion fix for different spi clk div                                      ////
////     1.3  - Mar 01, 2022, Dinesh A                                                                 ////
////            m1*res*fifo * m1*cmd*fifo status added into                                            ////
////            global config Register                                                                 ////
////     1.4  - Aug 29, 2022, Dinesh A                                                                 ////
////         A. Strap based CS#0 and CS#2 reset value change                                           ////
////            Added                                                                                  ////
////         B. Initialization bypass added to take care of case                                       ////
////            for when there is only local reboot                                                    ////    
////     1.5 -  Sept 2, 2022, Dinesh A                                                                 ////
////     A. Add previous power on strap from SRAM flash to take care of mode switching                 ////
////        1. If the current & previous sram strap is Single, then bypass mode switching              ////
////        2. If the current=Single and Previous: Quad, then switch mode by command 0xFF (RSTDQI)     ////
////        3. If the current=Quad and Previous: Quad, then bypass mode switching                      ////
////        4. If the current=Quad and Previous: Single, then switch mode by command 0x38 (ESQI)       ////
////     1.6 -  Jan 29, 2023, Dinesh A                                                                 ////
////          A. As part of MPW-2 Silicon Bug-Fx:-                                                     ////
////             SPI Flash Power Up command (0xAB) need 3 us delay before the next command             ////
////          B. FAST SIM connected to PORT for better GateSim control                                 ////
////     1.7 -  Feb 10, 2023, Dinesh A                                                                 ////
////            idle signal generated to support source clock gating                                   ////
////     1.8 -  Aug 2, 2023, Dinesh A                                                                  ////
////            A. Bug fix: SPI clock de-async de-assertion changed to spi-pos clock                   ////
////            B. Bug Fix: Made sure that CS de-assert start only after SPI clock in idle state       ////
////            C. added a 4 bit register to add delay between back-back command                       ////
////            D. Bug Fix: Added Support for partital byte based write access                         ////
////                                                                                                   ////
////                                                                                                   ////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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

/*******************************************************************
     strap_flash [1:0] - QSPI Flash Mode Selection for CS#0
                 2'b00 - Single
                 2'b01 - Double
                 2'b10 - Quad
                 2'b11 - QDDR

     strap_sram - QSPI SRAM Mode Selection for CS#2
                 1'b0 - Single
                 1'b1 - Quad

*********************************************************************/


module qspim_top
#( parameter WB_WIDTH = 32,
   parameter CMD_FIFO_WD = 50)
(
`ifdef USE_POWER_PINS
         input logic            vccd1,    // User area 1 1.8V supply
         input logic            vssd1,    // User area 1 digital ground
`endif
    input  logic                          mclk,
    input  logic                          rst_n,
    input  logic                          cfg_fast_sim, // 0 -> Normal, 1 -> Fast Sim

    input  logic   [1:0]                 strap_flash,
    input  logic                         strap_sram,
    input  logic                         strap_pre_sram,
    input  logic                         cfg_init_bypass, // Bypass initialization
    output logic                         qspim_idle,

    input  logic   [3:0]                 cfg_cska_sp_co, // spi clock skew adjust
    input  logic   [3:0]                 cfg_cska_spi,
    input  logic                         wbd_clk_int,
    output logic                         wbd_clk_spi,

    input  logic                         wbd_stb_i, // strobe/request
    input  logic   [WB_WIDTH-1:0]        wbd_adr_i, // address
    input  logic                         wbd_we_i,  // write
    input  logic   [WB_WIDTH-1:0]        wbd_dat_i, // data output
    input  logic   [3:0]                 wbd_sel_i, // byte enable
    input  logic   [9:0]                 wbd_bl_i,  // Burst Length
    input  logic                         wbd_bry_i, // Burst Ready
    output logic   [WB_WIDTH-1:0]        wbd_dat_o, // data input
    output logic                         wbd_ack_o, // acknowlegement
    output logic                         wbd_lack_o,// Last acknowlegement
    output logic                         wbd_err_o,  // error

    output logic                 [31:0]  spi_debug,

    // PAD I/f
    input logic [3:0]                    spi_sdi,
    output logic                         spi_clk,
    output logic [3:0]                   spi_csn,// No hold fix for CS#, as it asserted much eariler than SPI clock
    output logic [3:0]                   spi_sdo,
    output logic [3:0]                   spi_oen
);

  parameter P_SINGLE = 2'b00;
  parameter P_DOUBLE = 2'b01;
  parameter P_QUAD   = 2'b10;
  parameter P_QDDR   = 2'b11;

   
    logic                   [7:0] spi_clk_div      ;

    // Master 0 Configuration
    // Direct Memory CS# Address Mapping
    logic  [7:0]                  cfg_m0_cs0_addr;
    logic  [7:0]                  cfg_m0_cs1_addr;
    logic  [7:0]                  cfg_m0_cs2_addr;
    logic  [7:0]                  cfg_m0_cs3_addr;
    logic  [7:0]                  cfg_m0_cs0_amask;
    logic  [7:0]                  cfg_m0_cs1_amask;
    logic  [7:0]                  cfg_m0_cs2_amask;
    logic  [7:0]                  cfg_m0_cs3_amask;
    logic                         cfg_m0_fsm_reset ;
    logic                         cfg_dpft_dis ;

    logic [1:0]                   cfg_m0_g0_rd_spi_imode ;  // Init SPI Mode 
    logic [1:0]                   cfg_m0_g0_rd_spi_fmode ;  // Final SPI Mode 
    logic [1:0]                   cfg_m0_g0_rd_spi_switch;  // SPI Mode Switching Place
    logic [3:0]                   cfg_m0_g0_rd_spi_seq   ;  // SPI SEQUENCE
    logic [1:0]                   cfg_m0_g0_rd_addr_cnt  ;  // SPI Addr Count
    logic [3:0]                   cfg_m0_g0_rd_dummy_cnt ;  // SPI Dummy Count
    logic [7:0]                   cfg_m0_g0_rd_cmd_reg   ;  // SPI MEM COMMAND
    logic [7:0]                   cfg_m0_g0_rd_mode_reg  ;  // SPI MODE REG

    logic [1:0]                   cfg_m0_g0_wr_spi_imode ;  // Init SPI Mode 
    logic [1:0]                   cfg_m0_g0_wr_spi_fmode ;  // Final SPI Mode 
    logic [1:0]                   cfg_m0_g0_wr_spi_switch;  // SPI Mode Switching Place
    logic [3:0]                   cfg_m0_g0_wr_spi_seq   ;  // SPI SEQUENCE
    logic [1:0]                   cfg_m0_g0_wr_addr_cnt  ;  // SPI Addr Count
    logic [3:0]                   cfg_m0_g0_wr_dummy_cnt ;  // SPI Dummy Count
    logic [7:0]                   cfg_m0_g0_wr_cmd_reg   ;  // SPI MEM COMMAND
    logic [7:0]                   cfg_m0_g0_wr_mode_reg  ;  // SPI MODE REG

    logic [1:0]                   cfg_m0_g1_rd_spi_imode ;  // Init SPI Mode 
    logic [1:0]                   cfg_m0_g1_rd_spi_fmode ;  // Final SPI Mode 
    logic [1:0]                   cfg_m0_g1_rd_spi_switch;  // SPI Mode Switching Place
    logic [3:0]                   cfg_m0_g1_rd_spi_seq   ;  // SPI SEQUENCE
    logic [1:0]                   cfg_m0_g1_rd_addr_cnt  ;  // SPI Addr Count
    logic [3:0]                   cfg_m0_g1_rd_dummy_cnt ;  // SPI Dummy Count
    logic [7:0]                   cfg_m0_g1_rd_cmd_reg   ;  // SPI MEM COMMAND
    logic [7:0]                   cfg_m0_g1_rd_mode_reg  ;  // SPI MODE REG

    logic [1:0]                   cfg_m0_g1_wr_spi_imode ;  // Init SPI Mode 
    logic [1:0]                   cfg_m0_g1_wr_spi_fmode ;  // Final SPI Mode 
    logic [1:0]                   cfg_m0_g1_wr_spi_switch;  // SPI Mode Switching Place
    logic [3:0]                   cfg_m0_g1_wr_spi_seq   ;  // SPI SEQUENCE
    logic [1:0]                   cfg_m0_g1_wr_addr_cnt  ;  // SPI Addr Count
    logic [3:0]                   cfg_m0_g1_wr_dummy_cnt ;  // SPI Dummy Count
    logic [7:0]                   cfg_m0_g1_wr_cmd_reg   ;  // SPI MEM COMMAND
    logic [7:0]                   cfg_m0_g1_wr_mode_reg  ;  // SPI MODE REG

    logic [1:0]                   cfg_cs_early           ;  // Amount of cycle early CS asserted
    logic [1:0]                   cfg_cs_late            ;  // Amount of cycle late CS de-asserted
    logic [3:0]                   cfg_cmd_delay          ;  // Amount of cycle late CS de-asserted

    // Towards Reg I/F
    logic                         spim_reg_req     ;   // Reg Request
    logic [3:0]                   spim_reg_addr    ;   // Reg Address
    logic                         spim_reg_we      ;   // Reg Write/Read Command
    logic [3:0]                   spim_reg_be      ;   // Reg Byte Enable
    logic [31:0]                  spim_reg_wdata   ;   // Reg Write Data
    logic                         spim_reg_ack     ;   // Read Ack
    logic [31:0]                  spim_reg_rdata   ;   // Read Read Data

    // Towards m0 Command FIFO
    logic                         m0_cmd_fifo_full    ;   // Command FIFO full
    logic                         m0_cmd_fifo_afull   ;   // Command FIFO full
    logic                         m0_cmd_fifo_empty   ;   // Command FIFO empty
    logic                         m0_cmd_fifo_wr      ;   // Command FIFO Write
    logic                         m0_cmd_fifo_rd      ;   // Command FIFO read
    logic [CMD_FIFO_WD-1:0]       m0_cmd_fifo_wdata   ;   // Command FIFO WData
    logic [CMD_FIFO_WD-1:0]       m0_cmd_fifo_rdata   ;   // Command FIFO RData
    
    // Towards m0 Response FIFO
    logic                         m0_res_fifo_full    ;   // Response FIFO Empty
    logic                         m0_res_fifo_empty   ;   // Response FIFO Empty
    logic                         m0_res_fifo_wr      ;   // Response FIFO Write
    logic                         m0_res_fifo_rd      ;   // Response FIFO Read
    logic [31:0]                  m0_res_fifo_wdata   ;   // Response FIFO WData
    logic [31:0]                  m0_res_fifo_rdata   ;   // Response FIFO RData

    // Towards m1 Command FIFO
    logic                         m1_cmd_fifo_full    ;   // Command FIFO full
    logic                         m1_cmd_fifo_empty   ;   // Command FIFO empty
    logic                         m1_cmd_fifo_wr      ;   // Command FIFO Write
    logic                         m1_cmd_fifo_rd      ;   // Command FIFO Write
    logic [CMD_FIFO_WD-1:0]       m1_cmd_fifo_wdata   ;   // Command FIFO WData
    logic [CMD_FIFO_WD-1:0]       m1_cmd_fifo_rdata   ;   // Command FIFO RData
    
    // Towards m0 Response FIFO
    logic                         m1_res_fifo_full    ;   // Response FIFO Empty
    logic                         m1_res_fifo_empty   ;   // Response FIFO Empty
    logic                         m1_res_fifo_wr      ;   // Response FIFO Read
    logic                         m1_res_fifo_rd      ;   // Response FIFO Read
    logic [31:0]                  m1_res_fifo_wdata   ;   // Response FIFO WData
    logic [31:0]                  m1_res_fifo_rdata   ;   // Response FIFO RData

    logic                         m0_res_fifo_flush   ;   // m0 response fifo flush
    logic                         m1_res_fifo_flush   ;   // m0 response fifo flush

//-----------------------------------------------------
// SPI Debug monitoring
// ----------------------------------------------------
    logic [9:0]   spi_ctrl_status       ;
    logic [3:0]   m0_state         ;
    logic [3:0]   m1_state         ;
    logic [3:0]   ctrl_state        ;


    assign spi_debug  =   {m0_res_fifo_flush,m1_res_fifo_flush,spi_init_done,
		          m0_cmd_fifo_full,m0_cmd_fifo_empty,m0_res_fifo_full,m0_res_fifo_empty,
		          m1_cmd_fifo_full,m1_cmd_fifo_empty,m1_res_fifo_full,m1_res_fifo_empty,
		          ctrl_state[3:0], m0_state[3:0],m1_state[3:0],spi_ctrl_status[8:0]};

//-------------------------------------------------------
// SPI Interface moved inside to support carvel IO pad 
// -------------------------------------------------------

logic                    [1:0] spi_mode;
logic                          spi_en_tx;
logic                          spi_init_done;
logic  [3:0]                   spi_sdo_int;
logic                          spi_clk_int;
logic                          spi_sdo0_dl;
logic                          spi_sdo1_dl;
logic                          spi_sdo2_dl;
logic                          spi_sdo3_dl;
logic                          rst_ss_n;



// ADDing Delay cells for Interface hold fix
wire spi_sdo0_d1,spi_sdo0_d2;
ctech_delay_buf u_delay1_sdio0 (.X(spi_sdo0_d1),.A(spi_sdo_int[0]));
ctech_delay_buf u_delay2_sdio0 (.X(spi_sdo0_d2),.A(spi_sdo0_d1));
ctech_buf u_buf_sdio0    (.X(spi_sdo[0]),.A(spi_sdo0_d2));

wire spi_sdo1_d1,spi_sdo1_d2;
ctech_delay_buf u_delay1_sdio1 (.X(spi_sdo1_d1),.A(spi_sdo_int[1]));
ctech_delay_buf u_delay2_sdio1 (.X(spi_sdo1_d2),.A(spi_sdo1_d1));
ctech_buf u_buf_sdio1    (.X(spi_sdo[1]),.A(spi_sdo1_d2));

wire spi_sdo2_d1,spi_sdo2_d2;
ctech_delay_buf u_delay1_sdio2 (.X(spi_sdo2_d1),.A(spi_sdo_int[2]));
ctech_delay_buf u_delay2_sdio2 (.X(spi_sdo2_d2),.A(spi_sdo2_d1));
ctech_buf u_buf_sdio2    (.X(spi_sdo[2]),.A(spi_sdo2_d2));

wire spi_sdo3_d1,spi_sdo3_d2;
ctech_delay_buf u_delay1_sdio3 (.X(spi_sdo3_d1),.A(spi_sdo_int[3]));
ctech_delay_buf u_delay2_sdio3 (.X(spi_sdo3_d2),.A(spi_sdo3_d1));
ctech_buf u_buf_sdio3    (.X(spi_sdo[3]),.A(spi_sdo3_d2));


assign   spi_oen[0] = ((spi_mode == P_DOUBLE) || (spi_mode == P_QUAD) || (spi_mode == P_QDDR)) ? !spi_en_tx: 1'b0;  // SPI_DIO0
assign   spi_oen[1] = (spi_mode == P_SINGLE) ? 1'b1 : !spi_en_tx;  // SPI_DIO1
assign   spi_oen[2] =  ((spi_mode == P_QUAD) || (spi_mode == P_QDDR)) ? !spi_en_tx: 1'b0;   // HOLD
assign   spi_oen[3] =  ((spi_mode == P_QUAD) || (spi_mode == P_QDDR)) ? !spi_en_tx: 1'b0;   // 

// spi clock skew control
clk_skew_adjust u_skew_spi
       (
`ifdef USE_POWER_PINS
               .vccd1      (vccd1                      ),// User area 1 1.8V supply
               .vssd1      (vssd1                      ),// User area 1 digital ground
`endif
	       .clk_in     (wbd_clk_int                ), 
	       .sel        (cfg_cska_spi               ), 
	       .clk_out    (wbd_clk_spi                ) 
       );

// Clock Skey for SPI clock out
clk_skew_adjust u_skew_sp_co
       (
`ifdef USE_POWER_PINS
               .vccd1      (vccd1                      ),// User area 1 1.8V supply
               .vssd1      (vssd1                      ),// User area 1 digital ground
`endif
	       .clk_in     (spi_clk_int                ), 
	       .sel        (cfg_cska_sp_co             ), 
	       .clk_out    (spi_clk                    ) 
       );
//###################################
// Application Reset Synchronization
//###################################
reset_sync  u_app_rst (
	      .scan_mode  (1'b0        ),
              .dclk       (mclk        ), // Destination clock domain
	      .arst_n     (rst_n       ), // active low async reset
              .srst_n     (rst_ss_n    )
          );
qspim_if #( .WB_WIDTH(WB_WIDTH),.CMD_FIFO_WD(CMD_FIFO_WD)) u_wb_if(
        .mclk                           (mclk                         ),
        .rst_n                          (rst_ss_n                     ),

        .wbd_stb_i                      (wbd_stb_i                    ), // strobe/request
        .wbd_adr_i                      (wbd_adr_i                    ), // address
        .wbd_we_i                       (wbd_we_i                     ), // write
        .wbd_dat_i                      (wbd_dat_i                    ), // data output
        .wbd_sel_i                      (wbd_sel_i                    ), // byte enable
        .wbd_bl_i                       (wbd_bl_i                     ), // Busrt length
        .wbd_bry_i                      (wbd_bry_i                    ), // Busrt Ready
        .wbd_dat_o                      (wbd_dat_o                    ), // data input
        .wbd_ack_o                      (wbd_ack_o                    ), // acknowlegement
        .wbd_lack_o                     (wbd_lack_o                   ), // last acknowlegement
        .wbd_err_o                      (wbd_err_o                    ), // error

    // Configuration
	.cfg_m0_cs0_addr                (cfg_m0_cs0_addr              ),
	.cfg_m0_cs1_addr                (cfg_m0_cs1_addr              ),
	.cfg_m0_cs2_addr                (cfg_m0_cs2_addr              ),
	.cfg_m0_cs3_addr                (cfg_m0_cs3_addr              ),

	.cfg_m0_cs0_amask               (cfg_m0_cs0_amask             ),
	.cfg_m0_cs1_amask               (cfg_m0_cs1_amask             ),
	.cfg_m0_cs2_amask               (cfg_m0_cs2_amask             ),
	.cfg_m0_cs3_amask               (cfg_m0_cs3_amask             ),
        .cfg_dpft_dis                   (cfg_dpft_dis                 ),
        .cfg_fsm_reset                  (cfg_m0_fsm_reset             ),

        .cfg_m0_g0_rd_spi_imode        (cfg_m0_g0_rd_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g0_rd_spi_fmode        (cfg_m0_g0_rd_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g0_rd_spi_switch       (cfg_m0_g0_rd_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g0_rd_spi_seq          (cfg_m0_g0_rd_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g0_rd_addr_cnt         (cfg_m0_g0_rd_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g0_rd_dummy_cnt        (cfg_m0_g0_rd_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g0_rd_cmd_reg          (cfg_m0_g0_rd_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g0_rd_mode_reg         (cfg_m0_g0_rd_mode_reg       ), // SPI MODE REG

        .cfg_m0_g0_wr_spi_imode        (cfg_m0_g0_wr_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g0_wr_spi_fmode        (cfg_m0_g0_wr_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g0_wr_spi_switch       (cfg_m0_g0_wr_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g0_wr_spi_seq          (cfg_m0_g0_wr_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g0_wr_addr_cnt         (cfg_m0_g0_wr_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g0_wr_dummy_cnt        (cfg_m0_g0_wr_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g0_wr_cmd_reg          (cfg_m0_g0_wr_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g0_wr_mode_reg         (cfg_m0_g0_wr_mode_reg       ), // SPI MODE REG

        .cfg_m0_g1_rd_spi_imode        (cfg_m0_g1_rd_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g1_rd_spi_fmode        (cfg_m0_g1_rd_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g1_rd_spi_switch       (cfg_m0_g1_rd_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g1_rd_spi_seq          (cfg_m0_g1_rd_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g1_rd_addr_cnt         (cfg_m0_g1_rd_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g1_rd_dummy_cnt        (cfg_m0_g1_rd_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g1_rd_cmd_reg          (cfg_m0_g1_rd_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g1_rd_mode_reg         (cfg_m0_g1_rd_mode_reg       ), // SPI MODE REG

        .cfg_m0_g1_wr_spi_imode        (cfg_m0_g1_wr_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g1_wr_spi_fmode        (cfg_m0_g1_wr_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g1_wr_spi_switch       (cfg_m0_g1_wr_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g1_wr_spi_seq          (cfg_m0_g1_wr_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g1_wr_addr_cnt         (cfg_m0_g1_wr_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g1_wr_dummy_cnt        (cfg_m0_g1_wr_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g1_wr_cmd_reg          (cfg_m0_g1_wr_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g1_wr_mode_reg         (cfg_m0_g1_wr_mode_reg       ), // SPI MODE REG

        .spi_init_done                  (spi_init_done                ), // SPI internal Init completed

    // Towards Reg I/F
        .spim_reg_req                   (spim_reg_req                 ), // Reg Request
        .spim_reg_addr                  (spim_reg_addr                ), // Reg Address
        .spim_reg_we                    (spim_reg_we                  ), // Reg Write/Read Command
        .spim_reg_be                    (spim_reg_be                  ), // Reg Byte Enable
        .spim_reg_wdata                 (spim_reg_wdata               ), // Reg Write Data
        .spim_reg_ack                   (spim_reg_ack                 ), // Read Ack
        .spim_reg_rdata                 (spim_reg_rdata               ), // Read Read Data

    // Towards Command FIFO
        .cmd_fifo_full                  (m0_cmd_fifo_full             ), // Command FIFO full
        .cmd_fifo_afull                 (m0_cmd_fifo_afull            ), // Command FIFO full
        .cmd_fifo_empty                 (m0_cmd_fifo_empty            ), // Command FIFO empty
        .cmd_fifo_wr                    (m0_cmd_fifo_wr               ), // Command FIFO Write
        .cmd_fifo_wdata                 (m0_cmd_fifo_wdata            ), // Command FIFO WData
    
    // Towards Response FIFO
        .res_fifo_empty                 (m0_res_fifo_empty            ), // Response FIFO Empty
        .res_fifo_rd                    (m0_res_fifo_rd               ), // Response FIFO Read
        .res_fifo_rdata                 (m0_res_fifo_rdata            ), // Response FIFO Data

	.state                          (m0_state                     )

    );


qspim_regs
    #( .WB_WIDTH(WB_WIDTH),.CMD_FIFO_WD(CMD_FIFO_WD))
    u_spim_regs
    (
        .mclk                           (mclk                         ),
        .rst_n                          (rst_ss_n                     ),

         .cfg_init_bypass               (cfg_init_bypass              ),
         .strap_flash                   (strap_flash                  ),
         .strap_pre_sram                (strap_pre_sram               ),
         .strap_sram                    (strap_sram                   ),

	    .cfg_fast_sim                   (cfg_fast_sim                 ),

        .spi_clk_div                    (spi_clk_div                  ),
	    .spi_init_done                  (spi_init_done                ),

        .spi_debug                      (spi_debug                    ),


	.cfg_m0_cs0_addr                (cfg_m0_cs0_addr              ),
	.cfg_m0_cs1_addr                (cfg_m0_cs1_addr              ),
	.cfg_m0_cs2_addr                (cfg_m0_cs2_addr              ),
	.cfg_m0_cs3_addr                (cfg_m0_cs3_addr              ),

	.cfg_m0_cs0_amask               (cfg_m0_cs0_amask             ),
	.cfg_m0_cs1_amask               (cfg_m0_cs1_amask             ),
	.cfg_m0_cs2_amask               (cfg_m0_cs2_amask             ),
	.cfg_m0_cs3_amask               (cfg_m0_cs3_amask             ),

        .cfg_dpft_dis                   (cfg_dpft_dis                 ),
        .cfg_m0_fsm_reset               (cfg_m0_fsm_reset             ),

        .cfg_m0_g0_rd_spi_imode        (cfg_m0_g0_rd_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g0_rd_spi_fmode        (cfg_m0_g0_rd_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g0_rd_spi_switch       (cfg_m0_g0_rd_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g0_rd_spi_seq          (cfg_m0_g0_rd_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g0_rd_addr_cnt         (cfg_m0_g0_rd_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g0_rd_dummy_cnt        (cfg_m0_g0_rd_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g0_rd_cmd_reg          (cfg_m0_g0_rd_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g0_rd_mode_reg         (cfg_m0_g0_rd_mode_reg       ), // SPI MODE REG

        .cfg_m0_g0_wr_spi_imode        (cfg_m0_g0_wr_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g0_wr_spi_fmode        (cfg_m0_g0_wr_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g0_wr_spi_switch       (cfg_m0_g0_wr_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g0_wr_spi_seq          (cfg_m0_g0_wr_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g0_wr_addr_cnt         (cfg_m0_g0_wr_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g0_wr_dummy_cnt        (cfg_m0_g0_wr_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g0_wr_cmd_reg          (cfg_m0_g0_wr_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g0_wr_mode_reg         (cfg_m0_g0_wr_mode_reg       ), // SPI MODE REG

        .cfg_m0_g1_rd_spi_imode        (cfg_m0_g1_rd_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g1_rd_spi_fmode        (cfg_m0_g1_rd_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g1_rd_spi_switch       (cfg_m0_g1_rd_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g1_rd_spi_seq          (cfg_m0_g1_rd_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g1_rd_addr_cnt         (cfg_m0_g1_rd_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g1_rd_dummy_cnt        (cfg_m0_g1_rd_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g1_rd_cmd_reg          (cfg_m0_g1_rd_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g1_rd_mode_reg         (cfg_m0_g1_rd_mode_reg       ), // SPI MODE REG

        .cfg_m0_g1_wr_spi_imode        (cfg_m0_g1_wr_spi_imode      ), // Init SPI Mode 
        .cfg_m0_g1_wr_spi_fmode        (cfg_m0_g1_wr_spi_fmode      ), // Final SPI Mode 
        .cfg_m0_g1_wr_spi_switch       (cfg_m0_g1_wr_spi_switch     ), // SPI Mode Switching Place
        .cfg_m0_g1_wr_spi_seq          (cfg_m0_g1_wr_spi_seq        ), // SPI SEQUENCE
        .cfg_m0_g1_wr_addr_cnt         (cfg_m0_g1_wr_addr_cnt       ), // SPI Addr Count
        .cfg_m0_g1_wr_dummy_cnt        (cfg_m0_g1_wr_dummy_cnt      ), // SPI Dummy Count
        .cfg_m0_g1_wr_cmd_reg          (cfg_m0_g1_wr_cmd_reg        ), // SPI MEM COMMAND
        .cfg_m0_g1_wr_mode_reg         (cfg_m0_g1_wr_mode_reg       ), // SPI MODE REG

	    .cfg_cs_early                   (cfg_cs_early               ),
	    .cfg_cs_late                    (cfg_cs_late                ),
	    .cfg_cmd_delay                  (cfg_cmd_delay              ),

    // Towards Reg I/F
        .spim_reg_req                   (spim_reg_req                 ), // Reg Request
        .spim_reg_addr                  (spim_reg_addr                ), // Reg Address
        .spim_reg_we                    (spim_reg_we                  ), // Reg Write/Read Command
        .spim_reg_be                    (spim_reg_be                  ), // Reg Byte Enable
        .spim_reg_wdata                 (spim_reg_wdata               ), // Reg Write Data
        .spim_reg_ack                   (spim_reg_ack                 ), // Read Ack
        .spim_reg_rdata                 (spim_reg_rdata               ), // Read Read Data

    // Towards Command FIFO
        .cmd_fifo_full                  (m1_cmd_fifo_full             ), // Command FIFO empty
        .cmd_fifo_empty                 (m1_cmd_fifo_empty            ), // Command FIFO empty
        .cmd_fifo_wr                    (m1_cmd_fifo_wr               ), // Command FIFO Write
        .cmd_fifo_wdata                 (m1_cmd_fifo_wdata            ), // Command FIFO WData
    
    // Towards Response FIFO
        .res_fifo_full                  (m1_res_fifo_full             ), // Response FIFO Empty
        .res_fifo_empty                 (m1_res_fifo_empty            ), // Response FIFO Empty
        .res_fifo_rd                    (m1_res_fifo_rd               ), // Response FIFO Read
        .res_fifo_rdata                 (m1_res_fifo_rdata            ),  // Response FIFO Data

	.state                          (m1_state                     )

    );

 // Master 0 Command FIFO
qspim_fifo #(.W(CMD_FIFO_WD), .DP(4)) u_m0_cmd_fifo (
	 .clk                           (mclk                        ),
         .reset_n                       (rst_ss_n                    ),
	 .flush                         (1'b0                        ),
         .wr_en                         (m0_cmd_fifo_wr              ),
         .wr_data                       (m0_cmd_fifo_wdata           ),
         .full                          (m0_cmd_fifo_full            ),                 
         .afull                         (m0_cmd_fifo_afull           ),                 
         .rd_en                         (m0_cmd_fifo_rd              ),
         .empty                         (m0_cmd_fifo_empty           ),                
         .aempty                        (                            ),                
         .rd_data                       (m0_cmd_fifo_rdata           )
   );

 // Master 0 Response FIFO
qspim_fifo #(.W(32), .DP(8)) u_m0_res_fifo (
	 .clk                           (mclk                        ),
         .reset_n                       (rst_ss_n                    ),
	 .flush                         (m0_res_fifo_flush           ),
         .wr_en                         (m0_res_fifo_wr              ),
         .wr_data                       (m0_res_fifo_wdata           ),
         .full                          (m0_res_fifo_full            ),                 
         .afull                         (                            ),                 
         .rd_en                         (m0_res_fifo_rd              ),
         .empty                         (m0_res_fifo_empty           ),                
         .aempty                        (                            ),                
         .rd_data                       (m0_res_fifo_rdata           )
   );

 // Master 1 Command FIFO
qspim_fifo #(.W(CMD_FIFO_WD), .DP(4)) u_m1_cmd_fifo (
	 .clk                           (mclk                        ),
         .reset_n                       (rst_ss_n                    ),
	 .flush                         (1'b0                        ),
         .wr_en                         (m1_cmd_fifo_wr              ),
         .wr_data                       (m1_cmd_fifo_wdata           ),
         .full                          (m1_cmd_fifo_full            ),                 
         .afull                         (                            ),                 
         .rd_en                         (m1_cmd_fifo_rd              ),
         .empty                         (m1_cmd_fifo_empty           ),                
         .aempty                        (                            ),                
         .rd_data                       (m1_cmd_fifo_rdata           )
   );
 // Master 1 Response FIFO
qspim_fifo #(.W(32), .DP(8)) u_m1_res_fifo (
	 .clk                           (mclk                        ),
         .reset_n                       (rst_ss_n                    ),
	 .flush                         (m1_res_fifo_flush           ),
         .wr_en                         (m1_res_fifo_wr              ),
         .wr_data                       (m1_res_fifo_wdata           ),
         .full                          (m1_res_fifo_full            ),                 
         .afull                         (                            ),                 
         .rd_en                         (m1_res_fifo_rd              ),
         .empty                         (m1_res_fifo_empty           ),                
         .aempty                        (                            ),                
         .rd_data                       (m1_res_fifo_rdata           )
   );


qspim_ctrl #(.CMD_FIFO_WD(CMD_FIFO_WD)) u_spictrl
    (
        .clk                            (mclk                         ),
        .rstn                           (rst_ss_n                     ),

        .spi_clk_div                    (spi_clk_div                  ),
        .spi_status                     (spi_ctrl_status              ),


	    .cfg_cs_early                   (cfg_cs_early                 ),
	    .cfg_cs_late                    (cfg_cs_late                  ),
	    .cfg_cmd_delay                  (cfg_cmd_delay                ),

	.m0_cmd_fifo_empty              (m0_cmd_fifo_empty            ),
        .m0_cmd_fifo_rd                 (m0_cmd_fifo_rd               ),
	.m0_cmd_fifo_rdata              (m0_cmd_fifo_rdata            ),

	.m0_res_fifo_flush              (m0_res_fifo_flush            ),
	.m0_res_fifo_empty              (m0_res_fifo_empty            ),
	.m0_res_fifo_full               (m0_res_fifo_full             ),
	.m0_res_fifo_wr                 (m0_res_fifo_wr               ),
	.m0_res_fifo_wdata              (m0_res_fifo_wdata            ),

	.m1_cmd_fifo_empty              (m1_cmd_fifo_empty            ),
        .m1_cmd_fifo_rd                 (m1_cmd_fifo_rd               ),
	.m1_cmd_fifo_rdata              (m1_cmd_fifo_rdata            ),

	.m1_res_fifo_flush              (m1_res_fifo_flush            ),
	.m1_res_fifo_empty              (m1_res_fifo_empty            ),
	.m1_res_fifo_full               (m1_res_fifo_full             ),
	.m1_res_fifo_wr                 (m1_res_fifo_wr               ),
	.m1_res_fifo_wdata              (m1_res_fifo_wdata            ),

	.ctrl_state                     (ctrl_state                   ),

        .spi_clk                        (spi_clk_int                  ),
        .spi_csn0                       (spi_csn[0]                   ),
        .spi_csn1                       (spi_csn[1]                   ),
        .spi_csn2                       (spi_csn[2]                   ),
        .spi_csn3                       (spi_csn[3]                   ),
        .spi_mode                       (spi_mode                     ),
        .spi_sdo0                       (spi_sdo_int[0]               ),
        .spi_sdo1                       (spi_sdo_int[1]               ),
        .spi_sdo2                       (spi_sdo_int[2]               ),
        .spi_sdo3                       (spi_sdo_int[3]               ),
        .spi_sdi0                       (spi_sdi[0]                   ),
        .spi_sdi1                       (spi_sdi[1]                   ),
        .spi_sdi2                       (spi_sdi[2]                   ),
        .spi_sdi3                       (spi_sdi[3]                   ),
	    .spi_en_tx_out                  (spi_en_tx                    )
    );


//-------------------------------------------------------
// QSPIM Idle generation based on no internal activity
//-------------------------------------------------------


always@ (posedge mclk or negedge rst_ss_n)
begin
   if(rst_ss_n == 1'b0) begin
      qspim_idle <= 1'b0;
   end else begin
      qspim_idle <= spi_init_done & 
                    m0_cmd_fifo_empty & m0_res_fifo_empty & 
                    m1_cmd_fifo_empty & m1_cmd_fifo_empty &
                    (ctrl_state == 0) & 
                    (m0_state == 0) & 
                    (m1_state == 0) ;

   end
end
  

endmodule
