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
// SPDX-FileContributor: Created by Dinesh Annayya <dinesha@opencores.org>
//
//////////////////////////////////////////////////////////////////////
////                                                              ////
////  SPI WishBone Register I/F Module                            ////
////                                                              ////
////  This file is part of the YIFive cores project               ////
////  https://github.com/dineshannayya/yifive_r0.git              ////
////  http://www.opencores.org/cores/yifive/                      ////
////                                                              ////
////  Description                                                 ////
////     SPI WishBone I/F module                                  ////
////     This block support following functionality               ////
////        1. Direct SPI Read memory support for address rang    ////
////             0x0000 to 0x0FFF_FFFF - Use full for Instruction ////
////             Data Memory fetch                                ////
////        2. SPI Local Register Access                          ////
////        3. Indirect register way to access SPI Memory         ////
////                                                              ////
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision :                                                  ////
////     V.0  -  June 8, 2021                                     //// 
////     V.1  -  Jan 29, 2023                                     ////
////             As part of MPW-2 Silicon Bring-up noticed        ////
////             SPI Flash Power Up command (0xAB) need 3 us      ////
////             delay before the next command                    ////
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


module qspim_regs #( parameter WB_WIDTH = 32, parameter CMD_FIFO_WD = 40) (
    input  logic                         mclk             ,
    input  logic                         rst_n            ,

    input  logic                         cfg_init_bypass  ,
    input  logic   [1:0]                 strap_flash      ,
    input  logic                         strap_sram       ,
    input  logic                         strap_pre_sram   , // Previous Power-on SRAM Strap State


    input logic                          cfg_fast_sim    , // Set 1 for simulation

    output logic                   [7:0] spi_clk_div      ,
    output logic                         spi_init_done    , // SPI internal Init completed

    // Status Monitoring
    input logic                    [31:0] spi_debug       ,

    // Master 0 Configuration
    // Direct Memory CS# Address Mapping
    output logic  [7:0]                  cfg_m0_cs0_addr,
    output logic  [7:0]                  cfg_m0_cs1_addr,
    output logic  [7:0]                  cfg_m0_cs2_addr,
    output logic  [7:0]                  cfg_m0_cs3_addr,
    output logic  [7:0]                  cfg_m0_cs0_amask,
    output logic  [7:0]                  cfg_m0_cs1_amask,
    output logic  [7:0]                  cfg_m0_cs2_amask,
    output logic  [7:0]                  cfg_m0_cs3_amask,

    output logic                         cfg_dpft_dis ,      // Direct Mem Prefetch enable/disable
    output logic                         cfg_m0_fsm_reset ,

    output logic [1:0]                   cfg_m0_g0_rd_spi_imode  , // Initial SPI Mode 
    output logic [1:0]                   cfg_m0_g0_rd_spi_fmode  , // Final SPI Mode 
    output logic [1:0]                   cfg_m0_g0_rd_spi_switch,  // SPI Mode Switching Place
    output logic [3:0]                   cfg_m0_g0_rd_spi_seq   ,  // SPI SEQUENCE
    output logic [1:0]                   cfg_m0_g0_rd_addr_cnt  ,  // SPI Addr Count
    output logic [3:0]                   cfg_m0_g0_rd_dummy_cnt ,  // SPI Dummy Count
    output logic [7:0]                   cfg_m0_g0_rd_cmd_reg   ,  // SPI MEM COMMAND
    output logic [7:0]                   cfg_m0_g0_rd_mode_reg  ,  // SPI MODE REG

    output logic [1:0]                   cfg_m0_g0_wr_spi_imode  , // Initial SPI Mode 
    output logic [1:0]                   cfg_m0_g0_wr_spi_fmode  , // Final SPI Mode 
    output logic [1:0]                   cfg_m0_g0_wr_spi_switch,  // SPI Mode Switching Place
    output logic [3:0]                   cfg_m0_g0_wr_spi_seq   ,  // SPI SEQUENCE
    output logic [1:0]                   cfg_m0_g0_wr_addr_cnt  ,  // SPI Addr Count
    output logic [3:0]                   cfg_m0_g0_wr_dummy_cnt ,  // SPI Dummy Count
    output logic [7:0]                   cfg_m0_g0_wr_cmd_reg   ,  // SPI MEM COMMAND
    output logic [7:0]                   cfg_m0_g0_wr_mode_reg  ,  // SPI MODE REG

    output logic [1:0]                   cfg_m0_g1_rd_spi_imode  , // Initial SPI Mode 
    output logic [1:0]                   cfg_m0_g1_rd_spi_fmode  , // Final SPI Mode 
    output logic [1:0]                   cfg_m0_g1_rd_spi_switch,  // SPI Mode Switching Place
    output logic [3:0]                   cfg_m0_g1_rd_spi_seq   ,  // SPI SEQUENCE
    output logic [1:0]                   cfg_m0_g1_rd_addr_cnt  ,  // SPI Addr Count
    output logic [3:0]                   cfg_m0_g1_rd_dummy_cnt ,  // SPI Dummy Count
    output logic [7:0]                   cfg_m0_g1_rd_cmd_reg   ,  // SPI MEM COMMAND
    output logic [7:0]                   cfg_m0_g1_rd_mode_reg  ,  // SPI MODE REG

    output logic [1:0]                   cfg_m0_g1_wr_spi_imode  , // Initial SPI Mode 
    output logic [1:0]                   cfg_m0_g1_wr_spi_fmode  , // Final SPI Mode 
    output logic [1:0]                   cfg_m0_g1_wr_spi_switch,  // SPI Mode Switching Place
    output logic [3:0]                   cfg_m0_g1_wr_spi_seq   ,  // SPI SEQUENCE
    output logic [1:0]                   cfg_m0_g1_wr_addr_cnt  ,  // SPI Addr Count
    output logic [3:0]                   cfg_m0_g1_wr_dummy_cnt ,  // SPI Dummy Count
    output logic [7:0]                   cfg_m0_g1_wr_cmd_reg   ,  // SPI MEM COMMAND
    output logic [7:0]                   cfg_m0_g1_wr_mode_reg  ,  // SPI MODE REG


    output logic [1:0]                   cfg_cs_early     ,  // Amount of cycle early CS asserted
    output logic [1:0]                   cfg_cs_late      ,  // Amount of cycle late CS de-asserted
    output logic [3:0]                   cfg_cmd_delay    ,  // Delay between processing command

    // Towards Reg I/F
    input  logic                         spim_reg_req     ,   // Reg Request
    input  logic [3:0]                   spim_reg_addr    ,   // Reg Address
    input  logic                         spim_reg_we      ,   // Reg Write/Read Command
    input  logic [3:0]                   spim_reg_be      ,   // Reg Byte Enable
    input  logic [31:0]                  spim_reg_wdata   ,   // Reg Write Data
    output  logic                        spim_reg_ack     ,   // Read Ack
    output  logic [31:0]                 spim_reg_rdata    ,   // Read Read Data

    // Towards Command FIFO
    input  logic                         cmd_fifo_full    ,   // Command FIFO full
    input  logic                         cmd_fifo_empty   ,   // Command FIFO empty
    output logic                         cmd_fifo_wr      ,   // Command FIFO Write
    output logic [CMD_FIFO_WD-1:0]       cmd_fifo_wdata   ,   // Command FIFO WData
    
    // Towards Response FIFO
    input  logic                         res_fifo_full    ,   // Response FIFO Empty
    input  logic                         res_fifo_empty   ,   // Response FIFO Empty
    output logic                         res_fifo_rd      ,   // Response FIFO Read
    input  logic [31:0]                  res_fifo_rdata   ,   // Response FIFO Data

    output logic [3:0]                   state           
    );
//------------------------------------------------
// Parameter Decleration
// -----------------------------------------------
parameter SOC = 1'b1;    // START of COMMAND
parameter EOC = 1'b1;    // END of COMMAND
parameter NOC = 1'b0;    // NORMAL COMMAND

parameter BTYPE = 1'b0;  // Count is Byte Type
parameter WTYPE = 1'b1;  // Count is Word Type

parameter CNT1 = 2'b00; // BYTE/WORD Count1
parameter CNT2 = 2'b01; // BYTE/WORD Count2
parameter CNT3 = 2'b10; // BYTE/WORD Count3
parameter CNT4 = 2'b11; // BYTE/WORD Count4


// Type of command
parameter NWRITE = 2'b00; // Normal Write
parameter NREAD  = 2'b01; // Normal Read
parameter DWRITE = 2'b10; // Dummy Write
parameter DREAD  = 2'b11; // Dummy Read

// State Machine state
parameter FSM_IDLE        = 3'b000;
parameter FSM_ADR_PHASE   = 3'b001;
parameter FSM_WRITE_PHASE = 3'b010;
parameter FSM_READ_PHASE  = 3'b011;
parameter FSM_READ_BUSY   = 3'b100;
parameter FSM_WRITE_BUSY  = 3'b101;
parameter FSM_ACK_PHASE   = 3'b110;

//----------------------------
// Register Decoding
// ---------------------------
parameter GLBL_CTRL          = 4'b0000;
parameter DMEM_CS0_RD_CTRL   = 4'b0001;
parameter DMEM_CS0_WR_CTRL   = 4'b0010;
parameter DMEM_CS1_RD_CTRL   = 4'b0011;
parameter DMEM_CS1_WR_CTRL   = 4'b0100;
parameter DMEM_CS_AMAP       = 4'b0101;  // DIRECT MEMORY CS# ADDRESS MAP
parameter DMEM_CS_AMASK      = 4'b0110;  // DIRECT MEMORY CS# ADDRESS MASK
parameter IMEM_CTRL1         = 4'b0111;
parameter IMEM_CTRL2         = 4'b1000;
parameter IMEM_ADDR          = 4'b1001;
parameter IMEM_WDATA         = 4'b1010;
parameter IMEM_RDATA         = 4'b1011;
parameter SPI_STATUS         = 4'b1100;

// Init FSM
parameter SPI_INIT_PWUP         = 4'b0000;
parameter SPI_INIT_IDLE         = 4'b0001;
parameter SPI_POWERUP_CMD_WAIT  = 4'b0010;
parameter SPI_POWERUP_CMD_DELAY = 4'b0011;
parameter SPI_INIT_WREN_CMD     = 4'b0100;
parameter SPI_INIT_WREN_WAIT    = 4'b0101;
parameter SPI_INIT_WRR_CMD      = 4'b0110;
parameter SPI_INIT_WRR_WAIT     = 4'b0111;
parameter SPI_SRAM_STRAP_CHECK  = 4'b1000;
parameter SPI_SRAM_ESQI_WAIT    = 4'b1001;
parameter SPI_INIT_WAIT         = 4'b1010;

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
//---------------------------------------------------------
  parameter P_CS0 = 4'b0001;
  parameter P_CS1 = 4'b0010;
  parameter P_CS2 = 4'b0100;
  parameter P_CS3 = 4'b1000;

  parameter P_SINGLE = 2'b00;
  parameter P_DOUBLE = 2'b01;
  parameter P_QUAD   = 2'b10;
  parameter P_QDDR   = 2'b11;

  parameter P_MODE_SWITCH_IDLE     = 2'b00;
  parameter P_MODE_SWITCH_AT_ADDR  = 2'b01;
  parameter P_MODE_SWITCH_AT_DATA  = 2'b10;

  parameter P_FLASH_READ    = 8'h03;  // Normal Read
  parameter P_FLASH_FREAD   = 8'h0B;  // Fast Read
  parameter P_FLASH_DOR     = 8'h3B;  // Dual Read
  parameter P_FLASH_QOR     = 8'h6B;  // Quad Read
  parameter P_FLASH_DIOR    = 8'hBB;  // Dual I/O Read
  parameter P_FLASH_QIOR    = 8'hEB;  // Quad IO Read
  parameter P_FLASH_DDRFR   = 8'h0D;  // DDR Read
  parameter P_FLASH_DDRDIOR = 8'hBD;  // DDR Dual I/O Read
  parameter P_FLASH_DDRQIOR = 8'hED;  // DDR Quad I/O Read
  
  parameter P_FLASH_RES = 8'hAB;
  parameter P_FLASH_WEN = 8'h06;
  parameter P_FLASH_WRR = 8'h01;

  parameter P_SRAM_READ  = 8'h03;
  parameter P_SRAM_WRITE = 8'h02;
  parameter P_SRAM_ESDI  = 8'h3B; // Enter SDI (Dual)mode
  parameter P_SRAM_ESQI  = 8'h38; // Enter SQI (Quad) mode
  parameter P_SRAM_RSTDQI  = 8'hFF; // Reset SDI/SQI mode


  parameter P_BYTE_1   = 4'b0000;
  parameter P_BYTE_2   = 4'b0001;
  parameter P_BYTE_3   = 4'b0010;
  parameter P_BYTE_4   = 4'b0011;
  parameter P_BYTE_5   = 4'b0100;
  parameter P_BYTE_6   = 4'b0101;
  parameter P_BYTE_7   = 4'b0110;
  parameter P_BYTE_8   = 4'b0111;
  parameter P_BYTE_9   = 4'b1000;
  parameter P_BYTE_10  = 4'b1001;
  parameter P_BYTE_11  = 4'b1010;
  parameter P_BYTE_12  = 4'b1011;
  parameter P_BYTE_13  = 4'b1100;
  parameter P_BYTE_14  = 4'b1101;
  parameter P_BYTE_15  = 4'b1110;
  parameter P_BYTE_16  = 4'b1111;
//---------------------------------------------------------
// Variable declartion
// -------------------------------------------------------
logic   [3:0]        spi_init_state ;
logic                spim_reg_req_f ; 

logic [3:0]          cfg_m1_cs_reg    ;  // Chip select
logic [1:0]          cfg_m1_spi_imode ; // Initial SPI Mode 
logic [1:0]          cfg_m1_spi_fmode ; // Final SPI Mode 
logic [1:0]          cfg_m1_spi_switch;  // SPI Mode Switching Place
logic [1:0]          cfg_m1_fsm_reset ;
logic [3:0]          cfg_m1_spi_seq   ; // SPI SEQUENCE
logic [1:0]          cfg_m1_addr_cnt  ; // SPI Addr Count
logic [3:0]          cfg_m1_dummy_cnt ; // SPI Dummy Count
logic [7:0]          cfg_m1_data_cnt  ; // SPI Read Count
logic [7:0]          cfg_m1_cmd_reg   ; // SPI MEM COMMAND
logic [7:0]          cfg_m1_mode_reg  ; // SPI MODE REG
logic [31:0]         cfg_m1_addr      ;
logic [31:0]         cfg_m1_wdata     ;
logic [31:0]         cfg_m1_rdata     ;
logic                cfg_m1_wrdy      ;
logic                cfg_m1_req       ;

logic [31:0]         reg_rdata        ;


logic [5:0]           cur_cnt         ;
logic [5:0]           next_cnt        ;
logic [3:0]           next_state      ;


logic [31:0]          spim_m1_rdata   ;
logic                 spim_m1_ack     ;
logic                 spim_m1_rrdy    ;
logic                 spim_m1_wrdy    ;
logic  [15:0]         spi_delay_cnt  ;
logic                 spim_fifo_rdata_req  ;
logic                 spim_fifo_wdata_req  ;



//----------------------------------------------
// Consolidated Register Ack handling
//   1. Handles Normal Register Read
//   2. Indirect Memory Write
//   3. Indirect Memory Read
//----------------------------------------------
//
assign spim_fifo_rdata_req = spim_reg_req && spim_reg_we == 0 && (spim_reg_addr== IMEM_RDATA);
assign spim_fifo_wdata_req = spim_reg_req && spim_reg_we == 1 && (spim_reg_addr== IMEM_WDATA);

always_comb  begin
   spim_reg_ack   = 1'b0;
   spim_reg_rdata = 'h0;
   if(spi_init_done) begin
      if (spim_fifo_wdata_req && (spim_m1_wrdy == 1)) begin // Indirect Memory Write
	 // If FIFO Write DATA case, Make sure that there no previous pending
	 // need to processed
         spim_reg_ack  = 1'b1;
      end else if (spim_reg_req && spim_reg_we && (spim_reg_addr != IMEM_WDATA)) begin // Indirect memory Write
         spim_reg_ack  = 1'b1;
      end else if (spim_fifo_rdata_req && (spim_m1_rrdy == 1)) begin // Indirect mem Read
	 // If FIFO Read DATA case, Make sure that there Data is read from
         // External SPI Memory
         spim_reg_ack   = 1'b1;
         spim_reg_rdata = reg_rdata;
      end else if (spim_reg_req && spim_reg_we == 0 && (spim_reg_addr != IMEM_RDATA)) begin // Normal Read
	     // Read other than FIFO Read Data case
          spim_reg_ack   = 1'b1;
          spim_reg_rdata = reg_rdata;
      end
   end
end

  //---------------------------------------------
  // Manges the initial Config Phase of SPI Memory
  // 1. Power Up Command -  RES(0xAB) 
  // 2. Write Enable Command - WEN (0x06)
  // 3. WRITE CONFIG Reg - WRR (0x01) - Set Qaud Mode
  // --------------------------------------------
  
  logic  [15:0]          cfg_pup_cnt  ;
  logic  [15:0]          cfg_entry_cnt  ;
  logic  [15:0]          cfg_exit_cnt  ;
  //--------------------------------------------
  // With Assuming 100Mhz clock, Entry wait 50us 
  // and exit wait for 5us assumed
  //--------------------------------------------
  assign cfg_pup_cnt   = (cfg_fast_sim) ? 50: 300;
  assign cfg_entry_cnt = (cfg_fast_sim) ? 50: 5000;
  assign cfg_exit_cnt  = (cfg_fast_sim) ? 50: 500;

  integer byte_index;
  always_ff @(posedge mclk) begin
    if ( rst_n == 1'b0 ) begin
      cfg_m0_fsm_reset      <= 'h0;
      cfg_dpft_dis          <= 'h0;

      cfg_m0_g0_rd_cmd_reg        <= (strap_flash == P_SINGLE) ? P_FLASH_FREAD   :
                                     (strap_flash == P_DOUBLE) ? P_FLASH_DOR     :
                                     (strap_flash == P_QUAD)   ? P_FLASH_QIOR    : 
                                     (strap_flash == P_QDDR)   ? P_FLASH_DDRQIOR : 'h0;
      cfg_m0_g0_rd_mode_reg       <= 'h0;
      cfg_m0_g0_rd_spi_imode      <= (strap_flash == P_SINGLE) ? P_SINGLE              :
                                     (strap_flash == P_DOUBLE) ? P_SINGLE              :
                                     (strap_flash == P_QUAD)   ? P_SINGLE              : 
                                     (strap_flash == P_QDDR)   ? P_SINGLE              : 'h0;

      cfg_m0_g0_rd_spi_fmode      <= (strap_flash == P_SINGLE) ? P_SINGLE              :
                                     (strap_flash == P_DOUBLE) ? P_DOUBLE              :
                                     (strap_flash == P_QUAD)   ? P_QUAD                : 
                                     (strap_flash == P_QDDR)   ? P_QDDR                : 'h0;

      cfg_m0_g0_rd_spi_switch     <= (strap_flash == P_SINGLE) ? P_MODE_SWITCH_IDLE    :
                                     (strap_flash == P_DOUBLE) ? P_MODE_SWITCH_AT_DATA :
                                     (strap_flash == P_QUAD)   ? P_MODE_SWITCH_AT_ADDR : 
                                     (strap_flash == P_QDDR)   ? P_MODE_SWITCH_AT_ADDR : 'h0;
      cfg_m0_g0_rd_addr_cnt[1:0]  <= P_BYTE_3;
      cfg_m0_g0_rd_dummy_cnt[3:0] <= (strap_flash == P_SINGLE) ? P_BYTE_1       :
                                     (strap_flash == P_DOUBLE) ? P_BYTE_1       :
                                     (strap_flash == P_QUAD)   ? P_BYTE_2       : 
                                     (strap_flash == P_QDDR)   ? P_BYTE_5       : 'h0;
      cfg_m0_g0_rd_spi_seq[3:0]   <= (strap_flash == P_SINGLE) ? P_FSM_CADR     :
                                     (strap_flash == P_DOUBLE) ? P_FSM_CADR     :
                                     (strap_flash == P_QUAD)   ? P_FSM_CAMDR    : 
                                     (strap_flash == P_QDDR)   ? P_FSM_CAMDR    : 'h0;

      cfg_m0_g0_wr_cmd_reg        <= P_FLASH_QIOR;
      cfg_m0_g0_wr_mode_reg       <= 'h0;
      cfg_m0_g0_wr_spi_imode      <= P_QUAD;
      cfg_m0_g0_wr_spi_fmode      <= P_QUAD;
      cfg_m0_g0_wr_spi_switch     <= P_MODE_SWITCH_AT_ADDR;
      cfg_m0_g0_wr_addr_cnt[1:0]  <= P_BYTE_3;
      cfg_m0_g0_wr_dummy_cnt[3:0] <= P_BYTE_2;
      cfg_m0_g0_wr_spi_seq[3:0]   <= P_FSM_CAMDR;

      cfg_m0_g1_rd_cmd_reg        <= P_SRAM_READ;
      cfg_m0_g1_rd_mode_reg       <= 'h0;
      cfg_m0_g1_rd_spi_imode      <= (strap_sram == 1'b0) ? P_SINGLE   :
                                     (strap_sram == 1'b1) ? P_QUAD     : 'h0; 

      cfg_m0_g1_rd_spi_fmode      <= (strap_sram == 1'b0) ? P_SINGLE   :
                                     (strap_sram == 1'b1) ? P_QUAD     : 'h0; 

      cfg_m0_g1_rd_spi_switch     <= P_MODE_SWITCH_IDLE;

      cfg_m0_g1_rd_addr_cnt[1:0]  <= P_BYTE_3;
      cfg_m0_g1_rd_dummy_cnt[3:0] <= P_BYTE_1;
      cfg_m0_g1_rd_spi_seq[3:0]   <= P_FSM_CADR;

      cfg_m0_g1_wr_cmd_reg        <= P_SRAM_WRITE;
      cfg_m0_g1_wr_mode_reg       <= 'h0;
      cfg_m0_g1_wr_spi_imode      <= (strap_sram == 1'b0) ? P_SINGLE  :
                                     (strap_sram == 1'b1) ? P_QUAD    : 'h0; 
      cfg_m0_g1_wr_spi_fmode      <= (strap_sram == 1'b0) ? P_SINGLE  :
                                     (strap_sram == 1'b1) ? P_QUAD    : 'h0; 
      cfg_m0_g1_wr_spi_switch     <= P_MODE_SWITCH_IDLE;
      cfg_m0_g1_wr_addr_cnt[1:0]  <= P_BYTE_3;
      cfg_m0_g1_wr_dummy_cnt[3:0] <= P_BYTE_1;
      cfg_m0_g1_wr_spi_seq[3:0]   <= P_FSM_CAW;

      cfg_m1_fsm_reset      <= 'h0;
      cfg_m1_cs_reg         <= P_CS0;
      cfg_m1_spi_imode      <= P_QUAD;
      cfg_m1_spi_fmode      <= P_QUAD;
      cfg_m1_spi_switch     <= P_MODE_SWITCH_AT_DATA;
      cfg_m1_cmd_reg        <= P_FLASH_QOR;
      cfg_m1_mode_reg       <= 'h0;
      cfg_m1_spi_seq[3:0]   <= P_FSM_CADR;
      cfg_m1_addr_cnt[1:0]  <= P_BYTE_3;
      cfg_m1_dummy_cnt[3:0] <= P_BYTE_1;
      cfg_m1_data_cnt[7:0]  <= 0;
      cfg_m1_req            <= 0; 
      cfg_m1_wrdy           <= 1'b0;
      cfg_m1_wdata          <= 'h0; // Not Used

      cfg_cs_early         <= 'h1;
      cfg_cs_late          <= 'h1;
      cfg_cmd_delay        <= 'h4;
      spi_clk_div          <= 'h2;

      spi_init_done         <=  'h0;
      spi_delay_cnt         <= 'h0;
      spim_reg_req_f        <= 1'b0;
      spi_init_state        <=  SPI_INIT_PWUP;

      // Assumed Default Direct Address CS Mapping 
      // 0x0000_0000 to 0x03FF_FFFF - CS-0 (64MB)
      // 0x0400_0000 to 0x07FF_FFFF - CS-1 (64MB)
      // 0x0800_0000 to 0x0BFF_FFFF - CS-2 (64MB)
      // 0x0C00_0000 to 0x0FFF_FFFF - CS-3 (64MB)
      cfg_m0_cs0_addr        <= 8'h00;
      cfg_m0_cs1_addr        <= 8'h40;
      cfg_m0_cs2_addr        <= 8'h80;
      cfg_m0_cs3_addr        <= 8'hC0;
      cfg_m0_cs0_amask       <= 8'hF0;
      cfg_m0_cs1_amask       <= 8'hF0;
      cfg_m0_cs2_amask       <= 8'hF0;
      cfg_m0_cs3_amask       <= 8'hF0;
    end else begin 
        spim_reg_req_f        <= spim_reg_req; // Needed for finding Req Edge
        if(cfg_init_bypass) spi_init_done <= 1'b1;
        else if (spi_init_done == 0) begin
          case(spi_init_state)

              //----------------------------------------------
              // SPI MEMORY Need minimum 50Us after power up
              // With 100Mhz, 10ns translated to 5000 cycle
              // ---------------------------------------------
              SPI_INIT_PWUP:begin
                 if(spi_delay_cnt == cfg_entry_cnt) begin
                     spi_init_state   <=  SPI_INIT_IDLE;
           	 end else begin
           	     spi_delay_cnt <= spi_delay_cnt+1;
           	 end
              end

              SPI_INIT_IDLE:
              begin
                 cfg_m1_cs_reg        <= P_CS0;
                 cfg_m1_spi_imode     <= P_SINGLE;
                 cfg_m1_spi_fmode     <= P_SINGLE;
                 cfg_m1_spi_seq[3:0]  <= P_FSM_C;
                 cfg_m1_spi_switch    <= '0;
                 cfg_m1_cmd_reg       <= P_FLASH_RES; // POWER UP CMD
                 cfg_m1_mode_reg      <= 'h0; // Not Used
                 cfg_m1_addr_cnt[1:0] <= 'h0; // Not Used
                 cfg_m1_dummy_cnt[3:0]<= 'h0; // Not Used
                 cfg_m1_data_cnt[7:0] <= 'h0; // Not Used
                 cfg_m1_addr          <= 'h0; // Not Used
                 cfg_m1_wdata         <= 'h0; // Not Used
                 cfg_m1_req           <= 'h1;
                 spi_init_state       <=  SPI_POWERUP_CMD_WAIT;
              end
              SPI_POWERUP_CMD_WAIT:
              begin
                 if(spim_m1_ack)   begin
                    cfg_m1_req       <= 1'b0;
                    spi_delay_cnt    <= 'h0;
                    spi_init_state   <=  SPI_POWERUP_CMD_DELAY;
                 end
              end
              // After POWER-UP CMD, there is need for 3us delay for next cmd
              SPI_POWERUP_CMD_DELAY: begin
                   if(spi_delay_cnt == cfg_pup_cnt) begin
                       spi_init_state   <=  SPI_INIT_WREN_CMD;
           	       end else begin
           	           spi_delay_cnt <= spi_delay_cnt+1;
           	       end
                end


              SPI_INIT_WREN_CMD:
              begin
                 cfg_m1_cs_reg        <= P_CS0;
                 cfg_m1_spi_imode     <= P_SINGLE;
                 cfg_m1_spi_fmode     <= P_SINGLE;
                 cfg_m1_spi_seq[3:0]  <= P_FSM_C;
                 cfg_m1_spi_switch    <= '0;
                 cfg_m1_cmd_reg       <= P_FLASH_WEN;
                 cfg_m1_mode_reg      <= 'h0; // Not Used
                 cfg_m1_addr_cnt[1:0] <= 'h0; // Not Used
                 cfg_m1_dummy_cnt[3:0]<= 'h0; // Not Used
                 cfg_m1_data_cnt[7:0] <= 'h0; // Not Used
                 cfg_m1_addr          <= 'h0; // Not Used
                 cfg_m1_wdata         <= 'h0; // Not Used
                 cfg_m1_req           <= 'h1;
                 spi_init_state       <=  SPI_INIT_WREN_WAIT;
              end
              SPI_INIT_WREN_WAIT:
              begin
                 if(spim_m1_ack)   begin
                    cfg_m1_req      <= 1'b0;
                    //---------------------------------------------------------
                    // For SPI FLASH Memory QUAD and QDDR Mode, we need to enable
                    // enable set cr1[1] = 1 to enable. So we need configure it
                    // through WRR command
                    //-----------------------------------------------------------
                    if((strap_flash == P_QUAD) || (strap_flash == P_QDDR)) begin
                       spi_init_state    <=  SPI_INIT_WRR_CMD;
                    end else begin
                       spi_init_state    <=  SPI_SRAM_STRAP_CHECK;
                    end
                 end
              end
              SPI_INIT_WRR_CMD:
              begin
                 cfg_m1_cs_reg        <= P_CS0;
                 cfg_m1_spi_imode     <= P_SINGLE;
                 cfg_m1_spi_fmode     <= P_SINGLE;
                 cfg_m1_spi_seq[3:0]  <= P_FSM_CW;
                 cfg_m1_spi_switch    <= '0;
                 cfg_m1_cmd_reg       <= P_FLASH_WRR;
                 cfg_m1_mode_reg      <= 'h0; 
                 cfg_m1_addr_cnt[1:0] <= 'h0; 
                 cfg_m1_dummy_cnt[3:0]<= 'h0; 
                 cfg_m1_data_cnt[7:0] <= 'h2; // 2 Bytes
                 cfg_m1_addr          <= 'h0; 
                 cfg_m1_wrdy          <= 1'b1;
                 cfg_m1_wdata         <= {16'h0,8'h2,8'h0}; // <<cr1[7:0]><sr1[7:0]>> cr1[1] = 1 indicate quad mode cr1[7:6]=3 
                 cfg_m1_req           <= 'h1;
                 spi_init_state       <=  SPI_INIT_WRR_WAIT;
              end
              SPI_INIT_WRR_WAIT:
              begin
                 if(spim_m1_ack)   begin
		            spi_delay_cnt    <= 'h0;
                    cfg_m1_wrdy      <= 1'b0;
                    cfg_m1_req       <= 1'b0;
                    spi_init_state   <=  SPI_SRAM_STRAP_CHECK;
                 end
              end
              SPI_SRAM_STRAP_CHECK: // Check SRAM STRAP
              begin
                 //-----------------------------------------------------------------------
                 // Check the SPI SRAM Mode, If the Previous Strap State is Single
                 // If we need QUAD Mode, We need to issue ESQI command
                 //-----------------------------------------------------------------------
                 if(strap_pre_sram == 1'b0 && strap_sram == 1'b1) begin // If SRAM STRAP is QUAD Mode
                    cfg_m1_cs_reg        <= P_CS2;
                    cfg_m1_spi_imode     <= P_SINGLE;
                    cfg_m1_spi_fmode     <= P_SINGLE;
                    cfg_m1_spi_seq[3:0]  <= P_FSM_C;
                    cfg_m1_spi_switch    <= P_MODE_SWITCH_IDLE;
                    cfg_m1_cmd_reg       <= P_SRAM_ESQI;
                    cfg_m1_mode_reg      <= 'h0; 
                    cfg_m1_addr_cnt[1:0] <= 'h0; 
                    cfg_m1_dummy_cnt[3:0]<= 'h0; 
                    cfg_m1_data_cnt[7:0] <= 'h0; 
                    cfg_m1_addr          <= 'h0; 
                    cfg_m1_wrdy          <= 1'b1;
                    cfg_m1_wdata         <= 'h0; 
                    cfg_m1_req           <= 'h1;
                    spi_init_state       <=  SPI_SRAM_ESQI_WAIT;
                 end else if(strap_pre_sram == 1'b1 && strap_sram == 1'b0) begin 
                   // If PRE_STRAP=QUAD and STRAP=SINGLE, then we need RSTDQI command
                    cfg_m1_cs_reg        <= P_CS2;
                    cfg_m1_spi_imode     <= P_QUAD;
                    cfg_m1_spi_fmode     <= P_QUAD;
                    cfg_m1_spi_seq[3:0]  <= P_FSM_C;
                    cfg_m1_spi_switch    <= P_MODE_SWITCH_IDLE;
                    cfg_m1_cmd_reg       <= P_SRAM_RSTDQI;
                    cfg_m1_mode_reg      <= 'h0; 
                    cfg_m1_addr_cnt[1:0] <= 'h0; 
                    cfg_m1_dummy_cnt[3:0]<= 'h0; 
                    cfg_m1_data_cnt[7:0] <= 'h0; 
                    cfg_m1_addr          <= 'h0; 
                    cfg_m1_wrdy          <= 1'b1;
                    cfg_m1_wdata         <= 'h0; 
                    cfg_m1_req           <= 'h1;
                    spi_init_state       <=  SPI_SRAM_ESQI_WAIT;
                 end else begin
                    spi_init_state       <=  SPI_INIT_WAIT;
                 end
              end
              SPI_SRAM_ESQI_WAIT:
              begin
                 if(spim_m1_ack)   begin
		            spi_delay_cnt    <= 'h0;
                    cfg_m1_wrdy      <= 1'b0;
                    cfg_m1_req       <= 1'b0;
                    spi_init_state   <=  SPI_INIT_WAIT;
                 end
              end
              SPI_INIT_WAIT:
              begin // SPI MEMORY need 5us after WRR Command
                   if(spi_delay_cnt == cfg_exit_cnt) begin
                       spi_init_done    <=  'h1;
           	end else begin
           	    spi_delay_cnt <= spi_delay_cnt+1;
           	end
              end
          endcase
       end else if (spim_reg_req && spim_reg_we && spi_init_done )
       begin
         case(spim_reg_addr)
         GLBL_CTRL: begin
             if ( spim_reg_be[0] == 1 ) begin
                cfg_cs_early  <= spim_reg_wdata[1:0];
                cfg_cs_late   <= spim_reg_wdata[3:2];
                cfg_cmd_delay <= spim_reg_wdata[7:4];
             end
             if ( spim_reg_be[1] == 1 ) begin
                spi_clk_div <= spim_reg_wdata[15:8];
             end
             if ( spim_reg_be[3] == 1 ) begin
               cfg_dpft_dis         <= spim_reg_wdata[30];
               cfg_m0_fsm_reset     <= spim_reg_wdata[31];
	     end
         end
	 // CH0 READ CONTROL for direct access
        DMEM_CS0_RD_CTRL: begin // This register control Direct Memory Access Type
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_g0_rd_cmd_reg <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_g0_rd_mode_reg <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
               cfg_m0_g0_rd_spi_imode  <= spim_reg_wdata[17:16]; // SPI init Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g0_rd_spi_fmode  <= spim_reg_wdata[19:18]; // SPI final Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g0_rd_spi_switch <= spim_reg_wdata[21:20]; // Phase where to switch the SPI Mode
               cfg_m0_g0_rd_addr_cnt   <= spim_reg_wdata[23:22];
             end

             if ( spim_reg_be[3] == 1 ) begin
               cfg_m0_g0_rd_dummy_cnt[3:0]<= spim_reg_wdata[27:24];
               cfg_m0_g0_rd_spi_seq[3:0]  <= spim_reg_wdata[31:28];
             end
         end
	 // CH0 WRITE CONTROL for direct access
        DMEM_CS0_WR_CTRL: begin // This register control Direct Memory Access Type
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_g0_wr_cmd_reg <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_g0_wr_mode_reg <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
               cfg_m0_g0_wr_spi_imode  <= spim_reg_wdata[17:16]; // SPI init Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g0_wr_spi_fmode  <= spim_reg_wdata[19:18]; // SPI final Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g0_wr_spi_switch <= spim_reg_wdata[21:20]; // Phase where to switch the SPI Mode
               cfg_m0_g0_wr_addr_cnt   <= spim_reg_wdata[23:22];
             end

             if ( spim_reg_be[3] == 1 ) begin
               cfg_m0_g0_wr_dummy_cnt[3:0]<= spim_reg_wdata[27:24];
               cfg_m0_g0_wr_spi_seq[3:0]  <= spim_reg_wdata[31:28];
             end
         end

	 // CH1 READ CONTROL for direct access
        DMEM_CS1_RD_CTRL: begin // This register control Direct Memory Access Type
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_g1_rd_cmd_reg <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_g1_rd_mode_reg <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
               cfg_m0_g1_rd_spi_imode  <= spim_reg_wdata[17:16]; // SPI init Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g1_rd_spi_fmode  <= spim_reg_wdata[19:18]; // SPI final Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g1_rd_spi_switch <= spim_reg_wdata[21:20]; // Phase where to switch the SPI Mode
               cfg_m0_g1_rd_addr_cnt   <= spim_reg_wdata[23:22];
             end

             if ( spim_reg_be[3] == 1 ) begin
               cfg_m0_g1_rd_dummy_cnt[3:0]<= spim_reg_wdata[27:24];
               cfg_m0_g1_rd_spi_seq[3:0]  <= spim_reg_wdata[31:28];
             end
         end
	 // CH1 WRITE CONTROL for direct access
        DMEM_CS1_WR_CTRL: begin // This register control Direct Memory Access Type
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_g1_wr_cmd_reg <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_g1_wr_mode_reg <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
               cfg_m0_g1_wr_spi_imode  <= spim_reg_wdata[17:16]; // SPI init Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g1_wr_spi_fmode  <= spim_reg_wdata[19:18]; // SPI final Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - QDDR
               cfg_m0_g1_wr_spi_switch <= spim_reg_wdata[21:20]; // Phase where to switch the SPI Mode
               cfg_m0_g1_wr_addr_cnt   <= spim_reg_wdata[23:22];
             end

             if ( spim_reg_be[3] == 1 ) begin
               cfg_m0_g1_wr_dummy_cnt[3:0]<= spim_reg_wdata[27:24];
               cfg_m0_g1_wr_spi_seq[3:0]  <= spim_reg_wdata[31:28];
             end
         end

	 DMEM_CS_AMAP: begin
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_cs0_addr    <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_cs1_addr    <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
                cfg_m0_cs2_addr    <= spim_reg_wdata[23:16];
	     end
             if ( spim_reg_be[3] == 1 ) begin
                cfg_m0_cs3_addr    <= spim_reg_wdata[31:24];
	     end
	 end
	 DMEM_CS_AMAP: begin
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m0_cs0_amask    <= spim_reg_wdata[7:0];
	     end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m0_cs1_amask    <= spim_reg_wdata[15:8];
	     end
             if ( spim_reg_be[2] == 1 ) begin
                cfg_m0_cs2_amask    <= spim_reg_wdata[23:16];
	     end
             if ( spim_reg_be[3] == 1 ) begin
                cfg_m0_cs3_amask    <= spim_reg_wdata[31:24];
	     end
	 end
         IMEM_CTRL1: begin
             if ( spim_reg_be[0] == 1 ) begin
               cfg_m1_cs_reg     <= spim_reg_wdata[3:0]; // Chip Select for Memory Interface
               cfg_m1_spi_imode  <= spim_reg_wdata[5:4]; // Init SPI Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - DDR
               cfg_m1_spi_fmode  <= spim_reg_wdata[7:6]; // Final SPI Mode, 0 - Normal, 1- Double, 2 - Qard, 3 - DDR
             end
             if ( spim_reg_be[0] == 1 ) begin
               cfg_m1_spi_switch    <= spim_reg_wdata[9:8]; // Phase where to switch the SPI Mode
               cfg_m1_dummy_cnt[3:0]<= spim_reg_wdata[13:10];
               cfg_m1_fsm_reset     <= spim_reg_wdata[15];
             end
         end
         IMEM_CTRL2: begin // This register control Direct Memory Access Type
             if ( spim_reg_be[0] == 1 ) begin
                cfg_m1_cmd_reg <= spim_reg_wdata[7:0];
             end
             if ( spim_reg_be[1] == 1 ) begin
                cfg_m1_mode_reg <= spim_reg_wdata[15:8];
             end
             if ( spim_reg_be[2] == 1 ) begin
                cfg_m1_spi_seq[3:0]  <= spim_reg_wdata[19:16];
                cfg_m1_addr_cnt[1:0] <= spim_reg_wdata[21:20];
             end
             if ( spim_reg_be[3] == 1 ) begin
                cfg_m1_data_cnt[7:0]  <= spim_reg_wdata[31:24];
             end
         end
         IMEM_ADDR: begin
           for (byte_index = 0; byte_index < 4; byte_index = byte_index+1 )
               if ( spim_reg_be[byte_index] == 1 )
                 cfg_m1_addr[byte_index*8 +: 8] <= spim_reg_wdata[(byte_index*8) +: 8];
         end
         endcase
         end 
     end 
  end 



  // implement slave model register read mux
  always_comb
    begin
      reg_rdata = '0;
      if(spim_reg_req) begin
          case(spim_reg_addr)
            GLBL_CTRL:         reg_rdata[31:0] = {cfg_m0_fsm_reset,cfg_dpft_dis,10'h0,
		                                  {res_fifo_full,res_fifo_empty,cmd_fifo_full,cmd_fifo_empty},
						  spi_clk_div,cfg_cmd_delay,cfg_cs_late,cfg_cs_early};
	    DMEM_CS0_RD_CTRL:  reg_rdata[31:0] = {cfg_m0_g0_rd_spi_seq   ,cfg_m0_g0_rd_dummy_cnt,cfg_m0_g0_rd_addr_cnt,
		                                  cfg_m0_g0_rd_spi_switch,cfg_m0_g0_rd_spi_fmode,cfg_m0_g0_rd_spi_imode,
						  cfg_m0_g0_rd_mode_reg  ,cfg_m0_g0_rd_cmd_reg};
	    DMEM_CS0_WR_CTRL:  reg_rdata[31:0] = {cfg_m0_g0_wr_spi_seq   ,cfg_m0_g0_wr_dummy_cnt,cfg_m0_g0_wr_addr_cnt,
		                                  cfg_m0_g0_wr_spi_switch,cfg_m0_g0_wr_spi_fmode,cfg_m0_g0_wr_spi_imode,
						  cfg_m0_g0_wr_mode_reg  ,cfg_m0_g0_wr_cmd_reg};
	    DMEM_CS1_RD_CTRL:  reg_rdata[31:0] = {cfg_m0_g1_rd_spi_seq   ,cfg_m0_g1_rd_dummy_cnt,cfg_m0_g1_rd_addr_cnt,
		                                  cfg_m0_g1_rd_spi_switch,cfg_m0_g1_rd_spi_fmode,cfg_m0_g1_rd_spi_imode,
						  cfg_m0_g1_rd_mode_reg  ,cfg_m0_g1_rd_cmd_reg};
	    DMEM_CS1_WR_CTRL:  reg_rdata[31:0] = {cfg_m0_g1_wr_spi_seq   ,cfg_m0_g1_wr_dummy_cnt,cfg_m0_g1_wr_addr_cnt,
		                                  cfg_m0_g1_wr_spi_switch,cfg_m0_g1_wr_spi_fmode,cfg_m0_g1_wr_spi_imode,
						  cfg_m0_g1_wr_mode_reg  ,cfg_m0_g1_wr_cmd_reg};

	    DMEM_CS_AMAP:      reg_rdata[31:0] = {cfg_m0_cs3_addr,cfg_m0_cs2_addr,cfg_m0_cs1_addr,cfg_m0_cs0_addr};
	    DMEM_CS_AMASK:     reg_rdata[31:0] = {cfg_m0_cs3_amask,cfg_m0_cs2_amask,cfg_m0_cs1_amask,cfg_m0_cs0_amask};

            IMEM_CTRL1:        reg_rdata[31:0] =  {16'h0, cfg_m1_fsm_reset,1'b0,cfg_m1_dummy_cnt,cfg_m1_spi_switch,cfg_m1_spi_fmode,cfg_m1_spi_imode,cfg_m1_cs_reg};
	    IMEM_CTRL2:        reg_rdata[31:0] =  {cfg_m1_data_cnt,2'b00,cfg_m1_addr_cnt,cfg_m1_spi_seq,cfg_m1_mode_reg,cfg_m1_cmd_reg};
            IMEM_ADDR:         reg_rdata[31:0] = cfg_m1_addr;
            IMEM_WDATA:        reg_rdata[31:0] = cfg_m1_wdata;
            IMEM_RDATA:        reg_rdata[31:0] = cfg_m1_rdata;
            SPI_STATUS:        reg_rdata[31:0] = spi_debug;
          endcase
       end
    end 

// FSM

always_ff @(negedge rst_n or posedge mclk) begin
    if ( rst_n == 1'b0 ) begin
	cur_cnt <= 'h0;
	state    <= FSM_IDLE;
    end else begin
       if(cfg_m1_fsm_reset) begin
          cur_cnt <= 'h0;
	  state    <= FSM_IDLE;
       end else begin
           cur_cnt <= next_cnt;
       	   state <= next_state;
       end
    end
end

/***********************************************************************************
* This block interface with WishBone Request and Write Command & Read Response FIFO
* **********************************************************************************/

logic [7:0] cfg_data_cnt;
logic [31:0] spim_fifo_wdata;
logic       spim_fifo_req;
assign cfg_data_cnt = cfg_m1_data_cnt-1;

assign spim_fifo_req = cfg_m1_req || spim_fifo_rdata_req || spim_fifo_wdata_req;

assign spim_fifo_wdata = (cfg_m1_req) ?  cfg_m1_wdata :  spim_reg_wdata;

always_comb
begin
   cmd_fifo_wr    = '0;
   cmd_fifo_wdata = '0;

   res_fifo_rd    = 0;
   spim_m1_rdata   = '0;

   spim_m1_ack    = 0;
   spim_m1_rrdy   = 0;
   next_cnt      = cur_cnt;
   next_state    = state;
   spim_m1_rrdy  = 0;
   spim_m1_wrdy  = 0;
   cfg_m1_rdata  = 0;

   case(state)
   FSM_IDLE:  begin
        next_cnt      = 0;
	if(spim_fifo_req && cmd_fifo_empty) begin
	   case(cfg_m1_spi_seq)
	      P_FSM_C: begin
	              cmd_fifo_wdata = {SOC,EOC, {4'b0,cfg_m1_data_cnt[7:0]},
			                cfg_m1_dummy_cnt[3:0], cfg_m1_addr_cnt[1:0],
			                cfg_m1_spi_switch[1:0],
			                cfg_m1_spi_fmode[1:0], cfg_m1_spi_imode[1:0],
					cfg_m1_spi_seq[3:0],cfg_m1_cs_reg[3:0],
					cfg_m1_mode_reg[7:0],cfg_m1_cmd_reg[7:0]};
	              spim_m1_wrdy = 1;
	              next_state = FSM_ACK_PHASE;
	      end
	      P_FSM_CW, 
	      P_FSM_CDW:
	      begin
	          cmd_fifo_wdata = {SOC,NOC, {4'b0,cfg_m1_data_cnt[7:0]},
			                cfg_m1_dummy_cnt[3:0], cfg_m1_addr_cnt[1:0],
			                cfg_m1_spi_switch[1:0],
			                cfg_m1_spi_fmode[1:0], cfg_m1_spi_imode[1:0],
					cfg_m1_spi_seq[3:0],cfg_m1_cs_reg[3:0],
					cfg_m1_mode_reg[7:0],cfg_m1_cmd_reg[7:0]};
	          next_state = FSM_WRITE_PHASE;
	      end
	      P_FSM_CA, 
	      P_FSM_CAR, 
	      P_FSM_CADR,
	      P_FSM_CAMR, 
	      P_FSM_CAMDR, 
	      P_FSM_CAW, 
	      P_FSM_CADW, 
	      P_FSM_CAMW: 
	      begin
	          cmd_fifo_wdata = {SOC,NOC, {4'b0,cfg_m1_data_cnt[7:0]},
			            cfg_m1_dummy_cnt[3:0], cfg_m1_addr_cnt[1:0],
			            cfg_m1_spi_switch[1:0],
			            cfg_m1_spi_fmode[1:0], cfg_m1_spi_imode[1:0],
				    cfg_m1_spi_seq[3:0],cfg_m1_cs_reg[3:0],
				    cfg_m1_mode_reg[7:0],cfg_m1_cmd_reg[7:0]};
	          next_state = FSM_ADR_PHASE;
	      end
	       P_FSM_CDR,
	       P_FSM_CR: 
               begin
	          cmd_fifo_wdata = {SOC,EOC, {4'b0,cfg_m1_data_cnt[7:0]},
			            cfg_m1_dummy_cnt[3:0], cfg_m1_addr_cnt[1:0],
			            cfg_m1_spi_switch[1:0],
			            cfg_m1_spi_fmode[1:0], cfg_m1_spi_imode[1:0],
				    cfg_m1_spi_seq[3:0],cfg_m1_cs_reg[3:0],
				    cfg_m1_mode_reg[7:0],cfg_m1_cmd_reg[7:0]};
	          next_state = FSM_READ_PHASE;
	       end


	   endcase
	   cmd_fifo_wr    = 1;
	end
   end
   // ADDRESS PHASE
   FSM_ADR_PHASE: begin
	  if(!cmd_fifo_full) begin
	      case(cfg_m1_spi_seq)
	         P_FSM_CA:   // COMMAND + ADDRESS PHASE
	         begin
                       cmd_fifo_wdata = {NOC,EOC,16'h0, cfg_m1_addr[31:0]};
	               spim_m1_wrdy = 1;
	               next_state = FSM_ACK_PHASE;
	         end
	         P_FSM_CAR,  // COMMAND + ADDRESS + READ PHASE
                 P_FSM_CADR, // COMMAND + ADDRESS + DUMMY + READ PHASE
	         P_FSM_CAMR, // COMMAND + ADDRESS + MODE + READ PHASE
	         P_FSM_CAMDR: // COMMAND + ADDRESS + MODE + DUMMY + READ PHASE
		 begin
                    cmd_fifo_wdata = {NOC,EOC,16'h0,cfg_m1_addr[31:0]};
	            next_cnt  = 'h0;
	            next_state = FSM_READ_PHASE;
	         end

		 P_FSM_CAW,
		 P_FSM_CADW,
		 P_FSM_CAMW: 
		 begin
                    cmd_fifo_wdata = {NOC,NOC,16'h0,cfg_m1_addr[31:0]};
	            next_cnt  = 'h0;
	            next_state = FSM_WRITE_PHASE;
	         end
	      endcase
              cmd_fifo_wr      = 1;
	  end
   end

   //----------------------------------------------------------
   // Check Resonse FIFO is not empty then read the data from response fifo
   // ---------------------------------------------------------
   FSM_READ_PHASE: begin
	if(res_fifo_empty != 1 && spim_fifo_rdata_req) begin
	   spim_m1_rrdy = 1;
           cfg_m1_rdata = res_fifo_rdata;
	   res_fifo_rd  = 1;
	   if(cfg_data_cnt[7:2] == cur_cnt) begin
	      next_state = FSM_ACK_PHASE;
	   end else begin
	      next_state = FSM_READ_BUSY;
	      next_cnt  = cur_cnt+1;
	    end
	 end
   end
   //----------------------------------------------
   // Wait for Previous Read Data Read
   // ---------------------------------------------
   FSM_READ_BUSY: begin
        spim_m1_rrdy = 0;
	if(spim_fifo_rdata_req == 0) begin
           next_state    = FSM_READ_PHASE;
	end
   end

   //----------------------------------------------------------
   // Check command FIFO is not full and Write Data is available
   // ---------------------------------------------------------
   FSM_WRITE_PHASE: begin
        if(cmd_fifo_full != 1 && spim_fifo_req) begin
	   // If this a single word config cycle or 
           // in crrent spim_fifo_wr request
	   spim_m1_wrdy = 1;
	   if(cfg_data_cnt[7:2] == cur_cnt) begin
              cmd_fifo_wdata = {NOC,EOC,16'b0,spim_fifo_wdata[31:0]};
	      next_state     = FSM_ACK_PHASE;
	   end else begin
              cmd_fifo_wdata = {NOC,NOC,16'b0,spim_fifo_wdata[31:0]};
	      next_state     = FSM_WRITE_BUSY;
	      next_cnt      = cur_cnt+1;
	    end
	   cmd_fifo_wr  = 1;
	 end
   end
   //----------------------------------------------
   // Wait for NEXT Data Ready
   // ---------------------------------------------
   FSM_WRITE_BUSY: begin
	spim_m1_wrdy = 0;
	if(spim_fifo_wdata_req == 0) begin
           next_state    = FSM_WRITE_PHASE;
	end
   end

   FSM_ACK_PHASE: begin
	   spim_m1_ack = 1;
	   next_state = FSM_IDLE;
	end

   endcase


end

endmodule
