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
////  SPI WishBone I/F Module                                     ////
////                                                              ////
////  This file is part of the YIFive cores project               ////
////  https://github.com/dineshannayya/yifive_r0.git              ////
////  http://www.opencores.org/cores/yifive/                      ////
////                                                              ////
////  Description                                                 ////
////     SPI WishBone I/F module                                  ////
////     This block support following functionality               ////
////        1. This block Response to Direct Memory Read and      ////
////           Register Write and Read Command                    ////
////        2. In case of Direct Memory Read, It check send the   ////
////           SPI Read command to SPI Ctrl logic and wait for    ////
////           Read data through Response                         ////
////                                                              ////
////  To Do:                                                      ////
////    1. Add 4 Word Memory Fetch for better Through Put         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision :                                                  ////
////     V.0  -  June 30, 2021                                    //// 
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


module qspim_if #( parameter WB_WIDTH = 32,parameter CMD_FIFO_WD=50) (
    input  logic                         mclk,
    input  logic                         rst_n,

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


    // Configuration
    // Direct Memory CS# Address Mapping
    input logic  [7:0]                   cfg_m0_cs0_addr,
    input logic  [7:0]                   cfg_m0_cs1_addr,
    input logic  [7:0]                   cfg_m0_cs2_addr,
    input logic  [7:0]                   cfg_m0_cs3_addr,
    input logic  [7:0]                   cfg_m0_cs0_amask,
    input logic  [7:0]                   cfg_m0_cs1_amask,
    input logic  [7:0]                   cfg_m0_cs2_amask,
    input logic  [7:0]                   cfg_m0_cs3_amask,

    input logic                          cfg_dpft_dis,   // Direct Mem Prefetch enable/disable
    input logic                          cfg_fsm_reset,

    input logic [1:0]                    cfg_m0_g0_rd_spi_imode ,  // Init SPI Mode 
    input logic [1:0]                    cfg_m0_g0_rd_spi_fmode ,  // Final SPI Mode 
    input logic [1:0]                    cfg_m0_g0_rd_spi_switch,  // SPI Mode Switching Place
    input logic [3:0]                    cfg_m0_g0_rd_spi_seq   ,  // SPI SEQUENCE
    input logic [1:0]                    cfg_m0_g0_rd_addr_cnt  ,  // SPI Addr Count
    input logic [3:0]                    cfg_m0_g0_rd_dummy_cnt ,  // SPI Dummy Count
    input logic [7:0]                    cfg_m0_g0_rd_cmd_reg   ,  // SPI MEM COMMAND
    input logic [7:0]                    cfg_m0_g0_rd_mode_reg  ,  // SPI MODE REG

    input logic [1:0]                    cfg_m0_g0_wr_spi_imode ,  // Init SPI Mode 
    input logic [1:0]                    cfg_m0_g0_wr_spi_fmode ,  // Final SPI Mode 
    input logic [1:0]                    cfg_m0_g0_wr_spi_switch,  // SPI Mode Switching Place
    input logic [3:0]                    cfg_m0_g0_wr_spi_seq   ,  // SPI SEQUENCE
    input logic [1:0]                    cfg_m0_g0_wr_addr_cnt  ,  // SPI Addr Count
    input logic [3:0]                    cfg_m0_g0_wr_dummy_cnt ,  // SPI Dummy Count
    input logic [7:0]                    cfg_m0_g0_wr_cmd_reg   ,  // SPI MEM COMMAND
    input logic [7:0]                    cfg_m0_g0_wr_mode_reg  ,  // SPI MODE REG

    input logic [1:0]                    cfg_m0_g1_rd_spi_imode ,  // Init SPI Mode 
    input logic [1:0]                    cfg_m0_g1_rd_spi_fmode ,  // Final SPI Mode 
    input logic [1:0]                    cfg_m0_g1_rd_spi_switch,  // SPI Mode Switching Place
    input logic [3:0]                    cfg_m0_g1_rd_spi_seq   ,  // SPI SEQUENCE
    input logic [1:0]                    cfg_m0_g1_rd_addr_cnt  ,  // SPI Addr Count
    input logic [3:0]                    cfg_m0_g1_rd_dummy_cnt ,  // SPI Dummy Count
    input logic [7:0]                    cfg_m0_g1_rd_cmd_reg   ,  // SPI MEM COMMAND
    input logic [7:0]                    cfg_m0_g1_rd_mode_reg  ,  // SPI MODE REG

    input logic [1:0]                    cfg_m0_g1_wr_spi_imode ,  // Init SPI Mode 
    input logic [1:0]                    cfg_m0_g1_wr_spi_fmode ,  // Final SPI Mode 
    input logic [1:0]                    cfg_m0_g1_wr_spi_switch,  // SPI Mode Switching Place
    input logic [3:0]                    cfg_m0_g1_wr_spi_seq   ,  // SPI SEQUENCE
    input logic [1:0]                    cfg_m0_g1_wr_addr_cnt  ,  // SPI Addr Count
    input logic [3:0]                    cfg_m0_g1_wr_dummy_cnt ,  // SPI Dummy Count
    input logic [7:0]                    cfg_m0_g1_wr_cmd_reg   ,  // SPI MEM COMMAND
    input logic [7:0]                    cfg_m0_g1_wr_mode_reg  ,  // SPI MODE REG


    input logic                          spi_init_done,  // SPI internal Init completed


    // Towards Reg I/F
    output logic                         spim_reg_req,     // Reg Request
    output logic [3:0]                   spim_reg_addr,    // Reg Address
    output logic                         spim_reg_we,      // Reg Write/Read Command
    output logic [3:0]                   spim_reg_be,      // Reg Byte Enable
    output logic [31:0]                  spim_reg_wdata,    // Reg Write Data
    input  logic                         spim_reg_ack,     // Read Ack
    input  logic [31:0]                  spim_reg_rdata,    // Read Read Data

    // Towards Command FIFO
    input  logic                         cmd_fifo_full,   // Command FIFO full
    input  logic                         cmd_fifo_afull,  // Command FIFO full
    input  logic                         cmd_fifo_empty,  // Command FIFO empty
    output logic                         cmd_fifo_wr,     // Command FIFO Write
    output logic [CMD_FIFO_WD-1:0]       cmd_fifo_wdata,  // Command FIFO WData
    
    // Towards Response FIFO
    input  logic                         res_fifo_empty,   // Response FIFO Empty
    output logic                         res_fifo_rd,      // Response FIFO Read
    input  logic [31:0]                  res_fifo_rdata,    // Response FIFO Data

    output  logic [3:0]                  state         
    );

//------------------------------------------------
// Parameter Decleration
// -----------------------------------------------
parameter SOC = 1'b1;    // START of COMMAND
parameter EOC = 1'b1;    // END of COMMAND
parameter NOC = 1'b0;    // NORMAL COMMAND

// State Machine state
parameter IDLE       = 4'b0000;
parameter ADR_PHASE  = 4'b0001;
parameter CMD_WAIT   = 4'b0010;
parameter READ_DATA  = 4'b0011;
parameter WRITE_DATA = 4'b0100;


//---------------------------------------------------------
// Variable declartion
// -------------------------------------------------------
logic                 spim_mem_req   ;  // Current Request is Direct Memory Read


logic [WB_WIDTH-1:0]  spim_wb_wdata  ;
logic [WB_WIDTH-1:0]  spim_wb_addr   ;
logic                 spim_wb_we     ;
logic [3:0]           spim_wb_be     ;
logic [WB_WIDTH-1:0]  spi_mem_rdata  ;
logic [WB_WIDTH-1:0]  spim_wb_rdata  ;
logic                 wbd_stb_l      ;

logic [9:0]           wbd_bl_cnt     ;
logic [9:0]           next_wbd_bl_cnt;
logic                 spim_mem_ack   ;
logic [3:0]           next_state     ;


logic [3:0]           cfg_m0_cs_reg           ;
logic [1:0]           cfg_m0_spi_imode ;  // Init SPI Mode 
logic [1:0]           cfg_m0_spi_fmode ;  // Final SPI Mode 
logic [1:0]           cfg_m0_spi_switch;  // SPI Mode Switching Place
logic [3:0]           cfg_m0_spi_seq   ;  // SPI SEQUENCE
logic [1:0]           cfg_m0_addr_cnt  ;  // SPI Addr Count
logic [3:0]           cfg_m0_dummy_cnt ;  // SPI Dummy Count
logic [7:0]           cfg_m0_cmd_reg   ;  // SPI MEM COMMAND
logic [7:0]           cfg_m0_mode_reg  ;  // SPI MODE REG
logic [11:0]          cfg_m0_data_cnt  ;
logic [31:0]          cfg_m0_wdata     ;
logic [31:0]          cfg_m0_addr     ;
  //---------------------------------------------------------------
  // Address Decoding
  // 0x0000_0000 - 0x0FFF_FFFF  - SPI FLASH MEMORY ACCESS - 256MB
  // 0x1000_0000 -              - SPI Register Access
  // 
  //
  // Note: Only Bit[28] is decoding done here, other Bit decoding 
  // will be done inside the wishbone inter-connect 
  // --------------------------------------------------------------

  assign spim_mem_req = (spi_init_done && wbd_stb_i && !wbd_lack_o && (wbd_adr_i[28] == 1'b0));

  // Generate Once cycle delayed wbd_stb_l
  assign spim_reg_req = (spi_init_done && wbd_stb_i && !wbd_lack_o && (wbd_adr_i[28] == 1'b1)) ;

  assign spim_reg_addr  = wbd_adr_i[5:2];
  assign spim_reg_wdata = wbd_dat_i;
  assign spim_reg_we    = wbd_we_i;
  assign spim_reg_be    = wbd_sel_i;
        
  
  wire wbd_ack  =     (spim_mem_req) ? spim_mem_ack : 
	              (spim_reg_req) ? spim_reg_ack : 1'b0;

always_ff @(negedge rst_n or posedge mclk) begin
    if ( rst_n == 1'b0 ) begin
       wbd_dat_o  <=  'h0;
       wbd_err_o  <=  1'b0;
       wbd_ack_o  <=  'b0;
       wbd_lack_o  <=  'b0;
    end else begin
        wbd_dat_o  <=  (spim_mem_req && !wbd_we_i && wbd_ack ) ? spi_mem_rdata :
	               (spim_reg_req && !wbd_we_i && wbd_ack ) ? spim_reg_rdata: 'h0;
        wbd_err_o  <=  1'b0;

        wbd_ack_o  <=  (spim_mem_req) ? spim_mem_ack : 
	              (spim_reg_req) ? spim_reg_ack : 1'b0;

       wbd_lack_o  <= (spim_mem_req) ? ((wbd_bl_i == 'h1) ? spim_mem_ack : (wbd_bl_cnt == 'h1) &  spim_mem_ack) :
	                ((spim_reg_req) ? spim_reg_ack : 1'b0);
  end
end


  // Detect Pos Edge of strobe
  wire wbd_stb_pedge = (wbd_stb_i && !wbd_stb_l);

  // To reduce the load/Timing Wishbone I/F, all the variable are registered
always_ff @(negedge rst_n or posedge mclk) begin
    if ( rst_n == 1'b0 ) begin
        spim_wb_wdata <= '0;
        spim_wb_rdata <= '0;
        spim_wb_addr  <= '0;
        spim_wb_be    <= '0;
        spim_wb_we    <= '0;
	wbd_stb_l     <= '0;
   end else begin
	if(spi_init_done) begin // Wait for internal SPI Init Done
	    wbd_stb_l     <= wbd_stb_i;
      	    wbd_bl_cnt   <= next_wbd_bl_cnt;
	    if(wbd_stb_pedge) begin
	       spim_wb_addr <= wbd_adr_i;
            end else if(spim_mem_ack) begin
               spim_wb_addr <= spim_wb_addr+4;
	    end

            spim_wb_we    <= wbd_we_i;
    
    
    	    if(!spim_wb_we && spim_mem_req && spim_mem_ack) 
                   spim_wb_rdata <= spi_mem_rdata;
            else if (spim_reg_req && spim_reg_ack)
                   spim_wb_rdata <= spim_reg_rdata;
    
       end
   end
end


// Generate CS# based on the Direct Address Map [27:20]
//
assign  cfg_m0_cs_reg[0] =  ((wbd_adr_i[27:20] & cfg_m0_cs0_amask) == cfg_m0_cs0_addr) ;
assign  cfg_m0_cs_reg[1] =  ((wbd_adr_i[27:20] & cfg_m0_cs1_amask) == cfg_m0_cs1_addr) ;
assign  cfg_m0_cs_reg[2] =  ((wbd_adr_i[27:20] & cfg_m0_cs2_amask) == cfg_m0_cs2_addr) ;
assign  cfg_m0_cs_reg[3] =  ((wbd_adr_i[27:20] & cfg_m0_cs3_amask) == cfg_m0_cs3_addr) ;



// Select the configuration based on the chip select
// Note: CS0/CS1 share same config of g0
//       CS2/CS3 Share same config of g1
always_comb begin
         cfg_m0_spi_imode     = cfg_m0_g0_rd_spi_imode;
         cfg_m0_spi_fmode     = cfg_m0_g0_rd_spi_fmode;
         cfg_m0_spi_switch    = cfg_m0_g0_rd_spi_switch;
         cfg_m0_spi_seq       = cfg_m0_g0_rd_spi_seq;
         cfg_m0_addr_cnt      = cfg_m0_g0_rd_addr_cnt;
         cfg_m0_dummy_cnt     = cfg_m0_g0_rd_dummy_cnt;
         cfg_m0_cmd_reg       = cfg_m0_g0_rd_cmd_reg;
         cfg_m0_mode_reg      = cfg_m0_g0_rd_mode_reg;

	case({cfg_m0_cs_reg[3:0],wbd_we_i})
		5'b00010,5'b00100: begin // CS0/CS1 Read Phase
			cfg_m0_spi_imode     = cfg_m0_g0_rd_spi_imode;
                        cfg_m0_spi_fmode     = cfg_m0_g0_rd_spi_fmode;
                        cfg_m0_spi_switch    = cfg_m0_g0_rd_spi_switch;
                        cfg_m0_spi_seq       = cfg_m0_g0_rd_spi_seq;
                        cfg_m0_addr_cnt      = cfg_m0_g0_rd_addr_cnt;
                        cfg_m0_dummy_cnt     = cfg_m0_g0_rd_dummy_cnt;
                        cfg_m0_cmd_reg       = cfg_m0_g0_rd_cmd_reg;
                        cfg_m0_mode_reg      = cfg_m0_g0_rd_mode_reg;
		end
		5'b00011,5'b00101: begin // CS0/CS1 Write Phase
			            cfg_m0_spi_imode     = cfg_m0_g0_wr_spi_imode;
                        cfg_m0_spi_fmode     = cfg_m0_g0_wr_spi_fmode;
                        cfg_m0_spi_switch    = cfg_m0_g0_wr_spi_switch;
                        cfg_m0_spi_seq       = cfg_m0_g0_wr_spi_seq;
                        cfg_m0_addr_cnt      = cfg_m0_g0_wr_addr_cnt;
                        cfg_m0_dummy_cnt     = cfg_m0_g0_wr_dummy_cnt;
                        cfg_m0_cmd_reg       = cfg_m0_g0_wr_cmd_reg;
                        cfg_m0_mode_reg      = cfg_m0_g0_wr_mode_reg;

		end
		5'b01000,5'b10000: begin // CS2/CS3 Read Phase
			cfg_m0_spi_imode     = cfg_m0_g1_rd_spi_imode;
                        cfg_m0_spi_fmode     = cfg_m0_g1_rd_spi_fmode;
                        cfg_m0_spi_switch    = cfg_m0_g1_rd_spi_switch;
                        cfg_m0_spi_seq       = cfg_m0_g1_rd_spi_seq;
                        cfg_m0_addr_cnt      = cfg_m0_g1_rd_addr_cnt;
                        cfg_m0_dummy_cnt     = cfg_m0_g1_rd_dummy_cnt;
                        cfg_m0_cmd_reg       = cfg_m0_g1_rd_cmd_reg;
                        cfg_m0_mode_reg      = cfg_m0_g1_rd_mode_reg;
		end
		5'b01001,5'b10001: begin // CS2/CS3 Write Phase
			            cfg_m0_spi_imode     = cfg_m0_g1_wr_spi_imode;
                        cfg_m0_spi_fmode     = cfg_m0_g1_wr_spi_fmode;
                        cfg_m0_spi_switch    = cfg_m0_g1_wr_spi_switch;
                        cfg_m0_spi_seq       = cfg_m0_g1_wr_spi_seq;
                        cfg_m0_addr_cnt      = cfg_m0_g1_wr_addr_cnt;
                        cfg_m0_dummy_cnt     = cfg_m0_g1_wr_dummy_cnt;
                        cfg_m0_cmd_reg       = cfg_m0_g1_wr_cmd_reg;
                        cfg_m0_mode_reg      = cfg_m0_g1_wr_mode_reg;
		end


	endcase


end

// To Support byte enabled based write access,
always_comb begin
   cfg_m0_data_cnt = {wbd_bl_i[9:0],2'b0};
   if(wbd_we_i == 0) begin // Read case
      cfg_m0_data_cnt = {wbd_bl_i[9:0],2'b0};
   end else begin
       case(wbd_sel_i)
       4'b0001 : cfg_m0_data_cnt = 12'h1; 
       4'b0010 : cfg_m0_data_cnt = 12'h1; 
       4'b0100 : cfg_m0_data_cnt = 12'h1; 
       4'b1000 : cfg_m0_data_cnt = 12'h1; 
       4'b0011 : cfg_m0_data_cnt = 12'h2; 
       4'b0110 : cfg_m0_data_cnt = 12'h2; 
       4'b1100 : cfg_m0_data_cnt = 12'h2; 
       4'b0111 : cfg_m0_data_cnt = 12'h3; 
       4'b1110 : cfg_m0_data_cnt = 12'h3; 
       default : cfg_m0_data_cnt = {wbd_bl_i[9:0],2'b0};
       endcase
    end
end

always_comb begin
   cfg_m0_wdata = wbd_dat_i;
   case(wbd_sel_i)
   4'b0001 : cfg_m0_wdata  = {24'h0,wbd_dat_i[7:0]};
   4'b0010 : cfg_m0_wdata  = {24'h0,wbd_dat_i[15:8]};
   4'b0100 : cfg_m0_wdata  = {24'h0,wbd_dat_i[23:16]};
   4'b1000 : cfg_m0_wdata  = {24'h0,wbd_dat_i[31:24]};
   4'b0011 : cfg_m0_wdata  = {16'h0,wbd_dat_i[15:0]};
   4'b0110 : cfg_m0_wdata  = {16'h0,wbd_dat_i[23:8]};
   4'b1100 : cfg_m0_wdata  = {16'h0,wbd_dat_i[31:16]};
   4'b0111 : cfg_m0_wdata  = {8'h0,wbd_dat_i[23:0]};
   4'b1110 : cfg_m0_wdata  = {8'h0,wbd_dat_i[31:8]};
   default : cfg_m0_wdata  = wbd_dat_i;
   endcase
end

always_comb begin
   cfg_m0_addr = {spim_wb_addr[31:2],2'b00};
   if(wbd_we_i == 0) begin // Read case
      cfg_m0_addr = {spim_wb_addr[31:2],2'b00};
   end else begin
      case(wbd_sel_i)
      4'b0001 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b00};
      4'b0010 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b01};
      4'b0100 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b10};
      4'b1000 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b11};
      4'b0011 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b00};
      4'b0110 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b01};
      4'b1100 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b10};
      4'b0111 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b00};
      4'b1110 : cfg_m0_addr  = {spim_wb_addr[31:2],2'b01};
      default : cfg_m0_addr  = {spim_wb_addr[31:2],2'b00};
      endcase
   end
end


always_ff @(negedge rst_n or posedge mclk) begin
    if ( rst_n == 1'b0 ) begin
	state <= IDLE;
    end else begin
	if(cfg_fsm_reset) state <= IDLE;
	else state <= next_state;
    end
end

/***********************************************************************************
* This block interface with WishBone Request and Write Command & Read Response FIFO
* **********************************************************************************/

always_comb
begin
   cmd_fifo_wr    = '0;
   cmd_fifo_wdata = '0;
   res_fifo_rd    = 0;
   spi_mem_rdata = '0;
   next_wbd_bl_cnt  = wbd_bl_cnt;

   spim_mem_ack   = 0;
   next_state     = state;
   case(state)
   IDLE:  begin
	if(spim_mem_req && cmd_fifo_empty) begin
	   cmd_fifo_wdata = {SOC,NOC,cfg_m0_data_cnt,
		                  cfg_m0_dummy_cnt[3:0],cfg_m0_addr_cnt[1:0],
		                  cfg_m0_spi_switch[1:0],cfg_m0_spi_fmode[1:0],
		                  cfg_m0_spi_imode[1:0],
				          cfg_m0_spi_seq[3:0],cfg_m0_cs_reg[3:0],
		                  cfg_m0_mode_reg[7:0],cfg_m0_cmd_reg[7:0]};
	   next_wbd_bl_cnt = wbd_bl_i;
	   cmd_fifo_wr    = 1;
	   next_state = ADR_PHASE;
	end
   end
   ADR_PHASE: begin
	  if(spim_wb_we) begin
             cmd_fifo_wdata = {NOC,NOC,16'h0,cfg_m0_addr};
             next_state = WRITE_DATA;
          end else begin
              cmd_fifo_wdata = {NOC,EOC,16'h0,cfg_m0_addr};
             next_state = CMD_WAIT;
          end
          cmd_fifo_wr      = 1;
   end
   CMD_WAIT: begin
	  // Wait for Command Accepted, before reading data
	  // to take care of staled data being read due to pre-fetch logic
	  if(cmd_fifo_empty) next_state = READ_DATA;
    end


   READ_DATA: begin
	if(res_fifo_empty != 1 && wbd_bry_i) begin
       spi_mem_rdata    = res_fifo_rdata;
	   next_wbd_bl_cnt  = wbd_bl_cnt-1;
	   res_fifo_rd   = 1;
       spim_mem_ack  = 1;
	   if(next_wbd_bl_cnt == 0) 
               next_state    = IDLE;
	end
   end
   WRITE_DATA: begin
       if(wbd_ack_o)begin 
           cmd_fifo_wdata = {NOC,EOC,16'b0,cfg_m0_wdata[31:0]};
           cmd_fifo_wr    = 1;
	end
	if(cmd_fifo_full != 1 && !(cmd_fifo_afull && wbd_ack_o) &&  wbd_bry_i) begin
	   if(next_wbd_bl_cnt == 0)  begin
              next_state   = IDLE;
           end else begin
	      next_wbd_bl_cnt = wbd_bl_cnt-1;
              spim_mem_ack = 1;
           end
	end
   end
   endcase
end

    


endmodule
