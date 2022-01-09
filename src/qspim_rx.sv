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
////  SPI RX  Module                                              ////
////                                                              ////
////  This file is part of the YIFive cores project               ////
////  https://github.com/dineshannayya/yifive_r0.git              ////
////  http://www.opencores.org/cores/yifive/                      ////
////                                                              ////
////  Description                                                 ////
////     SPI RX module                                            ////
////                                                              ////
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision :                                                  ////
////     V.0  -  June 8, 2021                                     //// 
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


module qspim_rx #(
		parameter ENDIEAN = 0  // 0 - Little, 1 - Big endian, since RISV is Little indian default set 0
	)	
(
    input  logic        clk,
    input  logic        rstn,
    input  logic        flush,
    input  logic        en,
    input  logic        rx_edge,
    output logic        rx_done,
    input  logic        sdi0,
    input  logic        sdi1,
    input  logic        sdi2,
    input  logic        sdi3,
    input  logic [1:0]  s_spi_mode,
    input  logic [15:0] counter_in,
    input  logic        counter_in_upd,
    output logic [31:0] data,
    input  logic        data_ready,
    output logic        data_valid,
    output logic        clk_en_o
);
//------------------------------------------------------
// Parameter Decleration
// -----------------------------------------------------
  parameter P_SINGLE = 2'b00;
  parameter P_DOUBLE = 2'b01;
  parameter P_QUAD   = 2'b10;
  parameter P_QDDR   = 2'b11;

//------------------------------------------------------
// Variable Decleration
// -----------------------------------------------------

  logic [31:0] data_int;
  logic [31:0] data_int_next;
  logic [15:0] counter;
  logic [15:0] counter_trgt;
  logic [15:0] counter_next;
  logic        reg_done;
  logic        data_valid_i;
  logic        qddr_rx_en;
  enum logic [1:0] { IDLE, RECEIVE, WAIT_FIFO, WAIT_FIFO_DONE } rx_CS, rx_NS;


  assign reg_done  = (s_spi_mode == P_SINGLE && (counter[4:0] == 5'b11111)) || 
	             (s_spi_mode == P_DOUBLE && (counter[3:0] == 4'b1111)) ||
	             (s_spi_mode == P_QUAD && (counter[2:0] == 3'b111))    ||
	             (s_spi_mode == P_QDDR && (counter[2:0] == 3'b111));



  always_comb
  begin
    rx_NS         = rx_CS;
    data_int_next = data_int;
    data_valid_i    = 1'b0;
    counter_next  = counter;

    case (rx_CS)
      IDLE: begin

        // check first if there is available space instead of later
        if (en) begin
          rx_NS = RECEIVE;
        end
      end

      RECEIVE: begin

        if (rx_edge || qddr_rx_en) begin
          counter_next = counter + 1;
          if ((s_spi_mode == P_QUAD ) || (s_spi_mode == P_QDDR ))
             data_int_next = {data_int[27:0],sdi3,sdi2,sdi1,sdi0};
          else if (s_spi_mode == P_DOUBLE )
             data_int_next = {data_int[29:0],sdi1,sdi0};
          else
             data_int_next = {data_int[30:0],sdi1};

          if (rx_done) begin
               counter_next = 0;
	       if (data_ready) begin
                 data_valid_i   = 1'b1;
                 rx_NS = IDLE;
	       end else
                 rx_NS = WAIT_FIFO_DONE;
          end else if (reg_done) begin
	    if (data_ready) begin
              data_valid_i   = 1'b1;
            end else begin
              // no space in the FIFO, wait for free space
              rx_NS    = WAIT_FIFO;
            end
          end
        end
      end

      WAIT_FIFO_DONE: begin
	  if (data_ready) begin
             data_valid_i = 1'b1;
             rx_NS = IDLE;
	 end
      end

      WAIT_FIFO: begin
	 if (data_ready) begin
            data_valid_i = 1'b1;
            rx_NS = RECEIVE;
	 end
      end
    endcase
  end


  always_ff @(posedge clk, negedge rstn)
  begin
    if (rstn == 0)
    begin
      counter      <= 0;
      counter_trgt <= 'h8;
      data_int     <= '0;
      rx_done      <= '0;
      clk_en_o     <= '0;
      data         <= 'b0;
      data_valid   <= 1'b0;
      qddr_rx_en   <= '0;
      rx_CS        <= IDLE;
    end else if(flush && rx_edge) begin
        counter      <= 0;
        counter_trgt <= 'h8;
        data_int     <= '0;
        rx_done      <= '0;
        clk_en_o     <= '0;
        data         <= 'b0;
        data_valid   <= 1'b0;
	qddr_rx_en   <= 0;
        rx_CS        <= IDLE;
    end else begin
        // Enable qddr rx after first rx edge
        if(en && rx_edge && (rx_CS == RECEIVE) && (s_spi_mode ==P_QDDR)) begin
	   qddr_rx_en <= 1;
        end else if(!en || rx_done) begin
	   qddr_rx_en <= 0;
	end
	
       data_valid <= data_valid_i;
       data <= (ENDIEAN) ? data_int_next : {data_int_next[7:0],data_int_next[15:8],data_int_next[23:16],data_int_next[31:24]};
       clk_en_o     <= (rx_NS == RECEIVE);
       if (rx_edge ||  qddr_rx_en) begin
          counter      <= counter_next;
          data_int     <= data_int_next;
          rx_CS        <= rx_NS;
          rx_done      <= (counter_next == (counter_trgt-1)) && (rx_NS == RECEIVE);
          clk_en_o     <= (rx_NS == RECEIVE);
       end
       if (en && counter_in_upd) begin
           counter_trgt <= (s_spi_mode ==P_QDDR )   ? {2'b00,counter_in[15:2]} : 
		           (s_spi_mode ==P_QUAD )   ? {2'b00,counter_in[15:2]} : 
		           (s_spi_mode ==P_DOUBLE ) ? {1'b0,counter_in[15:1]} : counter_in;
       end
    end
  end

endmodule
