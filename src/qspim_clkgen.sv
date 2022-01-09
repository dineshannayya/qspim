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
////  SPI Clkgen  Module                                          ////
////                                                              ////
////  This file is part of the YIFive cores project               ////
////  https://github.com/dineshannayya/yifive_r0.git              ////
////  http://www.opencores.org/cores/yifive/                      ////
////                                                              ////
////  Description                                                 ////
////      This is SPI Master Clock Generation control logic.      ////
////      This logic also generate spi clock rise and fall pulse  ////
////      Basis assumption is master clock is 2x time spi clock   ////
////         1. spi fall pulse is used to transmit spi data       ////
////         2. spi rise pulse is used to received spi data       ////
////     SPI Master Top module                                    ////
////                                                              ////
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision:                                                   ////
////      0.1 - 16th Feb 2021, Dinesh A                           ////
////            Initial version                                   ////
////      0.2 - 24th Mar 2021, Dinesh A                           ////
////            1. Comments are added                             ////
////            2. RTL clean-up done and the output are registred ////
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

module qspim_clkgen
(
    input  logic                        clk,
    input  logic                        rstn,
    input  logic                        en,
    input  logic          [5:0]         cfg_sck_period,
    output logic                        spi_clk,
    output logic                        spi_fall,
    output logic                        spi_rise
);

	logic [5:0] sck_half_period;
	logic [5:0] clk_cnt;

    assign sck_half_period = {1'b0, cfg_sck_period[5:1]};
   
    // The first transition on the sck_toggle happens one SCK period
    // after en is asserted
    always @(posedge clk or negedge rstn) begin
    	if(!rstn) begin
    	   spi_clk    <= 1'b1;
    	end // if (!reset_n)
    	else 
    	begin
    	   if(en) 
    	   begin
    	      if(clk_cnt == sck_half_period) 
    	      begin
    		 spi_clk    <= 1'b0;
    	      end // if (clk_cnt == sck_half_period)
    	      else if(clk_cnt == cfg_sck_period) begin
    		    spi_clk    <= 1'b1;
    	      end 
    	   end else begin
    	      spi_clk    <= 1'b1;
    	   end // else: !if(en)
    	end // else: !if(!reset_n)
    end // always @ (posedge clk or negedge reset_n)

    // Generate Free runnng spi_fall and rise pulse
    // after en is asserted
    always @(posedge clk or negedge rstn) begin
    	if(!rstn) begin
    	   clk_cnt    <= 'h1;
	   spi_fall   <= 1'b0;
	   spi_rise   <= 1'b0;
    	end // if (!reset_n)
    	else 
    	begin
    	   if(clk_cnt == sck_half_period) 
    	   begin
	      spi_fall   <= 1'b0;
	      spi_rise   <= 1'b1;
    	      clk_cnt    <= clk_cnt + 1'b1;
    	   end // if (clk_cnt == sck_half_period)
    	   else begin
    	      if(clk_cnt == cfg_sck_period) 
    	      begin
	         spi_fall   <= 1'b1;
	         spi_rise   <= 1'b0;
    	         clk_cnt    <= 'h1;
    	      end // if (clk_cnt == cfg_sck_period)
    	      else 
    	      begin
    	         clk_cnt    <= clk_cnt + 1'b1;
	         spi_fall   <= 1'b0;
	         spi_rise   <= 1'b0;
    	       end // else: !if(clk_cnt == cfg_sck_period)
    	   end // else: !if(clk_cnt == sck_half_period)
    	end // else: !if(!reset_n)
    end // always @ (posedge clk or negedge reset_n)

endmodule
