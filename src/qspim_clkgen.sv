/*****************************************************************************************************
 * Copyright (c) 2024 SiPlusPlus Semiconductor
 *
 * FileContributor: Dinesh Annayya <dinesha@opencores.org>                       
 * FileContributor: Dinesh Annayya <dinesh@siplusplus.com>                       
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************************************/
/****************************************************************************************************
                                                                  
      SPI Clkgen  Module                                          
                                                                  
                                                                  
      Description                                                 
          This is SPI Master Clock Generation control logic.      
          This logic also generate spi clock rise and fall pulse  
          Basis assumption is master clock is 2x time spi clock   
             1. spi fall pulse is used to transmit spi data       
             2. spi rise pulse is used to received spi data       
         SPI Master Top module                                    
                                                                  
      To Do:                                                      
        nothing                                                   
                                                                  
      Author(s):                                                  
          - Dinesh Annayya <dinesha@opencores.org>               
          - Dinesh Annayya <dinesh@siplusplus.com>               
                                                                  
      Revision:                                                   
          0.1 - 16th Feb 2021, Dinesh A                           
                Initial version                                   
          0.2 - 24th Mar 2021, Dinesh A                           
                1. Comments are added                             
                2. RTL clean-up done and the output are registred 
          0.3 - 1 Aug 2023, Dinesh A                              
                As part of 2206Q bug fix:                         
                spi-clk de-assertion at end of transaction was not
                aligned to spi clock config. Not we are de-assert 
                spi_rise edge, previously it abortly de-asserting 
                at system clock.
                                                                  
 ***************************************************************************************************/

module qspim_clkgen
(
    input  logic                        clk,
    input  logic                        rstn,
    input  logic                        en,
    input  logic          [5:0]         cfg_sck_period,
    output logic                        spi_clk,
    output logic                        spi_fall,
    output logic                        spi_rise,
    output logic                        spi_clk_idle  // Indicate SPI clock in idle phase
);

	logic [5:0] cfg_sck_period_r;
	logic [5:0] sck_half_period;
	logic [5:0] clk_cnt;

    assign sck_half_period = {1'b0, cfg_sck_period[5:1]};
   
    // The first transition on the sck_toggle happens one SCK period
    // after en is asserted
    always @(posedge clk or negedge rstn) begin
    	if(!rstn) begin
    	   spi_clk      <= 1'b1;
           spi_clk_idle <= 1'b1;
    	end // if (!reset_n)
    	else 
    	begin
    	   if(en) 
    	   begin
    	      if(spi_fall) 
    	      begin
    		    spi_clk      <= 1'b0;
                spi_clk_idle <= 1'b0;
    	      end // if (clk_cnt == sck_half_period)
    	      else if(spi_rise) begin
    		    spi_clk      <= 1'b1;
                spi_clk_idle <= 1'b0;
    	      end 
    	   end else begin
    	      if(spi_rise) begin
                 spi_clk      <= 1'b1;
                 spi_clk_idle <= 1'b1;
              end
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
           cfg_sck_period_r <= 'h0;
    	end // if (!reset_n)
    	else 
    	begin
          cfg_sck_period_r <= cfg_sck_period;
          // if there is no change in sck clock, then do normal operation
          if(cfg_sck_period_r == cfg_sck_period) begin 
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
           end else begin // if there is change in sck_period, then reset counter
    	      clk_cnt    <= 'h1;
	          spi_fall   <= 1'b0;
	          spi_rise   <= 1'b0;
           end
    	end // else: !if(!reset_n)
    end // always @ (posedge clk or negedge reset_n)

endmodule
