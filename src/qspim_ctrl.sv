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

      SPI CTRL I/F Module                                         
                                                                  
      This file is part of the YIFive cores project               
      https://github.com/dineshannayya/yifive_r0.git              
      http://www.opencores.org/cores/yifive/                      
                                                                  
      Description                                                 
                                                                  
      To Do:                                                      
        nothing                                                   
                                                                  
      Author(s):                                                  
          - Dinesh Annayya <dinesha@opencores.org>               
          - Dinesh Annayya <dinesh@siplusplus.com>               
                                                                  
      Revision :                                                  
         V.0  -  June 8, 2021                                      
                                                                  
 ***************************************************************************************************/

module qspim_ctrl  #(
     parameter CMD_FIFO_WD=50,
     parameter ENDIEAN = 0  // 0 - Little, 1 - Big endian, since RISV is Little indian default set 0
     )

(
    input  logic                          clk,
    input  logic                          rstn,

    input  logic                    [7:0] g0_spi_clk_div,
    input  logic                    [7:0] g1_spi_clk_div,
    output logic                    [9:0] spi_status,

    // Master 0 Configuration

    input  logic [1:0]                   cfg_cs_early     ,  // Amount of cycle early CS asserted
    input  logic [1:0]                   cfg_cs_late      ,  // Amount of cycle late CS de-asserted
    input  logic [3:0]                   cfg_cmd_delay    ,  // Amount of cycle delay between command

    // Master 0 Command FIFO Interface
    input  logic                         m0_cmd_fifo_empty,
    output logic                         m0_cmd_fifo_rd,
    input  logic [CMD_FIFO_WD-1:0]       m0_cmd_fifo_rdata,

    // Master 0 response FIFO Interface
    output logic 	                     m0_res_fifo_flush,
    input  logic                         m0_res_fifo_empty,
    input  logic                         m0_res_fifo_full,
    input  logic                         m0_res_fifo_afull,
    output logic                         m0_res_fifo_wr,
    output logic [31:0]                  m0_res_fifo_wdata,

    // Master 1 Command FIFO Interface
    output logic 	                     m1_res_fifo_flush,
    input  logic                         m1_cmd_fifo_empty,
    output logic                         m1_cmd_fifo_rd,
    input  logic [CMD_FIFO_WD-1:0]       m1_cmd_fifo_rdata,

    // Master 1 response FIFO Interface
    input  logic                         m1_res_fifo_empty,
    input  logic                         m1_res_fifo_full,
    input  logic                         m1_res_fifo_afull,
    output logic                         m1_res_fifo_wr,
    output logic [31:0]                  m1_res_fifo_wdata,

    output logic [3:0]                   ctrl_state,

    output logic                          spi_clk,
    output logic                          spi_csn0,
    output logic                          spi_csn1,
    output logic                          spi_csn2,
    output logic                          spi_csn3,
    output logic                    [1:0] spi_mode,
    output logic                          spi_sdo0,
    output logic                          spi_sdo1,
    output logic                          spi_sdo2,
    output logic                          spi_sdo3,
    input  logic                          spi_sdi0,
    input  logic                          spi_sdi1,
    input  logic                          spi_sdi2,
    input  logic                          spi_sdi3,
    output logic                          spi_en_tx_out // Spi Direction control
);

//--------------------------------------
// Parameter
// --------------------------------------
parameter P_SINGLE = 2'b00;
parameter P_DOUBLE = 2'b01;
parameter P_QUAD   = 2'b10;
parameter P_QDDR   = 2'b11;


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

//---------------------
  parameter P_8BIT   = 2'b00;
  parameter P_16BIT  = 2'b01;
  parameter P_24BIT  = 2'b10;
  parameter P_32BIT  = 2'b11;

//---- Phase where to switch the SPI Mode
//---- This need to decided based on command
  parameter P_MODE_SWITCH_IDLE     = 2'b00;
  parameter P_MODE_SWITCH_AT_ADDR  = 2'b01;
  parameter P_MODE_SWITCH_AT_DATA  = 2'b10;
//----------------------------------------
// Local Variable
// ---------------------------------------
  logic spi_rise;
  logic spi_fall;
  logic spi_clk_idle; // Indicate SPI clock in idle phase
  logic [7:0] spi_clk_div;

  logic spi_clock_en;

  logic spi_en_rx;
  logic spi_en_tx;


  logic [15:0] counter_tx;
  logic        counter_tx_valid;
  logic [15:0] counter_rx;
  logic        counter_rx_valid;

  logic        dummy_phase;
  logic [31:0] data_to_tx;
  logic        data_to_tx_valid;
  logic        data_to_tx_ready;
  logic        tx_data_ready;


  logic       tx_done;
  logic       rx_done;

  logic [1:0] s_spi_mode;

  logic       ctrl_data_valid;

  logic       spi_cs;

  logic        tx_clk_en;
  logic        rx_clk_en;
  logic [3:0]  cnt; // counter for cs assertion and de-assertion
  logic [3:0]  nxt_cnt;
  logic [1:0]  gnt;


  logic  [11:0] cfg_data_cnt    ;
  logic  [3:0] cfg_dummy_cnt   ;
  logic  [1:0] cfg_addr_cnt    ;
  logic  [3:0] cfg_spi_seq     ;
  logic [7:0]  spi_mode_cmd    ;
  

  enum logic [2:0] {DATA_NULL,DATA_EMPTY,DATA_CMD,DATA_ADDR,DATA_MODE,DATA_FIFO} ctrl_data_mux;

  enum logic [4:0] {
            FSM_IDLE        = 5'h0,
            FSM_CMD_WAIT    = 5'h1,
	        FSM_CS_ASSERT   = 5'h2,
		    FSM_CMD_PHASE   = 5'h3,
		    FSM_ADR_PHASE   = 5'h4,
		    FSM_DUMMY_PHASE = 5'h5,
		    FSM_MODE_PHASE  = 5'h6,
		    FSM_WRITE_CMD   = 5'h7,
		    FSM_WRITE_PHASE = 5'h8,
	        FSM_READ_WAIT   = 5'h9,
		    FSM_READ_PHASE  = 5'hA,
		    FSM_TX_DONE     = 5'hB,
		    FSM_FLUSH       = 5'hC,
		    FSM_CLK_IDLE    = 5'hD,
		    FSM_CS_DEASEERT = 5'hE} state,next_state;

 
  assign ctrl_state =  state[3:0];

  assign spi_mode = s_spi_mode;

  //----------------------------
  // Configuration
  //----------------------------
  logic [3:0]  cfg_cs_reg    ;  // Chip select
  logic [1:0]  cfg_spi_imode ;  // Init SPI Mode 
  logic [1:0]  cfg_spi_fmode ;  // Final SPI Mode 
  logic [1:0]  cfg_spi_switch;  // SPI Mode Switching Place

  

  //----------------------------
  // Command FIFO
  //----------------------------
  logic                         cmd_fifo_empty;
  logic                         cmd_fifo_rd;
  logic [CMD_FIFO_WD-1:0]       cmd_fifo_rdata;

  assign cmd_fifo_empty = (gnt == 2'b01) ? m0_cmd_fifo_empty : m1_cmd_fifo_empty;
  assign cmd_fifo_rdata = (gnt == 2'b01) ? m0_cmd_fifo_rdata : m1_cmd_fifo_rdata;

  assign m0_cmd_fifo_rd = (gnt == 2'b01) ? cmd_fifo_rd : 1'b0;
  assign m1_cmd_fifo_rd = (gnt == 2'b10) ? cmd_fifo_rd : 1'b0;

  //----------------------------
  // Response FIFO
  //----------------------------
  logic              res_fifo_empty;
  logic              res_fifo_full;
  logic              res_fifo_wr;
  logic [31:0]       res_fifo_wdata;

  assign res_fifo_empty = (gnt == 2'b01) ? m0_res_fifo_empty : m1_res_fifo_empty;
  assign res_fifo_full  = (gnt == 2'b01) ? (m0_res_fifo_full | m0_res_fifo_afull)  : (m1_res_fifo_full | m1_res_fifo_afull) ;

  assign m0_res_fifo_wr = (gnt == 2'b01) ? res_fifo_wr : 1'b0;
  assign m1_res_fifo_wr = (gnt == 2'b10) ? res_fifo_wr : 1'b0;

  assign m0_res_fifo_wdata = (gnt == 2'b01) ? res_fifo_wdata : 1'b0;
  assign m1_res_fifo_wdata = (gnt == 2'b10) ? res_fifo_wdata : 1'b0;

  //---------------------------------------------------------------------------
  // To take care of partial/stall data in response fifo
  // we are flushing the content
  //
  // WARNING: This will work well for burst size 4,
  // If User given 6 Word Burst and Read only one location
  // Read Path will hang waiting for Response FIFO to empty, User need to take
  // care of partial reading case.
  //---------------------------------------------------------------------------
  
  logic  fsm_flush;
  logic  spi_dummy;
  assign m0_res_fifo_flush   =  (gnt == 2'b01) ? fsm_flush : 1'b0;
  assign m1_res_fifo_flush   =  (gnt == 2'b10) ? fsm_flush : 1'b0;

  assign spi_clock_en =  tx_clk_en |  rx_clk_en;


  // Map the SPI clock divider based on chip select
  // As some of the Device Access delay are different, we have given support for 
  // two different speed device support
  always_comb begin
      case(cfg_cs_reg)
      4'b0001 : spi_clk_div = g0_spi_clk_div;
      4'b0010 : spi_clk_div = g0_spi_clk_div;
      4'b0100 : spi_clk_div = g1_spi_clk_div;
      4'b1000 : spi_clk_div = g1_spi_clk_div;
      default : spi_clk_div = g0_spi_clk_div;
      endcase
  end

  qspim_clkgen u_clkgen
  (
    .clk            ( clk                    ),
    .rstn           ( rstn                   ),
    .en             ( spi_clock_en           ),
    .cfg_sck_period ( spi_clk_div [5:0]      ),
    .spi_clk        ( spi_clk                ),
    .spi_fall       ( spi_fall               ),
    .spi_rise       ( spi_rise               ),
    .spi_clk_idle   ( spi_clk_idle           )
  );
  qspim_tx u_txreg
  (
    .clk            ( clk                    ),
    .rstn           ( rstn                   ),
    .flush          ( fsm_flush              ),
    .en             ( spi_en_tx              ),
    .tx_edge        ( spi_fall               ),
    .tx_done        ( tx_done                ),
    .sdo0           ( spi_sdo0               ),
    .sdo1           ( spi_sdo1               ),
    .sdo2           ( spi_sdo2               ),
    .sdo3           ( spi_sdo3               ),
    .s_spi_mode     ( s_spi_mode             ),
    .counter_in     ( counter_tx             ),
    .counter_in_upd ( counter_tx_valid       ),
    .dummy_phase    ( dummy_phase            ),
    .txdata         ( data_to_tx             ),
    .data_valid     ( data_to_tx_valid       ),
    .data_ready     ( tx_data_ready          ),
    .spi_dummy      ( spi_dummy              ),
    .clk_en_o       ( tx_clk_en              )
  );
  qspim_rx #(.ENDIEAN(ENDIEAN)) u_rxreg
  (
    .clk            ( clk                    ),
    .rstn           ( rstn                   ),
    .flush          ( fsm_flush              ),
    .en             ( spi_en_rx              ),
    .rx_edge        ( spi_rise               ),
    .rx_done        ( rx_done                ),
    .sdi0           ( spi_sdi0               ),
    .sdi1           ( spi_sdi1               ),
    .sdi2           ( spi_sdi2               ),
    .sdi3           ( spi_sdi3               ),
    .s_spi_mode     ( s_spi_mode             ),
    .counter_in     ( counter_rx             ),
    .counter_in_upd ( counter_rx_valid       ),
    .data           ( res_fifo_wdata         ),
    .data_valid     ( res_fifo_wr            ),
    .data_ready     ( !res_fifo_full         ),
    .clk_en_o       ( rx_clk_en              )
  );

  always_comb
  begin
      data_to_tx       =  'h0;
      data_to_tx_valid = 1'b0;
      dummy_phase       = 1'b0;

      case(ctrl_data_mux)
          DATA_NULL:
          begin
              data_to_tx       =  '0;
              data_to_tx_valid = 1'b0;
          end

          DATA_EMPTY:
          begin
	      dummy_phase       =  1'b1;
              data_to_tx       =  '0;
              data_to_tx_valid = 1'b1;
          end

          DATA_CMD:
          begin
              data_to_tx       = {cmd_fifo_rdata[7:0],24'h0};
              data_to_tx_valid = ctrl_data_valid;
          end
          DATA_MODE:
          begin
              data_to_tx       = {spi_mode_cmd,24'h0};
              data_to_tx_valid = ctrl_data_valid;
          end

          DATA_ADDR:
          begin
              data_to_tx       = (cfg_addr_cnt == P_8BIT)  ? {cmd_fifo_rdata[7:0],24'h0}  :
		                 (cfg_addr_cnt == P_16BIT) ? {cmd_fifo_rdata[15:0],16'h0} :
		                 (cfg_addr_cnt == P_24BIT) ? {cmd_fifo_rdata[23:0],8'h0}  : {cmd_fifo_rdata[31:0]};
              data_to_tx_valid = ctrl_data_valid;
          end

	  // RISV is little endian, so data is converted to little endian format
          DATA_FIFO: begin
             data_to_tx     = (ENDIEAN) ? cmd_fifo_rdata[31:0] : 
		                 {cmd_fifo_rdata[7:0],cmd_fifo_rdata[15:8],cmd_fifo_rdata[23:16],cmd_fifo_rdata[31:24]};
             data_to_tx_valid  = !cmd_fifo_empty;
          end
      endcase
  end

  always_comb
  begin
    fsm_flush          = 0;
    counter_tx         =  '0;
    counter_tx_valid   = 1'b0;
    counter_rx         =  '0;
    counter_rx_valid   = 1'b0;
    next_state         = state;
    ctrl_data_mux      = DATA_NULL;
    ctrl_data_valid    = 1'b0;
    spi_en_rx          = 1'b0;
    spi_en_tx          = 1'b0;
    spi_status         =  '0;
    cmd_fifo_rd        = 1'b0;
    nxt_cnt            = cnt;
    case(state)
      FSM_IDLE:
      begin
        spi_status[0] = 1'b1;
	    if(!m0_cmd_fifo_empty || !m1_cmd_fifo_empty )  begin
	       if(cfg_cmd_delay == cnt) begin // If between cmd wait delay met
               nxt_cnt     = 0;
	           next_state  = FSM_CS_ASSERT;
           end else begin // if between cmd wait delay not met
               nxt_cnt = nxt_cnt+1;
	           next_state  = FSM_CMD_WAIT;
           end
	    end else begin
	       if(cfg_cmd_delay != cnt) begin
              nxt_cnt = nxt_cnt+1;
           end
        end
      end

      // To avoid one cycle between two back to back command
      // We added command wait
      FSM_CMD_WAIT:
      begin
        spi_status[0] = 1'b1;
	    if(cfg_cmd_delay == cnt) begin
           nxt_cnt     = 0;
	       next_state  = FSM_CS_ASSERT;
	    end else begin
           nxt_cnt = nxt_cnt+1;
        end
      end

      // Asserted CS# low
      FSM_CS_ASSERT: begin
	    fsm_flush=1; // Flush stale data in response fifo
	    if(cfg_cs_early == cnt) begin
	       next_state  = FSM_CMD_PHASE;
	    end else begin
           nxt_cnt = nxt_cnt+1;
	    end
      end

      // WAIT for COMMAND Phase Completed
      FSM_CMD_PHASE: begin
        spi_status[1] = 1'b1;
              counter_tx       = 8'h8;
              ctrl_data_mux    = DATA_CMD;
              ctrl_data_valid  = 1'b1;
              counter_tx       = 'd8;
              counter_tx_valid = 1'b1;
              spi_en_tx        = 1'b1;
	 if (tx_data_ready) begin
	      cmd_fifo_rd      = 1'b1;
	      case(cfg_spi_seq)
	      P_FSM_C:     next_state = FSM_TX_DONE;
	      P_FSM_CW:    next_state = FSM_WRITE_CMD;
	      P_FSM_CA:    next_state = FSM_ADR_PHASE;
	      P_FSM_CAR:   next_state = FSM_ADR_PHASE;
              P_FSM_CADR:  next_state = FSM_ADR_PHASE;
	      P_FSM_CAMR:  next_state = FSM_ADR_PHASE;
	      P_FSM_CAMDR: next_state = FSM_ADR_PHASE;
	      P_FSM_CAW:   next_state = FSM_ADR_PHASE;
	      P_FSM_CADW:  next_state = FSM_ADR_PHASE;
	      P_FSM_CAMW:  next_state = FSM_ADR_PHASE;
	      P_FSM_CDR:   next_state = FSM_DUMMY_PHASE;
	      P_FSM_CDW:   next_state = FSM_DUMMY_PHASE;
	      P_FSM_CR:    next_state = FSM_READ_WAIT;
	      default  :   next_state = FSM_TX_DONE;
              endcase
	  end
      end

      // WAIT for ADDR Command Accepted
      FSM_ADR_PHASE: begin
          nxt_cnt          = 0;
          ctrl_data_mux    = DATA_ADDR;
          ctrl_data_valid  = 1'b1;
          counter_tx       =  (cfg_addr_cnt == P_8BIT) ? 'd8 :
	                      (cfg_addr_cnt == P_16BIT) ? 'd16 :
	                      (cfg_addr_cnt == P_24BIT) ? 'd24 : 'd32;
          counter_tx_valid = 1'b1;
          spi_en_tx        = 1'b1;
	  if (tx_data_ready) begin
              ctrl_data_valid  = 1'b0;
	      cmd_fifo_rd      = 1'b1;
	      case(cfg_spi_seq)
	      P_FSM_CA:    next_state = FSM_TX_DONE;
	      P_FSM_CAR:   next_state = FSM_READ_WAIT;
              P_FSM_CADR:  next_state = FSM_DUMMY_PHASE;
	      P_FSM_CAMR:  next_state = FSM_MODE_PHASE;
	      P_FSM_CAMDR: next_state = FSM_MODE_PHASE;
	      P_FSM_CAW:   next_state = FSM_WRITE_CMD;
	      P_FSM_CADW:  next_state = FSM_DUMMY_PHASE;
	      P_FSM_CAMW:  next_state = FSM_MODE_PHASE;
	      default  :   next_state = FSM_TX_DONE;
              endcase
           end
        end

      // WAIT for DUMMY command Accepted
      FSM_DUMMY_PHASE: begin
          nxt_cnt          = 0;
          ctrl_data_mux    = DATA_EMPTY;
          ctrl_data_valid  = 1'b1;
          counter_tx_valid = 1'b1;
	  // Convert into Bit format
          counter_tx       =  'd8 + (cfg_dummy_cnt << 3); // Convert into Bit format
          spi_en_tx        = 1'b1;
	  if (tx_data_ready) begin
              ctrl_data_valid = 1'b0;
	      case(cfg_spi_seq)
              P_FSM_CADR:  next_state = FSM_READ_WAIT;
	          P_FSM_CAMDR: next_state = FSM_READ_WAIT;
	          P_FSM_CADW:  next_state = FSM_WRITE_CMD;
	          P_FSM_CDR:   next_state = FSM_READ_WAIT;
	          P_FSM_CDW:   next_state = FSM_WRITE_CMD;
	          default  :   next_state = FSM_CLK_IDLE;
              endcase
           end
        end
      // WAIT for MODE command accepted
      FSM_MODE_PHASE: begin
          nxt_cnt          = 0;
          ctrl_data_mux    = DATA_MODE;
          ctrl_data_valid  = 1'b1;
          counter_tx_valid = 1'b1;
          counter_tx       = 'd8;
          spi_en_tx        = 1'b1;
	  if (tx_data_ready) begin
	      case(cfg_spi_seq)
	      P_FSM_CAMR:  next_state = FSM_READ_WAIT;
	      P_FSM_CAMDR: next_state = FSM_DUMMY_PHASE;
	      P_FSM_CAMW:  next_state = FSM_WRITE_CMD;
	      default  :   next_state = FSM_CLK_IDLE;
              endcase
           end
        end

      // Wait for WRITE COMMAND ACCEPTED
      FSM_WRITE_CMD: begin
          spi_status[2] = 1'b1;
          nxt_cnt          = 0;
          ctrl_data_mux    = DATA_FIFO;
          ctrl_data_valid  = 1'b1;
          counter_tx_valid = 1'b1;
          counter_tx       = {1'b0,cfg_data_cnt[11:0],3'b000}; // Convert Byte to Bit Count
          spi_en_tx        = 1'b1;
	  if (tx_data_ready) begin
	      cmd_fifo_rd      = 1'b1;
	      next_state = FSM_WRITE_PHASE;
	   end
        end

      // Wait for ALL WRITE DATA ACCEPTED
      FSM_WRITE_PHASE: begin
          spi_status[3] = 1'b1;
          nxt_cnt          = 0;
          ctrl_data_mux    = DATA_FIFO;
          ctrl_data_valid  = 1'b1;
          spi_en_tx        = 1'b1;
	  if (tx_done) begin
	      next_state = FSM_CLK_IDLE;
           end else if(tx_data_ready  && cmd_fifo_empty == 0) begin
	      // Once Current Data is accepted by TX FSM, check FIFO not empty
	      // and read next location
	      cmd_fifo_rd      = 1'b1;
	   end
        end

      // Wait for Previous TX Completeion
      FSM_READ_WAIT: begin
          spi_status[4] = 1'b1;
          spi_en_tx        = 1'b1;
	  if (tx_done) begin
	      next_state = FSM_READ_PHASE;
	  end
      end

      FSM_READ_PHASE: begin
          spi_status[5] = 1'b1;
          nxt_cnt          = 0;
          counter_rx_valid = 1'b1;
          counter_rx       = {1'b0,cfg_data_cnt[11:0],3'b000}; // Convert Byte to Bit Count
          spi_en_rx        = 1'b1;
	  if(!cmd_fifo_empty) begin
             // If you see new command request, then abort the current request
	      next_state = FSM_FLUSH;
	  end else begin
	     if (rx_done && spi_rise) begin
	         next_state = FSM_CLK_IDLE;
             end 
	  end
        end

      FSM_FLUSH: begin
       spi_status[6] = 1'b1;
	   fsm_flush = 1;
	   // Wait for safe SPI-clock de-assertion phase
	   if(spi_clock_en ==0) begin 
	         next_state = FSM_CLK_IDLE;
	   end
      end
      // Wait for TX Done
      FSM_TX_DONE: begin
       spi_status[7] = 1'b1;
       spi_en_tx        = 1'b1;
	     if(tx_done) next_state  = FSM_CLK_IDLE;
      end

      // WAIT for SPI CLOCK entering into IDLE Phase
      FSM_CLK_IDLE: begin
       spi_status[8] = 1'b1;
       nxt_cnt          = 0;
         if(spi_en_tx_out) // If current transaction, then extend the tx drive phase 
            spi_en_tx        = 1'b1;
	     if(spi_clk_idle) next_state  = FSM_CS_DEASEERT;
      end

      // De-assert CS#
      FSM_CS_DEASEERT: begin
      spi_status[9] = 1'b1;
	 if(cfg_cs_late == cnt) begin
	     next_state  = FSM_IDLE;
	     nxt_cnt     = 0;
	 end else begin
             nxt_cnt = nxt_cnt+1;
	 end
      end
   endcase
end




  always @(posedge clk or negedge rstn) begin
    if (rstn == 1'b0) begin
      state       <= FSM_IDLE;
      cnt         <= 'h0;
    end else begin
       state <= next_state;
       cnt   <= nxt_cnt;
    end
  end

  //---------------------------------------------------------------------
  //  Grant Generation Based on FIFO empty, priority given to Master 0
  //  Grant switch happens only at FSM IDLE State
  // ---------------------------------------------------------------------

  always @(posedge clk or negedge rstn) begin
    if (rstn == 1'b0) begin
      gnt             <= 0;
      spi_mode_cmd    <= 'h0;
      cfg_spi_seq     <= 'h0;
      cfg_addr_cnt    <= 'h0;
      cfg_dummy_cnt   <= 'h0;
      cfg_data_cnt    <= 'h0;
      cfg_cs_reg      <= 'h0;
      cfg_spi_imode   <= 'h0;
      cfg_spi_fmode   <= 'h0;
      cfg_spi_switch  <= 'h0;
      spi_en_tx_out   <= 1'b0;
    end else begin
       spi_en_tx_out  <= spi_en_tx && (spi_dummy ==0); // Don't Drive Tx On Dummy Phase
       if(state == FSM_IDLE) begin
           if(!m0_cmd_fifo_empty) begin
              cfg_data_cnt    <= m0_cmd_fifo_rdata[47:36];
              cfg_dummy_cnt   <= m0_cmd_fifo_rdata[35:32];
              cfg_addr_cnt    <= m0_cmd_fifo_rdata[31:30];
	          cfg_spi_switch  <= m0_cmd_fifo_rdata[29:28];
              cfg_spi_fmode   <= m0_cmd_fifo_rdata[27:26];
              cfg_spi_imode   <= m0_cmd_fifo_rdata[25:24];
              cfg_spi_seq     <= m0_cmd_fifo_rdata[23:20];
	          cfg_cs_reg      <= m0_cmd_fifo_rdata[19:16];
              spi_mode_cmd    <= m0_cmd_fifo_rdata[15:8];
              gnt             <= 2'b01;
           end
           else if(!m1_cmd_fifo_empty ) begin
              cfg_data_cnt    <= m1_cmd_fifo_rdata[47:36];
              cfg_dummy_cnt   <= m1_cmd_fifo_rdata[35:32];
              cfg_addr_cnt    <= m1_cmd_fifo_rdata[31:30];
	      cfg_spi_switch  <= m1_cmd_fifo_rdata[29:28];
              cfg_spi_fmode   <= m1_cmd_fifo_rdata[27:26];
              cfg_spi_imode   <= m1_cmd_fifo_rdata[25:24];
              cfg_spi_seq     <= m1_cmd_fifo_rdata[23:20];
	      cfg_cs_reg      <= m1_cmd_fifo_rdata[19:16];
              spi_mode_cmd    <= m1_cmd_fifo_rdata[15:8];
              gnt             <= 2'b10;
           end
        end
      end
   end


  //-----------------------------------------------------------------------
  // SPI Mode Switch Control Logic
  // Note: SPI Protocl Start with SPI_STD Mode (Sigle Bit Mode) Base on the
  // Command, Type it Switch the mode at ADDRESS/DUMMY/DATA Phase
  // QIOR(0xEB) -> Mode switch at Idle Phase (sram)
  // QIOR(0xEB) -> Mode switch at Address Phase (Flash)
  // DIOR(0xBB) -> Mode Switch at Address Phase
  // QOR (0x6B) -> Mode Switch at Data Phase
  // DOR (0x3B) -> Mode Switch at Data Phase
  // QPP (0x32) -> Mode Switch at Data Phase 
  // ----------------------------------------------------------------------
  always @(posedge clk or negedge rstn) begin
     if (rstn == 1'b0) begin
        s_spi_mode <= P_SINGLE;
     end else begin
	if(state == FSM_IDLE) begin // Reset the Mode at IDLE State
            s_spi_mode <= P_SINGLE;
	end else if(state == FSM_CS_ASSERT && cfg_spi_switch == P_MODE_SWITCH_IDLE) begin
            s_spi_mode <= cfg_spi_imode;
	end else if(state == FSM_ADR_PHASE && cfg_spi_switch == P_MODE_SWITCH_AT_ADDR) begin
            s_spi_mode <= cfg_spi_fmode;
	end else if(((state == FSM_READ_PHASE) || state == FSM_WRITE_CMD ) && cfg_spi_switch == P_MODE_SWITCH_AT_DATA) begin
            s_spi_mode <= cfg_spi_fmode;
	end
     end
  end

  // SPI Chip Select Logic
  always @(posedge clk or negedge rstn) begin
     if (rstn == 1'b0) begin
        spi_csn0 <= 1'b1;
        spi_csn1 <= 1'b1;
        spi_csn2 <= 1'b1;
        spi_csn3 <= 1'b1;
     end else begin
	if(state != FSM_IDLE && state != FSM_CMD_WAIT) begin
           spi_csn0 <= ~cfg_cs_reg[0];
           spi_csn1 <= ~cfg_cs_reg[1];
           spi_csn2 <= ~cfg_cs_reg[2];
           spi_csn3 <= ~cfg_cs_reg[3];
	end else begin
           spi_csn0 <= 1'b1;
           spi_csn1 <= 1'b1;
           spi_csn2 <= 1'b1;
           spi_csn3 <= 1'b1;
	end
     end
  end

endmodule
