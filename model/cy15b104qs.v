///////////////////////////////////////////////////////////////////////////////
//  File name : cy15b104qs.v
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//  Copyright (C) 2014 Spansion, LLC.
//
//  MODIFICATION HISTORY :
//
//  version: |   author:     |  mod date:  |  changes made:
//    V1.0      M.Stojanovic   17 Mar 25      Initial version
//    V1.1      S.Stevanovic   17 Apr 15      Adding ECC ADDTRAP_reg to RDAR
//                                            Address range.
//                                            Fixing setup and hold times for
//                                            ddr read instructions. First read
//                                            data was checked wrong. Setup and
//                                            hold are now checked only for
//                                            input memory data.
//    V1.2      M.Stojanovic   17 Oct 18      Update to datasheet 002-18293 Rev. *B
//
///////////////////////////////////////////////////////////////////////////////
//  PART DESCRIPTION:
//
//  Library:    FLASH
//  Technology: FRAM MEMORY
//  Part:       CY15B104QS
//
//  Description: 4 Megabit Serial Flash Memory
//
//////////////////////////////////////////////////////////////////////////////
//  Comments :
//      For correct simulation, simulator resolution should be set to 1 ps
//      A device ordering (trim) option determines whether a feature is enabled
//      or not, or provide relevant parameters:
//        -15th character in TimingModel determines if enhanced high
//         performance option is available
//            (0,2) General Market
//
//////////////////////////////////////////////////////////////////////////////
//  Known Bugs:
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// MODULE DECLARATION                                                       //
//////////////////////////////////////////////////////////////////////////////
`timescale 1 ps/1 ps

module cy15b104qs
    (
        // Data Inputs/Outputs
        SI     ,
        SO     ,
        // Controls
        SCK    ,
        CSNeg  ,
        WPNeg  ,
        RESETNeg
    );

///////////////////////////////////////////////////////////////////////////////
// Port / Part Pin Declarations
///////////////////////////////////////////////////////////////////////////////

    inout   SI            ;
    inout   SO            ;

    input   SCK           ;
    input   CSNeg         ;
    inout   WPNeg         ;
    inout   RESETNeg      ;

    // interconnect path delay signals
    wire   SCK_ipd        ;
    wire   SI_ipd         ;
    wire   SO_ipd         ;
    wire   CSNeg_ipd      ;
    wire   WPNeg_ipd      ;
    wire   RESETNeg_ipd   ;

    wire SI_in            ;
    assign SI_in = SI_ipd ;

    wire SI_out           ;
    assign SI_out = SI    ;

    wire SO_in            ;
    assign SO_in = SO_ipd ;

    wire SO_out           ;
    assign SO_out = SO    ;

    wire   WPNeg_in                 ;
    //Internal pull-up
    assign WPNeg_in = (WPNeg_ipd === 1'bx) ? 1'b1 : WPNeg_ipd;

    wire   WPNeg_out                ;
    assign WPNeg_out = WPNeg        ;

    wire   RESETNeg_in              ;
    //Internal pull-up
    assign RESETNeg_in = (RESETNeg_ipd === 1'bx) ? 1'b1 : RESETNeg_ipd;

    wire   RESETNeg_out             ;
    assign RESETNeg_out = RESETNeg  ;

    // internal delays
    reg RST_in      ;
    reg RST_out     ;
    reg SWRST_in    ;
    reg SWRST_out   ;
    reg PRGSUSP_in  ;
    reg PRGSUSP_out ;
    reg DPD_in      ;
    reg DPD_out     ;
    reg RES_in      ;
    reg RES_out     ;
    reg HBN_in      ;
    reg HBN_out     ;
    reg REC_in      ;
    reg REC_out     ;

    reg rising_edge_CSNeg_ipd  = 1'b0;
    reg falling_edge_CSNeg_ipd = 1'b0;
    reg rising_edge_SCK_ipd    = 1'b0;
    reg falling_edge_SCK_ipd   = 1'b0;
    reg rising_edge_RESETNeg   = 1'b0;
    reg falling_edge_RESETNeg  = 1'b0;
    reg falling_edge_RST       = 1'b0;
    reg rising_edge_RST_out    = 1'b0;
    reg rising_edge_SWRST_out  = 1'b0;
    reg rising_edge_reseted    = 1'b0;

    reg falling_edge_write     = 1'b0;

    reg rising_edge_PoweredUp  = 1'b0;
    reg rising_edge_PSTART     = 1'b0;
    reg rising_edge_PDONE      = 1'b0;
    reg falling_edge_PDONE     = 1'b0;
    reg rising_edge_WSTART     = 1'b0;
    reg rising_edge_WDONE      = 1'b0;
    reg rising_edge_CSDONE     = 1'b0;
    reg rising_edge_DPD_out    = 1'b0;
    reg rising_edge_RES_out    = 1'b0;
    reg rising_edge_HBN_out    = 1'b0;
    reg rising_edge_REC_out    = 1'b0;
    reg rising_edge_CRCSTART   = 1'b0;
    reg rising_edge_CRCDONE    = 1'b0;
    reg rising_edge_START_T1_in= 1'b0;

    reg RST;

    reg SOut_zd            = 1'bZ;
    reg SIOut_zd           = 1'bZ;
    reg WPNegOut_zd        = 1'bZ;
    reg RESETNegOut_zd     = 1'bZ;

    parameter UserPreload       = 1;
    parameter mem_file_name     = "none";//"cy15b104qs.mem";
    parameter otp_file_name     = "none";//"none";

    parameter TimingModel       = "CY15B104QSN-108SXI";

    parameter  PartID           = "cy15b104qs";
    parameter  MaxData          = 255;
    parameter  MemSize          = 20'h7FFFF;
    parameter  SecSize          = 12'h7FF;
    parameter  SecNumUni        = 255;
    parameter  AddrRANGE        = 20'h7FFFF;
    parameter  HiAddrBit        = 23;
    parameter  OTPSize          = 1023;
    parameter  OTPLoAddr        = 8'h00;
    parameter  OTPHiAddr        = 8'hFF;
    parameter  BYTE             = 8;

    integer    SECURE_OPN; //  Trim Options active

    //varaibles to resolve architecture used
    reg [24*8-1:0] tmp_timing;//stores copy of TimingModel
    reg [7:0] tmp_char1; //Define General Market or Secure Device
    integer found = 1'b0;

    // powerup
    reg PoweredUp;

    // Memory Array Configuration
    reg BottomBoot = 1'b0;
    reg TopBoot    = 1'b0;
    reg UniformSec = 1'b0;

    // FSM control signals
    reg PDONE     ;
    reg PSTART    ;
    reg PGSUSP    ;
    reg PGRES     ;

    reg RES_TO_SUSP_TIME;

    reg CSDONE    ;
    reg CSSTART   ;

    reg WDONE     ;
    reg WSTART    ;

    reg CRCSTART  ;
    reg CRCDONE   ;
    reg CRCSUSP   ;
    reg CRCRES    ;
    
    reg reseted   ;

    reg INITIAL_CONFIG    = 1'b0;
    reg CHECK_FREQ        = 1'b0;

    reg ZERO_DETECTED     = 1'b0;

    // Wrap Length
    integer WrapLength;

    integer CRC_Start_Addr_reg = 0;
    integer CRC_End_Addr_reg   = 0;

    // Programming buffer
    integer WByte[0:511];
    // OTP Memory Array
    integer OTPMem[OTPLoAddr:OTPHiAddr];
    // Flash Memory Array
    integer Mem[0:AddrRANGE];

    //-----------------------------------------
    //  Registers
    //-----------------------------------------
    reg [7:0] SR1_in    = 8'h00;

    //Status Register 1
    reg [7:0] SR1     = 8'h00;

    wire       SRWD;
    wire       TBPROT;
    wire [2:0] BP;
    wire       WEL;
    wire       WIP;

    assign SRWD   = SR1[7]  ;
    assign TBPROT = SR1[5]  ;
    assign BP     = SR1[4:2];
    assign WEL    = SR1[1]  ;
    assign WIP    = SR1[0]  ;

    //Volatile Status Register 2
    reg [7:0] SR2_V     = 8'h00;

    wire CRCS;
    wire CRCA;

    assign CRCS  = SR2_V[4];
    assign CRCA  = SR2_V[3];

    //Nonvolatile Configuration Register 1
    reg [7:0] CR1_in    = 8'h00;

    reg [7:0] CR1_NV    = 8'h00;

    wire   QUAD;

    assign QUAD = CR1_NV[1];
    
    //Nonvolatile Configuration Register 2
    reg [7:0] CR2_NV    = 8'h00;

    wire   QPI;
    wire   DPI;

    assign QPI  = CR2_NV[6];
    assign DPI  = CR2_NV[4];

    //Nonvolatile Configuration Register 4
    reg [7:0] CR4_NV    = 8'h28;

    //Nonvolatile Configuration Register 5
    reg [7:0] CR5_NV    = 8'h00;

    //ID Register
    reg [63:0] ID_reg   = 64'h0000000006821054;

    //Serial Number Register
    reg [63:0] SERNUM_reg  = 64'h0000000000000000;

    // CRC Register
    reg[31:0] CRC_reg          = 32'h00000000;
    reg[31:0] CRC_reg_in       = 32'h00000000;
    // Unique ID Register
    reg[63:0] UID_reg          = 64'h0000000000000000;

    reg [7:0] WRAR_reg_in = 8'h00;
    reg [7:0] RDAR_reg    = 8'h00;

    // ECC Register
    reg [7:0] ECC_reg      = 8'h00;
    // Error Detection Counter Register
    reg [15:0] EDC_reg     = 32'h00000000;
    // ECC Address Trap Register
    reg [31:0] ADDTRAP_reg = 32'h00000000;

    reg write;
    reg cfg_write;
    reg read_out;
    reg dpd_act;
    reg dual          = 1'b0;
    reg rd_slow       = 1'b0;
    reg ddr           = 1'b0;
    reg any_read      = 1'b0;

    reg DOUBLE        = 1'b0; //Double Data Rate (DDR) flag

    reg change_BP     = 0;
    reg[2:0] BP_bits  = 3'b0;

    integer PageSize = 511;

    integer RESET_EN = 0; //Reset Enable Flag

    reg     change_addr;
    integer Address = 0;

    reg oe   = 1'b0;
    reg oe_z = 1'b0;

    reg sSTART_T1 = 1'b0;
    reg START_T1_in = 1'b0;

    integer Byte_number = 0;

    // Sector is protect if Sec_Prot(SecNum) = '1'
    reg [SecNumUni:0] Sec_Prot  = 256'b0;

    // timing check violation
    reg Viol = 1'b0;

    integer AddrLo;
    integer AddrHi;

    reg[7:0]  old_bit, new_bit;
    integer old_int, new_int;
    integer wr_cnt;
    integer cnt;

    integer read_cnt  = 0;
    integer read_addr = 0;
    integer byte_cnt  = 1;
    integer pgm_page = 0;

    reg[7:0] data_out;

    time SCK_cycle = 0;
    time prev_SCK;
    reg  glitch = 1'b0;
    reg  DataDriveOut_SO = 1'bZ ;
    reg  DataDriveOut_SI = 1'bZ ;
    reg  DataDriveOut_RESET = 1'bZ ;
    reg  DataDriveOut_WP = 1'bZ ;

///////////////////////////////////////////////////////////////////////////////
//Interconnect Path Delay Section
///////////////////////////////////////////////////////////////////////////////
    buf   (SCK_ipd, SCK);
    buf   (SI_ipd, SI);
    buf   (SO_ipd, SO);
    buf   (CSNeg_ipd, CSNeg);
    buf   (WPNeg_ipd, WPNeg);
    buf   (RESETNeg_ipd, RESETNeg);

///////////////////////////////////////////////////////////////////////////////
// Propagation  delay Section
///////////////////////////////////////////////////////////////////////////////
    nmos   (SI,       SIOut_zd       , 1);
    nmos   (SO,       SOut_zd        , 1);
    nmos   (RESETNeg, RESETNegOut_zd , 1);
    nmos   (WPNeg,    WPNegOut_zd    , 1);

    // Needed for TimingChecks
    // VHDL CheckEnable Equivalent

    //Single Data Rate Operations
    wire sdro;
    assign sdro = PoweredUp && ~DOUBLE;
    wire sdro_quad_io0;
    assign sdro_quad_io0 = PoweredUp && ~DOUBLE && ~dual && QUAD;
    wire sdro_io1;
    assign sdro_io1 = PoweredUp && ~DOUBLE && ~dual;
    wire sdro_quad_io2;
    assign sdro_quad_io2 = PoweredUp && ~DOUBLE && ~dual && QUAD;
    wire sdro_quad_io3;
    assign sdro_quad_io3 = PoweredUp && ~DOUBLE && ~dual && QUAD && ~CSNeg;

    //Dual Data Rate Operations
    wire ddro;
    assign ddro = PoweredUp && ddr;

    wire ddro_quad_io0;
    assign ddro_quad_io0 = PoweredUp && DOUBLE && ~dual && QUAD;
    wire ddro_io1;
    assign ddro_io1 = PoweredUp && DOUBLE && ~dual;
    wire ddro_quad_io2;
    assign ddro_quad_io2 = PoweredUp && DOUBLE && ~dual && QUAD;
    wire ddro_quad_io3;
    assign ddro_quad_io3 = PoweredUp && DOUBLE && ~dual && QUAD && ~CSNeg;

    wire rd ;
    wire ddrd ;
    assign rd      = rd_slow;
    assign ddrd    = ddr;

    wire wr_prot;
    assign wr_prot = SRWD && ~QUAD;

    wire REGS_PROTECTED;
    assign REGS_PROTECTED = SRWD && (~WPNeg_ipd) && (~QUAD);

    wire reset_act;
    assign reset_act = CR2_NV[5] && (~QUAD || (QUAD && CSNeg_ipd));

    wire rst_not_quad;
    assign rst_not_quad = CR2_NV[5] && ~QUAD;

    wire rst_quad;
    assign rst_quad = CR2_NV[5] && QUAD;

    wire RD_EQU_1;
    assign RD_EQU_1 = any_read && ~rst_quad;

    wire QRD_EQU_1;
    assign QRD_EQU_1 = any_read && rst_quad;

    wire RD_EQU_0;
    assign RD_EQU_0 = ~any_read;

    wire datain;
    assign datain = SOut_zd === 1'bz;

    wire datain_ddr;
    assign datain_ddr = datain & ddr & ~read_out;

specify
        // tipd delays: interconnect path delays , mapped to input port delays.
        // In Verilog is not necessary to declare any tipd_ delay variables,
        // they can be taken from SDF file
        // With all the other delays real delays would be taken from SDF file

    // tpd delays
    specparam        tpd_SCK_SO              = 1; // tV
    specparam        tpd_CSNeg_SO_normal_rd  = 1; // tDIS
    specparam        tpd_CSNeg_SO_ddr_rd     = 1; // tDIS

    //tsetup values: setup times
    specparam        tsetup_CSNeg_SCK_normal_rd= 1; // tCSS edge /
    specparam        tsetup_CSNeg_SCK_ddr_rd = 1;   // tCSS edge /
    specparam        tsetup_SI_SCK_sdr       = 1;   // tSU  edge /
    specparam        tsetup_SI_SCK_ddr       = 1;   // tSU
    specparam        tsetup_WPNeg_CSNeg      = 1;   // tWPS edge \
    specparam        tsetup_RESETNeg_CSNeg   = 1;   // tRS  edge \

    //thold values: hold times
    specparam        thold_CSNeg_SCK_normal_rd= 1;  // tCSH edge /
    specparam        thold_CSNeg_SCK_ddr_rd  = 1;   // tCSH edge /
    specparam        thold_SI_SCK_sdr        = 1;   // tHD  edge /
    specparam        thold_SI_SCK_ddr        = 1;   // tHD
    specparam        thold_WPNeg_CSNeg       = 1;   // tWPH edge /
    specparam        thold_CSNeg_RESETNeg    = 1;   // tRH  edge /

    // tpw values: pulse width
    specparam        tpw_SCK_normal_rd       = 1;
    specparam        tpw_SCK_ddr_rd          = 1;
    specparam        tpw_CSNeg_posedge       = 1;   // tCS
    specparam        tpw_CSNeg_rst_quad_posedge = 1;// tCS
    specparam        tpw_CSNeg_wip_posedge   = 1;   // tCS
    specparam        tpw_RESETNeg_negedge    = 1;   // tRP
    specparam        tpw_RESETNeg_posedge    = 1;   // tRS

    // tperiod min (calculated as 1/max freq)
    specparam        tperiod_SCK_normal_rd   = 1;   // 108 MHz
    specparam        tperiod_SCK_ddr_rd      = 1;   // 54 MHz

    // RESET# Low to CS# Low
    specparam        tdevice_RPH               = 300e6; //tRPH = 300 us
    // CS# High before HW Reset (Quad mode and Reset Feature are enabled)
    specparam        tdevice_CS                = 40e3; //tCS = 40 ns
    // VDD (min) to CS# Low
    specparam        tdevice_PU                = 1; // 450e6;//tPU = 450us // Dinesh A
    // CRC setup time
    specparam        tdevice_CRCSETUP          = 425e9;//tCRCSETUP = 425ms
    // CRC suspend latency
    specparam        tdevice_CRCSL             = 50e6;//tCRCSL = 50us
    // CRC Resume to next suspend
    specparam        tdevice_CRCRL             = 50e6;//tCRCRL = 50us
    // CS# High to Power Down Mode
    specparam        tdevice_DPD               = 3e6;     // 3 us
    // CS# High to Standby without Electronic Signature
    specparam        tdevice_RES               = 2e6;     // 2 us
    // CS# High to Hibernate Mode
    specparam        tdevice_HBN               = 3e6;   //tHBN = 3 us
    // CS# Low to ready for access
    specparam        tdevice_REC               = 450e6; //tREC = 450us

///////////////////////////////////////////////////////////////////////////////
// Input Port  Delays  don't require Verilog description
///////////////////////////////////////////////////////////////////////////////
// Path delays                                                               //
///////////////////////////////////////////////////////////////////////////////
    if (~glitch)     (SCK => SO) = tpd_SCK_SO;
    if (~glitch)     (SCK => SI) = tpd_SCK_SO;
    if (~glitch)     (SCK => RESETNeg) = tpd_SCK_SO;
    if (~glitch)     (SCK => WPNeg)    = tpd_SCK_SO;

    if (rd)          (CSNeg => SO) = tpd_CSNeg_SO_normal_rd;
    if (rd)          (CSNeg => SI) = tpd_CSNeg_SO_normal_rd;
    if (rd)          (CSNeg => RESETNeg) = tpd_CSNeg_SO_normal_rd;
    if (rd)          (CSNeg => WPNeg)    = tpd_CSNeg_SO_normal_rd;

    if (ddrd)        (CSNeg => SI) = tpd_CSNeg_SO_ddr_rd;
    if (ddrd)        (CSNeg => SO) = tpd_CSNeg_SO_ddr_rd;
    if (ddrd)        (CSNeg => RESETNeg) = tpd_CSNeg_SO_ddr_rd;
    if (ddrd)        (CSNeg => WPNeg)    = tpd_CSNeg_SO_ddr_rd;

///////////////////////////////////////////////////////////////////////////////
// Timing Violation                                                          //
///////////////////////////////////////////////////////////////////////////////
    $setup ( CSNeg  , posedge SCK &&& rd,       tsetup_CSNeg_SCK_normal_rd , Viol);
    $setup ( CSNeg  , posedge SCK &&& ddrd,        tsetup_CSNeg_SCK_ddr_rd , Viol);

    $setup ( SI     , posedge SCK &&& datain,            tsetup_SI_SCK_sdr , Viol);
    $setup ( SI     , negedge SCK &&& datain_ddr,        tsetup_SI_SCK_ddr , Viol);
    $setup ( SO     , posedge SCK &&& datain,            tsetup_SI_SCK_sdr , Viol);
    $setup ( SO     , negedge SCK &&& datain_ddr,        tsetup_SI_SCK_ddr , Viol);
    $setup ( RESETNeg,posedge SCK &&& datain,            tsetup_SI_SCK_sdr , Viol);
    $setup ( RESETNeg,negedge SCK &&& datain_ddr,        tsetup_SI_SCK_ddr , Viol);
    $setup ( WPNeg  , posedge SCK &&& datain,            tsetup_SI_SCK_sdr , Viol);
    $setup ( WPNeg  , negedge SCK &&& datain_ddr,        tsetup_SI_SCK_ddr , Viol);

    $setup ( WPNeg  , negedge CSNeg &&& wr_prot,        tsetup_WPNeg_CSNeg , Viol);

    $hold  ( posedge SCK &&& rd,   CSNeg,        thold_CSNeg_SCK_normal_rd , Viol);
    $hold  ( posedge SCK &&& ddrd, CSNeg,           thold_CSNeg_SCK_ddr_rd , Viol);

    $hold  ( posedge SCK &&& datain,     SI ,             thold_SI_SCK_sdr , Viol);
    $hold  ( negedge SCK &&& datain_ddr, SI ,             thold_SI_SCK_ddr , Viol);
    $hold  ( posedge SCK &&& datain,     SO ,             thold_SI_SCK_sdr , Viol);
    $hold  ( negedge SCK &&& datain_ddr, SO ,             thold_SI_SCK_ddr , Viol);
    $hold  ( posedge SCK &&& datain,     RESETNeg,        thold_SI_SCK_sdr , Viol);
    $hold  ( negedge SCK &&& datain_ddr, RESETNeg,        thold_SI_SCK_ddr , Viol);
    $hold  ( posedge SCK &&& datain,     WPNeg,           thold_SI_SCK_sdr , Viol);
    $hold  ( negedge SCK &&& datain_ddr, WPNeg,           thold_SI_SCK_ddr , Viol);

    $hold  ( posedge CSNeg &&& wr_prot,  WPNeg,        thold_WPNeg_CSNeg   , Viol);

    $width ( posedge SCK &&& rd           , tpw_SCK_normal_rd);
    $width ( negedge SCK &&& rd           , tpw_SCK_normal_rd);
    $width ( posedge SCK &&& ddrd         , tpw_SCK_ddr_rd);
    $width ( negedge SCK &&& ddrd         , tpw_SCK_ddr_rd);

    $width ( posedge CSNeg &&& any_read          , tpw_CSNeg_posedge);
    $width ( posedge CSNeg &&& rst_quad          , tpw_CSNeg_rst_quad_posedge);
    $width ( negedge RESETNeg                    , tpw_RESETNeg_negedge);

    $period ( posedge SCK &&& rd                 , tperiod_SCK_normal_rd);
    $period ( posedge SCK &&& ddrd               , tperiod_SCK_ddr_rd);

endspecify

///////////////////////////////////////////////////////////////////////////////
// Main Behavior Block                                                       //
///////////////////////////////////////////////////////////////////////////////
// FSM states
 parameter IDLE             = 4'd0;
 parameter RESET_STATE      = 4'd1;
 parameter PGERS_ERROR      = 4'd2;
 parameter WRITE_SR         = 4'd3;
 parameter WRITE_ALL_REG    = 4'd4;
 parameter PAGE_PG          = 4'd5;
 parameter OTP_PG           = 4'd6;
 parameter CRC_Calc         = 4'd7;
 parameter CRC_SUSP         = 4'd8;
 parameter DP_DOWN          = 4'd9;
 parameter HIBERNATE        = 4'd10;

 reg [3:0] current_state;
 reg [3:0] next_state;


// Instruction type
 parameter NONE            = 7'd0;
 parameter WREN            = 7'd1;
 parameter WRDI            = 7'd2;
 parameter WRR             = 7'd3;
 parameter WRAR            = 7'd4;
 parameter RDAR            = 7'd5;
 parameter RDSR1           = 7'd6;
 parameter RDSR2           = 7'd7;
 parameter RDCR1           = 7'd8;
 parameter RDCR2           = 7'd9;
 parameter RDCR4           = 7'd10;
 parameter RDCR5           = 7'd11;
 parameter RDID            = 7'd12;
 parameter RUID            = 7'd13;
 parameter WRSN            = 7'd14;
 parameter RDSN            = 7'd15;
 parameter ECCRD           = 7'd16;
 parameter CLECC           = 7'd17;
 parameter CRCC            = 7'd18;
 parameter RBCRC           = 7'd19;
 parameter READ            = 7'd20;
 parameter FAST_READ       = 7'd21;
 parameter DDRFR           = 7'd22;
 parameter DOR             = 7'd23;
 parameter DIOR            = 7'd24;
 parameter QOR             = 7'd26;
 parameter QIOR            = 7'd27;
 parameter DDRQIOR         = 7'd28;
 parameter WRITE_MEM       = 7'd29;
 parameter DDRWRITE        = 7'd30;
 parameter FAST_WRITE      = 7'd31;
 parameter DDR_FAST_WRITE  = 7'd32;
 parameter DIW             = 7'd33;
 parameter DIOW            = 7'd34;
 parameter QIW             = 7'd36;
 parameter QIOW            = 7'd37;
 parameter DDRQIOW         = 7'd38;
 parameter SSWR            = 7'd39;
 parameter SSRD            = 7'd40;
 parameter EPCS            = 7'd41;
 parameter EPCR            = 7'd42;
 parameter RSTEN           = 7'd43;
 parameter RSTCMD          = 7'd44;
 parameter HBN             = 7'd46;
 parameter DPD             = 7'd47;

// Command Register
 reg [5:0] Instruct;

//Bus cycle state
 parameter STAND_BY        = 3'd0;
 parameter OPCODE_BYTE     = 3'd1;
 parameter ADDRESS_BYTES   = 3'd2;
 parameter DUMMY_BYTES     = 3'd3;
 parameter MODE_BYTE       = 3'd4;
 parameter DATA_BYTES      = 3'd5;

 reg [2:0] bus_cycle_state;

    //Power Up time;
    initial
    begin
        PoweredUp = 1'b0;
        #tdevice_PU PoweredUp = 1'b1;
    end

    initial
    begin : Init
        write       = 1'b0;
        cfg_write   = 1'b0;
        read_out    = 1'b0;
        Address     = 0;
        change_addr = 1'b0;
        RST         = 1'b0;
        RST_in      = 1'b0;
        RST_out     = 1'b1;
        SWRST_in    = 1'b0;
        SWRST_out   = 1'b1;
        PDONE       = 1'b1;
        PSTART      = 1'b0;
        PGSUSP      = 1'b0;
        PGRES       = 1'b0;
        RES_TO_SUSP_TIME = 1'b0;

        CRCDONE     = 1'b1;
        CRCSTART    = 1'b0;
        CRCSUSP     = 1'b0;
        CRCRES      = 1'b0;

        WDONE       = 1'b1;
        WSTART      = 1'b0;

        DPD_in      = 1'b0;
        DPD_out     = 1'b0;
        RES_in      = 1'b0;
        RES_out     = 1'b0;

        CSDONE      = 1'b1;
        CSSTART     = 1'b0;

        reseted     = 1'b0;

        Instruct        = NONE;
        bus_cycle_state = STAND_BY;
        current_state   = RESET_STATE;
        next_state      = RESET_STATE;
    end

    // initialize memory and load preload files if any
    initial
    begin: InitMemory
        integer i;

        for (i=0;i<=AddrRANGE;i=i+1)
        begin
            Mem[i] = MaxData;
        end

        if ((UserPreload) && !(mem_file_name == "none"))
        begin
           // Memory Preload
           //cy15b104qs.mem, memory preload file
           //  @aaaaaaa - <aaaaaaa> stands for address
           //  dd       - <dd> is byte to be written at Mem(aaaaaaa++)
           // (aaaaaaa is incremented at every load)
           $readmemh(mem_file_name,Mem);
        end

        for (i=OTPLoAddr;i<=OTPHiAddr;i=i+1)
        begin
            OTPMem[i] = MaxData;
        end

        if (UserPreload && !(otp_file_name == "none"))
        begin
        //cy15b104qs_otp memory file
        //   /        - comment
        //   @aaa - <aaa> stands for address
        //   dd  - <dd> is byte to be written at OTPMem(aaa++)
        //   (aaa is incremented at every load)
        //   only first 1-4 columns are loaded. NO empty lines !!!!!!!!!!!!!!!!
           $readmemh(otp_file_name,OTPMem);
        end
    end

    // initialize memory and load preload files if any
    initial
    begin: InitTimingModel
    integer i;
    integer j;
        //UNIFORM OR HYBRID arch model is used
        //assumptions:
        //1. TimingModel has format as CY15B104QSN-108SXI
        //2. TimingModel does not have more then 24 characters
        tmp_timing = TimingModel;//copy of TimingModel

        i = 16;
        while ((i >= 0) && (found != 1'b1))//search for first non null character
        begin        //i keeps position of first non null character
            j = 7;
            while ((j >= 0) && (found != 1'b1))
            begin
                if (tmp_timing[i*8+j] != 1'd0)
                    found = 1'b1;
                else
                    j = j-1;
            end
            i = i - 1;
        end
        i = i +1;
        if (found)//if non null character is found
        begin
            for (j=0;j<=7;j=j+1)
            begin
            //Security character is 15
                tmp_char1[j] = TimingModel[(i-14)*8+j];
            end
        end

        if (tmp_char1 == "0" || tmp_char1 == "2" || tmp_char1 == "3")
        begin
            SECURE_OPN = 0;
        end
        else if (tmp_char1 == "Y" || tmp_char1 == "Z")
        begin
            SECURE_OPN = 1;
        end

    end

    always @(next_state or PoweredUp or falling_edge_RST or RST_out or SWRST_out)
    begin: StateTransition1
        if (PoweredUp)
        begin
            if ((~reset_act || (RESETNeg_in && reset_act)) && RST_out &&
                  SWRST_out)
            begin
                current_state = next_state;
                reseted = 1;
            end
            else if (((~RESETNeg_in && reset_act) ||
                     (rising_edge_RESETNeg && reset_act)) &&
                       falling_edge_RST)
            begin
            // no state transition while RESET# low
                current_state = RESET_STATE;
                RST_in = 1'b1;
                #1 RST_in = 1'b0;
                reseted   = 1'b0;
            end
        end
    end

    always @(falling_edge_write)
    begin: StateTransition2
        if (Instruct == RSTCMD && RESET_EN)
        begin
            // no state transition while RESET is in progress
            current_state = RESET_STATE;
            SWRST_in = 1'b1;
            #1 SWRST_in = 1'b0;
            reseted   = 1'b0;
            RESET_EN = 0;
        end
    end

    ////////////////////////////////////////////////////////////////////////////
    // Timing control for the Hardware Reset
    ////////////////////////////////////////////////////////////////////////////
    always @(posedge RST_in)
    begin:Threset
        RST_out = 1'b0;
        #(tdevice_RPH -200000) RST_out = 1'b1;
    end

    always @(RESETNeg)
        begin
        RST <= #199000 RESETNeg;
    end

    ////////////////////////////////////////////////////////////////////////////
    // Timing control for the Software Reset
    ////////////////////////////////////////////////////////////////////////////
    always @(posedge SWRST_in)
    begin:Tswreset
        SWRST_out = 1'b0;
        #tdevice_RPH SWRST_out = 1'b1;
    end

    always @(negedge CSNeg_ipd)
    begin:CheckCSOnPowerUP
        if (~PoweredUp)
            $display ("Device is selected during Power Up");
    end

    ///////////////////////////////////////////////////////////////////////////
    //// Internal Delays
    ///////////////////////////////////////////////////////////////////////////

    always @(posedge DPD_in)
    begin:DPDown
        DPD_out = 1'b0;
        #tdevice_DPD DPD_out = 1'b1;
    end

    always @(posedge RES_in)
    begin:RESDPDown
        RES_out = 1'b0;
        #tdevice_RES RES_out = 1'b1;
    end

    always @(posedge HBN_in)
    begin:HYBERNATE
        HBN_out = 1'b0;
        #tdevice_HBN HBN_out = 1'b1;
    end

    always @(posedge REC_in)
    begin:RESHYB
        REC_out = 1'b0;
        #tdevice_REC REC_out = 1'b1;
    end

///////////////////////////////////////////////////////////////////////////////
// write cycle decode
///////////////////////////////////////////////////////////////////////////////
    integer opcode_cnt = 0;
    integer addr_cnt   = 0;
    integer mode_cnt   = 0;
    integer dummy_cnt  = 0;
    integer data_cnt   = 0;
    integer bit_cnt    = 0;

    reg [4095:0] Data_in = {4096{1'b1}};
    reg    [7:0] opcode;
    reg    [7:0] opcode_in;
    reg    [7:0] opcode_tmp;
    reg   [23:0] addr_bytes;
    reg   [23:0] Address_in;
    reg    [7:0] mode_bytes;
    reg    [7:0] mode_in;
    integer Latency_code;
    integer Register_Latency;
    integer quad_data_in [0:1023];
    reg [3:0] quad_nybble = 4'b0;
    reg [3:0] Quad_slv;
    reg [7:0] Byte_slv;

    reg CRC_ACT      = 1'b0; // CRC Active
    reg CRC_RD_SETUP = 1'b0; // CRC read setup
    reg [15:0] crc_in;
    reg [31:0] crc_out;
    reg crc_tmp;

   always @(rising_edge_CSNeg_ipd or falling_edge_CSNeg_ipd or
            rising_edge_SCK_ipd or falling_edge_SCK_ipd)
   begin: Buscycle
        integer i;
        integer j;
        integer k;
        time CLK_PER;
        time LAST_CLK;

        if (falling_edge_CSNeg_ipd)
        begin
            if (bus_cycle_state==STAND_BY)
            begin
                Instruct = NONE;
                write = 1'b1;
                cfg_write  = 0;
                opcode_cnt = 0;
                addr_cnt   = 0;
                mode_cnt   = 0;
                dummy_cnt  = 0;
                data_cnt   = 0;

                Data_in = {4096{1'b1}};

                CLK_PER    = 1'b0;
                LAST_CLK   = 1'b0;

                ZERO_DETECTED = 1'b0;
                DOUBLE = 1'b0;
                bus_cycle_state = OPCODE_BYTE;

            end
        end

        if (rising_edge_SCK_ipd) // Instructions, addresses or data present at
        begin                    // input are latched on the rising edge of SCK
            if (~CSNeg_ipd)
            begin
                case (bus_cycle_state)
                    OPCODE_BYTE:
                    begin
                        Latency_code = CR1_NV[7:4];
                        Register_Latency = CR5_NV[7:6];

                        if (QPI)
                        begin
                            opcode_in[4*opcode_cnt]   = RESETNeg_in;
                            opcode_in[4*opcode_cnt+1] = WPNeg_in;
                            opcode_in[4*opcode_cnt+2] = SO_in;
                            opcode_in[4*opcode_cnt+3] = SI_in;
                        end
                        else if (DPI)
                        begin
                            opcode_in[2*opcode_cnt]   = SO_in;
                            opcode_in[2*opcode_cnt+1] = SI_in;
                        end
                        else
                        begin
                            opcode_in[opcode_cnt] = SI_in;
                        end

                        opcode_cnt = opcode_cnt + 1;

                        if ((QPI && (opcode_cnt == BYTE/4)) ||
                            (DPI && (opcode_cnt == BYTE/2)) ||
                            (opcode_cnt == BYTE))
                        begin
                            for(i=7;i>=0;i=i-1)
                            begin
                                opcode[i] = opcode_in[7-i];
                            end
                            case (opcode)

                                8'b00000001 : // 01h
                                begin
                                    Instruct = WRR;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b00000010 : // 02h
                                begin
                                    Instruct = WRITE_MEM;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b00000011 : // 03h
                                begin
                                    Instruct = READ;
                                    bus_cycle_state = ADDRESS_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00000100 : // 04h
                                begin
                                    Instruct = WRDI;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b00000101 : // 05h
                                begin
                                    Instruct = RDSR1;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00000110 : // 06h
                                begin
                                    Instruct = WREN;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b00000111 : // 07h
                                begin
                                    Instruct = RDSR2;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00001011 : // 0Bh
                                begin
                                    Instruct = FAST_READ;
                                    bus_cycle_state = ADDRESS_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00001101 : // 0Dh
                                begin
                                    Instruct = DDRFR;
                                    bus_cycle_state = ADDRESS_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00011001 : // 19h
                                begin
                                    Instruct = ECCRD;
                                    bus_cycle_state = ADDRESS_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00011011 : // 1Bh
                                begin
                                    Instruct = CLECC;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b00110010 : // 32h
                                begin
                                    Instruct = QIW;
                                    if (QPI || DPI || (WEL == 0))
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                        bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b00110101 : // 35h
                                begin
                                    Instruct = RDCR1;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b00111011 : // 3Bh
                                begin
                                    Instruct = DOR;
                                    if (QPI || DPI)
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                    begin
                                        bus_cycle_state = ADDRESS_BYTES;
                                        CHECK_FREQ = 1'b1;
                                    end
                                end

                                8'b00111111 : // 3Fh
                                begin
                                    Instruct = RDCR2;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b01000010 : // 42h
                                begin
                                    Instruct = SSWR;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b01000101 : // 45h
                                begin
                                    Instruct = RDCR4;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b01001011 : // 4Bh
                                begin
                                    Instruct = SSRD;
                                    bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b01001100 : // 4Ch
                                begin
                                    Instruct = RUID;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b01011011 : // 5Bh
                                begin
                                    Instruct = CRCC;
                                    bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b01011110 : // 5Eh
                                begin
                                    Instruct = RDCR5;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b01100100 : // 64h
                                begin
                                    Instruct = RBCRC;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b01100101 : // 65h
                                begin
                                    Instruct = RDAR;
                                    bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b01100110 : // 66h
                                begin
                                    Instruct = RSTEN;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b01101011 : // 6Bh
                                begin
                                    Instruct = QOR;
                                    if (QPI || DPI)
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                    begin
                                        bus_cycle_state = ADDRESS_BYTES;
                                        CHECK_FREQ = 1'b1;
                                    end
                                end

                                8'b01110001 : // 71h
                                begin
                                    Instruct = WRAR;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b01110101 : // 75h
                                begin
                                    Instruct = EPCS;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b01111010 : // 7Ah
                                begin
                                    Instruct = EPCR;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b10011001 : // 99h
                                begin
                                    Instruct = RSTCMD;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b10011111 : // 9Fh
                                begin
                                    Instruct = RDID;
                                    bus_cycle_state = DUMMY_BYTES;
                                end

                                8'b10100001 : // A1h
                                begin
                                    Instruct = DIOW;
                                    if (QPI || DPI || (WEL == 0))
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                        bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b10100010 : // A2h
                                begin
                                    Instruct = DIW;
                                    if (QPI || DPI || (WEL == 0))
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                        bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b10111001 : // B9h
                                begin
                                    Instruct = DPD;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b10111010 : // BAh
                                begin
                                    Instruct = HBN;
                                    bus_cycle_state = DATA_BYTES;
                                end

                                8'b10111011 : // BBh
                                begin
                                    Instruct = DIOR;
                                    if (QPI || DPI)
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                    begin
                                        bus_cycle_state = ADDRESS_BYTES;
                                        CHECK_FREQ = 1'b1;
                                    end
                                end

                                8'b11000010 : // C2h
                                begin
                                    Instruct = WRSN;
                                    if (WEL == 1)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b11000011 : // C3h
                                begin
                                    Instruct = RDSN;
                                    bus_cycle_state = DUMMY_BYTES;
                                    CHECK_FREQ = 1'b1;
                                end

                                8'b11010001 : // D1h
                                begin
                                    Instruct = DDRQIOW;
                                    if (QPI || DPI || (WEL == 0))
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                        bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b11010010 : // D2h
                                begin
                                    Instruct = QIOW;
                                    if (QPI || DPI || (WEL == 0))
                                    begin
                                    //Command not supported in QPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                        bus_cycle_state = ADDRESS_BYTES;
                                end

                                8'b11011010 : // DAh
                                begin
                                    Instruct = FAST_WRITE;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b11011101 : // DDh
                                begin
                                    Instruct = DDR_FAST_WRITE;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b11011110 : // DEh
                                begin
                                    Instruct = DDRWRITE;
                                    if (WEL == 1)
                                        bus_cycle_state = ADDRESS_BYTES;
                                    else
                                        bus_cycle_state = STAND_BY;
                                end

                                8'b11101011 : // EBh
                                begin
                                    Instruct = QIOR;
                                    if (DPI)
                                    begin
                                    //Command not supported in DPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                    begin
                                        bus_cycle_state = ADDRESS_BYTES;
                                        CHECK_FREQ = 1'b1;
                                    end
                                end

                                8'b11101101 : // EDh
                                begin
                                    Instruct = DDRQIOR;
                                    if (DPI)
                                    begin
                                    //Command not supported in DPI mode
                                        bus_cycle_state = STAND_BY;
                                    end
                                    else
                                    begin
                                        bus_cycle_state = ADDRESS_BYTES;
                                        CHECK_FREQ = 1'b1;
                                    end
                                end

                            endcase
                        end
                    end //end of OPCODE BYTE

                    ADDRESS_BYTES :
                    begin
                        if (CHECK_FREQ)
                        begin
                            CLK_PER = $time - LAST_CLK;
                            LAST_CLK = $time;
                            if (((Instruct == FAST_READ) && ~QPI) ||
                                (Instruct == DOR) || (Instruct == QOR))
                            begin
                                if (CLK_PER < 9250 && Latency_code >= 0) // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == FAST_READ) && DPI) || (Instruct == DIOR))
                            begin
                                if ((CLK_PER < 16670 && Latency_code == 0) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 1) || // <= 80MHz
                                    (CLK_PER <  9250 && Latency_code >= 2))   // <=108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if ((((Instruct == FAST_READ) || (Instruct == QIOR)) && QPI)
                            || ((Instruct == QIOR) && QUAD))
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 0) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 1) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 2) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 3) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 4) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 5) || // <= 100MHz
                                    (CLK_PER <  9250 && Latency_code >= 6))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && ~QPI)
                            begin
                                if ((CLK_PER < 19999 && Latency_code == 0) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 1) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 2) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 3) || // <= 100MHz
                                    (CLK_PER < 9250  && Latency_code >= 4))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && DPI)
                            begin
                                if ((CLK_PER < 33333 && Latency_code == 2) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 3) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 4) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 5) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 6) || // <= 100MHz
                                    (CLK_PER < 9250  && Latency_code >= 7))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && QPI)
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 2) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 3) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 4) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 5) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 6) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 7) || // <= 100MHz
                                    (CLK_PER <  9250 && Latency_code >= 8))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == DDRFR) && QPI) || (Instruct == DDRQIOR))
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 2) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 3) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 4) || // <= 50MHz
                                    (CLK_PER < 18510 && Latency_code >= 5))   // <= 54MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if ((Instruct == RDSR1) || (Instruct == RDSR2) ||
                                (Instruct == RDCR1) || (Instruct == RDCR2) ||
                                (Instruct == RDCR4) || (Instruct == RDCR5) ||
                                (Instruct == RDAR)  || (Instruct == RUID)  ||
                                (Instruct == RDID)  || (Instruct == RDSN))
                            begin
                                if ((CLK_PER < 20000 && Register_Latency == 0) || // <= 50MHz
                                    (CLK_PER < 9250 && Register_Latency >= 1))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end
                        end // CHECK_FREQ
                        
                        if ((Instruct == DDRFR) || (Instruct == DDRQIOR) ||
                            (Instruct == DDRWRITE) || (Instruct == DDR_FAST_WRITE))
                            DOUBLE = 1'b1;
                        else
                            DOUBLE = 1'b0;

                        if ((Instruct == QIOR) || (Instruct == QIOW))
                        begin
                        //Instruction + 3 Bytes Address + Dummy Byte
                            Address_in[4*addr_cnt]   = RESETNeg_in;
                            Address_in[4*addr_cnt+1] = WPNeg_in;
                            Address_in[4*addr_cnt+2] = SO_in;
                            Address_in[4*addr_cnt+3] = SI_in;
                            read_cnt = 0;
                            addr_cnt = addr_cnt + 1;
                            if (addr_cnt == 3*BYTE/4)
                            begin
                                addr_cnt = 0;
                                for(i=23;i>=0;i=i-1)
                                begin
                                    addr_bytes[23-i] = Address_in[i];
                                end
                                addr_bytes[23:20] = 4'b0000;
                                Address = addr_bytes;
                                change_addr = 1'b1;
                                #1 change_addr = 1'b0;
                                bus_cycle_state = MODE_BYTE;
                            end
                        end
                        else if ((Instruct == DOR) || (Instruct == QOR) ||
                        (Instruct == DIW) || (Instruct == QIW))
                        begin
                            Address_in[addr_cnt] = SI_in;
                            addr_cnt = addr_cnt + 1;
                            if (addr_cnt == 3*BYTE)
                            begin
                                addr_cnt = 0;
                                for(i=23;i>=0;i=i-1)
                                begin
                                    addr_bytes[23-i] = Address_in[i];
                                end
                                addr_bytes[23:20] = 4'b0000;
                                Address = addr_bytes;
                                change_addr = 1'b1;
                                #1 change_addr = 1'b0;
                                bus_cycle_state = MODE_BYTE;
                            end
                        end
                        else if ((Instruct == DIOR) || (Instruct == DIOW))
                        begin
                        //DUAL I/O High Performance Read(3 Bytes Addr)
                            Address_in[2*addr_cnt]     = SO_in;
                            Address_in[2*addr_cnt + 1] = SI_in;
                            read_cnt = 0;
                            addr_cnt = addr_cnt + 1;
                            if (addr_cnt == 3*BYTE/2)
                            begin
                                addr_cnt = 0;
                                for(i=23;i>=0;i=i-1)
                                begin
                                    addr_bytes[23-i] = Address_in[i];
                                end
                                addr_bytes[23:20] = 4'b0000;
                                Address = addr_bytes;
                                change_addr = 1'b1;
                                #1 change_addr = 1'b0;
                                bus_cycle_state = MODE_BYTE;
                            end
                        end
                        else if ((Instruct == DDRQIOR) || (Instruct == DDRQIOW)
                        && QUAD)
                        begin
                       //Quad I/O DDR Read Mode (3 Bytes Address)
                            Address_in[4*addr_cnt]   = RESETNeg_in;
                            Address_in[4*addr_cnt+1] = WPNeg_in;
                            Address_in[4*addr_cnt+2] = SO_in;
                            Address_in[4*addr_cnt+3] = SI_in;
                            opcode_tmp[addr_cnt/2]   = SI_in;
                            addr_cnt = addr_cnt +1;
                            read_cnt = 0;
                        end
                        else if ((Instruct == DDRFR) || (Instruct == DDRWRITE) ||
                        (Instruct == DDR_FAST_WRITE))
                        begin
                            if (QPI)
                            begin
                                Address_in[4*addr_cnt]   = RESETNeg_in;
                                Address_in[4*addr_cnt+1] = WPNeg_in;
                                Address_in[4*addr_cnt+2] = SO_in;
                                Address_in[4*addr_cnt+3] = SI_in;
                                addr_cnt = addr_cnt +1;
                                read_cnt = 0;
                            end
                            else if (DPI)
                            begin
                                Address_in[2*addr_cnt]   = SO_in;
                                Address_in[2*addr_cnt+1] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                            end
                            else
                            begin
                                Address_in[addr_cnt] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                            end
                        end
                        else if ((Instruct == FAST_READ) || (Instruct == FAST_WRITE))
                        begin
                        //Instruction + 3 Bytes Address + Dummy Byte
                            if (QPI)
                            begin
                                Address_in[4*addr_cnt]   = RESETNeg_in;
                                Address_in[4*addr_cnt+1] = WPNeg_in;
                                Address_in[4*addr_cnt+2] = SO_in;
                                Address_in[4*addr_cnt+3] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/4)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = MODE_BYTE;
                                end
                            end
                            else if (DPI)
                            begin
                                Address_in[2*addr_cnt]   = SO_in;
                                Address_in[2*addr_cnt+1] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/2)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = MODE_BYTE;
                                end
                            end
                            else
                            begin
                                Address_in[addr_cnt] = SI_in;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes ;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = MODE_BYTE;
                                end
                            end
                        end
                        else if ((Instruct == SSWR) || (Instruct == WRITE_MEM) ||
                                (Instruct == WRR) || (Instruct == WRAR))
                        begin
                        //Instruction + 3 Bytes Address + Dummy Byte
                            if (QPI)
                            begin
                                Address_in[4*addr_cnt]   = RESETNeg_in;
                                Address_in[4*addr_cnt+1] = WPNeg_in;
                                Address_in[4*addr_cnt+2] = SO_in;
                                Address_in[4*addr_cnt+3] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/4)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                            else if (DPI)
                            begin
                                Address_in[2*addr_cnt]   = SO_in;
                                Address_in[2*addr_cnt+1] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/2)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                            else
                            begin
                                Address_in[addr_cnt] = SI_in;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes ;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                        end
                        else if (Instruct == CRCC)
                        begin
                        // Instruction + 3 Bytes Address
                            if (QPI)
                            begin
                                Address_in[4*(addr_cnt % 6)]   = RESETNeg_in;
                                Address_in[4*(addr_cnt % 6)+1] = WPNeg_in;
                                Address_in[4*(addr_cnt % 6)+2] = SO_in;
                                Address_in[4*(addr_cnt % 6)+3] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/4)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                end
                                if (addr_cnt == 6*BYTE/4)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                            else if (DPI)
                            begin
                                Address_in[2*(addr_cnt % 12)]   = SO_in;
                                Address_in[2*(addr_cnt % 12)+1] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/2)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                end
                                if (addr_cnt == 6*BYTE/2)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                            else
                            begin
                                Address_in[addr_cnt % 24] = SI_in;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[31-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                end
                                if (addr_cnt == 6*BYTE)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    bus_cycle_state = DATA_BYTES;
                                end
                            end
                        end
                        else // All other instructions
                        begin
                        //Instruction + 3 Bytes Address
                            if (QPI)
                            begin
                                Address_in[4*addr_cnt]   = RESETNeg_in;
                                Address_in[4*addr_cnt+1] = WPNeg_in;
                                Address_in[4*addr_cnt+2] = SO_in;
                                Address_in[4*addr_cnt+3] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt +1;
                                if (addr_cnt == 3*BYTE/4)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    if (Register_Latency == 0)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                            else if (DPI)
                            begin
                                Address_in[2*addr_cnt]   = SO_in;
                                Address_in[2*addr_cnt+1] = SI_in;
                                read_cnt = 0;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE/2)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    if (Register_Latency == 0)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                            else
                            begin
                                Address_in[addr_cnt] = SI_in;
                                addr_cnt = addr_cnt + 1;
                                if (addr_cnt == 3*BYTE)
                                begin
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;

                                    if (Register_Latency == 0)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                        end
                    end //end of ADDRESS_BYTES

                    MODE_BYTE :
                    begin
                        if ((Instruct == DIOR) || (Instruct == DIOW) ||
                        ((Instruct == FAST_READ) && DPI))
                        begin
                            mode_in[2*mode_cnt]   = SO_in;
                            mode_in[2*mode_cnt+1] = SI_in;
                            mode_cnt = mode_cnt + 1;
                            if (mode_cnt == BYTE/2)
                            begin
                                mode_cnt = 0;
                                for(i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                if ((Latency_code == 0) || (Instruct == DIOW))
                                    bus_cycle_state = DATA_BYTES;
                                else
                                    bus_cycle_state = DUMMY_BYTES;
                            end
                        end
                        else if (((Instruct == QIOR) && QUAD) ||
                                ((Instruct == FAST_READ) && QPI))
                        begin
                            mode_in[4*mode_cnt]   = RESETNeg_in;
                            mode_in[4*mode_cnt+1] = WPNeg_in;
                            mode_in[4*mode_cnt+2] = SO_in;
                            mode_in[4*mode_cnt+3] = SI_in;
                            mode_cnt = mode_cnt + 1;
                            if (mode_cnt == BYTE/4)
                            begin
                                mode_cnt = 0;
                                for(i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                if (Latency_code == 0)
                                    bus_cycle_state = DATA_BYTES;
                                else
                                    bus_cycle_state = DUMMY_BYTES;
                            end
                        end
                        else if ((Instruct == QIOW) && QUAD)
                        begin
                            mode_in[4*mode_cnt]   = RESETNeg_in;
                            mode_in[4*mode_cnt+1] = WPNeg_in;
                            mode_in[4*mode_cnt+2] = SO_in;
                            mode_in[4*mode_cnt+3] = SI_in;
                            mode_cnt = mode_cnt + 1;
                            if (mode_cnt == BYTE/4)
                            begin
                                mode_cnt = 0;
                                for(i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                bus_cycle_state = DATA_BYTES;
                            end
                        end
                        else if ((Instruct == DIW) || (Instruct == QIW) ||
                                ((Instruct == FAST_READ) && (~QPI)))
                        begin
                            mode_in[mode_cnt] = SI_in;
                            mode_cnt = mode_cnt + 1;
                            if (mode_cnt == BYTE)
                            begin
                                mode_cnt = 0;
                                for(i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                if (Instruct != FAST_READ)
                                begin
                                    bus_cycle_state = DATA_BYTES;
                                end
                                else
                                begin
                                    if (Latency_code == 0)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                        end
                        else if ((Instruct == DOR) || (Instruct == QOR))
                        begin
                            mode_in[mode_cnt] = SI_in;
                            mode_cnt = mode_cnt + 1;
                            if (mode_cnt == BYTE)
                            begin
                                mode_cnt = 0;
                                for (i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                if (Latency_code == 0)
                                    bus_cycle_state = DATA_BYTES;
                                else
                                    bus_cycle_state = DUMMY_BYTES;
                            end
                        end
                        else if (Instruct == DDRFR)
                        begin
                            if (QPI)
                            begin
                                mode_in[0] = RESETNeg_in;
                                mode_in[1] = WPNeg_in;
                                mode_in[2] = SO_in;
                                mode_in[3] = SI_in;
                            end
                            else if (DPI)
                            begin
                                mode_in[4*mode_cnt]   = SO_in;
                                mode_in[4*mode_cnt+1] = SI_in;
                            end
                            else
                                mode_in[2*mode_cnt] = SI_in;
                        end
                        else if (((Instruct == DDRQIOR) || (Instruct == DDRQIOW))
                                 && QUAD)
                        begin
                            mode_in[0] = RESETNeg_in;
                            mode_in[1] = WPNeg_in;
                            mode_in[2] = SO_in;
                            mode_in[3] = SI_in;
                        end
                        dummy_cnt = 0;
                    end //end of MODE_BYTE

                    DUMMY_BYTES :
                    begin
                        if (CHECK_FREQ)
                        begin
                            CLK_PER = $time - LAST_CLK;
                            LAST_CLK = $time;
                            if (((Instruct == FAST_READ) && ~QPI) ||
                                (Instruct == DOR) || (Instruct == QOR))
                            begin
                                if (CLK_PER < 9250 && Latency_code >= 0) // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == FAST_READ) && DPI) || (Instruct == DIOR))
                            begin
                                if ((CLK_PER < 16670 && Latency_code == 0) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 1) || // <= 80MHz
                                    (CLK_PER <  9250 && Latency_code >= 2))   // <=108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if ((((Instruct == FAST_READ) || (Instruct == QIOR)) && QPI)
                            || ((Instruct == QIOR) && QUAD))
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 0) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 1) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 2) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 3) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 4) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 5) || // <= 100MHz
                                    (CLK_PER <  9250 && Latency_code >= 6))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && ~QPI)
                            begin
                                if ((CLK_PER < 19999 && Latency_code == 0) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 1) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 2) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 3) || // <= 100MHz
                                    (CLK_PER < 9250  && Latency_code >= 4))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && DPI)
                            begin
                                if ((CLK_PER < 33333 && Latency_code == 2) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 3) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 4) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 5) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 6) || // <= 100MHz
                                    (CLK_PER < 9250  && Latency_code >= 7))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == READ) || (Instruct == ECCRD) || (Instruct == SSRD)) && QPI)
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 2) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 3) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 4) || // <= 50MHz
                                    (CLK_PER < 16670 && Latency_code == 5) || // <= 60MHz
                                    (CLK_PER < 12500 && Latency_code == 6) || // <= 80MHz
                                    (CLK_PER < 10000 && Latency_code == 7) || // <= 100MHz
                                    (CLK_PER <  9250 && Latency_code >= 8))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if (((Instruct == DDRFR) && QPI) || (Instruct == DDRQIOR))
                            begin
                                if ((CLK_PER < 66670 && Latency_code == 2) || // <= 15MHz
                                    (CLK_PER < 33333 && Latency_code == 3) || // <= 30MHz
                                    (CLK_PER < 20000 && Latency_code == 4) || // <= 50MHz
                                    (CLK_PER < 18510 && Latency_code >= 5))   // <= 54MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end

                            if ((Instruct == RDSR1) || (Instruct == RDSR2) ||
                                (Instruct == RDCR1) || (Instruct == RDCR2) ||
                                (Instruct == RDCR4) || (Instruct == RDCR5) ||
                                (Instruct == RDAR)  || (Instruct == RUID)  ||
                                (Instruct == RDID)  || (Instruct == RDSN))
                            begin
                                if ((CLK_PER < 20000 && Register_Latency == 0) || // <= 50MHz
                                    (CLK_PER < 9250 && Register_Latency >= 1))   // <= 108MHz
                                begin
                                    $display ("More wait states are required for");
                                    $display ("this clock frequency value");
                                end
                                CHECK_FREQ = 0;
                            end
                        end // CHECK_FREQ

                        dummy_cnt = dummy_cnt + 1;
                    end //end of DUMMY_BYTES

                    DATA_BYTES :
                    begin
                        CHECK_FREQ = 0;

                        if ((Instruct == DDRQIOR) || (Instruct == DDRFR))
                        begin
                            read_out = 1'b1;
                            read_out <= #1 1'b0;
                        end

                        if (QPI)
                        begin
                            quad_nybble = {RESETNeg_in, WPNeg_in, SO_in, SI_in};
                            if (data_cnt > ((PageSize+1)*2-1))
                            begin
                            //In case of quad mode,if more than PageSize+1 bytes
                            //are sent to the device previously latched data
                            //are discarded and last 256/512 data bytes are
                            //guaranteed to be programmed correctly within
                            //the same page.
                                for(i=0;i<=(PageSize*2-1);i=i+1)
                                begin
                                    quad_data_in[i] = quad_data_in[i+1];
                                end
                                quad_data_in[(PageSize+1)*2-1] = quad_nybble;
                                data_cnt = data_cnt +1;
                            end
                            else
                            begin
                                if (quad_nybble !== 4'bZZZZ)
                                begin
                                    quad_data_in[data_cnt] = quad_nybble;
                                end
                                data_cnt = data_cnt +1;
                            end
                        end
                        else if (DPI)
                        begin
                            if (data_cnt > ((PageSize+1)*4-1))
                            begin
                            //In case of serial mode and PP,
                            //if more than PageSize are sent to the device
                            //previously latched data are discarded and last
                            //256/512 data bytes are guaranteed to be programmed
                            //correctly within the same page.
                                if (bit_cnt == 0)
                                begin
                                    for(i=0;i<=(PageSize*4-1);i=i+1)
                                    begin
                                        Data_in[i] = Data_in[i+4];
                                    end
                                end
                                Data_in[PageSize*4 + bit_cnt] = SI_in;
                                bit_cnt = bit_cnt + 1;
                                if (bit_cnt == 4)
                                begin
                                    bit_cnt = 0;
                                end
                                data_cnt = data_cnt + 1;
                            end
                            else
                            begin
                                Data_in[2*data_cnt]   = SO_in;
                                Data_in[2*data_cnt+1] = SI_in;
                                data_cnt = data_cnt + 1;
                                bit_cnt = 0;
                            end
                        end
                        else
                        begin
                            if (data_cnt > ((PageSize+1)*8-1))
                            begin
                            //In case of serial mode and PP,
                            //if more than PageSize are sent to the device
                            //previously latched data are discarded and last
                            //256/512 data bytes are guaranteed to be programmed
                            //correctly within the same page.
                                if (bit_cnt == 0)
                                begin
                                    for(i=0;i<=(PageSize*BYTE-1);i=i+1)
                                    begin
                                        Data_in[i] = Data_in[i+8];
                                    end
                                end
                                Data_in[PageSize*BYTE + bit_cnt] = SI_in;
                                bit_cnt = bit_cnt + 1;
                                if (bit_cnt == 8)
                                begin
                                    bit_cnt = 0;
                                end
                                data_cnt = data_cnt + 1;
                            end
                            else
                            begin
                                Data_in[data_cnt] = SI_in;
                                data_cnt = data_cnt + 1;
                                bit_cnt = 0;
                            end
                        end
                    end //end of DATA_BYTES

                endcase
            end
        end

        if (falling_edge_SCK_ipd)
        begin

            if (~CSNeg_ipd)
            begin
                case (bus_cycle_state)
                    ADDRESS_BYTES :
                    begin
                        if ((Instruct == DDRFR) || (Instruct == DDRWRITE) ||
                        (Instruct == DDR_FAST_WRITE))
                        begin
                            if (QPI)
                            begin
                                Address_in[4*addr_cnt]   = RESETNeg_in;
                                Address_in[4*addr_cnt+1] = WPNeg_in;
                                Address_in[4*addr_cnt+2] = SO_in;
                                Address_in[4*addr_cnt+3] = SI_in;
                                read_cnt = 0;
                                if (addr_cnt != 0)
                                begin
                                    addr_cnt = addr_cnt + 1;
                                end
                                if (addr_cnt == 3*BYTE/4)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    if (Instruct == DDRWRITE)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = MODE_BYTE;
                                end
                            end
                            else if (DPI)
                            begin
                                Address_in[2*addr_cnt]   = SO_in;
                                Address_in[2*addr_cnt+1] = SI_in;
                                read_cnt = 0;
                                if (addr_cnt != 0)
                                begin
                                    addr_cnt = addr_cnt + 1;
                                end
                                if (addr_cnt == 3*BYTE/2)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    if (Instruct == DDRWRITE)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = MODE_BYTE;
                                end
                            end
                            else
                            begin
                                // Fast DDR Read Mode
                                Address_in[addr_cnt] = SI_in;
                                read_cnt = 0;
                                if (addr_cnt != 0)
                                begin
                                    addr_cnt = addr_cnt + 1;
                                end
                                if (addr_cnt == 3*BYTE)
                                begin
                                    addr_cnt = 0;
                                    for(i=23;i>=0;i=i-1)
                                    begin
                                        addr_bytes[23-i] = Address_in[i];
                                    end
                                    addr_bytes[23:20] = 4'b0000;
                                    Address = addr_bytes;
                                    change_addr = 1'b1;
                                    #1 change_addr = 1'b0;
                                    if (Instruct == DDRWRITE)
                                        bus_cycle_state = DATA_BYTES;
                                    else
                                        bus_cycle_state = MODE_BYTE;
                                end
                            end
                        end
                        else if ((Instruct == DDRQIOR) && (QUAD || QPI))
                        begin
                        //Quad I/O DDR Read Mode (3 Bytes Address)
                            Address_in[4*addr_cnt]   = RESETNeg_in;
                            Address_in[4*addr_cnt+1] = WPNeg_in;
                            Address_in[4*addr_cnt+2] = SO_in;
                            Address_in[4*addr_cnt+3] = SI_in;
                            if (addr_cnt != 0)
                            begin
                                addr_cnt = addr_cnt + 1;
                            end
                            read_cnt = 0;
                            if (addr_cnt == 3*BYTE/4)
                            begin
                                addr_cnt = 0;
                                for(i=23;i>=0;i=i-1)
                                begin
                                    addr_bytes[23-i] = Address_in[i];
                                end
                                addr_bytes[23:20] = 4'b0000;
                                Address = addr_bytes;
                                change_addr = 1'b1;
                                #1 change_addr = 1'b0;

                                bus_cycle_state = MODE_BYTE;
                            end
                        end
                        else if ((Instruct == DDRQIOW) && QUAD)
                        begin
                            Address_in[4*addr_cnt]   = RESETNeg_in;
                            Address_in[4*addr_cnt+1] = WPNeg_in;
                            Address_in[4*addr_cnt+2] = SO_in;
                            Address_in[4*addr_cnt+3] = SI_in;
                            if (addr_cnt != 0)
                            begin
                                addr_cnt = addr_cnt + 1;
                            end
                            read_cnt = 0;
                            if (addr_cnt == 3*BYTE/4)
                            begin
                                addr_cnt = 0;
                                for(i=23;i>=0;i=i-1)
                                begin
                                    addr_bytes[23-i] = Address_in[i];
                                end
                                addr_bytes[23:20] = 4'b0000;
                                Address = addr_bytes;
                                change_addr = 1'b1;
                                #1 change_addr = 1'b0;

                                bus_cycle_state = MODE_BYTE;
                            end
                        end
                        else if (Instruct == CRCC)
                        begin
                            if (((addr_cnt == 3*BYTE/4) && QPI) ||
                                ((addr_cnt == 3*BYTE/2) && DPI) ||
                                ((addr_cnt == 3*BYTE) && (~QPI)))
                                CRC_Start_Addr_reg = Address;
                        end
                    end //end of ADDRESS_BYTES

                    MODE_BYTE :
                    begin
                        if (Instruct == DDRFR)
                        begin
                            if (QPI)
                            begin
                                mode_in[4] = RESETNeg_in;
                                mode_in[5] = WPNeg_in;
                                mode_in[6] = SO_in;
                                mode_in[7] = SI_in;
                                for(i=7;i>=0;i=i-1)
                                begin
                                    mode_bytes[i] = mode_in[7-i];
                                end
                                bus_cycle_state = DUMMY_BYTES;
                            end
                            else if (DPI)
                            begin
                                mode_in[4*mode_cnt+2] = SO_in;
                                mode_in[4*mode_cnt+3] = SI_in;
                                mode_cnt = mode_cnt + 1;
                                if (mode_cnt == BYTE/4)
                                begin
                                    mode_cnt = 0;
                                    for(i=7;i>=0;i=i-1)
                                    begin
                                        mode_bytes[i] = mode_in[7-i];
                                    end
                                    bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                            else
                            begin
                                mode_in[2*mode_cnt+1] = SI_in;
                                mode_cnt = mode_cnt + 1;
                                if (mode_cnt == BYTE/2)
                                begin
                                    mode_cnt = 0;
                                    for(i=7;i>=0;i=i-1)
                                    begin
                                        mode_bytes[i] = mode_in[7-i];
                                    end
                                    bus_cycle_state = DUMMY_BYTES;
                                end
                            end
                        end
                        else if (((Instruct == DDRQIOR) || (Instruct == DDRQIOW))
                                && QUAD)
                        begin
                            mode_in[4] = RESETNeg_in;
                            mode_in[5] = WPNeg_in;
                            mode_in[6] = SO_in;
                            mode_in[7] = SI_in;
                            for(i=7;i>=0;i=i-1)
                            begin
                                mode_bytes[i] = mode_in[7-i];
                            end

                            if ((Latency_code == 0) || (Instruct == DDRQIOW))
                                bus_cycle_state = DATA_BYTES;
                            else
                                bus_cycle_state = DUMMY_BYTES;
                        end
                    end //end of MODE_BYTE

                    DUMMY_BYTES :
                    begin
                        dummy_cnt = dummy_cnt + 1;

                        if ((Instruct == RDSR1) || (Instruct == RDSR2) ||
                            (Instruct == RDCR1) || (Instruct == RDCR2) ||
                            (Instruct == RDCR4) || (Instruct == RDCR5) ||
                            (Instruct == RDID)  || (Instruct == RDSN)  ||
                            (Instruct == RUID)  || (Instruct == RDAR))
                        begin
                            if (Register_Latency == dummy_cnt/2)
                            begin
                                bus_cycle_state = DATA_BYTES;
                                read_out = 1'b1;
                                read_out <= #1 1'b0;
                            end
                        end
                        else
                        begin
                            if (Latency_code == dummy_cnt/2)
                            begin
                                bus_cycle_state = DATA_BYTES;
                                read_out = 1'b1;
                                read_out <= #1 1'b0;
                            end
                        end
                    end //end of DUMMY_BYTES

                    DATA_BYTES :
                    begin
                        if ((((Instruct == DDRQIOR) || (Instruct == QIOR)) && QUAD) ||
                            (Instruct == READ)     || (Instruct == FAST_READ) ||
                            (Instruct == RDSR1)    || (Instruct == RDSR2) ||
                            (Instruct == RDCR1)    || (Instruct == RDCR2) ||
                            (Instruct == RDCR4)    || (Instruct == RDCR5) ||
                            (Instruct == RUID)     || (Instruct == RDID)  ||
                            (Instruct == DIOR)     || (Instruct == RDSN)  ||
                            (Instruct == SSRD)     || (Instruct == DOR)   ||
                            (Instruct == QOR)      || (Instruct == DDRFR) ||
                            (Instruct == ECCRD)    || (Instruct == RDAR))
                        begin
                            read_out = 1'b1;
                            read_out <= #1 1'b0;
                        end
                        else if (Instruct == CRCC)
                            CRC_End_Addr_reg = Address;
                    end //end of DATA_BYTES

                endcase
            end
        end

        if (rising_edge_CSNeg_ipd)
        begin
            if (bus_cycle_state != DATA_BYTES)
            begin
                bus_cycle_state = STAND_BY;
            end
            else
            begin
                if (((mode_bytes[7:4] == 4'b1010) &&
                     (Instruct == FAST_WRITE || Instruct == DIW ||
                      Instruct == DIOW || Instruct == QIW       ||
                      Instruct == QIOW || Instruct == FAST_READ ||
                      Instruct == DOR  || Instruct == DIOR      ||
                      Instruct == QOR  || Instruct == QIOR)) ||
                    ((mode_bytes == 8'b10100101) &&
                     (Instruct == DDR_FAST_WRITE || Instruct == DDRFR ||
                      Instruct == DDRQIOW || Instruct == DDRQIOR)))
                    bus_cycle_state = ADDRESS_BYTES;
                else
                    bus_cycle_state = STAND_BY;

                case (Instruct)
                    WREN,
                    WRDI,
                    DPD,
                    HBN,
                    RSTEN,
                    RSTCMD,
                    EPCS,
                    EPCR,
                    CRCC:
                    begin
                        if (data_cnt == 0)
                            write = 1'b0;
                    end

                    READ:
                    begin
                        if (dpd_act)
                            write = 1'b0;
                    end

                    WRR:
                    begin
                        if (QPI)
                        begin
                            if (data_cnt == 2)
                            //If CS# is driven high after eight
                            //cycle,only the Status Register is
                            //written.
                            begin
                                write = 1'b0;
                                for(i=1;i>=0;i=i-1)
                                begin
                                    Quad_slv = quad_data_in[1-i];
                                    SR1_in[(4*i+3) -: 4] = Quad_slv;
                                end
                            end
                            else if (data_cnt == 4)
                            //After the 16th cycle both the
                            //Status and Configuration Registers
                            //are written to.
                            begin
                                write = 1'b0;
                                cfg_write = 1'b1;

                                for(i=3;i>=0;i=i-1)
                                begin
                                    Quad_slv = quad_data_in[3-i];
                                    if (i == 3)
                                        SR1_in[7:4] = Quad_slv;
                                    else if (i == 2)
                                        SR1_in[3:0] = Quad_slv;
                                    else if (i == 1)
                                        CR1_in[7:4] = Quad_slv;
                                    else if (i == 0)
                                        CR1_in[3:0] = Quad_slv;
                                end
                            end
                        end
                        else if (DPI)
                        begin
                            if (data_cnt == 4)
                            //If CS# is driven high after eight
                            //cycle,only the Status Register is
                            //written.
                            begin
                                write = 1'b0;
                                for(i=0;i<=3;i=i+1)
                                begin
                                    SR1_in[2*i]   = Data_in[7-2*i];
                                    SR1_in[2*i+1] = Data_in[6-2*i];
                                end
                            end
                            else if (data_cnt == 8)
                            //After the 16th cycle both the
                            //Status and Configuration Registers
                            //are written to.
                            begin
                                write = 1'b0;
                                cfg_write = 1'b1;

                                for(i=0;i<=3;i=i+1)
                                begin
                                    SR1_in[2*i]   = Data_in[7-2*i];
                                    SR1_in[2*i+1] = Data_in[6-2*i];
                                    CR1_in[2*i]   = Data_in[15-2*i];
                                    CR1_in[2*i+1] = Data_in[14-2*i];
                                end
                            end
                        end
                        else
                        begin
                            if (data_cnt == 8)
                            //If CS# is driven high after eight
                            //cycle,only the Status Register is
                            //written.
                            begin
                                write = 1'b0;
                                for(i=0;i<=7;i=i+1)
                                begin
                                    SR1_in[i] = Data_in[7-i];
                                end
                            end
                            else if (data_cnt == 16)
                            //After the 16th cycle both the
                            //Status and Configuration Registers
                            //are written to.
                            begin
                                write = 1'b0;
                                cfg_write = 1'b1;

                                for(i=0;i<=7;i=i+1)
                                begin
                                    SR1_in[i] = Data_in[7-i];
                                    CR1_in[i] = Data_in[15-i];
                                end
                            end
                        end
                    end

                    WRAR:
                    begin
                        if (QPI)
                        begin
                            if (data_cnt == 2)
                            begin
                                write = 1'b0;
                                for(i=1;i>=0;i=i-1)
                                begin
                                   Quad_slv = quad_data_in[1-i];
                                   if (i==1)
                                        WRAR_reg_in[7:4] = Quad_slv;
                                    else if (i==0)
                                        WRAR_reg_in[3:0] = Quad_slv;
                                end
                            end
                        end
                        else if (DPI)
                        begin
                            if (data_cnt == 4)
                            begin
                                write = 1'b0;
                                for(i=0;i<=3;i=i+1)
                                begin
                                    WRAR_reg_in[2*i]   = Data_in[7-2*i];
                                    WRAR_reg_in[2*i+1] = Data_in[6-2*i];
                                end
                            end
                        end
                        else
                        begin
                            if (data_cnt == 8)
                            begin
                                write = 1'b0;
                                for(i=0;i<=7;i=i+1)
                                begin
                                    WRAR_reg_in[i] = Data_in[7-i];
                                end
                            end
                        end
                    end

                    WRSN:
                    begin
                        if (QPI)
                        begin
                            if (data_cnt == 16)
                            begin
                                write = 1'b0;
                                for(i=15;i>=0;i=i-1)
                                begin
                                    Quad_slv = quad_data_in[15-i];
                                    SERNUM_reg[4*i+3 -: 4] = Quad_slv;
                                end
                            end
                        end
                        else if (DPI)
                        begin
                            if (data_cnt == 32)
                            begin
                                write = 1'b0;
                                for(i=0;i<=31;i=i+1)
                                begin
                                    SERNUM_reg[2*i+1 -: 2] = {SO_in, SI_in};
                                end
                            end
                        end
                        else
                        begin
                            if (data_cnt == 64)
                            begin
                                write = 1'b0;
                                for(i=1;i<=8;i=i+1)
                                begin
                                    for(j=1;j<=8;j=j+1)
                                    begin
                                        SERNUM_reg[8*i-j] = Data_in[8*(i-1)+j-1];
                                    end
                                end
                            end
                        end
                    end

                    WRITE_MEM,
                    DDRWRITE,
                    FAST_WRITE,
                    DDR_FAST_WRITE,
                    SSWR:
                    begin
                        if (QPI)
                        begin
                            if (data_cnt > 0)
                            begin
                                if ((data_cnt % 2) == 0)
                                begin
                                    write = 1'b0;
                                    for(i=0;i<=PageSize;i=i+1)
                                    begin
                                        for(j=1;j>=0;j=j-1)
                                        begin
                                            Quad_slv =
                                            quad_data_in[(i*2)+(1-j)];
                                            if (j==1)
                                                Byte_slv[7:4] = Quad_slv;
                                            else if (j==0)
                                                Byte_slv[3:0] = Quad_slv;
                                        end
                                        WByte[i] = Byte_slv;
                                    end
                                    if (data_cnt > (PageSize+1)*2)
                                        Byte_number = PageSize;
                                    else
                                        Byte_number = ((data_cnt/2)-1);
                                end
                            end
                        end
                        else if (DPI)
                        begin
                            if (data_cnt > 0)
                            begin
                                if ((data_cnt % 4) == 0)
                                begin
                                    write = 1'b0;
                                    for(i=0;i<=PageSize;i=i+1)
                                    begin
                                        for(j=3;j>=0;j=j-1)
                                        begin
                                            if ((Data_in[(i*4)+(3-j)]) !== 1'bX)
                                            begin
                                                Byte_slv[2*j+1] =
                                                           Data_in[(i*4)+(6-2*j)];
                                                Byte_slv[2*j] =
                                                           Data_in[(i*4)+(6-2*j+1)];
                                            end
                                        end
                                        WByte[i] = Byte_slv;
                                    end

                                    if (data_cnt > (PageSize+1)*4)
                                        Byte_number = PageSize;
                                    else
                                        Byte_number = ((data_cnt/4)-1);
                                end
                            end
                        end
                        else
                        begin
                            if (data_cnt > 0)
                            begin
                                if ((data_cnt % 8) == 0)
                                begin
                                    write = 1'b0;
                                    for(i=0;i<=PageSize;i=i+1)
                                    begin
                                        for(j=7;j>=0;j=j-1)
                                        begin
                                            if ((Data_in[(i*8)+(7-j)]) !== 1'bX)
                                            begin
                                                Byte_slv[j] =
                                                           Data_in[(i*8)+(7-j)];
                                                if (Data_in[(i*8)+(7-j)]==1'b0)
                                                begin
                                                    ZERO_DETECTED = 1'b1;
                                                end
                                            end
                                        end
                                        WByte[i] = Byte_slv;
                                    end

                                    if (data_cnt > (PageSize+1)*BYTE)
                                        Byte_number = PageSize;
                                    else
                                        Byte_number = ((data_cnt/8) - 1);
                                end
                            end
                        end
                    end

                    DIW,
                    DIOW:
                    begin
                        if (data_cnt > 0)
                        begin
                            if ((data_cnt % 4) == 0)
                            begin
                                write = 1'b0;
                                for(i=0;i<=PageSize;i=i+1)
                                begin
                                    for(j=3;j>=0;j=j-1)
                                    begin
                                        if ((Data_in[(i*4)+(3-j)]) !== 1'bX)
                                        begin
                                            Byte_slv[2*j+1] =
                                                        Data_in[(i*4)+(6-2*j)];
                                            Byte_slv[2*j] =
                                                        Data_in[(i*4)+(6-2*j+1)];
                                        end
                                    end
                                    WByte[i] = Byte_slv;
                                end

                                if (data_cnt > (PageSize+1)*4)
                                    Byte_number = PageSize;
                                else
                                    Byte_number = ((data_cnt/4)-1);
                            end
                        end
                    end

                    QIW,
                    QIOW,
                    DDRQIOW:
                    begin
                        if (data_cnt > 0)
                        begin
                            if ((data_cnt % 2) == 0)
                            begin
                                write = 1'b0;
                                for(i=0;i<=PageSize;i=i+1)
                                begin
                                    for(j=1;j>=0;j=j-1)
                                    begin
                                        Quad_slv =
                                        quad_data_in[(i*2)+(1-j)];
                                        if (j==1)
                                            Byte_slv[7:4] = Quad_slv;
                                        else if (j==0)
                                            Byte_slv[3:0] = Quad_slv;
                                    end
                                    WByte[i] = Byte_slv;
                                end
                                if (data_cnt > (PageSize+1)*2)
                                    Byte_number = PageSize;
                                else
                                    Byte_number = ((data_cnt/2)-1);
                            end
                        end
                    end

                endcase
            end
        end
    end

///////////////////////////////////////////////////////////////////////////////
// Timing control for the Page Program
///////////////////////////////////////////////////////////////////////////////
    time  pob;
    event pdone_event;

    always @(rising_edge_PSTART or rising_edge_reseted)
    begin : ProgTime

        pob = 1000;

        if (rising_edge_reseted)
        begin
            PDONE = 1; // reset done, programing terminated
            disable pdone_process;
        end
        else if (reseted)
        begin
            if (rising_edge_PSTART && PDONE)
            begin
                PDONE = 1'b0;
                ->pdone_event;
            end
        end
    end

    always @(pdone_event)
    begin : pdone_process
        #pob PDONE = 1;
    end

///////////////////////////////////////////////////////////////////////////////
// Timing control for the Write Status Register
///////////////////////////////////////////////////////////////////////////////
    time  wob;
    event wdone_event;
    event csdone_event;

    always @(rising_edge_WSTART or rising_edge_reseted)
    begin:WriteTime

        wob = 1000;

        if (rising_edge_reseted)
        begin
            WDONE = 1; // reset done, Write terminated
            disable wdone_process;
        end
        else if (reseted)
        begin
            if (rising_edge_WSTART && WDONE)
            begin
                WDONE = 1'b0;
                -> wdone_event;
            end
        end
    end

    always @(wdone_event)
    begin : wdone_process
        #wob WDONE = 1;
    end

   always @(posedge CSSTART or rising_edge_reseted)
   begin:WriteVolatileBitsTime

        if (rising_edge_reseted)
        begin
            CSDONE = 1; // reset done, Write terminated
            disable csdone_process;
        end
        else if (reseted)
        begin
            if (CSSTART && CSDONE)
            begin
                CSDONE = 1'b0;
                -> csdone_event;
            end
        end
    end

    always @(csdone_event)
    begin : csdone_process
        #tdevice_CS CSDONE = 1;
    end

    ///////////////////////////////////////////////////////////////////
    // Timing control for the suspend process
    ///////////////////////////////////////////////////////////////////
    always @(rising_edge_START_T1_in)
    begin : Start_T1_time
        if (rising_edge_START_T1_in)
        begin
            if (CRC_ACT == 1'b1)
            begin
                sSTART_T1 = 1'b0;
                sSTART_T1 <= #tdevice_CRCSL 1'b1;
            end
        end
        else
        begin
            sSTART_T1 = 1'b0;
        end
    end

    ///////////////////////////////////////////////////////////////////
    // Timing control for the CRC calculation
    ///////////////////////////////////////////////////////////////////
    event crcdone_event;
    time elapsed_crc;
    time start_crc;
    time crc_duration;

    always @(rising_edge_CRCSTART or rising_edge_reseted)
    begin : CRCTime

        if (rising_edge_reseted)
        begin
            CRCDONE = 1;
            disable crcdone_process;
        end
        else if (reseted)
        begin
            if ((rising_edge_CRCSTART) && CRCDONE)
            begin
                crc_duration = tdevice_CRCSETUP;
                elapsed_crc = 0;
                CRCDONE = 1'b0;
                start_crc = $time;
                -> crcdone_event;
            end
        end
    end

    always @(posedge CRCSUSP)
    begin
        if (CRCSUSP && (~CRCDONE))
        begin
            disable crcdone_process;
            elapsed_crc = $time - start_crc;
            crc_duration = crc_duration - elapsed_crc;
            CRCDONE = 1'b0;
        end
    end

    always @(posedge CRCRES)
    begin
        start_crc = $time;
        ->crcdone_event;
    end

    always @(crcdone_event)
    begin : crcdone_process
        #(crc_duration) CRCDONE = 1;
    end

    ///////////////////////////////////////////////////////////////////
    // Process for clock frequency determination
    ///////////////////////////////////////////////////////////////////
    always @(posedge SCK_ipd)
    begin : clock_period
        if (SCK_ipd)
        begin
            SCK_cycle = $time - prev_SCK;
            prev_SCK = $time;
        end
    end

//    /////////////////////////////////////////////////////////////////////////
//    // Main Behavior Process
//    // combinational process for next state generation
//    /////////////////////////////////////////////////////////////////////////

    integer i;
    integer j;

    always @(rising_edge_PoweredUp or falling_edge_write or rising_edge_WDONE or
           rising_edge_PDONE or rising_edge_RST_out or
           rising_edge_SWRST_out or //rising_edge_CSDONE or
           rising_edge_CRCDONE or rising_edge_HBN_out or rising_edge_REC_out or
           rising_edge_DPD_out or rising_edge_RES_out or rising_edge_RESETNeg)
    begin: StateGen1

        integer sect;

        if (rising_edge_PoweredUp && SWRST_out && RST_out)
        begin
            next_state = IDLE;
        end
        else if (PoweredUp)
        begin
            if (RST_out == 1'b0)
                next_state = current_state;
            else if (falling_edge_write && Instruct == RSTCMD)
            begin
                next_state = IDLE;
            end
            else
            begin
                case (current_state)
                    RESET_STATE :
                    begin
                        if (rising_edge_RST_out || rising_edge_SWRST_out)
                        begin
                            next_state = IDLE;
                        end
                    end

                    IDLE :
                    begin
                        if (falling_edge_write)
                        begin
                            if (Instruct == WRR && WEL == 1 &&
                            ~(SRWD && ~WPNeg_in && ~QUAD))
                            begin
                                next_state = WRITE_SR;
                            end
                            else if (Instruct == WRAR && WEL == 1 &&
                                ~(SRWD && ~WPNeg_in && ~QUAD &&
                                    (Address == 16'h0000 ||
                                    Address == 16'h0002)))
                            begin
                            // can not execute if WEL bit is zero or Hardware
                            // Protection Mode is entered and SR1NV,SR1V,CR1NV or
                            // CR1V is selected (no error is set)
                                if ((Address == 16'h0001)  ||
                                    (Address == 16'h0004)  ||
                                    (Address == 16'h0007)  ||
                                   ((Address >  16'h0008)  &&
                                    (Address <  16'h0040)) ||
                                   ((Address >  16'h0041)  &&
                                    (Address <  16'h0089)) ||
                                   ((Address >  16'h008F)  &&
                                    (Address <  16'h0095)) ||
                                    (Address >  16'h0098))
                                begin
                                    $display ("WARNING: Undefined location ");
                                    $display (" selected. Command is ignored!");
                                end
                                else if ((Address > 16'h0094) &&
                                         (Address < 16'h0099)) // CRC
                                begin
                                    $display ("WARNING: CRC register cannot be ");
                                    $display ("written by the WRAR command. ");
                                    $display ("Command is ignored!");
                                end
                                else // Protection Mode not selected
                                begin
                                    next_state = WRITE_ALL_REG;
                                end
                            end
                            else if ((Instruct == WRITE_MEM || Instruct == DDRWRITE ||
                                    Instruct == FAST_WRITE || Instruct == DDR_FAST_WRITE ||
                                    Instruct == DIW || Instruct == DIOW ||
                                    Instruct == QIW || Instruct == QIOW ||
                                    Instruct == DDRQIOW) && WEL == 1)
                            begin
                                sect = Address/(SecSize+1);

                                if (Sec_Prot[sect] == 0)
                                begin
                                    next_state = PAGE_PG;
                                end
                                else
                                    next_state = PGERS_ERROR;
                            end
                            else if (Instruct == SSWR && WEL == 1)
                            begin
                                if (Address + Byte_number <= OTPHiAddr)
                                begin //Program within valid OTP Range
                                    next_state =  OTP_PG;
                                end
                            end
                            else if (Instruct == CRCC)
                            begin
                                if (Address >= CRC_Start_Addr_reg + 4)
                                // Condition for entering CRC_calc state is not complete
                                // it needs to have comparison of Addr to EndAddr
                                // Check datasheet for table of state transitions
                                    next_state = CRC_Calc;
                                else
                                    next_state = IDLE;
                            end
                            else if (Instruct == EPCS)
                                next_state = CRC_SUSP;
                            else
                                next_state = IDLE;
                        end
                        else if (rising_edge_DPD_out)
                            next_state = DP_DOWN;
                        else if (rising_edge_HBN_out)
                            next_state = HIBERNATE;
                    end

                    WRITE_SR :
                    begin
                        if (rising_edge_WDONE)
                            next_state = IDLE;
                    end

                    WRITE_ALL_REG :
                    begin
                        if (rising_edge_WDONE)
                            next_state = IDLE;
                    end

                    PAGE_PG :
                    begin
                        if (rising_edge_PDONE)
                            next_state = IDLE;
                    end

                    OTP_PG :
                    begin
                        if (rising_edge_PDONE)
                            next_state = IDLE;
                    end

                    CRC_Calc :
                    begin
                        if ((Instruct == EPCS) || rising_edge_START_T1_in)
                            next_state = CRC_SUSP;
                        if (rising_edge_CRCDONE)
                            next_state = IDLE;
                    end

                    CRC_SUSP :
                    begin
                        if (falling_edge_write)
                        begin
                            if (Instruct == EPCR)
                                next_state = CRC_Calc;
                            else if (Instruct == RSTEN)
                                next_state = RESET_STATE;
                        end
                    end

                    PGERS_ERROR :
                    begin
                        if (falling_edge_write)
                        begin
                            if (Instruct == WRDI)
                            begin
                            // A Clear Status Register (CLSR) followed by a Write
                            // Disable (WRDI) command must be sent to return the
                            // device to standby state
                                next_state = IDLE;
                            end
                        end
                    end

                    DP_DOWN:
                    begin
                        if (rising_edge_RES_out && (Instruct == READ))
                            next_state = IDLE;
                    end

                    HIBERNATE :
                    begin
                        if (rising_edge_REC_out && (Instruct == READ))
                            next_state = IDLE;
                    end

                endcase
            end
        end
    end

//    /////////////////////////////////////////////////////////////////////////
//    //FSM Output generation and general functionality
//    /////////////////////////////////////////////////////////////////////////
    reg change_addr_event    = 1'b0;
    reg Instruct_event       = 1'b0;
    reg current_state_event  = 1'b0;

    integer WData [0:511];
    integer WOTPData;
    integer Addr;
    integer Addr_tmp;
    integer Addr_idcfi;

    always @(Instruct_event)
    begin
        read_cnt  = 0;
        byte_cnt  = 1;
        rd_slow   = 1'b0;
        dual      = 1'b0;
        ddr       = 1'b0;
        any_read  = 1'b0;
        Addr_idcfi  = 0;
    end

    always @(posedge read_out)
    begin
        if (PoweredUp == 1'b1)
        begin
            oe_z = 1'b1;
            #1000 oe_z = 1'b0;

            if (CSNeg_ipd==1'b0)
            begin
                oe = 1'b1;
                #1000 oe = 1'b0;
            end
        end
    end

    always @(change_addr_event)
    begin
        if (change_addr_event)
        begin
            if (Address > AddrRANGE)
                read_addr = Address-(AddrRANGE+1);
            else
                read_addr = Address;
        end
    end

    always @(rising_edge_PoweredUp or posedge oe or posedge oe_z or rising_edge_CRCDONE or
           posedge WDONE or current_state_event or posedge PDONE or negedge PDONE or//  posedge CSDONE or
           rising_edge_PDONE or falling_edge_PDONE or
           falling_edge_write or rising_edge_DPD_out or rising_edge_RES_out or
           Instruct or Address or rising_edge_HBN_out or rising_edge_REC_out or
           rising_edge_reseted or change_addr_event) // rising_edge_CSNeg_ipd or 
    begin: Functionality
    integer i,j;
    integer sect;

        if (rising_edge_PoweredUp)
        begin
            // the default condition after power-up
            // During POR,the non-volatile version of the registers is copied to
            // volatile version to provide the default state of the volatile
            // register
            SR1 = {SR1[7:2],2'b00};

            BP_bits = {SR1[4],SR1[3],SR1[2]};
            change_BP = 1'b1;
            #1 change_BP = 1'b0;

            CRC_ACT = 1'b0;
            CRC_RD_SETUP = 1'b0;
        end

        if (rising_edge_RES_out)
        begin
            RES_in = 1'b0;
        end

        if (rising_edge_DPD_out)
        begin
            DPD_in = 1'b0;
        end

        if (rising_edge_REC_out)
        begin
            REC_in = 1'b0;
        end

        if (rising_edge_HBN_out)
        begin
            HBN_in = 1'b0;
        end

        case (current_state)
            IDLE :
            begin
                dpd_act = 1'b0;
                if (falling_edge_write && ((DPD_in == 1'b0) || (HBN_in == 1'b0)))
                begin
                    if (Instruct == WREN)
                        SR1[1] = 1'b1;
                    else if (Instruct == WRDI)
                        SR1[1] = 0;
                    else if (Instruct == WRR && WEL == 1)
                    begin
                        if (~(SRWD && ~WPNeg_in && ~QUAD))
                        begin
                            if ((TBPROT ==1 && CR1_in[5] == 1'b0) &&
                                cfg_write)
                            begin
                                SR1[1] = 1'b0; // WEL
                            end
                            else
                            begin
                                WSTART = 1'b1;
                                WSTART <= #5 1'b0;
                            end
                        end
                        else
                        // can not execute if Hardware Protection Mode
                        // is entered or if WEL bit is zero
                            SR1[1] = 1'b0; // WEL
                    end
                    else if (Instruct == WRAR && WEL == 1)
                    begin
                        if (~(SRWD && ~WPNeg_in && ~QUAD &&
                           (Address == 16'h0000 || Address == 16'h0002)))
                        begin
                        // can not execute if WEL bit is zero or Hardware
                        // Protection Mode is entered and SR1NV,SR1V,CR1NV or
                        // CR1V is selected (no error is set)
                            Addr = Address;

                            if ((Address == 16'h0001)  ||
                                (Address == 16'h0004)  ||
                                (Address == 16'h0007)  ||
                                ((Address >  16'h0008)  &&
                                (Address <  16'h0040)) ||
                                ((Address >  16'h0041)  &&
                                (Address <  16'h0089)) ||
                                ((Address >  16'h008F)  &&
                                (Address <  16'h0095)) ||
                                (Address >  16'h0098))
                            begin
                                SR1[1] = 1'b0; // WEL
                            end
                            else // Protection Mode not selected
                            begin
                                WSTART = 1'b1;
                                WSTART <= #5 1'b0;
                            end
                        end
                        else
                            // can not execute if Hardware Protection Mode
                            // is entered or if WEL bit is zero
                            SR1[1] = 1'b0; // WEL
                    end
                    else if ((Instruct == WRITE_MEM || Instruct == DDRWRITE ||
                            Instruct == FAST_WRITE || Instruct == DDR_FAST_WRITE ||
                            Instruct == DIW || Instruct == DIOW ||
                            Instruct == QIW || Instruct == QIOW ||
                            Instruct == DDRQIOW) && WEL == 1)
                    begin
                        pgm_page = Address / (PageSize+1);
                        sect = Address / (SecSize+1);

                        if (Sec_Prot[sect] == 0)
                        begin
                            PSTART  = 1'b1;
                            PSTART <= #5 1'b0;
                            INITIAL_CONFIG = 1;
                            Addr    = Address;
                            Addr_tmp= Address;
                            wr_cnt  = Byte_number;
                            for (i=wr_cnt;i>=0;i=i-1)
                            begin
                                if (Viol != 0)
                                    WData[i] = -1;
                                else
                                    WData[i] = WByte[i];
                            end
                        end
                    end
                    else if (Instruct == SSWR && WEL == 1)
                    begin
                        if (Address + Byte_number <= OTPHiAddr)
                        begin //Program within valid OTP Range
                            if (Address <= 8'hFF)
                            begin
                                PSTART  = 1'b1;
                                PSTART <= #5 1'b0;
                                Addr    = Address;
                                Addr_tmp= Address;
                                wr_cnt  = Byte_number;
                                for (i=wr_cnt;i>=0;i=i-1)
                                begin
                                    if (Viol != 0)
                                        WData[i] = -1;
                                    else
                                        WData[i] = WByte[i];
                                end
                            end
                            else
                            begin
                                Addr_tmp = 0;
                            end
                        end
                    end
                    else if (Instruct == CRCC)
                    begin
                        if (CRC_End_Addr_reg >= CRC_Start_Addr_reg + 4)
                        begin
                            CRCSTART = 1'b1;
                            CRCSTART <= #5 1'b0;
                            SR1[0]   = 1'b1; // WIP
                            SR2_V[3] = 1'b0; // CRCA
                            CRC_reg  = 32'h00000000;
                        end
                        else
                        begin
                            // Abort CRC calculation
                            $display ("CRC EndAddr is not StartAddr+4 ");
                            $display ("or greater; CRC calculation is aborted");
                            SR2_V[3] = 1'b1; // CRCA
                        end
                    end
                    else if (Instruct == EPCS && ~START_T1_in)
                    begin
                        START_T1_in = 1'b1;
                    end
                    else if (Instruct == DPD)
                    begin
                        DPD_in = 1'b1;
                    end
                    else if (Instruct == HBN)
                    begin
                        HBN_in = 1'b1;
                    end

                    if (Instruct == RSTEN)
                    begin
                        RESET_EN = 1;
                    end
                    else
                    begin
                        RESET_EN <= 0;
                    end
                end
                else if (oe_z)
                begin
                    if (Instruct == READ)
                    begin
                        rd_slow = 1'b1;
                        dual    = 1'b0;
                        ddr     = 1'b0;
                    end
                    else if (Instruct == DDRQIOR)
                    begin
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b1;
                    end
                    else if (Instruct == DIOR || Instruct == QIOR || Instruct == QOR)
                    begin
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b0;
                    end
                    else
                    begin
                        if (QPI)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b0;
                        end
                        else
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b0;
                            ddr     = 1'b0;
                        end
                    end
                end
                else if (oe)
                begin
                    any_read = 1'b1;
                    if (Instruct == RDSR1)
                    begin
                    //Read Status Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR1[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDSR2)
                    begin
                        //Read Status Register 2
                        if (QPI)
                        begin
                            data_out[7:0] = SR2_V;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR2_V;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR2_V[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR1)
                    begin
                        //Read Configuration Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = CR1_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR1_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR1_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR2)
                    begin
                        //Read Configuration Register 2
                        if (QPI)
                        begin
                            data_out[7:0] = CR2_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR2_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR2_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR4)
                    begin
                        //Read Configuration Register 4
                        if (QPI)
                        begin
                            data_out[7:0] = CR4_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR4_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR4_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR5)
                    begin
                        //Read Configuration Register 5
                        if (QPI)
                        begin
                            data_out[7:0] = CR5_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR5_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR5_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDAR)
                    begin
                        READ_ALL_REG(read_addr, RDAR_reg);

                        if (QPI)
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = RDAR_reg;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_SO = data_out[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == READ)
                    begin
                        //Read Memory array
                        rd_slow = 1'b1;
                        dual    = 1'b0;
                        ddr     = 1'b0;

                        if (Mem[read_addr] !== -1)
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO  = data_out[7-read_cnt];
                        end
                        else
                        begin
                            DataDriveOut_SO  = 8'bx;
                        end

                        read_cnt = read_cnt + 1;
                        if (read_cnt == 8)
                        begin
                            read_cnt = 0;
                            if (read_addr >= AddrRANGE)
                                read_addr = 0;
                            else
                                read_addr = read_addr + 1;
                        end
                    end
                    else if (Instruct == FAST_READ)
                    begin
                        //Read Memory array
                        rd_slow = 1'b0;
                        dual    = 1'b0;
                        ddr     = 1'b0;

                        if (Mem[read_addr] !== -1)
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO  = data_out[7-read_cnt];
                        end
                        else
                        begin
                            DataDriveOut_SO  = 8'bx;
                        end

                        read_cnt = read_cnt + 1;
                        if (read_cnt == 8)
                        begin
                            read_cnt = 0;

                            if (read_addr == AddrRANGE)
                                read_addr = 0;
                            else
                                read_addr = read_addr + 1;
                        end
                    end
                    else if (Instruct == DDRFR)
                    begin
                        //Read Memory array
                        if (QPI)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b1;

                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_RESET  = data_out[7-4*read_cnt];
                            DataDriveOut_WP     = data_out[6-4*read_cnt];
                            DataDriveOut_SO     = data_out[5-4*read_cnt];
                            DataDriveOut_SI     = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else if (DPI)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b0;
                            ddr     = 1'b1;

                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO <= data_out[7-2*read_cnt];
                            DataDriveOut_SI <= data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b0;
                            ddr     = 1'b1;

                            if (Mem[read_addr] !== -1)
                            begin
                                data_out[7:0] = Mem[read_addr];
                                DataDriveOut_SO  = data_out[7-read_cnt];
                            end
                            else
                            begin
                                DataDriveOut_SO  = 8'bx;
                            end
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if ((Instruct == DIOR) || (Instruct == DOR))
                    begin
                        //Read Memory array
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b0;

                        if (Mem[read_addr] !== -1)
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO = data_out[7-2*read_cnt];
                            DataDriveOut_SI = data_out[6-2*read_cnt];
                        end
                        else
                        begin
                            DataDriveOut_SO = 8'bx;
                            DataDriveOut_SI = 8'bx;
                        end

                        data_out[7:0] = Mem[read_addr];
                        read_cnt = read_cnt + 1;
                        if (read_cnt == 4)
                        begin
                            read_cnt = 0;

                            if (read_addr == AddrRANGE)
                                read_addr = 0;
                            else
                                read_addr = read_addr + 1;
                        end
                    end
                    else if ((Instruct == QIOR || Instruct == DDRQIOR ||
                            Instruct == QOR) && QUAD)
                    begin
                        //Read Memory array
                        if (Instruct == DDRQIOR)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b1;
                        end
                        else
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b0;
                        end

                        data_out[7:0]  = Mem[read_addr];
                        DataDriveOut_RESET = data_out[7-4*read_cnt];
                        DataDriveOut_WP    = data_out[6-4*read_cnt];
                        DataDriveOut_SO    = data_out[5-4*read_cnt];
                        DataDriveOut_SI    = data_out[4-4*read_cnt];
                        read_cnt = read_cnt + 1;
                        if (read_cnt == 2)
                        begin
                            read_cnt = 0;

                            if (read_addr == AddrRANGE)
                                read_addr = 0;
                            else
                                read_addr = read_addr + 1;
                        end
                    end
                    else if (Instruct == SSRD)
                    begin
                        if(read_addr>=OTPLoAddr && read_addr<=OTPHiAddr)
                        begin
                        //Read OTP Memory array
                            rd_slow = 1'b0;
                            dual    = 1'b0;
                            ddr     = 1'b0;
                            data_out[7:0] = OTPMem[read_addr];
                            if (QPI)
                            begin
                                data_out[7:0]  = Mem[read_addr];
                                DataDriveOut_RESET = data_out[7-4*read_cnt];
                                DataDriveOut_WP    = data_out[6-4*read_cnt];
                                DataDriveOut_SO    = data_out[5-4*read_cnt];
                                DataDriveOut_SI    = data_out[4-4*read_cnt];
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 2)
                                begin
                                    read_cnt = 0;
                                    read_addr = read_addr + 1;
                                end
                            end
                            else if (DPI)
                            begin
                                DataDriveOut_SO    = data_out[7-2*read_cnt];
                                DataDriveOut_SI    = data_out[6-2*read_cnt];
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 4)
                                begin
                                    read_cnt = 0;
                                    read_addr = read_addr + 1;
                                end
                            end
                            else
                            begin
                                DataDriveOut_SO  = data_out[7-read_cnt];
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 8)
                                begin
                                    read_cnt = 0;
                                    read_addr = read_addr + 1;
                                end
                            end
                        end
                        else if (read_addr > OTPHiAddr)
                        begin
                        //OTP Read operation will not wrap to the
                        //starting address after the OTP address is at
                        //its maximum; instead, the data beyond the
                        //maximum OTP address will be undefined.
                            if (QPI)
                            begin
                                DataDriveOut_RESET = 1'bX;
                                DataDriveOut_WP    = 1'bX;
                                DataDriveOut_SO    = 1'bX;
                                DataDriveOut_SI    = 1'bX;
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 2)
                                    read_cnt = 0;
                            end
                            else if (DPI)
                            begin
                                DataDriveOut_SO    = 1'bX;
                                DataDriveOut_SI    = 1'bX;
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 4)
                                    read_cnt = 0;
                            end
                            else
                            begin
                                DataDriveOut_SO = 1'bX;
                                read_cnt = read_cnt + 1;
                                if (read_cnt == 8)
                                    read_cnt = 0;
                            end
                        end
                    end

                    else if (Instruct == RDID)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = ID_reg[(8*byte_cnt-1)-4*read_cnt];
                            DataDriveOut_WP    = ID_reg[(8*byte_cnt-2)-4*read_cnt];
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-3)-4*read_cnt];
                            DataDriveOut_SI    = ID_reg[(8*byte_cnt-4)-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-1)-2*read_cnt];
                            DataDriveOut_SI    = ID_reg[(8*byte_cnt-2)-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-1)-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                    end

                    else if (Instruct == RUID)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = UID_reg[(8*byte_cnt-1)-4*read_cnt];
                            DataDriveOut_WP    = UID_reg[(8*byte_cnt-2)-4*read_cnt];
                            DataDriveOut_SO    = UID_reg[(8*byte_cnt-3)-4*read_cnt];
                            DataDriveOut_SI    = UID_reg[(8*byte_cnt-4)-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = UID_reg[(8*byte_cnt-1)-2*read_cnt];
                            DataDriveOut_SI    = UID_reg[(8*byte_cnt-2)-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO    = UID_reg[(8*byte_cnt-1)-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                    end

                    else if (Instruct == RDSN)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = SERNUM_reg[(8*byte_cnt-1)-4*read_cnt];
                            DataDriveOut_WP    = SERNUM_reg[(8*byte_cnt-2)-4*read_cnt];
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-3)-4*read_cnt];
                            DataDriveOut_SI    = SERNUM_reg[(8*byte_cnt-4)-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-1)-2*read_cnt];
                            DataDriveOut_SI    = SERNUM_reg[(8*byte_cnt-2)-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-1)-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                    end

                    else if (Instruct == ECCRD)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = ECC_reg[7-4*read_cnt];
                            DataDriveOut_WP    = ECC_reg[6-4*read_cnt];
                            DataDriveOut_SO    = ECC_reg[5-4*read_cnt];
                            DataDriveOut_SI    = ECC_reg[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                                read_cnt = 0;
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = ECC_reg[7-2*read_cnt];
                            DataDriveOut_SI    = ECC_reg[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                                read_cnt = 0;
                        end
                        else
                        begin
                            DataDriveOut_SO = ECC_reg[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                end
            end

            WRITE_SR:
            begin
                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (WDONE == 1)
                begin
                    SR1[1] = 1'b0; //WEL
                    //SRWD bit
                    SR1[7] = SR1_in[7];

                    SR1[4]  = SR1_in[4];//BP2
                    SR1[3]  = SR1_in[3];//BP1
                    SR1[2]  = SR1_in[2];//BP0

                    BP_bits = {SR1[4],SR1[3],SR1[2]};

                    if (TBPROT == 1'b0 && INITIAL_CONFIG == 1'b0)
                    begin
                        SR1[5]  = SR1_in[5];//TBPROT
                    end

                    change_BP = 1'b1;
                    #1 change_BP = 1'b0;
                end
            end

            WRITE_ALL_REG:
            begin

                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (WDONE == 1)
                begin
                    SR1[1] = 1'b0; // WEL

                    if (Addr == 24'h000000) // SR1;
                    begin
                        if (~REGS_PROTECTED)
                        begin
                            //SRWD bit
                            SR1[7] = WRAR_reg_in[7];

                            SR1[5]  = WRAR_reg_in[5];//TBPROT
                            SR1[4]  = WRAR_reg_in[4];//BP2
                            SR1[3]  = WRAR_reg_in[3];//BP1
                            SR1[2]  = WRAR_reg_in[2];//BP0

                            BP_bits = {SR1[4],SR1[3],SR1[2]};

                            change_BP    = 1'b1;
                            #1 change_BP = 1'b0;
                        end
                    end
                    else if (Addr == 24'h000002) // CR1_NV;
                    begin
                        if (~REGS_PROTECTED)
                        begin
                            CR1_NV[7:4] = WRAR_reg_in[7:4];// RL_NV(3:0)
                            CR1_NV[1]   = WRAR_reg_in[1];  // QUAD
                        end
                    end
                    else if (Addr == 24'h000003) // CR2_NV
                    begin
                        if (~REGS_PROTECTED)
                        begin
                            CR2_NV[6] = WRAR_reg_in[6];// QPI
                            CR2_NV[5] = WRAR_reg_in[5];// IO3R
                            CR2_NV[4] = WRAR_reg_in[4];// DPI
                        end
                    end
                    else if (Addr == 24'h000005) // CR4_NV
                    begin
                        if (~REGS_PROTECTED)
                        begin
                            CR4_NV[7:5] = WRAR_reg_in[7:5];// OI_O(2:0)
                            CR4_NV[2]   = WRAR_reg_in[2];  // DPDPOR
                        end
                    end
                    else if (Addr == 24'h000006) // CR5_NV
                    begin
                        if (~REGS_PROTECTED)
                        begin
                            CR5_NV[7:6] = WRAR_reg_in[7:6];// REG_LATENCY_NV[2:0]
                        end
                    end
                end
            end

            PAGE_PG :
            begin
                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (~PDONE)
                begin
                    ADDRHILO_PG(AddrLo, AddrHi, Addr);
                    cnt = 0;

                    for (i=0;i<=wr_cnt;i=i+1)
                    begin
                        new_int = WData[i];
                        old_int = Mem[Addr + i - cnt];
                        if (new_int > -1)
                        begin
                            new_bit = new_int;
                            if (old_int > -1)
                            begin
                                old_bit = old_int;
                                for(j=0;j<=7;j=j+1)
                                begin
                                    if (~old_bit[j])
                                        new_bit[j] = 1'b0;
                                end
                                new_int = new_bit;
                            end
                            WData[i] = new_int;
                        end
                        else
                        begin
                            WData[i] = -1;
                        end

                        Mem[Addr + i - cnt] = - 1;
                        if ((Addr + i) == AddrHi)
                        begin
                            Addr = AddrLo;
                            cnt = i + 1;
                        end
                    end
                    cnt = 0;
                end

                if (PDONE)
                begin
                    SR1[1] = 1'b0; //WEL

                    for (i=0;i<=wr_cnt;i=i+1)
                    begin
                        Mem[Addr_tmp + i - cnt] = WData[i];
                        if ((Addr_tmp + i) == AddrHi)
                        begin
                            Addr_tmp = AddrLo;
                            cnt = i + 1;
                        end
                    end
                end
            end

            OTP_PG:
            begin
                rd_slow = 1'b0;
                dual    = 1'b0;
                ddr     = 1'b0;

//                 if (~PDONE)
                if (falling_edge_PDONE)
                begin
                    for (i=0;i<=wr_cnt;i=i+1)
                    begin
                        OTPMem[Addr + i] = WData[i];
                    end
                    SR1[0] = 1'b0; // WIP
                    SR1[1] = 1'b0; // WEL
                end
            end

            CRC_Calc:
            begin
                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (oe)
                begin
                    any_read = 1'b1;
                    if (Instruct == RDSR1)
                    begin
                    //Read Status Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR1[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                end

                CRC_ACT      = 1'b1;
                CRC_RD_SETUP =  1'b1;

                if (rising_edge_CRCDONE)
                begin
                    crc_out = 32'h00000000;
                    for (i=CRC_Start_Addr_reg;i<=CRC_End_Addr_reg;i=i+1)
                    begin
                        crc_in = Mem[i];
                        for (j=15;j>=0;j=j-1)
                        begin
                            crc_tmp = crc_out[31] ^ crc_in[j];

                            crc_out[31] = crc_out[30];
                            crc_out[30] = crc_out[29];
                            crc_out[29] = crc_out[28];
                            crc_out[28] = crc_out[27] ^ crc_tmp;
                            crc_out[27] = crc_out[26] ^ crc_tmp;
                            crc_out[26] = crc_out[25] ^ crc_tmp;
                            crc_out[25] = crc_out[24] ^ crc_tmp;
                            crc_out[24] = crc_out[23];
                            crc_out[23] = crc_out[22] ^ crc_tmp;
                            crc_out[22] = crc_out[21] ^ crc_tmp;
                            crc_out[21] = crc_out[20];
                            crc_out[20] = crc_out[19] ^ crc_tmp;
                            crc_out[19] = crc_out[18] ^ crc_tmp;
                            crc_out[18] = crc_out[17] ^ crc_tmp;
                            crc_out[17] = crc_out[16];
                            crc_out[16] = crc_out[15];
                            crc_out[15] = crc_out[14];
                            crc_out[14] = crc_out[13] ^ crc_tmp;
                            crc_out[13] = crc_out[12] ^ crc_tmp;
                            crc_out[12] = crc_out[11];
                            crc_out[11] = crc_out[10] ^ crc_tmp;
                            crc_out[10] = crc_out[9] ^ crc_tmp;
                            crc_out[9] = crc_out[8] ^ crc_tmp;
                            crc_out[8] = crc_out[7] ^ crc_tmp;
                            crc_out[7] = crc_out[6];
                            crc_out[6] = crc_out[5] ^ crc_tmp;
                            crc_out[5] = crc_out[4];
                            crc_out[4] = crc_out[3];
                            crc_out[3] = crc_out[2];
                            crc_out[2] = crc_out[1];
                            crc_out[1] = crc_out[0];
                            crc_out[0] = crc_tmp;
                        end
                    end
                    CRC_reg = crc_out;
                    SR1[0] = 1'b0; // WIP
                end
            end

            CRC_SUSP:
            begin
                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (sSTART_T1 && START_T1_in)
                begin
                    START_T1_in = 1'b0;
                    //The WIP bit in the Status Register will indicate that
                    //the device is ready for another operation.
                    SR1[0] = 1'b0;
                    //The CRC Suspend (CRCS) bit in the Status Register will
                    //be set to the logical “1” state to indicate that the
                    //CRC operation has been suspended.
                    SR2_V[4] = 1'b1;
                end

                if (oe)
                begin
                    any_read = 1'b1;
                    if (Instruct == RDSR1)
                    begin
                    //Read Status Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR1[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDSR2)
                    begin
                        //Read Status Register 2
                        if (QPI)
                        begin
                            data_out[7:0] = SR2_V;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR2_V;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR2_V[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR1)
                    begin
                        //Read Configuration Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = CR1_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR1_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR1_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR2)
                    begin
                        //Read Configuration Register 2
                        if (QPI)
                        begin
                            data_out[7:0] = CR2_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR2_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR2_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR4)
                    begin
                        //Read Configuration Register 4
                        if (QPI)
                        begin
                            data_out[7:0] = CR4_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR4_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR4_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDCR5)
                    begin
                        //Read Configuration Register 5
                        if (QPI)
                        begin
                            data_out[7:0] = CR5_NV;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = CR5_NV;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = CR5_NV[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDAR)
                    begin
                        READ_ALL_REG(read_addr, RDAR_reg);

                        if (QPI)
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = RDAR_reg;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_SO = data_out[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == READ)
                    begin
                        //Read Memory array
                        rd_slow = 1'b1;
                        dual    = 1'b0;
                        ddr     = 1'b0;

                        if (pgm_page != read_addr / (PageSize+1))
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO  = data_out[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                if (read_addr >= AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO  = 8'bxxxxxxxx;
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if (Instruct == FAST_READ)
                    begin
                        if (pgm_page != read_addr / (PageSize+1))
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO  = data_out[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO  = 8'bxxxxxxxx;
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if (Instruct == DDRFR)
                    begin
                        //Read Memory array
                        rd_slow = 1'b0;
                        dual    = 1'b0;
                        ddr     = 1'b1;

                        if (pgm_page != read_addr / (PageSize+1))
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO = data_out[7-2*read_cnt];
                            DataDriveOut_SI = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = 1'bx;
                            DataDriveOut_SI = 1'bx;

                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if (Instruct == DOR || Instruct == DIOR)
                    begin
                        //Read Memory array
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b0;

                        if (pgm_page != read_addr / (PageSize+1))
                        begin
                            data_out[7:0] = Mem[read_addr];
                            DataDriveOut_SO = data_out[7-2*read_cnt];
                            DataDriveOut_SI = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO  = 1'bx;
                            DataDriveOut_SI  = 1'bx;

                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if ((Instruct == QOR || Instruct == QIOR ||
                              Instruct == DDRQIOR) && QUAD)
                    begin
                        //Read Memory array
                        if (Instruct == DDRQIOR)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b1;
                        end
                        else
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b0;
                        end

                        if (pgm_page != read_addr / (PageSize+1))
                        begin
                            data_out[7:0]  = Mem[read_addr];
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_RESET = 1'bx;
                            DataDriveOut_WP    = 1'bx;
                            DataDriveOut_SO    = 1'bx;
                            DataDriveOut_SI    = 1'bx;

                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;

                                if (read_addr == AddrRANGE)
                                    read_addr = 0;
                                else
                                    read_addr = read_addr + 1;
                            end
                        end
                    end
                    else if (Instruct == ECCRD)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = ECC_reg[7-4*read_cnt];
                            DataDriveOut_WP    = ECC_reg[6-4*read_cnt];
                            DataDriveOut_SO    = ECC_reg[5-4*read_cnt];
                            DataDriveOut_SI    = ECC_reg[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                                read_cnt = 0;
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = ECC_reg[7-2*read_cnt];
                            DataDriveOut_SI    = ECC_reg[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                                read_cnt = 0;
                        end
                        else
                        begin
                            DataDriveOut_SO = ECC_reg[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDID)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = ID_reg[(8*byte_cnt-1)-4*read_cnt];
                            DataDriveOut_WP    = ID_reg[(8*byte_cnt-2)-4*read_cnt];
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-3)-4*read_cnt];
                            DataDriveOut_SI    = ID_reg[(8*byte_cnt-4)-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-1)-2*read_cnt];
                            DataDriveOut_SI    = ID_reg[(8*byte_cnt-2)-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO    = ID_reg[(8*byte_cnt-1)-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                    end
                    else if (Instruct == RDSN)
                    begin
                        if (QPI)
                        begin
                            DataDriveOut_RESET = SERNUM_reg[(8*byte_cnt-1)-4*read_cnt];
                            DataDriveOut_WP    = SERNUM_reg[(8*byte_cnt-2)-4*read_cnt];
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-3)-4*read_cnt];
                            DataDriveOut_SI    = SERNUM_reg[(8*byte_cnt-4)-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else if (DPI)
                        begin
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-1)-2*read_cnt];
                            DataDriveOut_SI    = SERNUM_reg[(8*byte_cnt-2)-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO    = SERNUM_reg[(8*byte_cnt-1)-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                            begin
                                read_cnt = 0;
                                byte_cnt = byte_cnt + 1;
                                if (byte_cnt == 9)
                                   byte_cnt = 1;
                            end
                        end
                    end
                end
                else if (oe_z)
                begin
                    if (Instruct == READ)
                    begin
                        rd_slow = 1'b1;
                        dual    = 1'b0;
                        ddr     = 1'b0;
                    end
                    else if (Instruct == DIOR || Instruct == QIOR)
                    begin
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b0;
                    end
                    else if (Instruct == DDRQIOR)
                    begin
                        rd_slow = 1'b0;
                        dual    = 1'b1;
                        ddr     = 1'b1;
                    end
                    else
                    begin
                        if (QPI)
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b1;
                            ddr     = 1'b0;
                        end
                        else
                        begin
                            rd_slow = 1'b0;
                            dual    = 1'b0;
                            ddr     = 1'b0;
                        end
                    end
                end

                if (falling_edge_write)
                begin
                    if (Instruct == EPCR)
                    begin
                        SR2_V[4] = 1'b0; // CRCS
                        SR1[0]   = 1'b1; // WIP
                        CRCRES   = 1'b1;
                        CRCRES   <= #5 1'b0;
                        RES_TO_SUSP_TIME = 1'b1;
                        RES_TO_SUSP_TIME <= #tdevice_CRCRL 1'b0;// 50us
                    end

                    if (Instruct == RSTEN)
                    begin
                        RESET_EN = 1;
                    end
                    else
                    begin
                        RESET_EN <= 0;
                    end
                end
            end

            DP_DOWN:
            begin
                dpd_act = 1'b1;

                if (oe)
                begin
                    any_read = 1'b0;
                end

                if (falling_edge_write)
                begin
                    if (Instruct == READ)
                    begin
                        RES_in = 1'b1;
                        #5 RES_in = 1'b0;
                    end
                    else
                    begin
                        $display("Device is in Deep Power Down Mode");
                        $display("No instructions allowed");
                    end
                end
            end

            HIBERNATE:
            begin
                if (oe)
                begin
                    any_read = 1'b0;
                end

                if (falling_edge_write)
                begin
                    if (Instruct == READ)
                    begin
                        REC_in = 1'b1;
                        #5 REC_in = 1'b0;
                    end
                    else
                    begin
                        $display("Device is in HIBERNATE Mode");
                        $display("No instructions allowed");
                    end
                end
            end

            RESET_STATE:
            begin
            // During Reset,the non-volatile version of the registers is
            // copied to volatile version to provide the default state of
            // the volatile register
                SR1[1:0] = 2'b00;
                CRC_reg = 32'h00000000;

                //Loads the Program Buffer with all ones
                for(i=0;i<=511;i=i+1)
                begin
                    WData[i] = MaxData;
                end

                //When BPNV is set to '1'. the BP2-0 bits in Status
                //Register are volatile and will be reseted after
                //reset command
                BP_bits = {SR1[4],SR1[3],SR1[2]};
                change_BP = 1'b1;
                #1 change_BP = 1'b0;

                RESET_EN = 0;
            end

            PGERS_ERROR :
            begin
                if (QPI)
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b1;
                    ddr     = 1'b0;
                end
                else
                begin
                    rd_slow = 1'b0;
                    dual    = 1'b0;
                    ddr     = 1'b0;
                end

                if (oe)
                begin
                    any_read = 1'b1;
                    if (Instruct == RDSR1)
                    begin
                    //Read Status Register 1
                        if (QPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = SR1;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            DataDriveOut_SO = SR1[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                    else if (Instruct == RDAR)
                    begin
                        READ_ALL_REG(read_addr, RDAR_reg);

                        if (QPI)
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_RESET = data_out[7-4*read_cnt];
                            DataDriveOut_WP    = data_out[6-4*read_cnt];
                            DataDriveOut_SO    = data_out[5-4*read_cnt];
                            DataDriveOut_SI    = data_out[4-4*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 2)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else if (DPI)
                        begin
                            data_out[7:0] = RDAR_reg;
                            DataDriveOut_SO    = data_out[7-2*read_cnt];
                            DataDriveOut_SI    = data_out[6-2*read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 4)
                            begin
                                read_cnt = 0;
                            end
                        end
                        else
                        begin
                            data_out[7:0]  = RDAR_reg;
                            DataDriveOut_SO = data_out[7-read_cnt];
                            read_cnt = read_cnt + 1;
                            if (read_cnt == 8)
                                read_cnt = 0;
                        end
                    end
                end

                if (falling_edge_write)
                begin
                    if (Instruct == WRDI)
                    begin
                    // A Clear Status Register (CLSR) followed by a Write
                    // Disable (WRDI) command must be sent to return the
                    // device to standby state
                        SR1[1] = 1'b0; //WEL
                    end

                    if (Instruct == RSTEN)
                    begin
                        RESET_EN = 1;
                    end
                    else
                    begin
                        RESET_EN <= 0;
                    end
                end
            end

        endcase
        if (falling_edge_write)
        begin
            if (Instruct == RSTEN &&
                (current_state != DP_DOWN || current_state != HIBERNATE))
                RESET_EN <= 1;
            else
                RESET_EN <= 0;
        end
    end
    always @(posedge CSNeg_ipd)
    begin
        read_cnt = 0;
        //Output Disable Control
        SOut_zd                = 1'bZ;
        SIOut_zd               = 1'bZ;
        RESETNegOut_zd         = 1'bZ;
        WPNegOut_zd            = 1'bZ;
        DataDriveOut_SO        = 1'bZ;
        DataDriveOut_SI        = 1'bZ;
        DataDriveOut_RESET     = 1'bZ;
        DataDriveOut_WP        = 1'bZ;
    end

    always @(posedge change_BP)
    begin
        case (SR1[4:2])

            3'b000:
            begin
                Sec_Prot[SecNumUni:0] = {256{1'b0}};
            end

            3'b001:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni:(SecNumUni+1)*63/64] =   {4{1'b1}};
                    Sec_Prot[(SecNumUni+1)*63/64-1 : 0]     = {252{1'b0}};
                end
                else
                begin
                    Sec_Prot[(SecNumUni+1)/64-1 : 0]       =   {4{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/64] = {252{1'b0}};
                end
            end

            3'b010:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni : (SecNumUni+1)*31/32] = {8{1'b1}};
                    Sec_Prot[(SecNumUni+1)*31/32-1 : 0]       = {248{1'b0}};
                end
                else            // BP starts at Bottom
                begin
                    Sec_Prot[(SecNumUni+1)/32-1 : 0]       = {8{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/32] = {248{1'b0}};
                end
            end

            3'b011:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni : (SecNumUni+1)*15/16] = {16{1'b1}};
                    Sec_Prot[(SecNumUni+1)*15/16-1 : 0]       = {240{1'b0}};
                end
                else            // BP starts at Bottom
                begin
                    Sec_Prot[(SecNumUni+1)/16-1 : 0]       = {16{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/16] = {240{1'b0}};
                end
            end

            3'b100:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni : (SecNumUni+1)*7/8] = {32{1'b1}};
                    Sec_Prot[(SecNumUni+1)*7/8-1 : 0]       = {224{1'b0}};
                end
                else            // BP starts at Bottom
                begin
                    Sec_Prot[(SecNumUni+1)/8-1 : 0]       = {32{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/8] = {224{1'b0}};
                end
            end

            3'b101:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni : (SecNumUni+1)*3/4] = {64{1'b1}};
                    Sec_Prot[(SecNumUni+1)*3/4-1 : 0]       = {192{1'b0}};
                end
                else            // BP starts at Bottom
                begin
                    Sec_Prot[(SecNumUni+1)/4-1 : 0]       = {64{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/4] = {192{1'b0}};
                end
            end

            3'b110:
            begin
                if (~TBPROT)  // BP starts at Top
                begin
                    Sec_Prot[SecNumUni : (SecNumUni+1)/2] = {128{1'b1}};
                    Sec_Prot[(SecNumUni+1)/2-1 : 0]       = {128{1'b0}};
                end
                else            // BP starts at Bottom
                begin
                    Sec_Prot[(SecNumUni+1)/2-1 : 0]       = {128{1'b1}};
                    Sec_Prot[SecNumUni : (SecNumUni+1)/2] = {128{1'b0}};
                end
            end

            3'b111:
            begin
                Sec_Prot[SecNumUni:0] =  {256{1'b1}};
            end
        endcase
    end

    ///////////////////////////////////////////////////////////////////////////
    // functions & tasks
    ///////////////////////////////////////////////////////////////////////////
    // Procedure ADDRHILO_PG
    task ADDRHILO_PG;
    inout  AddrLOW;
    inout  AddrHIGH;
    input   Addr;
    integer AddrLOW;
    integer AddrHIGH;
    integer Addr;
    integer page;
    begin
        page = Addr / (PageSize + 1);
        AddrLOW = page * (PageSize + 1);
        AddrHIGH = page * (PageSize + 1) + PageSize;
    end
    endtask

    task READ_ALL_REG;
        input integer Addr;
        inout integer RDAR_reg;
    begin

        if (Addr == 24'h000000)
            RDAR_reg = SR1;
        else if (Addr == 24'h000001)
            RDAR_reg = SR2_V;
        else if (Addr == 24'h000002)
            RDAR_reg = CR1_NV;
        else if (Addr == 24'h000003)
            RDAR_reg = CR2_NV;
        else if (Addr == 24'h000005)
            RDAR_reg = CR4_NV;
        else if (Addr == 24'h000006)
            RDAR_reg = CR5_NV;
        else if (Addr == 24'h000089)
            RDAR_reg = ECC_reg;
        else if (Addr == 24'h00008A)
            RDAR_reg = EDC_reg[7:0];
        else if (Addr == 24'h00008B)
            RDAR_reg = EDC_reg[15:8];
        else if (Addr == 24'h00008E)
            RDAR_reg = ADDTRAP_reg[7:0];
        else if (Addr == 24'h00008F)
            RDAR_reg = ADDTRAP_reg[15:8];
        else if (Addr == 24'h000040)
            RDAR_reg = ADDTRAP_reg[23:16];
        else if (Addr == 24'h000041)
            RDAR_reg = ADDTRAP_reg[31:24];
        else if (Addr == 24'h000095)
            RDAR_reg = CRC_reg[7:0];
        else if (Addr == 24'h000096)
            RDAR_reg = CRC_reg[15:8];
        else if (Addr == 24'h000097)
            RDAR_reg = CRC_reg[23:16];
        else if (Addr == 24'h000098)
            RDAR_reg = CRC_reg[31:24];
        else
            RDAR_reg = 8'bXX;//N/A

    end
    endtask

    ///////////////////////////////////////////////////////////////////////////
    // edge controll processes
    ///////////////////////////////////////////////////////////////////////////

    always @(posedge PoweredUp)
    begin
        rising_edge_PoweredUp = 1;
        #1 rising_edge_PoweredUp = 0;
    end

    always @(posedge SCK_ipd)
    begin
       rising_edge_SCK_ipd = 1'b1;
       #1 rising_edge_SCK_ipd = 1'b0;
    end

    always @(negedge SCK_ipd)
    begin
       falling_edge_SCK_ipd = 1'b1;
       #1 falling_edge_SCK_ipd = 1'b0;
    end

    always @(posedge CSNeg_ipd)
    begin
        rising_edge_CSNeg_ipd = 1'b1;
        #1 rising_edge_CSNeg_ipd = 1'b0;
    end

    always @(negedge CSNeg_ipd)
    begin
        falling_edge_CSNeg_ipd = 1'b1;
        #1 falling_edge_CSNeg_ipd = 1'b0;
    end

    always @(negedge write)
    begin
        falling_edge_write = 1;
        #1 falling_edge_write = 0;
    end

    always @(posedge reseted)
    begin
        rising_edge_reseted = 1;
        #1 rising_edge_reseted = 0;
    end

    always @(negedge RESETNeg)
    begin
        falling_edge_RESETNeg = 1;
        #1 falling_edge_RESETNeg = 0;
    end

    always @(posedge RESETNeg)
    begin
        rising_edge_RESETNeg = 1;
        #1 rising_edge_RESETNeg = 0;
    end

    always @(posedge PSTART)
    begin
        rising_edge_PSTART = 1'b1;
        #1 rising_edge_PSTART = 1'b0;
    end

    always @(posedge PDONE)
    begin
        rising_edge_PDONE = 1'b1;
        #1 rising_edge_PDONE = 1'b0;
    end

    always @(negedge PDONE)
    begin
        falling_edge_PDONE = 1'b1;
        #1 falling_edge_PDONE = 1'b0;
    end

    always @(posedge WSTART)
    begin
        rising_edge_WSTART = 1;
        #1 rising_edge_WSTART = 0;
    end

    always @(posedge WDONE)
    begin
        rising_edge_WDONE = 1'b1;
        #1 rising_edge_WDONE = 1'b0;
    end

    always @(posedge CSDONE)
    begin
        rising_edge_CSDONE = 1'b1;
        #1 rising_edge_CSDONE = 1'b0;
    end

    always @(posedge START_T1_in)
    begin
        rising_edge_START_T1_in = 1'b1;
        #1 rising_edge_START_T1_in = 1'b0;
    end

    always @(posedge CRCSTART)
    begin
        rising_edge_CRCSTART = 1'b1;
        #1 rising_edge_CRCSTART = 1'b0;
    end

    always @(posedge CRCDONE)
    begin
        rising_edge_CRCDONE = 1'b1;
        #1 rising_edge_CRCDONE = 1'b0;
    end

    always @(change_addr)
    begin
        change_addr_event = 1'b1;
        #1 change_addr_event = 1'b0;
    end

    always @(current_state)
    begin
        current_state_event = 1'b1;
        #1 current_state_event = 1'b0;
    end

    always @(Instruct)
    begin
        Instruct_event = 1'b1;
        #1 Instruct_event = 1'b0;
    end

    always @(posedge DPD_out)
    begin
        rising_edge_DPD_out = 1'b1;
        #1 rising_edge_DPD_out = 1'b0;
    end

    always @(posedge HBN_out)
    begin
        rising_edge_HBN_out = 1'b1;
        #1 rising_edge_HBN_out = 1'b0;
    end

    always @(posedge RES_out)
    begin
        rising_edge_RES_out = 1'b1;
        #1 rising_edge_RES_out = 1'b0;
    end

    always @(posedge RST_out)
    begin
        rising_edge_RST_out = 1'b1;
        #1 rising_edge_RST_out = 1'b0;
    end

    always @(negedge RST)
    begin
        falling_edge_RST = 1'b1;
        #1 falling_edge_RST = 1'b0;
    end

    always @(posedge SWRST_out)
    begin
        rising_edge_SWRST_out = 1'b1;
        #1 rising_edge_SWRST_out = 1'b0;
    end

    integer DQt_01;
    integer DQt_0Z;

    reg  BuffInDQ;
    wire BuffOutDQ;

    reg  BuffInDQZ;
    wire BuffOutDQZ;

    BUFFER    BUF_DOut   (BuffOutDQ, BuffInDQ);
    BUFFER    BUF_DOutZ  (BuffOutDQZ, BuffInDQZ);

    initial
    begin
        BuffInDQ   = 1'b1;
        BuffInDQZ  = 1'b1;
    end

    always @(posedge BuffOutDQ)
    begin
        DQt_01 = $time;
    end

    always @(posedge BuffOutDQZ)
    begin
        DQt_0Z = $time;
    end

    always @(DataDriveOut_SO,DataDriveOut_SI,DataDriveOut_RESET,DataDriveOut_WP)
    begin
        if ((DQt_01 > SCK_cycle/2) && DOUBLE)
        begin
            glitch = 1;
            SOut_zd        <= #(DQt_01-1000) DataDriveOut_SO;
            SIOut_zd       <= #(DQt_01-1000) DataDriveOut_SI;
            RESETNegOut_zd <= #(DQt_01-1000) DataDriveOut_RESET;
            WPNegOut_zd    <= #(DQt_01-1000) DataDriveOut_WP;
        end
        else
        begin
            glitch = 0;
            SOut_zd        <= DataDriveOut_SO;
            SIOut_zd       <= DataDriveOut_SI;
            RESETNegOut_zd <= DataDriveOut_RESET;
            WPNegOut_zd    <= DataDriveOut_WP;
        end
    end

endmodule

module BUFFER (OUT,IN);
    input IN;
    output OUT;
    buf   ( OUT, IN);
endmodule
