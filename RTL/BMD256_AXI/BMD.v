//--------------------------------------------------------------------------------
//--
//-- This file is owned and controlled by Xilinx and must be used solely
//-- for design, simulation, implementation and creation of design files
//-- limited to Xilinx devices or technologies. Use with non-Xilinx
//-- devices or technologies is expressly prohibited and immediately
//-- terminates your license.
//--
//-- Xilinx products are not intended for use in life support
//-- appliances, devices, or systems. Use in such applications is
//-- expressly prohibited.
//--
//--            **************************************
//--            ** Copyright (C) 2005, Xilinx, Inc. **
//--            ** All Rights Reserved.             **
//--            **************************************
//--
//--------------------------------------------------------------------------------
//-- Filename: BMD.v
//--
//-- Description: Bus Master Device (BMD) Module
//--              
//--              The module designed to operate with 32 bit and 64 bit interfaces.
//--
//--------------------------------------------------------------------------------
`timescale 1ns/1ns

module BMD #
  (
   parameter INTERFACE_WIDTH = 64,
   parameter INTERFACE_TYPE = 4'b0010,
   parameter FPGA_FAMILY = 8'h14
   )
      (

       trn_clk,
       trn_reset_n,
       trn_lnk_up_n,


       s_axis_rq_tlast,
       s_axis_rq_tdata,
       s_axis_rq_tuser,
       s_axis_rq_tkeep,
       s_axis_rq_tready,
       s_axis_rq_tvalid,

       s_axis_cc_tdata,
       s_axis_cc_tuser,
       s_axis_cc_tlast,
       s_axis_cc_tkeep,
       s_axis_cc_tvalid,
       s_axis_cc_tready,

       // Completer Request Interface
       m_axis_cq_tdata,
       m_axis_cq_tlast,
       m_axis_cq_tvalid,
       m_axis_cq_tuser,
       m_axis_cq_tkeep,
       pcie_cq_np_req_count,
       m_axis_cq_tready,
       pcie_cq_np_req,

       // Requester Completion Interface

       m_axis_rc_tdata,
       m_axis_rc_tlast,
       m_axis_rc_tvalid,
       m_axis_rc_tkeep,
       m_axis_rc_tuser,
       m_axis_rc_tready,
      
       trn_rcpl_streaming_n, //only for Block Plus
      
       cfg_to_turnoff_n,
       cfg_turnoff_ok_n,
      
       cfg_interrupt_n,
       cfg_interrupt_rdy_n,
       cfg_interrupt_assert_n,
       cfg_interrupt_di,
       cfg_interrupt_do,
       cfg_interrupt_mmenable,
       cfg_interrupt_msienable,

       cfg_neg_max_lnk_width,
       cfg_prg_max_payload_size,
       cfg_max_rd_req_size,
       cfg_rd_comp_bound,

       cfg_phant_func_en,
       cfg_phant_func_supported,


       cfg_dwaddr,
       cfg_rd_en_n,
       cfg_do,
       cfg_rd_wr_done_n,

       cpld_data_size_hwm,     // HWMark for Completion Data (DWs)
       cur_rd_count_hwm,       // HWMark for Read Count Allowed
       cpld_size,
       cur_mrd_count,

       cfg_completer_id,
       cfg_ext_tag_en,
       cfg_bus_mstr_enable,


       //memory interface
      
       fifo_read_data, //I
       fifo_prog_empty, //
       fifo_read_en, //O
       fifo_prog_full, //
       fifo_write_en, //O
       fifo_write_data, //
       fifo_read_valid, //I

       fifo_sender_data_prepare_ok, //I
       packet_data_num,

       calc_start,
       calc_finish,
       io_command,
       io_command_en,
       random_seed,
       fpga_state,
       receive_mode

       ); // synthesis syn_hier = "hard"

      ///////////////////////////////////////////////////////////////////////////////
      // Port Declarations
      ///////////////////////////////////////////////////////////////////////////////

      input         trn_clk;         
      input         trn_reset_n;
      input         trn_lnk_up_n;

      output 		s_axis_rq_tlast;
      output [255:0] 	s_axis_rq_tdata;
      output [59:0] 	s_axis_rq_tuser;
      output [7:0] 	s_axis_rq_tkeep;
      input [3:0] 		s_axis_rq_tready;
      output 		s_axis_rq_tvalid;
      
      output [255:0] 	s_axis_cc_tdata;
      output [32:0] 	s_axis_cc_tuser;
      output 		s_axis_cc_tlast;
      output [7:0] 	s_axis_cc_tkeep;
      output 		s_axis_cc_tvalid;
      input [3:0] 		s_axis_cc_tready;


      // Completer Request Interface
      input [255:0]      m_axis_cq_tdata;
      input 	       m_axis_cq_tlast;
      input 	       m_axis_cq_tvalid;
      input [84:0]        m_axis_cq_tuser;
      input [7:0] 	       m_axis_cq_tkeep;
      input [5:0] 	       pcie_cq_np_req_count;
      output 	       m_axis_cq_tready;
      output 	       pcie_cq_np_req;

      // Requester Completion Interface

      input [255:0] m_axis_rc_tdata;
      input 	    m_axis_rc_tlast;
      input 		    m_axis_rc_tvalid;
      input [7:0] 	    m_axis_rc_tkeep;
      input [74:0] 	    m_axis_rc_tuser;
      output 		    m_axis_rc_tready;

      output        trn_rcpl_streaming_n;
      input         cfg_to_turnoff_n;
      output        cfg_turnoff_ok_n;

      output        cfg_interrupt_n;
      input         cfg_interrupt_rdy_n;
      output        cfg_interrupt_assert_n;
      output  [7:0] cfg_interrupt_di;
      input   [7:0] cfg_interrupt_do;
      input   [2:0] cfg_interrupt_mmenable;
      input         cfg_interrupt_msienable;

      input [15:0]  cfg_completer_id;
      input         cfg_ext_tag_en;
      input         cfg_bus_mstr_enable;
      input [5:0]   cfg_neg_max_lnk_width;
      input [2:0]   cfg_prg_max_payload_size;
      input [2:0]   cfg_max_rd_req_size;
      input         cfg_rd_comp_bound;

      input         cfg_phant_func_en;
      input [1:0]   cfg_phant_func_supported;

      
      output [9:0]  cfg_dwaddr;
      output        cfg_rd_en_n;
      input  [31:0] cfg_do;
      input         cfg_rd_wr_done_n;

      output [31:0] cpld_data_size_hwm;     // HWMark for Completion Data (DWs)
      output [15:0] cur_rd_count_hwm;       // HWMark for Read Count Allowed
      output [31:0] cpld_size;
      output [15:0] cur_mrd_count;


      //memory interface
      input [255:0] 	fifo_read_data;
      input 		fifo_prog_empty;
      output 		fifo_read_en;
      input 		fifo_prog_full;
      output 		fifo_write_en;
//      output [191:0] 	fifo_write_data;
      output [255:0] 	fifo_write_data;
      input             fifo_read_valid;

      input fifo_sender_data_prepare_ok;
      //mwr_lenの代用
      input [31:0] packet_data_num; //from test_sender

      input 		calc_start;
      input 		calc_finish;
      
      output [3:0] 	io_command;
      output 		io_command_en;
      output [31:0] 	random_seed;
      input [2:0] 		fpga_state;
      output 		receive_mode;

      
      //signal to tx_data_cont module
      wire 		mem_completion_req, io_completion_req;
      wire 		completion_end;
      wire [2:0] 		rd_tc;
      wire 		rd_tlp_digest, rd_ep;
      wire [1:0] 		rd_attribute;
      wire [9:0] 		rd_length;
      wire [15:0] 		rd_request_id;
      wire [7:0] 		rd_tag;
      wire [2:0] 		rd_byte_count;
      wire [6:0] 		rd_lower_addr;

      //fpga state interface
      wire [3:0] 		io_command;
      wire 		io_command_en;

      wire [2:0] 		fpga_state;


      // Local wires

      wire          req_compl;
      wire          compl_done;
      wire          bmd_reset_n = trn_reset_n & ~trn_lnk_up_n;
      wire [5:0]    cfg_cap_max_lnk_width;
      wire [2:0]    cfg_cap_max_payload_size;

      BMD_EP# 
	(
         .INTERFACE_WIDTH(INTERFACE_WIDTH),
         .INTERFACE_TYPE(INTERFACE_TYPE),
         .FPGA_FAMILY(FPGA_FAMILY)
	    
         )
      BMD_EP (

              .clk  ( trn_clk ),                           // I
              .rst_n ( bmd_reset_n ),                      // I

              // LocalLink Tx
	      .s_axis_rq_tlast(s_axis_rq_tlast),
	      .s_axis_rq_tdata(s_axis_rq_tdata),
	      .s_axis_rq_tuser(s_axis_rq_tuser),
	      .s_axis_rq_tkeep(s_axis_rq_tkeep),
	      .s_axis_rq_tready(s_axis_rq_tready),
	      .s_axis_rq_tvalid(s_axis_rq_tvalid),

	      .s_axis_cc_tdata(s_axis_cc_tdata),
	      .s_axis_cc_tuser(s_axis_cc_tuser),
	      .s_axis_cc_tlast(s_axis_cc_tlast),
	      .s_axis_cc_tkeep(s_axis_cc_tkeep),
	      .s_axis_cc_tvalid(s_axis_cc_tvalid),
	      .s_axis_cc_tready(s_axis_cc_tready),
	    
              // LocalLink Rx
	      // Completer Request Interface
	      .m_axis_cq_tdata(m_axis_cq_tdata),
	      .m_axis_cq_tlast(m_axis_cq_tlast),
              .m_axis_cq_tvalid(m_axis_cq_tvalid),
              .m_axis_cq_tuser(m_axis_cq_tuser),
	      .m_axis_cq_tkeep(m_axis_cq_tkeep),
              .pcie_cq_np_req_count(pcie_cq_np_req_count),
              .m_axis_cq_tready(m_axis_cq_tready),
              .pcie_cq_np_req(pcie_cq_np_req),

	      // Requester Completion Interface

	      .m_axis_rc_tdata(m_axis_rc_tdata),
              .m_axis_rc_tlast(m_axis_rc_tlast),
              .m_axis_rc_tvalid(m_axis_rc_tvalid),
	      .m_axis_rc_tkeep(m_axis_rc_tkeep),
              .m_axis_rc_tuser(m_axis_rc_tuser),
              .m_axis_rc_tready(m_axis_rc_tready),


              .req_compl_o(req_compl),                     // O
              .compl_done_o(compl_done),                   // O

              .cfg_interrupt_n(cfg_interrupt_n),           // O
              .cfg_interrupt_rdy_n(cfg_interrupt_rdy_n),   // I
              .cfg_interrupt_assert_n(cfg_interrupt_assert_n), // O

              .cfg_interrupt_di ( cfg_interrupt_di ),      // O       
              .cfg_interrupt_do ( cfg_interrupt_do ),      // I       
              .cfg_interrupt_mmenable ( cfg_interrupt_mmenable ),     // I
              .cfg_interrupt_msienable ( cfg_interrupt_msienable ),   // I
              .cfg_completer_id ( cfg_completer_id ),      // I [15:0]

              .cfg_ext_tag_en ( cfg_ext_tag_en ),              // I
              .cfg_cap_max_lnk_width( cfg_cap_max_lnk_width ), // I [5:0]
              .cfg_neg_max_lnk_width( cfg_neg_max_lnk_width ), // I [5:0]

              .cfg_cap_max_payload_size( cfg_cap_max_payload_size ), // I [2:0]
              .cfg_prg_max_payload_size( cfg_prg_max_payload_size ), // I [2:0]
              .cfg_max_rd_req_size( cfg_max_rd_req_size ),           // I [2:0]
              .cfg_msi_enable(cfg_interrupt_msienable),              // I
              .cfg_rd_comp_bound( cfg_rd_comp_bound ),               // I
              .cfg_phant_func_en(cfg_phant_func_en),                 // I
              .cfg_phant_func_supported(cfg_phant_func_supported),   // I [1:0] 

              .cpld_data_size_hwm(cpld_data_size_hwm),
              .cur_rd_count_hwm(cur_rd_count_hwm),       
              .cpld_size(cpld_size), //Completion data size //O //come from 256_RX
              .cur_mrd_count(cur_mrd_count),

              .cfg_bus_mstr_enable ( cfg_bus_mstr_enable ),     // I


	      //bram interface
	      //appにてpcie_fifoが宣言されている。これにfifo_read,writeの信号が書かれている
	      .fifo_prog_full(fifo_prog_full),
	      .fifo_write_en(fifo_write_en),
	      .fifo_write_data(fifo_write_data), //write は Requester Completion(RX) からの 192bit width dataを書いている. 
	      .fifo_read_data(fifo_read_data), //TXがこのfifo dataをrdしている.
	      .fifo_prog_empty(fifo_prog_empty),
	      .fifo_read_en( fifo_read_en ), 
	      .fifo_read_valid( fifo_read_valid ), //I

	      .fifo_sender_data_prepare_ok( fifo_sender_data_prepare_ok ), //I
	      .packet_data_num( packet_data_num ),

	      .calc_start(calc_start),
	      .calc_finish(calc_finish),
	      .io_command(io_command),
	      .io_command_en(io_command_en),
	      .random_seed(random_seed),
	      .fpga_state(fpga_state),
	      .receive_mode(receive_mode)
              );

      BMD_TO_CTRL BMD_TO  (

			   .clk( trn_clk ),                             // I
			   .rst_n( bmd_reset_n ),                       // I

			   .req_compl_i( req_compl ),                   // I
			   .compl_done_i( compl_done ),                 // I

			   .cfg_to_turnoff_n( cfg_to_turnoff_n ),       // I
			   .cfg_turnoff_ok_n( cfg_turnoff_ok_n )        // O
	    
			   );

      BMD_CFG_CTRL BMD_CF  (

			    .clk( trn_clk ),                             // I
			    .rst_n( bmd_reset_n ),                       // I

			    .cfg_bus_mstr_enable( cfg_bus_mstr_enable ), // I

			    .cfg_dwaddr( cfg_dwaddr ),                    // O [9:0]
			    .cfg_rd_en_n( cfg_rd_en_n ),                  // O
			    .cfg_do( cfg_do ),                            // I [31:0]
			    .cfg_rd_wr_done_n( cfg_rd_wr_done_n ),        // I

			    .cfg_cap_max_lnk_width( cfg_cap_max_lnk_width ),       // O [5:0]
			    .cfg_cap_max_payload_size( cfg_cap_max_payload_size )  // O [2:0]
			    //                  .cfg_msi_enable(cfg_msi_enable)                        // O
	    
			    );      
      

endmodule // BMD
