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
//-- Filename: BMD_EP.v
//--
//-- Description: Bus Master Device I/O Endpoint module. 
//--
//--------------------------------------------------------------------------------
`timescale 1ns/1ns


module BMD_EP#
  (
   parameter INTERFACE_WIDTH = 64,
   parameter INTERFACE_TYPE = 4'b0010,
   parameter FPGA_FAMILY = 8'h14

   )
      (
       clk,                 
       rst_n,              

       // LocalLink Tx

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
      
       // LocalLink Rx

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


       trn_rcpl_streaming_n,

       // Turnoff access

       req_compl_o,
       compl_done_o,

       // Configuration access
      
       cfg_interrupt_n,
       cfg_interrupt_rdy_n,
       cfg_interrupt_assert_n,
       cfg_interrupt_di,
       cfg_interrupt_do,
       cfg_interrupt_mmenable,
       cfg_interrupt_msienable,
       cfg_completer_id,

       cfg_ext_tag_en,

       cfg_cap_max_lnk_width,
       cfg_neg_max_lnk_width,

       cfg_cap_max_payload_size,
       cfg_prg_max_payload_size,
       cfg_max_rd_req_size,
       cfg_msi_enable,
       cfg_rd_comp_bound,

       cfg_phant_func_en,
       cfg_phant_func_supported,


       cpld_data_size_hwm,
       cpld_size,
       cur_rd_count_hwm,
       cur_mrd_count,

       cfg_bus_mstr_enable,
      
       //memory interface
      
       fifo_read_data,
       fifo_prog_empty,
       fifo_read_en,
       fifo_prog_full,
       fifo_write_en, 
       fifo_write_data,
       fifo_read_valid, //I

       fifo_sender_data_prepare_ok, //I
       packet_data_num, //I
       
       calc_start,
       calc_finish,
       io_command,
       io_command_en,
       random_seed,
       fpga_state,
       receive_mode
   
       );

      input              clk;
      input              rst_n;

      // LocalLink Tx
      
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

      //packet 送信数(DW/8)
      input [31:0] packet_data_num;

      input 		calc_start;
      input 		calc_finish;
      
      output [3:0] 	io_command;
      output 		io_command_en;
      output [31:0] 	random_seed;
      input [2:0] 		fpga_state;
      output 		receive_mode;
      assign receive_mode = mrd_start;
      
      // LocalLink Rx

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
      input 		    m_axis_rc_tlast;
      input 		    m_axis_rc_tvalid;
      input [7:0]   m_axis_rc_tkeep;
      input [74:0] 	    m_axis_rc_tuser;
      output		    m_axis_rc_tready;


      output            trn_rcpl_streaming_n;
      
      output            req_compl_o;
      output            compl_done_o;
      
      output            cfg_interrupt_n;
      input             cfg_interrupt_rdy_n;
      output            cfg_interrupt_assert_n;

      output [7:0]      cfg_interrupt_di;
      input  [7:0]      cfg_interrupt_do;
      input  [2:0]      cfg_interrupt_mmenable;
      input             cfg_interrupt_msienable;

      input [15:0]      cfg_completer_id;
      input             cfg_ext_tag_en;
      input             cfg_bus_mstr_enable;
      input [5:0]       cfg_cap_max_lnk_width;
      input [5:0]       cfg_neg_max_lnk_width;

      input [2:0]       cfg_cap_max_payload_size;
      input [2:0]       cfg_prg_max_payload_size;
      input [2:0]       cfg_max_rd_req_size;
      input             cfg_msi_enable;
      input             cfg_rd_comp_bound;

      input             cfg_phant_func_en;
      input [1:0]       cfg_phant_func_supported;

      output [31:0]     cpld_data_size_hwm;     // HWMark for Completion Data (DWs)
      output [15:0]     cur_rd_count_hwm;       // HWMark for Read Count Allowed
      output [31:0]     cpld_size;
      output [15:0]     cur_mrd_count;
      

      // Local wires
      
      wire  [10:0]      rd_addr; 
      wire  [3:0]       rd_be; 
      wire  [31:0]      rd_data; 

      wire  [10:0]      req_addr; 

      wire  [7:0]       wr_be; 
      wire  [31:0]      wr_data; 
      wire              wr_en;
      wire              wr_busy;

      wire              req_compl;
      wire              compl_done;

      wire  [2:0]       req_tc;
      wire              req_td; 
      wire              req_ep; 
      wire  [1:0]       req_attr; 
      wire  [9:0]       req_len;
      wire  [15:0]      req_rid;
      wire  [7:0]       req_tag;
      wire  [7:0]       req_be;

      wire              init_rst;

      wire              mwr_start;
      wire              mwr_int_dis_o; 
      wire              mwr_done;
      wire  [31:0]      mwr_len; //use at EP_MEM only
      wire  [7:0]       mwr_tag;
      wire  [3:0]       mwr_lbe;
      wire  [3:0]       mwr_fbe;
      wire  [31:0]      mwr_addr;
      wire  [31:0]      mwr_count;
      wire  [31:0]      mwr_data;
      wire  [2:0]       mwr_tlp_tc_o;  
      wire              mwr_64b_en_o;
      wire              mwr_phant_func_en1;
      wire  [7:0]       mwr_up_addr_o;
      wire              mwr_relaxed_order;
      wire              mwr_nosnoop;
      wire  [7:0]       mwr_wrr_cnt;

      wire              mrd_start;
      wire              mrd_int_dis_o; 
      wire              mrd_done;
      wire  [31:0]      mrd_len; //use at EP_MEM, EP_256_TX. 
      wire  [7:0]       mrd_tag;
      wire  [3:0]       mrd_lbe;
      wire  [3:0]       mrd_fbe;
      wire  [31:0]      mrd_addr;
      wire  [31:0]      mrd_count;
      wire  [2:0]       mrd_tlp_tc_o;  
      wire              mrd_64b_en_o;
      wire              mrd_phant_func_en1;
      wire  [7:0]       mrd_up_addr_o;
      wire              mrd_relaxed_order;
      wire              mrd_nosnoop;
      wire  [7:0]       mrd_wrr_cnt;
      
      wire  [7:0]       cpl_ur_found;
      wire  [7:0]       cpl_ur_tag;

      wire  [31:0]      cpld_data;
      wire  [31:0]      cpld_found;
      wire  [31:0]      cpld_size;
      wire              cpld_malformed;
      wire              cpld_data_err;

      wire              mrd_start_o;
      wire [15:0]       cur_mrd_count;

      wire              cpl_streaming;              
      wire              rd_metering;
      wire              trn_rnp_ok_n_o;
      wire              trn_tstr_n_o;
      wire              cfg_interrupt_legacyclr;

      wire Receiver_side_trans_start;
      
      //debug signal
      wire m_axis_rc_tlast_o;
      wire [255:0] m_axis_rc_tdata_o;
      wire m_axis_rc_tvalid_o;
      //


      assign            trn_rnp_ok_n = trn_rnp_ok_n_o;
      assign            trn_tstr_n = trn_tstr_n_o;


      assign            trn_rcpl_streaming_n = ~cpl_streaming;


      wire 	      cpld_receive;
      reg 		      mwr_start_reg;
      reg 		      init_rst_reg;
      reg 		      init_rst_reg_delay;
      reg 		      calc_finished;
      reg 		      calc_start_delay;
      wire 	      calc_start_p = calc_start && !calc_start_delay;

      
      always @ (posedge clk or negedge rst_n) begin
	    if (!rst_n) begin
		  mwr_start_reg <= 0;
		  init_rst_reg <= 0;
	    end
	    else if (init_rst) begin
		  mwr_start_reg <= init_rst_reg_delay;
		  init_rst_reg <= 0;
	    end
	    else if (init_rst_reg) begin
		  init_rst_reg <= 0;
	    end
	    else if (calc_start_p) begin
		  mwr_start_reg <= 0;
		  init_rst_reg <= 1;
	    end
      end // always @ (posedge clk or negedge rst_n)

      always @ (posedge clk or negedge rst_n) begin
	    if (!rst_n) begin
		  init_rst_reg_delay <= 0;
		  calc_start_delay <= 0;
	    end
	    else begin
		  init_rst_reg_delay <= init_rst_reg;
		  calc_start_delay <= calc_start;
	    end
      end

      always @ (posedge clk or negedge rst_n) begin
	    if (!rst_n)
	      calc_finished <= 1'b0;
	    else if (calc_start_p)
	      calc_finished <= 1'b0;
	    else if (calc_finish)
	      calc_finished <= 1'b1;
      end


      //////////////////////          
      //////////////////////      
      //Dword countに適�?, 送信数を設�?
      reg [31:0] packet_DW_count;
      wire [31:0] tx_send_num;
      always @( posedge clk ) begin
	    if( !rst_n ) begin
		  packet_DW_count <= 32'd0;
	    end
	    else begin
//		  packet_DW_count[31:3] <= packet_data_num;

		  packet_DW_count <= tx_send_num;
	    end
      end
      
      //////////////////////
      //送信DW数を記�?�
      //////////////////////
      vio_num_trans vio_num_trans
	(
	 .clk( clk ),
	 .probe_in0( 1'b0 ), //1bit
	 .probe_out0( tx_send_num ) //32bit(DW数を�?��?)
	 );





      //
      // ENDPOINT MEMORY : 
      // 

      BMD_EP_MEM_ACCESS#(
			 .INTERFACE_TYPE(INTERFACE_TYPE),
			 .FPGA_FAMILY(FPGA_FAMILY)
			 )
      EP_MEM (

              .clk(clk),                           // I
              .rst_n(rst_n),                       // I

              .cfg_cap_max_lnk_width(cfg_cap_max_lnk_width), // I [5:0]
              .cfg_neg_max_lnk_width(cfg_neg_max_lnk_width), // I [5:0]

              .cfg_cap_max_payload_size(cfg_cap_max_payload_size), // I [2:0]
              .cfg_prg_max_payload_size(cfg_prg_max_payload_size), // I [2:0]
              .cfg_max_rd_req_size(cfg_max_rd_req_size),           // I [2:0]

              .addr_i(req_addr[6:0]),              // I [10:0]

              // Read Port

              .rd_be_i(rd_be),                     // I [3:0]
              .rd_data_o(rd_data),                 // O [31:0]

              // Write Port

              .wr_be_i(wr_be),                     // I [7:0]
              .wr_data_i(wr_data),                 // I [31:0]
              .wr_en_i(wr_en),                     // I
              .wr_busy_o(wr_busy),                 // O

              .init_rst_o(init_rst),               // O

              .mrd_start_o(mrd_start),             // O
              .mrd_int_dis_o(mrd_int_dis_o),       // O
              .mrd_done_o(mrd_done),               // O
              .mrd_addr_o(mrd_addr),               // O [31:0]
              .mrd_len_o(mrd_len),                 // O [31:0] //It declared at BMD_EP_MEM.v //this is a transaction packet DW size.
              .mrd_count_o(mrd_count),             // O [31:0]
              .mrd_tlp_tc_o(mrd_tlp_tc_o),         // O [2:0]
              .mrd_64b_en_o(mrd_64b_en_o),         // O
              .mrd_phant_func_dis1_o(mrd_phant_func_dis1), // O
              .mrd_up_addr_o(mrd_up_addr_o),       // O [7:0]
              .mrd_relaxed_order_o(mrd_relaxed_order), // O
              .mrd_nosnoop_o(mrd_nosnoop),         // O
              .mrd_wrr_cnt_o(mrd_wrr_cnt),         // O [7:0]

              .mwr_start_i(mwr_start_reg),             // I
              .mwr_int_dis_o(mwr_int_dis_o),       // O
              .mwr_done_i(mwr_done),               // I
              .mwr_addr_o(mwr_addr),               // O [31:0]
              .mwr_len_o(mwr_len),                 // O [31:0]
              .mwr_count_o(mwr_count),             // O [31:0]
              .mwr_data_o(mwr_data),               // O [31:0]
              .mwr_tlp_tc_o(mwr_tlp_tc_o),         // O [2:0]
              .mwr_64b_en_o(mwr_64b_en_o),         // O
              .mwr_phant_func_dis1_o(mwr_phant_func_dis1), // O
              .mwr_up_addr_o(mwr_up_addr_o),       // O [7:0]
              .mwr_relaxed_order_o(mwr_relaxed_order), // O
              .mwr_nosnoop_o(mwr_nosnoop),         // O
              .mwr_wrr_cnt_o(mwr_wrr_cnt),         // O [7:0]

              .cpl_ur_found_i(cpl_ur_found),       // I [7:0]
              .cpl_ur_tag_i(cpl_ur_tag),           // I [7:0]

              .cpld_data_o(cpld_data),             // O [31:0]
              .cpld_found_i(cpld_found),           // I [31:0]
              .cpld_data_size_i(cpld_size),        // I [31:0]
              .cpld_malformed_i(cpld_malformed),   // I 
              .cpld_data_err_i(cpld_data_err),     // I
              .cpl_streaming_o(cpl_streaming),     // O
              .rd_metering_o(rd_metering),         // O
              .cfg_interrupt_di(cfg_interrupt_di),         // O
              .cfg_interrupt_do(cfg_interrupt_do),         // I
              .cfg_interrupt_mmenable(cfg_interrupt_mmenable),     // I
              .cfg_interrupt_msienable(cfg_interrupt_msienable),   // I
              .cfg_interrupt_legacyclr(cfg_interrupt_legacyclr),   // O

              .trn_rnp_ok_n_o(trn_rnp_ok_n_o),      // O
              .trn_tstr_n_o ( trn_tstr_n_o  ),       // O
	    
	    
	      //fpga state interface
	      .io_command(io_command),
	      .io_command_en(io_command_en),
	      
	      //IO read completion data
	      .fpga_state(fpga_state),

	      .init_rst_reg(init_rst_reg),
	      .calc_start(calc_start_p),
	      .calc_finish(calc_finish),
	      .random_seed(random_seed)


              );


      //
      // Local-Link Receive Controller :
      //
      wire latency_reset_signal;
      wire [63:0] latency_counter;
            
      wire [63:0] bram_wr_data;
      wire bram_wea;
      wire [9:0] bram_wr_addr;
      wire bram_reb;
      wire [9:0] bram_rd_addr;
      wire [63:0] bram_rd_data;

      wire [31:0] vio_settings_sender_address_for_sender;

      BMD_RX_ENGINE EP_RX (
	    
			   .clk(clk),                           // I
			   .rst_n(rst_n),                       // I

			   .init_rst_i(init_rst),               // I

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

			   // Handshake with Tx engine 

			   .req_compl_o(req_compl),             // O
			   .compl_done_i(compl_done),           // I

			   .addr_o(req_addr),                   // O [10:0]

			   .req_tc_o(req_tc),                   // O [2:0]
			   .req_td_o(req_td),                   // O
			   .req_ep_o(req_ep),                   // O
			   .req_attr_o(req_attr),               // O [1:0]
			   .req_len_o(req_len),                 // O [9:0]
			   .req_rid_o(req_rid),                 // O [15:0]
			   .req_tag_o(req_tag),                 // O [7:0]
			   .req_be_o(req_be),                   // O [7:0]

			   // Memory Write Port

			   .wr_be_o(wr_be),                     // O [7:0]
			   .wr_data_o(wr_data),                 // O [31:0]
			   .wr_en_o(wr_en),                     // O
			   .wr_busy_i(wr_busy),                 // I
			   
			   .cpl_ur_found_o(cpl_ur_found),       // O [7:0]
			   .cpl_ur_tag_o(cpl_ur_tag),           // O [7:0]

			   .cpld_data_i(cpld_data),             // I [31:0]
			   .cpld_found_o(cpld_found),           // O [31:0]
			   .cpld_data_size_o(cpld_size),        // O [31:0]
			   .cpld_malformed_o(cpld_malformed),   // O 
			   .cpld_data_err_o(cpld_data_err),      // O


			   .cpld_receive_o(cpld_receive),
			   //bram interface
			   .fifo_write_en(fifo_write_en), //O
			   .fifo_write_data(fifo_write_data), //O

			   .packet_DW_setting( packet_DW_count ), //I

			   .Receiver_side_trans_start( Receiver_side_trans_start ), //O

			   .latency_reset_signal( latency_reset_signal ),  //O  //send to BMD_256_** 
			   .latency_counter( latency_counter[63:0] ), //I //64bit //come from TX_ENGINE
			   .latency_data_en( latency_data_en ), //O
			   .bram_rd_data( bram_rd_data ), //I //64bit //come from check_latency
			   .bram_reb( bram_reb ), //O
			   .bram_rd_addr( bram_rd_addr[9:0] ),//O
			   .vio_settings_sender_address_for_sender( vio_settings_sender_address_for_sender[31:0] ) //I
			   );

      
      //
      // Local-Link Transmit Controller
      // 

      BMD_TX_ENGINE EP_TX (

			   .clk(clk),                         // I
			   .rst_n(rst_n),                     // I

			   // LocalLink Tx
			   .s_axis_rq_tlast(s_axis_rq_tlast),
			   .s_axis_rq_tdata(s_axis_rq_tdata),
			   .s_axis_rq_tuser(s_axis_rq_tuser), //O
			   .s_axis_rq_tkeep(s_axis_rq_tkeep),
			   .s_axis_rq_tready(s_axis_rq_tready),
			   .s_axis_rq_tvalid(s_axis_rq_tvalid),

			   .s_axis_cc_tdata(s_axis_cc_tdata),
			   .s_axis_cc_tuser(s_axis_cc_tuser),
			   .s_axis_cc_tlast(s_axis_cc_tlast),
			   .s_axis_cc_tkeep(s_axis_cc_tkeep),
			   .s_axis_cc_tvalid(s_axis_cc_tvalid),
			   .s_axis_cc_tready(s_axis_cc_tready),

			   // Handshake with Rx engine 
			   .req_compl_i(req_compl),           // I
			   .compl_done_o(compl_done),         // 0

			   .req_tc_i(req_tc),                 // I [2:0]
			   .req_td_i(req_td),                 // I
			   .req_ep_i(req_ep),                 // I
			   .req_attr_i(req_attr),             // I [1:0]
			   .req_len_i(req_len),               // I [9:0]
			   .req_rid_i(req_rid),               // I [15:0]
			   .req_tag_i(req_tag),               // I [7:0]
			   .req_be_i(req_be),                 // I [7:0]
			   .req_addr_i(req_addr),             // I [10:0]
            
			   // Read Port

			   .rd_addr_o(rd_addr[6:0]),         // I [10:0] //どこにもつながって�?な�?感じがあ�?
			   .rd_be_o(rd_be),                  // I [3:0]
			   .rd_data_i(rd_data),              // O [31:0]

			   // Initiator Controls

			   .init_rst_i(init_rst),            // I

			   .mrd_start_i(mrd_start_o),        // I
			   .mrd_int_dis_i(1'b1),    // I
			   .mrd_done_i(mrd_done),            // I
			   .mrd_addr_i(mrd_addr),            // I [31:0]
			   
//			   .mrd_len_i(mrd_len),              // I [31:0]
//			   .mrd_len_i( 32'd1 ),              // I [31:0] //changed
			   .mrd_len_i( packet_DW_count ), //data DW数 //changed
//			   .mrd_len_i( 32'd960 ), //120packet

			   
//			   .mrd_count_i(mrd_count),          // I [31:0]
//			   .mrd_count_i( 24'd1 ),          // I [31:0] //changed
			   .mrd_count_i( packet_DW_count ), //temporaly
//			   .mrd_count_i( 24'd960 ),


			   .mrd_tlp_tc_i(mrd_tlp_tc_o),      // I [2:0]
			   .mrd_64b_en_i(mrd_64b_en_o),      // I
			   .mrd_phant_func_dis1_i(1'b1 /*mrd_phant_func_dis1*/), // I
			   .mrd_up_addr_i(mrd_up_addr_o),    // I [7:0]
			   .mrd_lbe_i(4'hF),        
			   .mrd_fbe_i(4'hF),
			   .mrd_tag_i(8'h0),
			   .cur_mrd_count_o(cur_mrd_count),  // O[15:0]
			   .mrd_relaxed_order_i(mrd_relaxed_order), // I
			   .mrd_nosnoop_i(mrd_nosnoop),             // I
			   .mrd_wrr_cnt_i(mrd_wrr_cnt),      // I [7:0]

			   .mwr_start_i(mwr_start_reg),          // I
			   //.mwr_start_i(mwr_start),          // I
			   .mwr_int_dis_i(1'b1),    // I
			   .mwr_done_o(mwr_done),            // O
			   .mwr_addr_i(mwr_addr),            // I [31:0]
			   
//			   .mwr_len_i(mrd_len),              // I [31:0] //mwr_len_i == mrd_len_i になって�?�?
//			   .mwr_len_i( 32'd1 ),              // I [31:0] //mwr_len_i == mrd_len_i になって�?�? //changed
			   .mwr_len_i( packet_DW_count ), //data DW数 //changed
//			   .mwr_len_i( 32'd960 ),

//			   .mwr_count_i(mwr_count),          // I [31:0] //TXでのrmwr_count()とな�?
//			   .mwr_count_i( 24'd1 ),          // I [31:0] //TXでのrmwr_count()とな�? //changed
			   .mwr_count_i( packet_DW_count ), //temporaly //TXでのrmwr_count()とな�? //changed
//			   .mwr_count_i( 24'd960 ),


			   .mwr_data_i(mwr_data),            // I [31:0] 
			   .mwr_tlp_tc_i(mwr_tlp_tc_o),      // I [2:0]
			   .mwr_64b_en_i(1'b0),      // I
			   .mwr_phant_func_dis1_i(1'b1 /*mwr_phant_func_dis1*/), // I
			   .mwr_up_addr_i(mwr_up_addr_o),    // I [7:0]
			   .mwr_lbe_i(4'hF),
			   .mwr_fbe_i(4'hF),
			   .mwr_tag_i(8'h0),
			   .mwr_relaxed_order_i(mwr_relaxed_order), // I
			   .mwr_nosnoop_i(mwr_nosnoop),             // I
			   .mwr_wrr_cnt_i(mwr_wrr_cnt),       // I [7:0]

			   .cfg_msi_enable_i(cfg_msi_enable),            // I
			   .cfg_interrupt_n_o(cfg_interrupt_n),          // O
			   .cfg_interrupt_assert_n_o(cfg_interrupt_assert_n), // O
			   .cfg_interrupt_rdy_n_i(cfg_interrupt_rdy_n),  // I
			   .cfg_interrupt_legacyclr(cfg_interrupt_legacyclr),  // I
			   .completer_id_i(cfg_completer_id),            // I [15:0]
			   .cfg_ext_tag_en_i(cfg_ext_tag_en),            // I
			   .cfg_bus_mstr_enable_i(cfg_bus_mstr_enable),  // I
			   .cfg_phant_func_en_i(cfg_phant_func_en),                  // I
			   .cfg_phant_func_supported_i(cfg_phant_func_supported),    // I [1:0]


			   .cpld_receive_i(cpld_receive),
			   .calc_finished(calc_finished),


			   //memory interface

			   .fifo_prog_full(fifo_prog_full),
			   
			   .fifo_read_data(fifo_read_data),
			   .fifo_prog_empty(fifo_prog_empty),
			   // for test
			   //.pat_read_data(0),
			   //.color_read_data(0),
			   //.pat_prog_empty(1'd0),
			   //.color_prog_empty(1'd0),
			   .fifo_read_en(fifo_read_en),
			   .fifo_read_valid( fifo_read_valid ),
			   .fifo_sender_data_prepare_ok( fifo_sender_data_prepare_ok ),

			   .Receiver_side_trans_start( Receiver_side_trans_start ), //I

			   //bram
			   .latency_reset_signal( latency_reset_signal ), //I
			   .latency_data_en( latency_data_en ), //I
			   .latency_counter( latency_counter[63:0] ), //O //64bit //send to RX_ENGINE. これは絶対時刻
			   .bram_wr_data( bram_wr_data ), //O //64bit //send to check_latency. これはある�?ータの送信時�?�時刻
			   .bram_wea( bram_wea ), //O
			   .bram_wr_addr( bram_wr_addr[9:0] ), //O //10bit

			   .vio_settings_sender_address_for_sender( vio_settings_sender_address_for_sender[31:0] ), //O

			   //debug signal
			   .m_axis_rc_tdata_i(m_axis_rc_tdata),
			   .m_axis_rc_tlast_i(m_axis_rc_tlast),
			   .m_axis_rc_tvalid_i(m_axis_rc_tvalid)
			   
			   );
      

      assign req_compl_o  = req_compl;
      assign compl_done_o = compl_done;


      //
      // Read Transmit Throttle Unit :
      // 
/*
      BMD_RD_THROTTLE RD_THR (

			      .clk(clk),                           // I
			      .rst_n(rst_n),                       // I

			      .init_rst_i(init_rst),               // I

			      .mrd_start_i(mrd_start),             // I
			      
//			      .mrd_len_i(mrd_len),                 // I
//			      .mrd_len_i( 32'd1 ),                 // I //changed
			      .mrd_len_i( packet_DW_count ), //data DW数 //changed
//			      .mrd_len_i( 32'd960 ),

			      .mrd_cur_rd_count_i(cur_mrd_count),  // I [15:0]    

			      .cpld_found_i(cpld_found),           // I [31:0]
			      .cpld_data_size_i(cpld_size),        // I [31:0]
			      .cpld_malformed_i(cpld_malformed),   // I
			      .cpld_data_err_i(cpld_data_err),     // I

			      .cpld_data_size_hwm(cpld_data_size_hwm), // O [31:0]
			      .cur_rd_count_hwm(cur_rd_count_hwm),     // O [15:0]

			      .cfg_rd_comp_bound_i(cfg_rd_comp_bound), // I
			      .rd_metering_i(1'b0),             // I

			      .mrd_start_o(mrd_start_o)                // O

			      );

*/


  /****************************************************************************************************************/
  //new module declaration for check latency
  /****************************************************************************************************************/

  BMD_256_check_latency BMD_256_check_latency
	(
		.clk( clk ),
	 	.rst_n( rst_n ),
	 	.latency_reset_signal( latency_reset_signal ), //comes from RX_ENGINE, user reset.
	 	.latency_counter( latency_counter[63:0] ), //I //64bit //come from TX_ENGINE

		//write domain, comes from TX_ENGINE
		.bram_wea( bram_wea ),
		.bram_wr_addr( bram_wr_addr[9:0] ),
		.bram_wr_data( bram_wr_data[63:0] ),

		//read domain, comes from RX_ENGINE
		.bram_reb( bram_reb ),
		.bram_rd_addr( bram_rd_addr[9:0] ),
		//read domain send to RX_ENGINE
		.bram_rd_data( bram_rd_data[63:0] ) //O
	);


endmodule // BMD_EP

