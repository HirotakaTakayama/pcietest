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
//--            ** Copyright (C) 2009, Xilinx, Inc. **
//--            ** All Rights Reserved.             **
//--            **************************************
//--
//--------------------------------------------------------------------------------
//-- Filename: BMD_128_TX_ENGINE.v
//--
//-- Description: 128 bit Local-Link Transmit Unit.
//--
//--------------------------------------------------------------------------------

`timescale 1ns/1ns

module BMD_TX_ENGINE (
                      input 					  clk, //it's a user_clk(250MHz).
                      input 					  rst_n, //!user_reset
		      
		      output reg 				  s_axis_rq_tlast, //O
		      output reg [255:0] 			  s_axis_rq_tdata, //O
		      output [59:0] 				  s_axis_rq_tuser, //O
		      output reg [7:0] 				  s_axis_rq_tkeep, //O
		      input [3:0] 				  s_axis_rq_tready, //input from IP. 4bit
		      output reg 				  s_axis_rq_tvalid, //O

		      output reg [255:0] 			  s_axis_cc_tdata,
		      output reg [32:0] 			  s_axis_cc_tuser,
		      output reg 				  s_axis_cc_tlast,
		      output reg [7:0] 				  s_axis_cc_tkeep,
		      output reg 				  s_axis_cc_tvalid,
		      input [3:0] 				  s_axis_cc_tready,
   
		      input 					  req_compl_i,
		      output reg 				  compl_done_o,

		      input [2:0] 				  req_tc_i,
		      input 					  req_td_i,
		      input 					  req_ep_i,
		      input [1:0] 				  req_attr_i,
		      input [9:0] 				  req_len_i,
		      input [15:0] 				  req_rid_i,
		      input [7:0] 				  req_tag_i,
		      input [7:0] 				  req_be_i,
		      input [10:0] 				  req_addr_i, //It comes from 256_RX. It uses EP_MEM_ACCESS too.		      

                      // BMD Read Access
		      output [6:0] 				  rd_addr_o, //どこにもつながっていないっぽさあある
		      output [3:0] 				  rd_be_o,
		      input [31:0] 				  rd_data_i, 

                      // Initiator Reset
                      input 					  init_rst_i,

                      // Write Initiator
		      input 					  mwr_start_i,
		      input 					  mwr_int_dis_i,
		      input [31:0] 				  mwr_len_i, //mwr_len_i == mrd_len_i. BMD_EPでそうなっている. wr_d_i[15:0]で決定される.
		      input [7:0] 				  mwr_tag_i,
		      input [3:0] 				  mwr_lbe_i, //set rq_last_be. 4'hF
		      input [3:0] 				  mwr_fbe_i, //set rq_first_be. 4'hF
		      input [31:0] 				  mwr_addr_i, //RXのcq_dataによりこれにinputされるかが決まり, It is used when the rq client circuit makes transaction. It comes from BMD_EP_MEM. it decided at mem_wr_data.
		      input [31:0] 				  mwr_data_i,
		      input [31:0] 				  mwr_count_i, //It comes from BMD_EP_MEM. it decided at mem_wr_data. 
		      output reg 				  mwr_done_o,
		      input [2:0] 				  mwr_tlp_tc_i,
		      input 					  mwr_64b_en_i,
		      input 					  mwr_phant_func_dis1_i,
		      input [7:0] 				  mwr_up_addr_i,
		      input 					  mwr_relaxed_order_i,
		      input 					  mwr_nosnoop_i,
		      input [7:0] 				  mwr_wrr_cnt_i,
                      
                      // Read Initiator
		      input 					  mrd_start_i,
		      input 					  mrd_int_dis_i,
		      input [31:0] 				  mrd_len_i,
		      input [7:0] 				  mrd_tag_i,
		      input [3:0] 				  mrd_lbe_i, //set rq_last_be
		      input [3:0] 				  mrd_fbe_i, //set rq_first_be
		      input [31:0] 				  mrd_addr_i,
		      input [31:0] 				  mrd_count_i,
		      input 					  mrd_done_i,
		      input [2:0] 				  mrd_tlp_tc_i,
		      input 					  mrd_64b_en_i,
		      input 					  mrd_phant_func_dis1_i,
		      input [7:0] 				  mrd_up_addr_i,
		      input 					  mrd_relaxed_order_i,
		      input 					  mrd_nosnoop_i,
		      input [7:0] 				  mrd_wrr_cnt_i,

		      output [15:0] 				  cur_mrd_count_o,
                      
		      // CN ADDED FOR CS
		      output [23:0] 				  cur_wr_count_o, //Never used by all other circuit...?

		      input 					  cfg_msi_enable_i,
		      output 					  cfg_interrupt_n_o,
		      output 					  cfg_interrupt_assert_n_o,
		      input 					  cfg_interrupt_rdy_n_i,
		      input 					  cfg_interrupt_legacyclr,

		      input [15:0] 				  completer_id_i,
		      input 					  cfg_ext_tag_en_i,
		      input 					  cfg_bus_mstr_enable_i,

		      input 					  cfg_phant_func_en_i,
		      input [1:0] 				  cfg_phant_func_supported_i,

		      input 					  cpld_receive_i, //it comes from rc always of 256_RX. request completed signal.
		      input 					  calc_finished,

		      //old FIFO.
		      input [255:0] 				  fifo_read_data,
		      input 					  fifo_prog_empty,
		      output 					  fifo_read_en,
		      input 					  fifo_read_valid, 
		      input 					  fifo_prog_full,
		      input 					  fifo_sender_data_prepare_ok, 
		      /*
		       //build in FIFO. It comes 250MHz.
		       b_fifo_from_sender_read_data, //input. 256bit.
		       b_fifo_from_sender_read_en, //output          
		       b_fifo_from_sender_empty, //input
		       */

		      //受信側FPGAでのパケット転送開始信号
		      input 					  Receiver_side_trans_start,

		      //latency signal
		      input 					  latency_reset_signal,
		      input 					  latency_data_en,
		      output reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_counter, //send to RX_ENGINE. これは送信側FPGAの絶対時刻

		      //BRAM
		      output reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] bram_wr_data, //send to check_latency. これは送信側FPGAのあるデータの送信時の時刻
		      output reg 				  bram_wea,
		      output reg 				  bram_ena,
		      output reg [12:0] 			  bram_wr_addr,

		      output reg [31:0] 			  vio_settings_sender_address_for_sender_out,
		      output reg 				  Tlp_stop_interrupt,

              //count_wait
		      input 					  fifo_read_trigger,
		      output reg 				  fifo_counter_read_en,
		      input [RX_SIDE_WAITING_VALUE - 1:0] 	  waiting_counter,
		      input [RX_SIDE_WAITING_VALUE - 1:0] 	  fifo_counter_value_out,

		      //debug signal
		      input 					  m_axis_rc_tlast_i,
		      input [255:0] 				  m_axis_rc_tdata_i,
		      input 					  m_axis_rc_tvalid_i
                      );

   //localparameter
   localparam BRAM_ADDRESS_MAX   = 13'd8191;
   localparam TAG_FIELD_SIZE     = 4'd8; //タグフィールドのサイズ（5 or 8）
   localparam ECHO_TRANS_COUNTER_WIDTH = 8'd38; //レイテンシ測定（echo転送）時のカウンタサイズ設定
   localparam RX_SIDE_WAITING_VALUE  = 8'd30; //30bitだと4秒ぐらい
   
   //受信側FPGAの番地が送信側FPGAの番地であれば（つまり，受信側FPGAからechoを返す時にだけ一致する）
   assign      FPGA_RECEIVER_SIDE = ( receiveside_fpga_address == vio_settings_sender_address_for_sender[31:0] );

   /* Present address and byte enable to memory module */
   assign      rd_addr_o          = req_addr_i[10:2];
   assign      rd_be_o            = req_be_i[3:0];


   // Local registers
   reg [12:0]  byte_count;
   reg [6:0]   lower_addr;

   reg         req_compl_q;

   reg [2:0]   bmd_256_tx_state;

   reg         mrd_done;

   reg [23:0]  cur_wr_count;
   reg [23:0]  cur_rd_count;
   
   reg [9:0]   cur_mwr_dw_count; //packetが処理されるたびに -8(1clkで処理されるパケット分)される
   
   reg [12:0]  mwr_len_byte;
   reg [12:0]  mrd_len_byte;

   reg [31:0]  pmwr_addr;
   reg [31:0]  pmrd_addr;

   reg [31:0]  tmwr_addr;
   reg [31:0]  tmrd_addr;

   reg [23:0]  rmwr_count; //mwr_count_i[23:0]が入る. BMD_EPで決定される
   reg [23:0]  rmrd_count;

   reg         serv_mwr;
   reg         serv_mrd;

   reg  [7:0]  tmwr_wrr_cnt;
   reg  [7:0]  tmrd_wrr_cnt;

   //RQ tuser
   reg [3:0]   rq_first_be;
   reg [3:0]   rq_last_be;
   assign      s_axis_rq_tuser[59:0] = { 52'd0, rq_last_be, rq_first_be }; //[8]から上位は全て0fill. [8]より上位はparityなど

   //FIFO ctrl
   reg         fifo_reading;
   reg [9:0]   fifo_read_count;
   //read request to tx_side fifo
   assign      fifo_read_en          = ( fifo_reading && s_axis_rq_tready[0] ); //rq_tready is sent from rq IP.
   
   assign      cur_mrd_count_o[15:0] = cur_rd_count;
   // CN ADDED FOR CS
   assign      cur_wr_count_o[23:0]  = cur_wr_count;

   
   reg [3:0]   request_count;

   reg         cpld_waiting;
   reg         mem_writing;

   // Local wires   
   wire        cfg_bm_en       = 1'b1;
   wire [31:0] mwr_addr        = mwr_addr_i;
   wire [31:0] mrd_addr        = mrd_addr_i;

   wire [31:0] vio_settings_sender_address_for_sender;

   function [31:0] dword_data_align;
      input [31:0]    data;
      begin
         dword_data_align = data;
      end
   endfunction // dword_data_align   

   function [255:0] packet_data_align;
      input [255:0]   data;
      begin
         packet_data_align[31:0]    = dword_data_align(data[255:224]);
         packet_data_align[63:32]   = dword_data_align(data[223:192]);
         packet_data_align[95:64]   = dword_data_align(data[191:160]);
         packet_data_align[127:96]  = dword_data_align(data[159:128]);
         packet_data_align[159:128] = dword_data_align(data[127:96]);
         packet_data_align[191:160] = dword_data_align(data[95:64]);
         packet_data_align[223:192] = dword_data_align(data[63:32]);
         packet_data_align[255:224] = dword_data_align(data[31:0]);
      end
   endfunction // dword_data_align
   
   /*
    * Calculate byte count based on byte enable
    */

   always @ ( rd_be_o ) begin
      casex ( rd_be_o[3:0] )
        4'b1xx1 : byte_count = 13'h004;
        4'b01x1 : byte_count = 13'h003;
        4'b1x10 : byte_count = 13'h003;
        4'b0011 : byte_count = 13'h002;
        4'b0110 : byte_count = 13'h002;
        4'b1100 : byte_count = 13'h002;
        4'b0001 : byte_count = 13'h001;
        4'b0010 : byte_count = 13'h001;
        4'b0100 : byte_count = 13'h001;
        4'b1000 : byte_count = 13'h001;
        4'b0000 : byte_count = 13'h001;
      endcase // casex (rd_be_o[3:0])
   end

   /*
    * Calculate lower address based on  byte enable
    */

   always @ ( rd_be_o or req_addr_i ) begin
      casex ( rd_be_o[3:0] )
        4'b0000 : lower_addr = {req_addr_i[4:0], 2'b00};
        4'bxxx1 : lower_addr = {req_addr_i[4:0], 2'b00};
        4'bxx10 : lower_addr = {req_addr_i[4:0], 2'b01};
        4'bx100 : lower_addr = {req_addr_i[4:0], 2'b10};
        4'b1000 : lower_addr = {req_addr_i[4:0], 2'b11};
      endcase // casex (rd_be_o[3:0])
   end

   always @ ( posedge clk ) begin
      if (!rst_n ) begin
         req_compl_q <= 1'b0;
      end 
      else begin
         req_compl_q <= req_compl_i;
      end
   end

   always @ ( posedge clk ) begin
        if (!rst_n ) begin
            vio_settings_sender_address_for_sender_out <= 32'd0;
        end
        else begin
            vio_settings_sender_address_for_sender_out <= vio_settings_sender_address_for_sender;
        end
    end


   /* Interrupt Controller */
   BMD_INTR_CTRL BMD_INTR_CTRL  (
				 .clk(clk),                                     // I
				 .rst_n(rst_n),                                 // I

				 .init_rst_i(init_rst_i),                       // I

				 .mrd_done_i(mrd_done_i & !mrd_int_dis_i),      // I
				 .mwr_done_i(mwr_done_o & !mwr_int_dis_i),      // I

				 .msi_on(cfg_msi_enable_i),                     // I

				 .cfg_interrupt_rdy_n_i(cfg_interrupt_rdy_n_i), // I
				 .cfg_interrupt_assert_n_o(cfg_interrupt_assert_n_o), // O
				 .cfg_interrupt_n_o(cfg_interrupt_n_o),        // O
				 .cfg_interrupt_legacyclr(cfg_interrupt_legacyclr) // I
				 );
   
   /* Tx State Machine */
   /**************************/
   //Completer Completion Client
   /**************************/
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
         s_axis_cc_tdata         <= 256'b0;
         s_axis_cc_tkeep         <= 8'b0;
         s_axis_cc_tlast         <= 1'b0;
         s_axis_cc_tvalid        <= 1'b0;
         s_axis_cc_tuser         <= 33'b0;

         compl_done_o            <= 1'b0;
         cpld_waiting            <= 1'b0;
      end
      else begin
         if ( init_rst_i ) begin
            s_axis_cc_tdata      <= 256'b0;
            s_axis_cc_tkeep      <= 8'b0;
            s_axis_cc_tlast      <= 1'b0;
            s_axis_cc_tvalid     <= 1'b0;
            s_axis_cc_tuser      <= 33'b0;
            compl_done_o         <= 1'b0;
            cpld_waiting         <= 1'b0;
         end

         if ( !cpld_waiting ) begin
            compl_done_o         <= 1'b0;
            // PIO read completions always get highest priority.  if req_compl_i signal(from cq client of RX_ENGINE ) will be asserted, the header will start.
            if ( req_compl_q && !compl_done_o ) begin 
               s_axis_cc_tvalid  <= 1'b1;
               s_axis_cc_tlast   <= 1'b1;
               s_axis_cc_tkeep   <= 8'h0F; //
               
               s_axis_cc_tdata   <= {128'b0,        // Tied to 0 for 3DW completion descriptor
				     rd_data_i,       // 32- bit read data
				     1'b0,          // Force ECRC
				     1'b0, req_attr_i,// Attr. 3- bits
				     req_tc_i,        // TC. 3- bits
				     1'b0,          // Disable Completer ID
				     // Supplied Bus number
				     //completer_id_i, //Completer ID
				     16'b0, //Completer ID (not use)
				     //8'hAA,         // Completer Bus number - selected if Compl ID    = 1
				     //{5'b11111, 3'b000},//Compl Dev / Func no - sel if Compl ID = 1
				     req_tag_i,  // Select Client Tag or core's internal tag
				     req_rid_i,       // Requester ID - 16 bits
				     1'b0,          // Rsvd
				     1'b0,          // Posioned completion
				     3'b000,        // SuccessFull completion
				     {1'b0, req_len_i},         // DWord Count 0 - IO Write completions
				     2'b0,          // Rsvd [31:30]
				     1'b0,  // Locked Read Completion [29]
				     byte_count,      // Byte Count [28:16]
				     6'b0,          // Rsvd
				     {2'b0},        // Adress Type - 2 bits
				     1'b0,          // Rsvd
				     lower_addr};   // Starting address of the mem byte - 7 bits
               

               /*
                //It uses data test for getting latency of transportation 1DW.
                s_axis_cc_tdata   <= {128'b0,        // Tied to 0 for 3DW completion descriptor
                rd_data_i,       // 32- bit read data
                1'b0,          // Force ECRC
                3'd0, // Attr.3- bits
                3'd0,        // TC. 3- bits
                1'b0,          // Disable Completer ID
                // Supplied Bus number
                //completer_id_i, //Completer ID
                16'b0, //Completer ID (not use)
                req_tag_i,  // Select Client Tag or core's internal tag. from cq
                req_rid_i,       // Requester ID - 16 bits. from cq
                1'b0,          // Rsvd
                1'b0,          // Posioned completion
                3'b000,        // SuccessFull completion
                11'd1,         // DWord Count 0 - IO Write completions. from cq
                2'b0,          // Rsvd [31:30]
                1'b0,  // Locked Read Completion [29]
                13'd1,      // Byte Count [28:16] //testではfirst_beに0を書いているので, bytecount = 1. length0のmem readではmust1
                6'b0,          // Rsvd
                {2'b0},        // Adress Type - 2 bits
                1'b0,          // Rsvd
                lower_addr};   // Starting address of the mem byte - 7 bits
                */

               s_axis_cc_tuser   <= 33'b0;

               compl_done_o      <= 1'b1; //256_RXのcompletion確認のための信号, Requester Completionへ送る
               cpld_waiting      <= 1'b1;
            end // if ( req_compl_q && !compl_done_o )
         end // if ( !cpld_waiting )
         //cpld_waiting == 1'b1. completionなのでheaderを送信した次のclkで終了するだけ.
         else begin
            if ( s_axis_cc_tready[0] ) begin
               s_axis_cc_tvalid  <= 1'b0;
               s_axis_cc_tlast   <= 1'b0;

               cpld_waiting      <= 1'b0;
            end
         end // else: !if( !cpld_waiting )
      end // else: !if(!rst_n )
   end // always @ ( posedge clk )



   

   /**************************/
   // Requester reQuest Client
   /**************************/
   //local regs
   reg         start_wr_test_edge;
   reg         start_rd_test_edge;
   reg         tag_offset;
   reg [7:0]   tag_num;
   reg [9:0]   incr_data_counter;
   
   reg [12:0]  echo_tlp_num;

   //local wires
   wire [31:0] receiveside_fpga_address;      
   wire [31:0] requester_payload_data;
   
   wire        vio_latency_count_continue;
   wire        vio_echo_mode;
   
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
         s_axis_rq_tdata    <= 256'b0;
         s_axis_rq_tkeep    <= 8'b0;
         s_axis_rq_tlast    <= 1'b0;
         s_axis_rq_tvalid   <= 1'b0;
         fifo_reading       <= 1'b0;
         request_count      <= 4'd0;
         
         cur_mwr_dw_count   <= 10'b0;

         mwr_done_o         <= 1'b0;
         mrd_done           <= 1'b0;

         cur_wr_count       <= 24'b0;
         cur_rd_count       <= 24'b1;

         mwr_len_byte       <= 13'b0;
         mrd_len_byte       <= 13'b0;

         pmwr_addr          <= 32'b0;
         pmrd_addr          <= 32'b0;

         rmwr_count         <= 24'b0;
         rmrd_count         <= 24'b0;

         serv_mwr           <= 1'b1;
         serv_mrd           <= 1'b1;

         tmwr_wrr_cnt       <= 8'h00;
         tmrd_wrr_cnt       <= 8'h00;

         mem_writing        <= 1'b0;
         tag_offset         <= 1'b0;
         incr_data_counter  <= 10'd0;

         //bram 
         bram_wr_addr       <= 13'd0;
         bram_wea           <= 1'b0;
         bram_ena           <= 1'b0;

         echo_tlp_num       <= 13'd0;
         tag_num            <= 8'd0;
         Tlp_stop_interrupt <= 1'b0;

         fifo_counter_read_en <= 1'b0;
      end

      else begin // if (!rst_n )
         //user reset
         if( latency_reset_signal ) begin
            incr_data_counter  <= 10'd0;
            echo_tlp_num       <= 13'd0;
            bram_wr_addr       <= 13'd0;
            bram_wea           <= 1'b0;
            bram_ena           <= 1'b0;
            Tlp_stop_interrupt <= 1'b0;

            fifo_counter_read_en <= 1'b0;
            cur_wr_count       <= 24'd0; //tag
         end

         //受信側FPGAから送るパケット数をカウント, 受信側FPGAのみで動作
         if( Receiver_side_trans_start && FPGA_RECEIVER_SIDE ) begin
            echo_tlp_num       <= echo_tlp_num  + 1'b1;
         end
         bram_wea <= 1'b0;
         bram_ena <= 1'b0;

	 //LATENCY評価用.
         if( vio_latency_count_continue ) begin //BRAMに書き続けてチェックするver
   	    if( bram_wr_addr == BRAM_ADDRESS_MAX ) begin
   	       bram_wr_addr   <= 13'd0;
   	    end
   	    else begin
   	       bram_wr_addr   <= bram_wr_addr + ( bram_wea && bram_ena ); //書き込みが始まったらアドレス遷移
   	    end
   	 end
   	 if( !vio_latency_count_continue ) begin //一度BRAMに書ききったらそれ以降書かないようにするver
   	    if( bram_wr_addr == BRAM_ADDRESS_MAX ) begin
   	       bram_wr_addr       <= BRAM_ADDRESS_MAX;
   	       bram_wea           <= 1'b0;
               bram_ena           <= 1'b0;
   	       Tlp_stop_interrupt <= 1'b1;
   	    end
   	    else begin
   	       bram_wr_addr   <= bram_wr_addr + ( bram_wea && bram_ena ); //書き込みが始まったらアドレス遷移
   	    end
   	 end

	 
         if ( fifo_read_en ) begin
            fifo_read_count    <= fifo_read_count - 10'd1;
	 end         
         if ( fifo_read_en && fifo_read_count == 10'd1 ) begin
            fifo_reading       <= 1'b0;
	 end
         if ( cpld_receive_i ) begin
            request_count      <= request_count - 4'd1;
	 end
         
         if ( init_rst_i ) begin
            s_axis_rq_tdata    <= 256'b0;
            s_axis_rq_tkeep    <= 8'b0;
            s_axis_rq_tlast    <= 1'b0;
            s_axis_rq_tvalid   <= 1'b0;

            fifo_reading       <= 1'b0;
            
            cur_mwr_dw_count   <= 10'b0;
            
            mwr_done_o         <= 1'b0;
            mrd_done           <= 1'b0;
            
            cur_wr_count       <= 24'b0;
            cur_rd_count       <= 24'b1;

            mwr_len_byte       <= 13'b0;
            mrd_len_byte       <= 13'b0;

            pmwr_addr          <= 32'b0;
            pmrd_addr          <= 32'b0;
            rmwr_count         <= 24'b0;
            rmrd_count         <= 24'b0;

            serv_mwr           <= 1'b1;
            serv_mrd           <= 1'b1;
            tmwr_wrr_cnt       <= 8'h00;
            tmrd_wrr_cnt       <= 8'h00;

            mem_writing        <= 1'b0;
            request_count      <= 4'd0;
         end
         
         mwr_len_byte         <= 4 * mwr_len_i[10:0]; //change to Byte unit
         mrd_len_byte         <= 4 * mrd_len_i[10:0]; //change to Byte unit
         rmwr_count           <= mwr_count_i[23:0]; //
         rmrd_count           <= mrd_count_i[23:0]; //BMD_EPで設定できる

         fifo_counter_read_en <= 1'b0; //基本的には0， header受信時のみ1

         //!mem_writing : header実行前はassertされ、payloadを処理しきるまでdeassertされる。
         if ( !mem_writing ) begin
            fifo_reading   <= 1'b0;

            /*Memory Write access*/         
            //mwr_start_i : BMD_EPの calc_start_p がassertされた次クロックにassert. calc_start_p はcalc_pcie_synchronizerから出力.
            //mwr_done_o : メモリへのwrが終了したときにassertされ、ヘッダ転送もdeassertされる.
            //!fifo_prog_empty : sender側のfifoが空でなければ転送が開始される
            //calc_finished : BMD_EPの信号. calc_start_pによりdeassertされ、assertはcalc_pcie_synchronizerからのcalc_finish_pcie信号でされる。
            //start_wr_test_edge : VIOから指定する信号
            

            /************/
            /* Write test. It uses data test for getting latency and throughput of transportation multi DW. */
            /************/

            //test_sender_start_vio: TLP送信側FPGAのパケット転送開始信号
            //Receiver_side_trans_start: TLP受信側(echo側)FPGAのTLP受信完了&&echo送信開始信号
            //echo_tlp_num 送信側FPGAからTLPが届いた数 - echoしてあるtlpの数
            //receiveside_fpga_address: 受信側FPGA（ここだとecho側FPGAから見た送信側FPGA）のマッピングされたアドレス番地を指定する．TLP送信側FPGAがechoしないようにするために必要．
            //vio_settings_sender_address_for_sender: 送信側FPGAにとっての送信側FPGAのアドレス番地設定．複数のFPGAを使っていても同じ設定となる．
            //receiveside_fpga_address == vio_settings_sender_address_for_sender[31:0], 送信側FPGAがfffffffだとしたら，どのFPGAでもvio_settings_sender_address_for_senderをfffffffとする．
            if( s_axis_rq_tready[0] && !Tlp_stop_interrupt &&
            ( test_sender_start_vio ||
                  ( fifo_read_trigger && FPGA_RECEIVER_SIDE && vio_echo_mode ) ) ) begin //receiver FPGAからのechoあり
	       //            if( s_axis_rq_tready[0] && test_sender_start_vio ) begin //receuver FPGAからのechoなし

               cur_wr_count       <= cur_wr_count + 1'b1;

               s_axis_rq_tvalid   <= 1'b1;
               s_axis_rq_tlast    <= 1'b0;
               s_axis_rq_tkeep    <= 8'hFF;


	       //tagの設定を変えて，送信側と受信側でタグ番号が被らないようにする．　こうすれば詰まらないと想定した //bram domain
               if( FPGA_RECEIVER_SIDE ) begin
                  tag_offset      <= 1'b0;
                  echo_tlp_num    <= echo_tlp_num - 1'b1; //減算
               end
               else if( !FPGA_RECEIVER_SIDE ) begin //receiver_FPGA
                  tag_offset      <= 1'b1;
               end
               
               if( TAG_FIELD_SIZE == 4'd8 ) begin //拡張タグフィールド
                    tag_num       = { cur_wr_count[6:0], tag_offset }; //tagの値
               end
               else if( TAG_FIELD_SIZE == 4'd5 ) begin //通常タグフィールド
                    tag_num       = { 3'd0, cur_wr_count[3:0], tag_offset }; //tagの値
               end
               
               //Requester Requestからデータを発行するためのヘッダ, ここの指定を変えればFPGAからのアクセスになるはず。後々はこれを残しつつFPGAからのアクセスも送れるようにする                  
               s_axis_rq_tdata  <= {128'b0, // padding 
                                    1'b0, // Force ECRC
                                    3'd0, // Attr //3bit
                                    3'd0, // Transaction Class //3bit
                                    1'b0, // Disable requester ID
                                    16'd0, // completer ID (not use)
                                    tag_num[7:0], // tag　拡張タグ空間
				    //                           { 3'd0, tag_offset, cur_wr_count[3:0] }, // tag 標準タグ空間
                                    16'd0, // requester ID (not use)
                                    1'd0, // Posioned completion
                                    4'b0001, // Req type (memory write)
                                    {1'b0, mwr_len_i[9:0]}, // dword count. It decided test_sender
                                    {24'b0},
				    //                                              mwr_up_addr_i,
				    //                                              tmwr_addr[31:2],//Address (64-bit)
				    8'd0,
				    receiveside_fpga_address[31:2],
                                    {2'b0} //Address type
                                    };


               rq_last_be       <= ( mwr_len_i[9:0] == 10'd1 ) ? 4'b0 : mwr_lbe_i; //1DW write requestの時
               rq_first_be      <= mwr_fbe_i;
               fifo_reading     <= 1'b1; //If this signal and rq_tready are asserted, fifo_read_en will be enable.
               fifo_read_count  <= mwr_len_i[12:3]; //10bit size.[3]から見ることで、Dword / 8(つまりfifoの残数)を取得する

               cur_mwr_dw_count <= mwr_len_i[9:0]; //DWord Countが設定される
               mem_writing      <= 1'b1;

               fifo_counter_read_en <= ( fifo_read_trigger && FPGA_RECEIVER_SIDE && vio_echo_mode );
            end


            //not read, not write state..., but !mem_writing.
            else begin
               if ( s_axis_rq_tready[0] ) begin
                  s_axis_rq_tvalid <= 1'b0;
                  s_axis_rq_tlast  <= 1'b0;
                  s_axis_rq_tkeep  <= 8'hFF;
                  s_axis_rq_tdata  <= 256'b0;

                  mrd_done         <= 1'b0; //test用. rd headerの後、deassertされ、またrdができるようになる。
               end
            end // else: !if( !mrd_done && !request_count[3] && s_axis_rq_tready[0] &&...
         end // if ( !mem_writing )


         ////////////////////
         //Write test. It uses data test for getting latency of transportation multi words.
         ////////////////////
         // mem_writingのとき
         else begin
            s_axis_rq_tvalid  <= 1'b1; //treadyがenの時はvldもen. when transaction issue, this signal will assert.
            //rq_tvalid && rq_treadyの時にデータを送る. treadyはIPが受け付けられる状態かを示す.当然IPからくる.
            if ( s_axis_rq_tready[0] ) begin
	       //                s_axis_rq_tdata   <= fifo_read_data;
               s_axis_rq_tdata   <= { 196'd0, waiting_counter[RX_SIDE_WAITING_VALUE - 1:0], fifo_counter_value_out[RX_SIDE_WAITING_VALUE - 1:0] }; //256bytes{  [パケットを送るときの時間] - [パケットを受け取った時の時間] }
               s_axis_rq_tkeep   <= 8'hFF; //all of the data are enabled.
               cur_mwr_dw_count  <= cur_mwr_dw_count - 4'h8; //decrement 256bit(8DW)

               //残り8DWなら次は終了.assert rq_tlast.上の式と同タイミングで処理されるので、8DWのデータであればすぐにこの式が評価される
               if ( cur_mwr_dw_count == 4'h8 ) begin
                  s_axis_rq_tlast <= 1'b1; //transaction 終了信号
                  mem_writing     <= 1'b0; //これで再びheader送信可能に
                  fifo_reading    <= 1'b0; //FIFOから受信終了
                  bram_wea        <= 1'b1; //BRAMに入れる
                  bram_ena        <= 1'b1;

		  //                  if ( cur_wr_count == rmwr_count )  begin //rmwr_countはヘッダの送られる最大回数.
                  if ( cur_wr_count[6:0] == 7'b111_1111 ) begin //tag reset 7bitで判断．8bit目はFPGAごとに異なる
                     cur_wr_count <= 24'd0;
                     // mwr_done_o   <= 1'b1; //これを使うとこの後からヘッダ送れなくなる.
                  end

                  //送信data用
                  if( incr_data_counter == 10'd1023 ) begin
                     incr_data_counter <= 10'd0;
                  end
                  if( incr_data_counter != 10'd1023 ) begin
                     incr_data_counter <= incr_data_counter + 1'b1; //data incr
                  end                  
               end
               //まだDWが残っていればtlastはassertされない
               else begin
                  fifo_reading    <= 1'b1; //FIFOから受信
                  s_axis_rq_tlast <= 1'b0;
               end
            end
            //treadyが立っていない時はFIFOからreadしない
            else begin
               fifo_reading       <= 1'b0;
            end
            
         end // else: !if( !mem_writing )

      end // else: !if(!rst_n )
   end // always @ ( posedge clk )




   /***************************/
   /*packet send start edge signal*/
   /***************************/
   //local regs
   reg wr_test_perm_1;
   reg rd_test_perm_1;   
   reg start_rd_test_edge_d1;

   //local wires
   wire requester_wr_start_signal;
   wire requester_rd_start_signal;

   //write packet send start edge signal
   always @( posedge clk ) begin
      if( init_rst_i ) begin
         start_wr_test_edge    <= 1'b0;
         wr_test_perm_1        <= 1'b0;
      end
      else begin
         if( requester_wr_start_signal && !wr_test_perm_1 ) begin
            start_wr_test_edge <= 1'b1;
            wr_test_perm_1     <= 1'b1;
         end
         else if( requester_wr_start_signal && wr_test_perm_1 ) begin
            start_wr_test_edge <= 1'b0;
         end
         else if( !requester_wr_start_signal ) begin
            wr_test_perm_1     <= 1'b0;
         end
      end
   end

   //read packet send start edge signal
   always @( posedge clk ) begin
      if( init_rst_i ) begin
         start_rd_test_edge    <= 1'b0;
         start_rd_test_edge_d1 <= 1'b0;
         rd_test_perm_1        <= 1'b0;
      end
      else begin
         if( requester_rd_start_signal && !rd_test_perm_1 ) begin
            start_rd_test_edge <= 1'b1;
            rd_test_perm_1     <= 1'b1;
         end
         else if( requester_rd_start_signal && rd_test_perm_1 ) begin
            start_rd_test_edge <= 1'b0;
         end
         else if( !requester_rd_start_signal ) begin
            rd_test_perm_1     <= 1'b0;
         end

         start_rd_test_edge_d1 <= start_rd_test_edge; //delay 1clk
      end
   end



   /*********************************************************************************/
   /*********************************************************************************/
   //Latency check
   /*********************************************************************************/
   // for latency check, TLP送信を開始した時点からの時間を計測．
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
         bram_wr_data    <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         latency_counter <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
      end
      else if( latency_reset_signal ) begin
         bram_wr_data    <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         latency_counter <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
      end
      else if( test_sender_start_vio ) begin //TLP送信かつ，latency測定開始時にカウント開始
         bram_wr_data    <= bram_wr_data + 1'b1;
         latency_counter <= latency_counter + 1'b1;
      end
   end


   /***************************************************************************************************************************************/
   /***************************************************************************************************************************************/
   /*Declaration of vio*/
   /***************************************************************************************************************************************/
   //VIO IP core for input receiver side fpga BAR1 address.
   vio_256bit1in_32bit1out_1bit1out vio_addr_sendto_senderfpga
     (
      .clk( clk ),
      .probe_in0( 1'b0 ),
      .probe_out0( receiveside_fpga_address ),  //32bit //address of receiver side fpga BAR1.
      .probe_out1( test_sender_start_vio ) //1bit //wr tx start signal

      //   .probe_out1( requester_wr_start_signal ), //1bit //start signal of requester write request.
      //   .probe_out2( requester_rd_start_signal ), //1bit //start signal of requester read request.
      //   .probe_out3( requester_payload_data ) //32bit //It uses only 1DW transaction.
      );



   //レイテンシチェック 送信側FPGAの番地設定と，echo側でのデータたまりチェック
   vio_sender_fpga_address vio_sender_fpga_address
     (
      .clk( clk ),
      .probe_in0( echo_tlp_num[12:0] ), //13bit
      .probe_out0( vio_settings_sender_address_for_sender[31:0] ), //32bit //address of sender side fpga BAR1.
      .probe_out1( vio_latency_count_continue ), //1bit //レイテンシチェックを継続して行うか，ちょっとの間だけ行うかを決める.0:継続しない．1:継続してBRAMへ書き込み
      .probe_out2( vio_echo_mode ) //1bit //レイテンシを測るか（受信側FPGAからechoを返すか）どうかを決める.0:echoなし（スループットチェック用）.1:echoあり（レイテンシチェック用）
      );

endmodule // BMD_TX_ENGINE
