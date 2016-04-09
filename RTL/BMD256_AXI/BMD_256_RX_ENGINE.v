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
//-- Filename: BMD_256_RX_ENGINE.v
//--
//-- Description: 128 bit Local-Link Receive Unit.
//--
//--------------------------------------------------------------------------------

`timescale 1ns/1ns

module BMD_RX_ENGINE (
		      input 				       clk,
		      input 				       rst_n,

                      /* Initiator reset */
		      input 				       init_rst_i,

		      // Completer Request Interface
		      input [255:0] 			       m_axis_cq_tdata,
		      input 				       m_axis_cq_tlast,
		      input 				       m_axis_cq_tvalid,
		      input [84:0] 			       m_axis_cq_tuser,
		      input [7:0] 			       m_axis_cq_tkeep,
		      input [5:0] 			       pcie_cq_np_req_count,
		      output reg 			       m_axis_cq_tready,
		      output 				       pcie_cq_np_req,

		      // Requester Completion Interface
		      input [255:0] 			       m_axis_rc_tdata,
		      input 				       m_axis_rc_tlast,
		      input 				       m_axis_rc_tvalid,
		      input [7:0] 			       m_axis_rc_tkeep,
		      input [74:0] 			       m_axis_rc_tuser,
		      output reg 			       m_axis_rc_tready,

                      /*
                       * Memory Read data handshake with Completion 
                       * transmit unit. Transmit unit reponds to 
                       * req_compl assertion and responds with compl_done
                       * assertion when a Completion w/ data is transmitted. 
                       */
		      output reg 			       req_compl_o,
		      input 				       compl_done_i,

                      output reg [10:0] 		       addr_o, //EP_MEM_ACCESS と256_TXにつながっているっぽい // Memory Read Address


		      output reg [2:0] 			       req_tc_o, // Memory Read TC
		      output reg 			       req_td_o, // Memory Read TD
		      output reg 			       req_ep_o, // Memory Read EP
		      output reg [1:0] 			       req_attr_o, // Memory Read Attribute
		      output reg [9:0] 			       req_len_o, // Memory Read Length
		      output reg [15:0] 		       req_rid_o, // Memory Read Requestor ID
		      output reg [7:0] 			       req_tag_o, // Memory Read Tag
		      output reg [7:0] 			       req_be_o, // Memory Read Byte Enables

                      /* 
                       * Memory interface used to save 1 DW data received 
                       * on Memory Write 32 TLP. Data extracted from
                       * inbound TLP is presented to the Endpoint memory
                       * unit. Endpoint memory unit reacts to wr_en_o
                       * assertion and asserts wr_busy_i when it is 
                       * processing written information.
                       */
		      output reg [7:0] 			       wr_be_o, // Memory Write Byte Enable
		      output reg [31:0] 		       wr_data_o, //memory writeで使われる。 // Memory Write Data
		      output reg 			       wr_en_o, // Memory Write Enable
		      input 				       wr_busy_i, // Memory Write Busy   

                      /* Completion no Data */
		      output reg [7:0] 			       cpl_ur_found_o,
		      output reg [7:0] 			       cpl_ur_tag_o,

                      /* Completion with Data */
		      input [31:0] 			       cpld_data_i,
		      output reg [31:0] 		       cpld_found_o,
		      output reg [31:0] 		       cpld_data_size_o,
		      output reg 			       cpld_malformed_o,
		      output 				       cpld_data_err_o,

                      output reg 			       cpld_receive_o,

		      //FIFO
		      output reg [255:0] 		       fifo_write_data,
		      output reg 			       fifo_write_en, //b_fifo_receiver side write enable

		      //settings for PCIe
		      input [31:0] 			       packet_DW_setting,
		      output reg 			       Receiver_side_trans_start,

		      //latency calc
		      input [ECHO_TRANS_COUNTER_WIDTH - 1:0]   latency_counter, //come from TX_ENGINE
		      output reg 			       latency_reset_signal_out,
		      output 				       latency_data_en,

		      input 				       Tlp_stop_interrupt,
		      input [31:0] 			       vio_settings_sender_address_for_sender,

		      //bram access
		      input [ECHO_TRANS_COUNTER_WIDTH - 1:0]   bram_rd_data, //come from check_latency
		      output reg 			       bram_reb,
		      output reg [12:0] 		       bram_rd_addr,

		      //time calc fifo access
		      output reg [RX_SIDE_WAITING_VALUE - 1:0] waiting_counter, //最初のパケットが到達した段階からカウントし始めるカウンタ．echo開始までの時間測定用．
		      output reg 			       cq_sop_out,
		      input 				       fifo_counter_empty,

		      input 				       FPGA_RECEIVER_SIDE //受信側FPGAなら1
		      );
   //FIFO
   /*
    output 	      fifo_write_en;
    output [191:0]      fifo_write_data;
    reg 		      fifo_write_en;
    reg [191:0] 	      fifo_write_data;
    */

   //Req type 
   //      localparam PIO_RX_MEM_RD_FMT_TYPE    = 4'b0000;    // Memory Read
   //      localparam PIO_RX_MEM_WR_FMT_TYPE    = 4'b0001;    // Memory Write
   localparam RX_MEM_RD_FMT_TYPE       = 4'b0000;    // Memory Read
   localparam RX_MEM_WR_FMT_TYPE       = 4'b0001;    // Memory Write

   localparam BMD_256_RX_RST           = 8'b0000_0001;
   localparam BMD_256_RX_MEM_RD32_WT   = 8'b0000_0100;
   localparam BMD_256_RX_MEM_WR32_WT   = 8'b0001_0000;

   //benchmark param
   localparam THROUGHPUT_100MS         = 32'd25000000; //250MHzだと1clk4nsなので、1秒は250Mclk. 100msは25Mclk.
   localparam BRAM_ADDRESS_MAX         = 13'd8191; //bram depth value 
   localparam ECHO_TRANS_COUNTER_WIDTH = 8'd40; //レイテンシ測定（echo転送）時のカウンタサイズ設定
   localparam RX_SIDE_WAITING_VALUE    = 8'd30; //30bitだと4秒ぐらい
   
   assign     cpld_data_err_o          = 1'd0; // no error check
   assign     pcie_cq_np_req           = 1'b1;

   // Local Registers
   reg [7:0]  bmd_256_rx_state;
   
   //Local wires
   wire latency_reset_signal;

   always@( posedge clk ) begin
        if( !rst_n ) begin
            latency_reset_signal_out <= 1'b0;
        end
        else begin
            latency_reset_signal_out <= latency_reset_signal;
        end
    end

   // Generate a signal that indicates if we are currently receiving a packet.
   // This value is one clock cycle delayed from what is actually on the AXIS
   // data bus.
   wire       cq_sop;                   // Start of packet
   reg 	      cq_in_packet_q;
   wire       rc_sop;                   // Start of packet
   reg 	      rc_in_packet_q;

   //completer request start signal
   always@( posedge clk ) begin
      if( !rst_n ) begin
	 cq_in_packet_q <=  1'b0;
      end
      else if ( m_axis_cq_tvalid && m_axis_cq_tready && m_axis_cq_tlast ) begin
	 cq_in_packet_q <=  1'b0;
      end
      else if ( cq_sop && m_axis_cq_tready ) begin
	 cq_in_packet_q <=  1'b1;
      end
   end

   assign cq_sop        = ( !cq_in_packet_q && m_axis_cq_tvalid );

   //requester completion start signal
   always@( posedge clk ) begin
      if( !rst_n ) begin
	 rc_in_packet_q <=  1'b0;
      end
      else if ( m_axis_rc_tvalid && m_axis_rc_tready && m_axis_rc_tlast ) begin
	 rc_in_packet_q <=  1'b0;
      end
      else if ( rc_sop && m_axis_rc_tready ) begin
	 rc_in_packet_q <=  1'b1;
      end
   end
   assign rc_sop        = ( !rc_in_packet_q && m_axis_rc_tvalid );


   always@( posedge clk ) begin
        if( !rst_n ) begin
            cq_sop_out <= 1'b0;
        end
        else begin
            cq_sop_out <= cq_sop;
        end
    end


   /*
    //It uses at 192bit FIFO.
    function [23:0] dword_data_align;
    input [31:0] 	data;
    begin
    dword_data_align = data[23:0];
	    end
      endfunction

    //192bit長に変更、配列を変更
    function [191:0] packet_data_align;
    input [255:0]    data;
    //means m_axis_rc_tdata[23:0] -> packet_data_align[191:168] 183,180
    //m_axis_rc_tdata[87:64] -> packet_data_align[143:120]
    begin
    packet_data_align[191:168] = dword_data_align(data[31:0]);
    packet_data_align[167:144] = dword_data_align(data[63:32]);
    packet_data_align[143:120] = dword_data_align(data[95:64]);
    packet_data_align[119:96]  = dword_data_align(data[127:96]);
    packet_data_align[95:72]   = dword_data_align(data[159:128]);
    packet_data_align[71:48]   = dword_data_align(data[191:160]);
    packet_data_align[47:24]   = dword_data_align(data[223:192]);
    packet_data_align[23:0]    = dword_data_align(data[255:224]);
	    end
      endfunction // dword_data_align
    */
   
   //It uses at build in FIFO
   function [31:0] dword_data_align;
      input [31:0] 	data;
      begin
	 dword_data_align = data;
      end
   endfunction // dword_data_align
   
   //256bit build in fifoのためのalign
   function [255:0] packet_data_align_for_bfifo;
      input [255:0] data;
      begin
	 packet_data_align_for_bfifo[31:0]    = dword_data_align(data[255:224]);
	 packet_data_align_for_bfifo[63:32]   = dword_data_align(data[223:192]);
	 packet_data_align_for_bfifo[95:64]   = dword_data_align(data[191:160]);
	 packet_data_align_for_bfifo[127:96]  = dword_data_align(data[159:128]);
	 packet_data_align_for_bfifo[159:128] = dword_data_align(data[127:96]);
	 packet_data_align_for_bfifo[191:160] = dword_data_align(data[95:64]);
	 packet_data_align_for_bfifo[223:192] = dword_data_align(data[63:32]);
	 packet_data_align_for_bfifo[255:224] = dword_data_align(data[31:0]);
      end
   endfunction // dword_data_align


   /***************************/
   //Completer Request
   /***************************/
   reg         cq_receiving; //completer request受付中はen
   reg [127:0] cq_tdata_b1_128bit; //FIFOに送る下位128bit分
   reg [10:0]  wrDWpacket_num_remaining;
   reg [3:0]   memory_access_type;
   reg [10:0]  total_DW_count;
   reg timer_trigger;
   
   always @ ( posedge clk ) begin      
      if ( !rst_n ) begin
	 bmd_256_rx_state <= BMD_256_RX_RST;
	 m_axis_cq_tready <= 1'b1;
	 
	 req_compl_o      <= 1'b0;
	 
	 req_tc_o         <= 2'b0;
	 req_td_o         <= 1'b0;
	 req_ep_o         <= 1'b0;
	 req_attr_o       <= 2'b0;
	 req_len_o        <= 10'b0;
	 req_rid_o        <= 16'b0;
	 req_tag_o        <= 8'b0;
	 req_be_o         <= 8'b0;
	 addr_o           <= 11'b0;

	 wr_be_o          <= 8'b0;
	 wr_data_o        <= 32'b0;
	 wr_en_o          <= 1'b0;

	 //for fifo write
	 cq_receiving              <= 1'b0;
	 fifo_write_data           <= 256'd0;
	 fifo_write_en             <= 1'b0;
	 cq_tdata_b1_128bit        <= 128'd0;
	 wrDWpacket_num_remaining  <= 11'd0;
	 memory_access_type        <= 4'b0000;
	 total_DW_count            <= 11'd0;

	 Receiver_side_trans_start <= 1'b0; //not start receive
	 waiting_counter           <= { RX_SIDE_WAITING_VALUE{1'd0} };
	 timer_trigger             <= 1'b0;
      end // if ( !rst_n )
      //user reset.
      else  if( latency_reset_signal ) begin 
	 timer_trigger          <= 1'b0;
         waiting_counter        <= { RX_SIDE_WAITING_VALUE{1'd0} };
      end
      
      else begin
	 wr_en_o                   <= 1'b0;
	 req_compl_o               <= 1'b0;
	 m_axis_cq_tready          <= 1'b1; //treadyをassert、assert中はデータを受け付ける
	 Receiver_side_trans_start <= 1'b0; //not start receive
	 
	 if ( init_rst_i ) begin
	    bmd_256_rx_state       <= BMD_256_RX_RST;
	 end
	 	 
	 if( FPGA_RECEIVER_SIDE ) begin
	    //最初のパケットを受け取るまでは0. 繰り返しをする時にもカウントの初めは0
	    if( fifo_counter_empty && !timer_trigger ) begin
	       waiting_counter        <= { RX_SIDE_WAITING_VALUE{1'd0} };
	 end
	    //最初のパケットを受け取ったら，受信側FPGAはカウント開始
	    else if( timer_trigger ) begin
               waiting_counter        <= waiting_counter + 1'b1;
	    end
	 end
	 
	 case ( bmd_256_rx_state )
	   BMD_256_RX_RST : begin
	      /*
	       if ( cq_sop ) begin //Completer reQuest start signal ( m_axis_cq_tuser[40](sop)でも可能)
	       case ( m_axis_cq_tdata[78:75] ) //readなら4'b0000, writeなら4'b0001
	       //Memory Read Request. 
	       PIO_RX_MEM_RD_FMT_TYPE : begin 
	       // CN - Logic for upper QW. payload長さが1DWであればこれを行う
	       if ( m_axis_cq_tdata[74:64] == 11'b1 ) begin
	       
               req_tc_o     <= m_axis_cq_tdata[123:121];  
               req_td_o     <= 1'b0; 
               req_ep_o     <= 1'b0;
               req_attr_o   <= m_axis_cq_tdata[126:124]; 
               req_len_o    <= m_axis_cq_tdata[74:64]; 
               req_rid_o    <= m_axis_cq_tdata[95:80]; 
               req_tag_o    <= m_axis_cq_tdata[103:96]; 
               req_be_o     <= m_axis_cq_tuser[7:0];

	       // CN - Logic for lower QW
	       addr_o            <= m_axis_cq_tdata[31:2]; //[31:2]となっていたが実際はregが11bitである
               req_compl_o       <= 1'b1; //Completer Request IPからのデータを取得したことを,Completer CompletionのUser logicへ送る. Read
	       m_axis_cq_tready <= 1'b0; //取得をとめる.readではこれがたったら終了
               bmd_256_rx_state   <= BMD_256_RX_MEM_RD32_WT;

					    end 
	       else begin
               bmd_256_rx_state <= BMD_256_RX_RST;
					    end
				      end // case: PIO_RX_MEM_RD_FMT_TYPE

	       //Memory Write Request.
	       PIO_RX_MEM_WR_FMT_TYPE : begin  					    
	       // CN - Logic for upper QW
	       //PCからのPIOのみを想定しているから1DWのみの受け取り. include header.
	       if ( m_axis_cq_tdata[74:64] == 11'd1 ) begin  // Length = 1 DW 
               wr_be_o      <= m_axis_cq_tuser[7:0];   //  7:4 = Last DW BE  3:0 = 1st DW BE
	       
	       // CN - Logic for lower QW
	       addr_o           <= m_axis_cq_tdata[31:2]; //address
               wr_data_o        <= m_axis_cq_tdata[159:128]; //EP_MEMへのwr data.
               wr_en_o          <= 1'b1; //EP_MEMへのwr enable signal.
               m_axis_cq_tready   <= 1'b0; //IPからのデータ取得をとめる
               bmd_256_rx_state  <= BMD_256_RX_MEM_WR32_WT;						  
					    end					    

	       else begin
               bmd_256_rx_state <= BMD_256_RX_RST;  
					    end // else: !if(m_axis_cq_tdata[74:64] == 11'b1)
				      end // case: PIO_RX_MEM_WR_FMT_TYPE
	       
				  endcase // case (m_axis_cq_tdata[78:75])
			    end // if (cq_sop)
	       */

	      /***********************/
	      //複数DWのmemory writeができるようにする
	      /***********************/
	      /////////////////////////
	      //1clk目専用
	      /////////////////////////
	      if ( cq_sop ) begin //Completer reQuest start signal ( m_axis_cq_tuser[40](sop)ではだめ)
		 timer_trigger <= 1'b1; //echo用タイマートリガー

		 //Memory Read Request //readなら4'b0000
		 if( m_axis_cq_tdata[78:75] == RX_MEM_RD_FMT_TYPE ) begin 
		    // CN - Logic for upper QW. payload長さが1DWであればこれを行う
		    if ( m_axis_cq_tdata[74:64] == 11'b1 ) begin						  
                       req_tc_o         <= m_axis_cq_tdata[123:121];  
                       req_td_o         <= 1'b0; 
                       req_ep_o         <= 1'b0;
                       req_attr_o       <= m_axis_cq_tdata[126:124]; 
                       req_len_o        <= m_axis_cq_tdata[74:64]; 
                       req_rid_o        <= m_axis_cq_tdata[95:80]; 
                       req_tag_o        <= m_axis_cq_tdata[103:96]; 
                       req_be_o         <= m_axis_cq_tuser[7:0];

		       // CN - Logic for lower QW
		       addr_o           <= m_axis_cq_tdata[31:2]; //[31:2]となっていたが実際はregが11bitである
               	       req_compl_o      <= 1'b1; //Completer Request IPからのデータを取得したことを,Completer CompletionのUser logicへ送る. Read
		       m_axis_cq_tready <= 1'b0; //取得をとめる.readではこれがたったら終了
              	       bmd_256_rx_state <= BMD_256_RX_MEM_RD32_WT;

		    end 
		    else begin
                       bmd_256_rx_state <= BMD_256_RX_RST;
		    end
		 end // case: PIO_RX_MEM_RD_FMT_TYPE


		 ////////////////////////////////
		 //Memory Write Request. //writeなら4'b0001
		 ////////////////////////////////
		 else if( m_axis_cq_tdata[78:75] == RX_MEM_WR_FMT_TYPE ) begin
		    // CN - Logic for upper QW
		    //複数DW受け取り. メモリへの wrはとりあえず記述しない(11/23/2014)
		    //header and DW0~3 //1clkのみで受け取れるmulti DW receive.
		    if( m_axis_cq_tdata[74:64] <= 11'd4 ) begin
		       //fifo write
		       //DW0
		       if( m_axis_cq_tdata[74:64] == 11'd1 ) begin
			  fifo_write_data <= { 224'd0, m_axis_cq_tdata[159:128] };
		       end
		       //DW0, 1
		       else if( m_axis_cq_tdata[74:64] == 11'd2 ) begin
			  fifo_write_data <= { 192'd0, m_axis_cq_tdata[191:128] };
		       end
		       //DW0, 1, 2
		       else if( m_axis_cq_tdata[74:64] == 11'd3 ) begin
			  fifo_write_data <= { 160'd0, m_axis_cq_tdata[223:128] };
		       end
		       //DW0~3
		       else begin
			  fifo_write_data <= { 128'd0, m_axis_cq_tdata[255:128] };
		       end

		       fifo_write_en      <= 1'b1; //receiver fifoへのwr
		       cq_receiving       <= 1'b0; //パケット受付終了
		       

		       bmd_256_rx_state   <= BMD_256_RX_RST; //継続してpacketを受け取れるようにしておく
		       //					      bmd_256_rx_state  <= BMD_256_RX_MEM_WR32_WT;
		    end

		    //2clk以上になる複数DW受け取り. 2clk目以降はwrDWpacket_num_remainingにより決定
		    //DW数5~8, 9~... 
		    else if( m_axis_cq_tdata[74:64] > 11'd4 ) begin 
		       wrDWpacket_num_remaining <= m_axis_cq_tdata[74:64]; //wrDWpacket_num_remainingにDwordを代入.
		       total_DW_count           <= total_DW_count + m_axis_cq_tdata[74:64]; //分割されてきたときに最終パケットがわかるようにする
		       memory_access_type       <= RX_MEM_WR_FMT_TYPE; //memory access typeを保存
		       cq_receiving             <= 1'b1; //パケット受付中 from TX_ENGINE(RQ)
		       //					      fifo_write_en <= 1'b0; //DW5以上の時はこのタイミングでは受け取りを行わないためdeassert
		       fifo_write_en            <= cq_receiving; //cq_receivingの次のタイミングで立つ
		       
		       //DW0~3, 8~11, ... write 128bit before clk data. Cuz cq is DWord-Aligned.
		       cq_tdata_b1_128bit       <= m_axis_cq_tdata[255:128];
		       m_axis_cq_tready         <= 1'b1; //tready is enabled.
		       bmd_256_rx_state         <= BMD_256_RX_RST; //state continue
		    end // if ( m_axis_cq_tdata[74:64] > 11'd4 )

		    
		    else begin
                       bmd_256_rx_state         <= BMD_256_RX_RST;  
		    end // else: !if(m_axis_cq_tdata[74:64] == 11'b1)
		    
		 end // case: PIO_RX_MEM_WR_FMT_TYPE				  
	      end // if ( cq_sop )


	      /////////////////////////
	      //2clk目以降
	      /////////////////////////
	      else if ( cq_receiving ) begin //Completer reQuest continue
		 ////////////////////////////////
		 //Memory Write Request. //writeなら4'b0001
		 ////////////////////////////////
		 if( memory_access_type == RX_MEM_WR_FMT_TYPE ) begin
		    //2clk以上になる複数DW受け取り. wrDWpacket_num_remainingにDwordを代入. 1clk目はm_axis_cq_tdataの論理、2clk目以降はwrDWpacket_num_remainingにより決定
		    //DW数5~8, 9~... 
		    if( wrDWpacket_num_remaining != 11'd0 ) begin 			         
		       //RQからの受け取り終了(受け取ったpacketのDW数が5~8ならば、下のelse ifにいかずにすぐにここで処理される)
		       if( wrDWpacket_num_remaining <= 11'd8 ) begin
			  fifo_write_data           <= { m_axis_cq_tdata[127:0], cq_tdata_b1_128bit };
			  //						    fifo_write_en <= 1'b1; //receiver fifoへのwr
			  fifo_write_en             <= cq_receiving;
			  cq_receiving              <= 1'b0; //パケット受付終了 from TX_ENGINE(RQ)

			  Receiver_side_trans_start <= 1'b1; //受信側receiver開始
			  
			  //new
			  m_axis_cq_tready          <= 1'b0; //受け取らないように立ち下げる
			  wrDWpacket_num_remaining  <= 11'd0; //次のトランザクションもまた受け取れるように初期化

			  bmd_256_rx_state          <= BMD_256_RX_RST;  

			  memory_access_type        <= 4'b0000; //initialization

			  //求めるDW数が来たら初期化(分割されてもカウントできるように)
			  if( total_DW_count == packet_DW_setting ) begin
			     total_DW_count         <= 11'd0;
			  end
			  
		       end
		       //1clk目ではない(headerを含んでいない) and 残りのDWは8以上
		       else begin
			  //若いDWから順にFIFOの上位bitへ入っていく
			  fifo_write_data           <= { m_axis_cq_tdata[127:0], cq_tdata_b1_128bit };
			  //						    fifo_write_en <= 1'b1; //receiver fifoへのwr
			  fifo_write_en             <= cq_receiving;
			  cq_receiving              <= 1'b1; //パケット受付中 from TX_ENGINE(RQ)
			  //decrement 8DW.
			  wrDWpacket_num_remaining  <= wrDWpacket_num_remaining - 11'd8;
			  bmd_256_rx_state          <= BMD_256_RX_RST; //state continue
		       end
		       
		       //DW0~3, 8~11, ... write 128bit before clk data. Cuz cq is DWord-Aligned.
		       cq_tdata_b1_128bit           <= m_axis_cq_tdata[255:128];
		       m_axis_cq_tready             <= 1'b1; //tready is enabled. //elseはじめで明記しているけど
		    end //
		    
		    else begin
                       bmd_256_rx_state             <= BMD_256_RX_RST;  
		    end // else: !if( wrDWpacket_num_remaining != 11'd0 )					
		 end // if ( memory_access_type == RX_MEM_WR_FMT_TYPE )
	      end // if ( cq_receiving )
	      
	      //cq_sopでもなく、cq_receivingでもないとき
	      else begin
		 fifo_write_en    <= 1'b0; //FIFOへ送信はしない
		 bmd_256_rx_state <= BMD_256_RX_RST; 
	      end // else: !if( cq_receiving )

	   end // case: BMD_256_RX_RST


	   //Reading...
	   BMD_256_RX_MEM_RD32_WT: begin 
	      m_axis_cq_tready         <= 1'b0;
	      //Completer Completionのcompleteを送る処理が完了したらassertされる
	      if ( compl_done_i ) begin
              	 bmd_256_rx_state      <= BMD_256_RX_RST;
	      end
	      else begin
               	 req_compl_o           <= 1'b1;
		 m_axis_cq_tready      <= 1'b0;
              	 bmd_256_rx_state      <= BMD_256_RX_MEM_RD32_WT;
	      end
	   end

	   //Writing...
	   BMD_256_RX_MEM_WR32_WT: begin
	      //			    m_axis_cq_tready <= 1'b0; //もう受け取らないので立ち下げる
	      wrDWpacket_num_remaining <= 11'd0; //次のトランザクションもまた受け取れるように初期化
	      memory_access_type       <= 4'b0000; //initialization
	      fifo_write_en            <= 1'd0; //receiver fifoへのwr終了
	      total_DW_count           <= 11'd0;
	      if ( !wr_busy_i ) begin
               	 bmd_256_rx_state      <= BMD_256_RX_RST;
	      end
	      else begin
              	 bmd_256_rx_state      <= BMD_256_RX_MEM_WR32_WT;
	      end

	   end
	   
	   default : begin
	      bmd_256_rx_state         <= BMD_256_RX_RST;
	   end
	   
	 endcase // case ( bmd_256_rx_state )
      end // else: !if(!rst_n )
   end // always @ ( posedge clk )




   /************************************************************************************************************/
   /************************************************************************************************************/
   //throughput check.
   /************************************************************************************************************/
   /************************************************************************************************************/
   //local regs   
   reg [31:0] throughput_100ms_counter;
   reg [31:0] data_num_counter;

   //local wires
   wire throughput_reset_signal; //データの初期化

   /****************************************************************************************/
   //時間計測. THROUGHPUT_100MSは100msのclk数
   always @ ( posedge clk ) begin            
      if ( !rst_n ) begin
	 throughput_100ms_counter    <= 32'd0;
      end
      //user reset.
      else if( throughput_reset_signal ) begin
	 throughput_100ms_counter    <= 32'd0;
      end
      
      else begin
	 if( THROUGHPUT_100MS == throughput_100ms_counter ) begin
	    throughput_100ms_counter <= 32'd0;
	 end
	 else begin
	    throughput_100ms_counter <= throughput_100ms_counter + 1'b1;
	 end
      end
   end


   /****************************************************************************************/
   //受信データ数カウント。レジスタへの保存。
   always @ ( posedge clk ) begin            
      if ( !rst_n ) begin
	 data_num_counter       <= 32'd0;
      end
      //user reset.
      else if( throughput_reset_signal ) begin
	 data_num_counter       <= 32'd0;
      end

      else begin
	 //every 100ms, initialize and save data to register.
	 if( THROUGHPUT_100MS == throughput_100ms_counter ) begin
	    data_num_counter    <= 32'd0 + cq_receiving; //data数reset.もしこのタイミングでデータが来ていればcount
	 end
	 else begin
	    if( cq_receiving ) begin
	       data_num_counter <= data_num_counter + 1'b1; //data数カウント
	    end
	 end		 
      end
   end
   

   /***************************/
   //vioによるスループット測定
   reg [319:0] throughput_din_vio;      
   wire throughput_data_en;

   vio_check_throughput vio_check_throughput
     (
      .clk( clk ),
      .probe_in0( throughput_din_vio[31:0] ), //32bit
      .probe_in1( throughput_din_vio[63:32] ),
      .probe_in2( throughput_din_vio[95:64] ),
      .probe_in3( throughput_din_vio[127:96] ),
      .probe_in4( throughput_din_vio[159:128] ),
      .probe_in5( throughput_din_vio[191:160] ),
      .probe_in6( throughput_din_vio[223:192] ),
      .probe_in7( throughput_din_vio[255:224] ),
      .probe_in8( throughput_din_vio[287:256] ),
      .probe_in9( throughput_din_vio[319:288] ),

      .probe_out0( throughput_reset_signal ), //1bit //Input from VIO
      .probe_out1( throughput_data_en ) //これを立てたときにだけthroughputが測れる
      );


   /***************************/
   //VIOへのカウンタ情報の入力
   always @ ( posedge clk ) begin            
      if ( !rst_n ) begin
	 throughput_din_vio    <= 320'd0;		  
      end
      else begin
	 if( throughput_data_en && ( THROUGHPUT_100MS == throughput_100ms_counter ) ) begin	    
	    throughput_din_vio <= { throughput_din_vio[287:0], data_num_counter }; //287 = 319-32, shift register.
	 end
      end
   end






   /***************************************************************************************************************************************/
   /***************************************************************************************************************************************/
   //Latency check 
   /***************************************************************************************************************************************/
   /***************************************************************************************************************************************/
   //local regs
   reg bram_reb_d1;

   /***************************/
   //read process of latency BRAM
   /***************************/
   //BRAMからのlatency dataの受信。regへの保存。
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
	 //bram read domain
	 bram_reb           <= 1'b0;
	 bram_reb_d1        <= 1'b0;
	 bram_rd_addr       <= 13'd0;
      end
      //user reset.
      else if( latency_reset_signal ) begin
	 //bram read domain
	 bram_reb           <= 1'b0;
	 bram_reb_d1        <= 1'b0;
	 bram_rd_addr       <= 13'd0;
      end

      else begin
	 //reb(read enable) will be asserted, when 送信側FPGAのみBRAMリードを実行 and receive packet coming. 送信側FPGAでBRAMリードを行う．
	 if( ( m_axis_cq_tdata[31:0] == vio_settings_sender_address_for_sender[31:0] ) && cq_sop ) begin
	    //bram operation
	    bram_reb        <= 1'b1;
	 end
	 //rd_enの次のclkでaddressをincr. cq_sopが立つかに依らずアドレス加算はする（else ifとしない）
	 if( bram_reb ) begin
	    //bram operation
	    if( bram_rd_addr == BRAM_ADDRESS_MAX ) begin
	       bram_rd_addr <= 13'd0;
	    end
	    else begin
	       bram_rd_addr <= bram_rd_addr + 1'b1; //address coorperation
	    end
	    bram_reb        <= 1'b0;
	 end
	 
         bram_reb_d1        <= bram_reb;
      end
   end

   /***************************/
   //Latency save to Virtual I/O
   /***************************/
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d0_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d1_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d2_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d3_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d4_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d5_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d6_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d7_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d8_in_vio;
   reg [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_d9_in_vio;

   reg [13:0] counter_for_latency_store;
   
	always @ ( posedge clk ) begin
		if( latency_reset_signal ) begin
        	latency_d0_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d1_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d2_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d3_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d4_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d5_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d6_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d7_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d8_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };
         	latency_d9_in_vio <= { ECHO_TRANS_COUNTER_WIDTH{1'd0} };

         	counter_for_latency_store <= 14'd0;
      	end
     	else begin
     		/*
        	if( bram_reb_d1 ) begin //dataが来ていて，かつレイテンシ計測通信が止まっていない間
                case( bram_rd_addr )
                  13'd401  : latency_d0_in_vio <= latency_counter - bram_rd_data;
                  13'd1201 : latency_d1_in_vio <= latency_counter - bram_rd_data;
                  13'd2001 : latency_d2_in_vio <= latency_counter - bram_rd_data;
                  13'd2801 : latency_d3_in_vio <= latency_counter - bram_rd_data;
                  13'd3601 : latency_d4_in_vio <= latency_counter - bram_rd_data;
                  13'd4401 : latency_d5_in_vio <= latency_counter - bram_rd_data;
                  13'd5201 : latency_d6_in_vio <= latency_counter - bram_rd_data;
                  13'd6001 : latency_d7_in_vio <= latency_counter - bram_rd_data;
                  13'd6801 : latency_d8_in_vio <= latency_counter - bram_rd_data;
                  13'd7601 : latency_d9_in_vio <= latency_counter - bram_rd_data;
                endcase // case ( bram_rd_addr )
            end // if ( bram_reb_d1 )
            */
            if( cq_sop ) begin
            	case( counter_for_latency_store )
            		14'd401 : latency_d0_in_vio <= latency_counter - m_axis_cq_tdata[167:128]; //受信時の絶対時刻 - 送信時の絶対時刻
                  	14'd2001 : latency_d1_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd3601 : latency_d2_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd5201 : latency_d3_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd6801 : latency_d4_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd8601 : latency_d5_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd10201 : latency_d6_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd11801 : latency_d7_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd13401 : latency_d8_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                  	14'd15001 : latency_d9_in_vio <= latency_counter - m_axis_cq_tdata[167:128];
                endcase

                if( &counter_for_latency_store ) begin
                	counter_for_latency_store <= 14'd0;
                end
                else begin
                	counter_for_latency_store <= counter_for_latency_store + 1'b1;
                end
            end
	 	end // if ( latency_data_en && !FPGA_RECEIVER_SIDE )	 
   	end // always @ ( posedge clk )

   
   //latencyデータを受け取り、結果をここに突っ込む
   vio_check_latency vio_check_latency
     (
      .clk( clk ),
      .probe_in0( latency_d0_in_vio ), //38bit(もともとは64bitだったが，timingの都合でサイズを小さくした．これでも10分以上計測できるから問題なし)
      .probe_in1( latency_d1_in_vio ),
      .probe_in2( latency_d2_in_vio ),
      .probe_in3( latency_d3_in_vio ),
      .probe_in4( latency_d4_in_vio ),
      .probe_in5( latency_d5_in_vio ),
      .probe_in6( latency_d6_in_vio ),
      .probe_in7( latency_d7_in_vio ),
      .probe_in8( latency_d8_in_vio ),
      .probe_in9( latency_d9_in_vio ),
      //	 .probe_in11( guarantee_check ),

      .probe_out0( latency_reset_signal ), //1bit
      .probe_out1( latency_data_en ) //1bit //これを立てたときにだけlatencyが測れる
      );



   /***************************/
   /***************************/
   /*Declaration of vio*/
   /***************************/
   /***************************/
   /*
    //受信データ確認のためのVIOはさんだ
    wire test_en_signal;   
    reg [2:0] bar_id_for_vio;
    reg [5:0] bar_aparture_for_vio;
    reg [31:0] get_wr_data_for_vio;

    
    //inputには見たいdataを入れ, outputにはsoftから入力するための信号をいれる.
    vio_32bit3in_1bit1out vio_for_check_commu
    (
    .clk( clk ),
    .probe_in0( bar_id_for_vio ), //3bit
    .probe_in1( bar_aparture_for_vio ), //6bit
    .probe_out0( test_en_signal ) //1bit. 
    );


    //Completer Requestのdata保持用
    always @( posedge clk ) begin
    if( !rst_n ) begin
    bar_id_for_vio <= 3'd0;
    bar_aparture_for_vio <= 6'd0;
    get_wr_data_for_vio <= 32'd0;
	    end
    else begin
    if( test_en_signal && cq_sop ) begin
    bar_id_for_vio <= m_axis_cq_tdata[114:112];
    bar_aparture_for_vio <= m_axis_cq_tdata[120:115];
    get_wr_data_for_vio <= m_axis_cq_tdata[159:128];
		  end
	    end
      end

    */   

   
   /***************************/
   /*Declaration of ILA*/
   /***************************/
   /*
    ila_RX_ENGINE_check ila_RX_ENGINE_check
    (
    .clk( clk ),
    .probe0( m_axis_cq_tdata ), //256bit
    .probe1( cq_sop ), //1bit
    .probe2( cq_receiving ), //1bit
    .probe3( wrDWpacket_num_remaining ), //11bit
    .probe4( fifo_write_data ), //256bit
    .probe5( cq_tdata_b1_128bit ), //128bit
    .probe6( fifo_write_en ), //1bit
    .probe7( m_axis_cq_tready ), //1bit
    .probe8( m_axis_cq_tkeep ), //8bit
    .probe9( m_axis_cq_tlast ), //1bit
    .probe10( total_DW_count ), //11bit

    .probe11( incr_data ), //10bit
    .probe12( guarantee_check ), //1bit
    .probe13( cq_tlast_d1 ), //1bit
    .probe14( total_DW_count_d1 ) //1bit
    );
    */

   ila_RXENGINE_packetCheck ila_RXENGINE_packetCheck
     (
      .clk( clk ),
      .probe0( m_axis_cq_tdata ), //256bit
      .probe1( cq_sop ), //1bit
//      .probe2( cq_receiving ), //1bit
      .probe2( wrDWpacket_num_remaining ), //11bit
      .probe3( m_axis_cq_tvalid ), //1bit
      .probe4( m_axis_cq_tready ), //1bit
 //     .probe6( m_axis_cq_tkeep ), //8bit
 //     .probe7( m_axis_cq_tlast ), //1bit
      .probe5( total_DW_count ) //11bit
      );
   
endmodule // BMD_256_RX_ENGINE
