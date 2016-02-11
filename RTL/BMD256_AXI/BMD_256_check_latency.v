//-----------------------------------------------------------------------------
// Title         : latency check
// Project       : PCIe communication
//-----------------------------------------------------------------------------
// File          : BMD_256_check_latency.v
// Author        : Takayama
// Created       : 16.12.2014
// Last modified : 16.12.2014
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Copyright (c) 2014 by  This model is the confidential and
// proprietary property of  and the possession or use of this
// file requires a written license from .
//------------------------------------------------------------------------------
// Modification history :
// 16.12.2014 : created
//-----------------------------------------------------------------------------


module BMD_256_check_latency
       (
		input clk, //250MHz
		input rst_n,
		input latency_reset_signal, //comes from RX_ENGINE, user reset.
		input [ECHO_TRANS_COUNTER_WIDTH - 1:0] latency_counter, //48bit

		//write domain, comes from TX_ENGINE
		input bram_wea, //write enable
		input bram_ena,
		input [12:0] bram_wr_addr, //write address
		input [ECHO_TRANS_COUNTER_WIDTH - 1:0] bram_wr_data, //48bit

		//read domain, comes from RX_ENGINE
		input bram_reb, //read enable
		input [12:0] bram_rd_addr,
		//read domain send to RX_ENGINE
		output [ECHO_TRANS_COUNTER_WIDTH - 1:0] bram_rd_data
       );

       localparam ECHO_TRANS_COUNTER_WIDTH = 8'd40; //レイテンシ測定（echo転送）時のカウンタサイズ設定

      /******************************************************/
      //depth1024, write first, Simple Dual-port Block RAM. not common clock.
      //Port A:Write First, Always Enable. B:Write First, Use ENB Pin and RSTB Pin
      //Port B Reset Priority:SR( Set Reset ). CEだとenb信号が必要
      //Total Port B Read Latency : 2 Clock Cycle
      /******************************************************/
      //これの目的はデータのレイテンシをはかること。データにはカウントスタートの値を入れる。
      //AがWriteポート，BがReadポート．Read側は値リセット可能
      //データの整合性はデータを片っ端からincrしていったのを比較すればいいだけのことである。
      //8192depth
      blk_mem_check_latency blk_mem_check_latency
	(
	 .clka( clk ),
	 .ena( bram_ena ),
	 .wea( bram_wea ), //write enable port A.
	 .addra( bram_wr_addr ), //13bit //write address
	 .dina( bram_wr_data[ECHO_TRANS_COUNTER_WIDTH - 1:0] ), //40bit

	 .clkb( clk ),
	 .rstb( latency_reset_signal ), //reset around latency by VIO
	 .enb( bram_reb ), /*sender FPGAのRX_ENGINEにデータが来たとき*/
	 .addrb( bram_rd_addr ), //13bit //read address /* まだrdしていない番地。*/
	 .doutb( bram_rd_data[ECHO_TRANS_COUNTER_WIDTH - 1:0] ) //40bit
	 );


     

endmodule
