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
		input [63:0] latency_counter, //64bit

		//write domain, comes from TX_ENGINE
		input bram_wea, //write enable
		input [9:0] bram_wr_addr, //write address
		input [63:0] bram_wr_data, //64bit

		//read domain, comes from RX_ENGINE
		input bram_reb, //read enable
		input [9:0] bram_rd_addr,
		//read domain send to RX_ENGINE
		output [63:0] bram_rd_data
       );



      /***************************/
      //depth1024, write first, Simple Dual-port Block RAM.
      //A:Always Enable, B:Use ENB Pin and RSTB Pin
      //Total Port B Read Latency : 2 Clock Cycle
      /***************************/
      //これの目的はデータのレイテンシをはかること。データにはカウントスタートの値を入れる。
      //AがWriteポート，BがReadポート．Read側は値リセット可能
      //データの整合性はデータを片っ端からincrしていったのを比較すればいいだけのことである。

      blk_mem_check_latency blk_mem_check_latency
	(
	 .clka( clk ),
	 .wea( bram_wea ), //write enable port A.
	 .addra( bram_wr_addr[9:0] ), //10bit //write address
	 .dina( bram_wr_data[63:0] ), //64bit

	 .clkb( clk ),
	 .rstb( latency_reset_signal ), //reset around latency by VIO
	 .enb( bram_reb ), /*sender FPGAのRX_ENGINEにデータが来たとき*/
	 .addrb( bram_rd_addr[9:0] ), //10bit //read address /* まだrdしていない番地。*/
	 .doutb( bram_rd_data[63:0] ) //64bit
	 );


      ila_check_bram_access ila_check_bram_access
		(
			.clk( clk ),
			.probe0( bram_wea ), //1bit
      		.probe1( bram_wr_addr[9:0] ), //10bit
      		.probe2( bram_wr_data[63:0] ), //64bit
      		.probe3( bram_reb ), //1bit
      		.probe4( bram_rd_addr[9:0] ), //10bit
      		.probe5( bram_rd_data[63:0] ), //64bit
      		.probe6( latency_counter[63:0] ) //64bit
		);

endmodule