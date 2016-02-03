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
		input [47:0] latency_counter, //48bit

		//write domain, comes from TX_ENGINE
		input bram_wea, //write enable
		input [12:0] bram_wr_addr, //write address
		input [47:0] bram_wr_data, //48bit

		//read domain, comes from RX_ENGINE
		input bram_reb, //read enable
		input [12:0] bram_rd_addr,
		//read domain send to RX_ENGINE
		output [47:0] bram_rd_data
       );


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
	 .wea( bram_wea ), //write enable port A.
	 .addra( bram_wr_addr ), //13bit //write address
	 .dina( bram_wr_data[47:0] ), //48bit

	 .clkb( clk ),
	 .rstb( latency_reset_signal ), //reset around latency by VIO
	 .enb( bram_reb ), /*sender FPGAのRX_ENGINEにデータが来たとき*/
	 .addrb( bram_rd_addr ), //13bit //read address /* まだrdしていない番地。*/
	 .doutb( bram_rd_data[47:0] ) //48bit
	 );


      ila_check_bram_access ila_check_bram_access
		(
			.clk( clk ),
			.probe0( bram_wea ), //1bit
      		.probe1( bram_wr_addr ), //13bit
      		.probe2( bram_wr_data[47:0] ), //48bit
      		.probe3( bram_reb ), //1bit
      		.probe4( bram_rd_addr ), //13bit
      		.probe5( bram_rd_data[47:0] ), //48bit
      		.probe6( latency_counter[47:0] ) //48bit
		);

endmodule
