module BMD_256_count_wait 
	(
	 input 				      clk, //250MHz
	 input 				      rst_n,
	 input 				      latency_reset_signal,

	 input 				      cq_sop,
	 input [RX_SIDE_WAITING_VALUE - 1:0]  waiting_counter, //最初のパケットを受信してから動き出すカウンタ
		
	 input 				      fifo_counter_read_en, //from TX
	 output [RX_SIDE_WAITING_VALUE - 1:0] fifo_counter_value_out,

	 output reg 			      fifo_read_trigger //to TX
	);
   
   localparam RX_SIDE_WAITING_VALUE    = 8'd30; //30bitだと4秒ぐらい

   reg 					      cq_sop_reg;   
   reg [RX_SIDE_WAITING_VALUE - 1:0] waiting_counter_reg;
   
	   
   wire 			     rst_fifo = ( !rst_n || latency_reset_signal );
   wire 			     fifo_counter_full; //送信パケット数とdepthサイズを合わせれば，これが最後に立つ
   wire 			     fifo_counter_empty;
   
   //read trigger change
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
	 fifo_read_trigger <= 1'b0;
      end
      if( latency_reset_signal ) begin
	 fifo_read_trigger <= 1'b0;
      end
      else begin
    	 if( fifo_counter_full ) begin
    	    fifo_read_trigger <= 1'b1;
    	 end
    	 else if( fifo_counter_empty ) begin
    	    fifo_read_trigger <= 1'b0;
    	 end
      end
   end // always @ ( posedge clk )


   
   //入力前のbuffer
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
	 cq_sop_reg          <= 1'b0;
	 waiting_counter_reg <= { RX_SIDE_WAITING_VALUE{1'b0} };	 
      end
      else begin
	 cq_sop_reg <= cq_sop;
	 waiting_counter_reg <= waiting_counter;	 
      end
   end  


   fifo_generator_0 fifo_30in30out8192depth
     (
      .clk( clk ),
      .srst( rst_fifo ), 
      //wr
      .wr_en( cq_sop_reg && !fifo_counter_full ),
      .din( waiting_counter_reg ), //30bit
      
      //rd
      .rd_en( fifo_counter_read_en ), //1bit
      .dout( fifo_counter_value_out ), //30bit //O
      .full( fifo_counter_full ), //1bit //O
      .empty( fifo_counter_empty ) //1bit //O
      );


   /*
        ila_fifo_check ila_fifo_check (
            .clk( clk ),
            .probe0( cq_sop ), //1bit
            .probe1( waiting_counter ), //38bit
            .probe2( fifo_counter_read_en ), //1bit
            .probe3( fifo_counter_value_out ), //38bit
            .probe4( fifo_counter_full ), //1bit
            .probe5( fifo_counter_empty ) //1bit
            );
    */
   
endmodule
