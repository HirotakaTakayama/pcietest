module BMD_256_count_wait 
	(
	 input 				      clk, //250MHz
	 input 				      rst_n,
	 input 				      latency_reset_signal,

	 input 				      cq_sop,
	 input [RX_SIDE_WAITING_VALUE - 1:0]  waiting_counter, //最初のパケットを受信してから動き出すカウンタ
		
	 input 				      fifo_counter_read_en, //from TX
	 output [RX_SIDE_WAITING_VALUE - 1:0] fifo_counter_value_out,

	 output reg 			      fifo_read_trigger, //to TX
	 output reg 			      fifo_counter_empty_out,
	 output 			      fifo_counter_empty_wire, 
	 output reg 			      fifo_counter_full_out
	);
   
   localparam RX_SIDE_WAITING_VALUE    = 8'd30; //30bitだと4秒ぐらい

   assign fifo_counter_empty_wire = fifo_counter_empty;
   
   //local regs
   reg fifo_reset_tmp_signal;
   reg fifo_reset_every;

   //local wires
   wire 			     rst_fifo = ( !rst_n || latency_reset_signal || fifo_reset_every );
   wire 			     fifo_counter_full; //送信パケット数とdepthサイズを合わせれば，これが最後に立つ
   wire 			     fifo_counter_empty;
   wire [12:0]			     fifo_count;
   
   //read trigger change
   always @ ( posedge clk ) begin
      if ( !rst_n ) begin
	 fifo_read_trigger <= 1'b0;
      end
      else if( latency_reset_signal ) begin
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


   always @( posedge clk ) begin
      if( !rst_n ) begin
	 fifo_counter_empty_out <= 1'b0; 
      end
      else if( latency_reset_signal ) begin
	 fifo_counter_empty_out <= 1'b0; 
      end
      else begin
	 fifo_counter_empty_out <= fifo_counter_empty;	 
      end
   end // always @ ( posedge clk )

   always @( posedge clk ) begin
      if( !rst_n ) begin
	 fifo_counter_full_out <= 1'b0; 
      end
      else if( latency_reset_signal ) begin
	 fifo_counter_full_out <= 1'b0; 
      end
      else begin
	 fifo_counter_full_out <= fifo_counter_full;	 
      end
   end


   
   //fifo reset信号
   always @( posedge clk ) begin
      if( !rst_n ) begin
	 fifo_reset_every <= 1'b0;
	 fifo_reset_tmp_signal <= 1'b0;	 
      end
      else if( latency_reset_signal ) begin
	 fifo_reset_every <= 1'b0;
	 fifo_reset_tmp_signal <= 1'b0;
      end
      else begin
	 //tmp signalはfullで0にしておき，emptyになったら立てる
	 if( fifo_counter_full ) begin
	    fifo_reset_tmp_signal <= 1'b0;
	 end
	 //1clkだけresetを立てる
	 else if( fifo_counter_empty && !fifo_read_trigger && !fifo_reset_tmp_signal ) begin
	    fifo_reset_every <= 1'b1;
	    fifo_reset_tmp_signal <= 1'b1;
	 end
	 else if( fifo_reset_tmp_signal ) begin
	    fifo_reset_every <= 1'b0;
	 end
      end
   end // always @ ( posedge clk )
   
   
   fifo_generator_0 fifo_30in30out8192depth
     (
      .clk( clk ),
      .srst( rst_fifo ),

      .data_count( fifo_count ), //13bit
      //wr
      .wr_en( cq_sop && !fifo_counter_full ),
      .din( waiting_counter ), //30bit
      
      //rd
      .rd_en( fifo_counter_read_en ), //1bit
      .dout( fifo_counter_value_out ), //30bit //O
      .full( fifo_counter_full ), //1bit //O
      .empty( fifo_counter_empty ) //1bit //O
      );


   
   ila_fifo_check ila_fifo_check 
     (
      .clk( clk ),
      .probe0( cq_sop ), //1bit
      .probe1( waiting_counter ), //30bit
      .probe2( fifo_counter_read_en ), //1bit
      .probe3( fifo_counter_value_out ), //30bit
      .probe4( fifo_counter_full ), //1bit
      .probe5( fifo_counter_empty ), //1bit
      .probe6( fifo_count ) //13bit
      );
   
   
endmodule
