module BMD_256_count_wait 
	(
		input clk, //250MHz
		input rst_n,

		input [63:0] waiting_counter, //最初のパケットを受信してから動き出すカウンタ
		input cq_sop //パケット到着時に立つ信号をfifoへのen信号にする（cq_sopかな）
	)


	//ToDO: fifoの書き込み，読み出し機能の追加．fifoの作成．送信パケット（TX）に読み出し値を渡す回路作成．継続してechoをする機構作成（現時点ではtriggerが立ち下がらない限りカウントし続ける）送信側パケットが一定数送られたことを検知する必要もある
	fifo_64in64out16384depth
		(
			.clk( clk ),

			.( cq_sop ), //1bit
			.( waiting_counter ), //64bit
		); 


endmodule
