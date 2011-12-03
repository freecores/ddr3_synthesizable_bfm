/* 
*DDR3 Simple Synthesizable Memory BFM
*2010-2011 sclai <laikos@yahoo.com>
*
*This library is free software; you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published by 
* the Free Software Foundation; either version 2.1 of the License, 
* or (at your option) any later version.
* 
* This library is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
* USA
*
*
*  simple implementation of DDR3 Memory
*  will only reponse to write and read request
*  parameter
*  count start from t0,t2,t2...
*  ck _|-|_|-|_|-|_|-|_
* 
*  cs#---|___|---------
*
*        |   |    |
*        t0  t1  t2 ....
*
*2/12/2011: not able to handle multiple bank opening
*2/12/2011: not using DM signals
*
*/

`timescale 1ps / 1ps

module ddr3_simple4#(
parameter MEM_DQ_WIDTH 		=8,
parameter MEM_BA_WIDTH 		=3,
parameter MEM_ROW_WIDTH 	=13,
parameter MEM_COL_WIDTH		=13,
parameter MEM_TRTP 			=6,
parameter MEM_TRCD 			=11,
parameter MEM_TWL				=8,
parameter MEM_TRL				=6,
parameter MEM_ACT_CMD		=7 //activates to command
)(
input wire [MEM_ROW_WIDTH-1:0]	a,
input wire [ MEM_BA_WIDTH-1:0]	ba,
input wire								ck,
input wire 								ck_n,
input wire 								cke,
input wire								cs_n,
input wire								dm,     
input wire								ras_n,   
input wire								cas_n,   
input wire								we_n,    
input wire								reset_n, 
inout wire	[MEM_DQ_WIDTH-1:0]	dq,      
inout wire								dqs,     
inout wire								dqs_n,   
input wire								odt      
);


//definitions
localparam	   OPCODE_PRECHARGE 		= 4'b0010;
localparam		OPCODE_ACTIVATE 		= 4'b0011;
localparam		OPCODE_WRITE 		  	= 4'b0100;
localparam		OPCODE_READ 		   = 4'b0101;
localparam		OPCODE_MRS 			   = 4'b0000;
localparam		OPCODE_REFRESH 	 	= 4'b0001;
localparam		OPCODE_DES 			   = 4'b1000;
localparam		OPCODE_ZQC 			   = 4'b0110;
localparam		OPCODE_NOP 			   = 4'b0111;

localparam  BL8 = 1'b1;
localparam  BC4 = 1'b0;


//mode registers
reg [31:0] mr0;
reg [31:0] mr2;
reg [31:0] mr3;


wire [35:0] write_add;
wire [35:0] read_add;
wire [3:0]  write_cmd;
wire [3:0]  read_cmd;
(* keep *)wire [(MEM_DQ_WIDTH*2)-1:0] read_data;

reg [ 2:0] last_bank;
reg [15:0] last_row;
reg [3:0] last_write_cmd;
reg [3:0] last_read_cmd;
reg [35:0] last_write_add;
reg [35:0] last_read_add;

reg        write_address12;
reg        read_address12;

//bank tracker
reg [MEM_ROW_WIDTH-1:0]opened_row[(2**MEM_BA_WIDTH)-1:0]; 
//row  tracker

wire [MEM_DQ_WIDTH-1:0]  dq_out;
reg  [MEM_DQ_WIDTH-1:0]  dq_in0;

(* keep *) wire [MEM_DQ_WIDTH-1:0] data_hi; 
(* keep *) wire [MEM_DQ_WIDTH-1:0] data_lo;
(* keep *) wire						  data_hi_dm; 
(* keep *) wire 						  data_lo_dm;
//IDDR
my_iddrx8 iddrx8_inst(
	.clk(ck),
	.io(dq),
	.d0(data_lo),
	.d1(data_hi)
);

my_iddrx8 iddrx8_dm_inst(
	.clk(ck),
	.io(dm),
	.d0(data_lo_dm),
	.d1(data_hi_dm)
);

//ODDR
my_oddrx8 oddrx8_inst(
.clk(ck),
.d0(read_data[ MEM_DQ_WIDTH-1:0              ]),
.d1(read_data[(MEM_DQ_WIDTH*2)-1:MEM_DQ_WIDTH]),
.io(dq_out)
);

//double data rate
always @(posedge ck )
begin
if(reset_n==1'b0)
	begin	
		last_bank     <=4'h0;
		last_row      <=16'h0000;
	end
else
begin
	case({cs_n,ras_n,cas_n,we_n})
	/*
	OPCODE_PRECHARGE	:begin
								$display("t=%d,PRECHARGE",vip_clk);
							end
	*/
	OPCODE_ACTIVATE  	:begin								
								opened_row [ba] <={{(16-MEM_ROW_WIDTH){1'b0}},a[MEM_ROW_WIDTH-1:0]};
							end
	/*						
	OPCODE_DES			:begin
								$display("t=%d,DES",vip_clk);
							end
	OPCODE_MRS  		:begin
								$display("t=%d,MRS",vip_clk);
							end
	OPCODE_NOP  		:begin
								//$display("t=%d,NOP",vip_clk);
							end
	*/
	OPCODE_READ  		:begin							
								last_read_add 	<={ba,opened_row[ba],{{(16-MEM_COL_WIDTH){1'b0}},a[MEM_COL_WIDTH-1:0]}};								
								last_read_cmd 	<=OPCODE_READ;
							end
	OPCODE_WRITE  		:begin
								last_write_add  <={ba,opened_row[ba],{{(16-MEM_COL_WIDTH){1'b0}},a[MEM_COL_WIDTH-1:0]}};
								last_write_cmd  <=OPCODE_WRITE;
							end
							/*
	OPCODE_ZQC			:begin
								$display("t=%d,ZQC",vip_clk);
							end*/
		default:begin
				last_read_cmd 	<=OPCODE_NOP;
				last_write_cmd <=OPCODE_NOP;
				end
	endcase
end // end reset	
end // end always@(*)



//cmd
//read
ddr3_sr4 #(
.PIPE_LEN(MEM_TRL)
)ddr3_read_cmd_sr(
	.clk(ck),
	.shift_in(last_read_cmd),
	.shift_out(read_cmd)
);
//bank, row, col
ddr3_sr36 #(
.PIPE_LEN(MEM_TRL+1)
)ddr3_read_add_sr(
	.clk(ck),
	.shift_in(last_read_add),
	.shift_out(read_add)
);

//cmd
//write
ddr3_sr4#(
.PIPE_LEN(MEM_TWL)
)ddr3_write_cmd_sr(
	.clk(ck),
	.shift_in(last_write_cmd),
	.shift_out(write_cmd)
);

//bank, row, col
ddr3_sr36#(
.PIPE_LEN(MEM_TWL+1) //have to be a cycle late to wait for IDDR latency
) ddr3_write_add_sr(
	.clk(ck),
	.shift_in(last_write_add),
	.shift_out(write_add)
);


//write fsm
localparam WR_D0	=4'd0;
localparam WR_D1	=4'd1;
localparam WR_D2	=4'd2;
localparam WR_D3  =4'd3;
localparam WR_IDLE=4'd5;
reg [3:0] write_state;
reg 		 mem_we;
reg [2:0] write_col;
always@(posedge ck)
begin
	if(reset_n==1'b0)
		begin
			write_state<=WR_IDLE;
			mem_we<=1'b0;
			write_col<=0;
		end
	else
		begin
		case(write_state)
			WR_IDLE:begin
			write_col<=0;
			if(write_cmd==OPCODE_WRITE)
				begin				
					write_state<=WR_D0;
					mem_we<=1'b1;
				end
			else
				begin
					write_state<=WR_IDLE;	
					mem_we<=1'b0;				
				end
			end
			WR_D0:begin
				write_address12<=write_add[12];
				write_state<=WR_D1;
				write_col<=write_col+1'b1;
				$display("%m: at time %t\tWRITE BANK[%x]\tROW[%x]\tCOL[%x]\tWR D0: %x-%x",$time,write_add[34:32],write_add[31:16],write_add[15:0],data_hi,data_lo);
			end
			WR_D1:begin				
				if(write_address12==1'b1)
					begin
						write_state<=WR_D2;
						write_col<=write_col+1'b1;
					end
				else if (write_cmd==OPCODE_WRITE)
					begin
						write_state<=WR_D0;
						write_col<=0;
					end
				else
					begin
						write_state<=WR_IDLE;
						mem_we<=1'b0;
					end
				$display("%m: at time %t\tWRITE BANK[%x]\tROW[%x]\tCOL[%x]\tWR D1: %x-%x",$time,write_add[34:32],write_add[31:16],write_add[15:0],data_hi,data_lo);				
			end
			WR_D2:begin				
				write_state<=WR_D3;
				write_col<=write_col+1'b1;	
				$display("%m: at time %t\tWRITE BANK[%x]\tROW[%x]\tCOL[%x]\tWR D2: %x-%x",$time,write_add[34:32],write_add[31:16],write_add[15:0],data_hi,data_lo);			
			end
			WR_D3:begin
				$display("%m: at time %t\tWRITE BANK[%x]\tROW[%x]\tCOL[%x]\tWR D3: %x-%x",$time,write_add[34:32],write_add[31:16],write_add[15:0],data_hi,data_lo);
				
				//write_col<=write_col+1'b1;	
				if (write_cmd==OPCODE_WRITE)
					begin
						write_state<=WR_D0;
						write_col<=0;
					end
				else
					begin
						write_state<=WR_IDLE;
						mem_we<=1'b0;
					end
			end
		endcase
		end //endif
end


//read fsm
localparam RD_D0	=4'd0;
localparam RD_D1	=4'd1;
localparam RD_D2	=4'd2;
localparam RD_D3  =4'd3;
localparam RD_IDLE=4'd5;

reg [3:0] read_state;
reg [2:0] read_col;
reg		 send_dq;
reg		 send_dqs0;
reg		 send_dqs1;

always@(posedge ck)
begin
	if(reset_n==1'b0)
		begin
			read_state<=RD_IDLE;
			read_col	 <=0;
			send_dq	 <=0;	
		end
	else
		begin
			case(read_state)
			RD_IDLE:begin
			read_col<=0;
			send_dq<=0;
			if(read_cmd==OPCODE_READ)
				begin				
					read_state<=RD_D0;																
				end
			else
				begin
					read_state<=RD_IDLE;					
				end
			end
			RD_D0:begin
				read_address12<=read_add[12];
				read_state<=RD_D1;
				read_col<=read_col+1'b1;
				send_dq	 <=1'b1;
				$display("%m: at time %t\tREAD BANK[%x]\tROW[%x]\tCOL[%x]\tRD D0",$time,read_add[34:32],read_add[31:16],read_add[15:0]);
			end
			RD_D1:begin			
				if(read_address12==1'b1)
					begin
						read_state<=RD_D2;
						read_col<=read_col+1'b1;
					end
				else if (read_cmd==OPCODE_READ)
					begin
						read_state<=RD_D0;
						read_col<=0;
						send_dq	 <=1'b1;
					end
				else
					begin
						read_state<=RD_IDLE;
						//send_dq	 <=1'b0;
					end
				$display("%m: at time %t\tREAD BANK[%x]\tROW[%x]\tCOL[%x]\tRD D1",$time,read_add[34:32],read_add[31:16],read_add[15:0]);
			end
			RD_D2:begin				
				read_state<=RD_D3;
				read_col<=read_col+1'b1;
				send_dq	 <=1'b1;				
				$display("%m: at time %t\tREAD BANK[%x]\tROW[%x]\tCOL[%x]\tRD D2",$time,read_add[34:32],read_add[31:16],read_add[15:0]);
			end
			RD_D3:begin
				//write_col<=write_col+1'b1;	
				if (read_cmd==OPCODE_READ)
					begin
						read_state<=RD_D0;
						read_col<=0;
						send_dq	 <=1'b1;
					end
				else
					begin
						read_state<=RD_IDLE;
						read_col<=0;
						//send_dq	 <=1'b0;
					end
					$display("%m: at time %t\tREAD BANK[%x]\tROW[%x]\tCOL[%x]\tRD D3",$time,read_add[34:32],read_add[31:16],read_add[15:0]);
			end
			endcase
		end
		
end //end always

//dqs fsm
always @(posedge ck_n)
begin
if(reset_n==1'b0)
	begin
		send_dqs1<=0;
		send_dqs0<=0;
	end
else
	begin
		if(read_cmd==OPCODE_READ) 
			begin
				send_dqs1<=1'b1;
			end
		else
			begin
				send_dqs1<=1'b0;
			end
	end
send_dqs0<=send_dqs1;
end//end always

//ram here
dport_ram  #(
	.DATA_WIDTH(8), //data_hi,data_lo
	.ADDR_WIDTH(36)
)dport_ram_hi(
	.clk			(ck),
	.di			(data_hi),
	.read_addr	(read_add+read_col), 
	.write_addr (write_add+write_col),
	.we			(mem_we & data_hi_dm), 
	.do			(read_data[15:8])
);

dport_ram  #(
	.DATA_WIDTH(8), //data_hi,data_lo
	.ADDR_WIDTH(36)
)dport_ram_lo(
	.clk			(ck),
	.di			(data_lo),
	.read_addr	(read_add+read_col), 
	.write_addr (write_add+write_col),
	.we			(mem_we & data_lo_dm), 
	.do			(read_data[7:0])
);
assign dqs  =((send_dqs0==1'b1) || (send_dq==1'b1))?ck:1'bz;
assign dqs_n=((send_dqs0==1'b1) || (send_dq==1'b1))?ck_n:1'bz;
assign dq   = (send_dq==1'b1)?dq_out:8'hZZ;

/* utility functions to display information
*/
initial begin
        $timeformat (-9, 1, " ns", 1);
      end
      
always @(posedge ck )
begin
	case({cs_n,ras_n,cas_n,we_n})

	OPCODE_PRECHARGE	:begin
								$display("%m: at time %t PRECHARGE ",$time);
							end
	
	OPCODE_ACTIVATE  	:begin
								$display("%m: at time %t ACTIVATE - BANK[%x]\tROW[%x]",$time,ba,a);
							end
						
	OPCODE_DES			:begin
								$display("%m: at time %t DES ",$time);
							end
	OPCODE_MRS  		:begin
								$display("%m: at time %t MRS ",$time);
							end
	/*OPCODE_NOP  		:begin
								/$display("%m: at time %t WRITE ",$time);
							end
	*/
	/*
	OPCODE_READ  		:begin
								$display("%m: at time %t READ - BANK[%x]\tROW[%x]\tCOL[%x]",$time,ba,last_row,a);
							end
	OPCODE_WRITE  		:begin
								$display("%m: at time %t WRITE - BANK[%x]\tROW[%x],\tCOL[%x]",$time,ba,last_row,a);
							end
	*/						
	OPCODE_ZQC			:begin
								$display("%m: at time %t ZQC ",$time);
							end
	endcase

end // end always@(*)
/* end utility*/

endmodule
