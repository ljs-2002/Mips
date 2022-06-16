`timescale 10ns / 1ns
`define DATA_WIDTH 32
`define ADDR_WIDTH 5
`define ADD 3'b010 
`define SLT 3'b111
`define SLL 2'b00
`define ZERO 8'b0

module top_module(
	input  rst,
	input  clk,

	output reg [31:0] PC,
	input  [31:0] Instruction,

	output [31:0] Address,
	output MemWrite,
	output [31:0] Write_data,
	output [3:0] Write_strb,

	input  [31:0] Read_data,
	output MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW
	wire [5:0]  	opcode;
	wire [4:0]  	rs;
	wire [4:0]  	rt;
	wire [4:0]  	rd;
	wire [4:0]  	shamt;
	wire [5:0]  	func;
	wire [25:0] 	instr_index;
	wire [15:0] 	imm;
	wire [2:0]      ALUop;
	wire [4:0]		ShiftB;
	wire [1:0]      Shiftop;
	wire [4:0]		shiftime;
	wire [15:0]		extend;	
	wire [31:0]		load_data,mask;
	wire [31:0]     extend_imm,ShiftA;
	wire [31:0]     ShiftResult;
	wire [31:0]		A,B,rdataA,rdataB,ALUresult;
	wire [31:0]		PCReg,PCDefault,PCBranch;
	wire 			Overflow,CarryOut,Zero;
	wire 			RType,IType,JType,REGIMM,Jump_Reg,Mov;
	wire 			RegDst,Branch,Jump,MemToReg,ALUSrc,RegWrite;

	//Change to define 
	//======================================================================
	localparam ONE = 32'hffffffff;
	//======================================================================
	//IF
	//======================================================================
		always @(posedge clk, posedge rst) begin
			if(rst) begin
				PC <= 32'b0;
			end
			else begin
				PC<=PCReg;
			end
		end
	//======================================================================
	//ID
	//======================================================================
	
		//指令各部分含义
		assign opcode =       	Instruction[31:26];
		assign rs = 			Instruction[25:21];
		assign rt = 			Instruction[20:16];
		assign rd = 			Instruction[15:11];
		assign shamt = 			Instruction[10:6];
		assign func = 			Instruction[5:0];
		assign instr_index =	Instruction[25:0];
		assign imm = 			Instruction[15:0];

		//判断是何种类型指令
		assign RType = 			~|opcode;
		assign REGIMM =   		(~|opcode[5:1]) & opcode[0];
		assign JType =          (~|opcode[5:2]) & opcode[1];
		assign IType =          |opcode[5:2];
		assign Mov = 			RType&(func[5:1]==5'b00101);
		assign Jump_Reg = 		(JType&opcode[0]) | (RType&(func[0]&func[3])&(!func[2:1]));	
		//ALUop
		assign ALUop = (RType)?										//R-Type
			(func[5])?												//运算指令
				{
					func[1]^(func[3]&func[0]),
					~func[2],
					func[3]|func[2]&func[0]
				}
			:
				`ADD
		:(IType)?													//I-Type
			(opcode[5:3]==4'b001)?									//运算指令
				{
					opcode[1]&(opcode[2]|(~opcode[0])),
					~opcode[2],
					~opcode[2]&opcode[1]|opcode[2]&opcode[0]
				}
			:(opcode[5:2]==4'b0001)?								//分支指令
				{2'b11,opcode[1]}
			:
				`ADD												//读写指令及其他
		:(REGIMM)?													
			`SLT													//REGIMM
		:`ADD;														//J-Type,aluop default

		//Shiftop
		assign Shiftop = 		(RType&(!func[5:3]))?func[1:0]:`SLL;

		//RegDst
		assign RegDst = 		RType|REGIMM|JType|(IType&(!opcode[5:3]));

		//Branch
		assign Branch = 		REGIMM|(IType&(!opcode[5:3]));

		//Jump
		assign Jump = 			JType | (RType&(!func[5:4])&func[3]&(!func[1]));

		//MemRead(in MEM)
		//assign MemRead = 		opcode[5]&(~opcode[3]);

		//MemToReg(in WB)
		assign MemToReg =  		MemRead;

		//MemWrite(in MEM)
		//assign MemWrite = 		opcode[5]&opcode[3];
		
		//ALUSrc
		assign ALUSrc = 		~RegDst;

		//RegWrite
		assign RegWrite = 		(RType&(~(func[3]&(!func[2:0])))) | //除了jr
						  		(IType&(^opcode[5:3])) |
								(JType&(&opcode[1:0]));				//jal
		
		//extend_imm
		assign extend = ((opcode[5:2]==4'b0011)&(~(&opcode[1:0])))?16'b0:16'hffff;//andi, ori,xori需要0扩展
		assign extend_imm = 	{{16{imm[15]&(IType)}}&extend,imm};

	//======================================================================
	//EX
	//======================================================================
		//Register read
		reg_file REG(.clk(clk),.rst(rst),
					 .waddr(RF_waddr),.raddr1(rs),.raddr2(rt),
					 .wen(RF_wen),
					 .wdata(RF_wdata),.rdata1(rdataA),.rdata2(rdataB));
		
		//ALU
		assign A = 	(Mov)?32'b0:rdataA;
		assign B = 	(ALUSrc)?extend_imm:(REGIMM)?32'b0:rdataB;
		alu ALU(.A(A),.B(B),
				.ALUop(ALUop),
				.Overflow(Overflow),.CarryOut(CarryOut),.Zero(Zero),
				.Result(ALUresult));
		
		//Shifter
		assign ShiftA = 	(&opcode[3:0])?extend_imm:rdataB;
		assign ShiftB = 	(&opcode[3:0])?5'd16:(!shamt)?rdataA[4:0]:shamt;
		shifter Shift(.A(ShiftA),.B(ShiftB),
					  .Shiftop(Shiftop),.Result(ShiftResult));

		//PC
		//此处出现回环
		assign PCDefault = 	PC + 32'b100;
		assign PCBranch = 	PC + (extend_imm<<2) + 4;
		assign PCReg = 
		(rst)?
			32'b0
		:(Jump)?
			(JType)?
				{PC[31:28],instr_index,2'b00}
			:
				rdataA
		:(Branch)?
			(REGIMM)?
				(~(Zero^rt[0]))?PCBranch:PCDefault
			:(opcode[1])?
				(((~Zero)|(~|rdataA))^opcode[0])?PCBranch:PCDefault
			:
				(Zero^opcode[0]^(~opcode[2]))?PCBranch:PCDefault
		:
			PCDefault;
		
	//======================================================================
	//MEM
	//======================================================================
		wire [`DATA_WIDTH-1:0] 	sh_sb_data;
		wire [3:0]				sh_sb_strb;
		assign MemWrite = 		opcode[5]&opcode[3];
		assign MemRead = 		opcode[5]&(~opcode[3]);
		assign Address = 		{ALUresult[31:2],2'b0};
		assign sh_sb_data = 	rdataB<<{ALUresult[1:0],3'b0};
		//Write_data
		assign Write_data = 
		(~opcode[1])? 								
			sh_sb_data								//sh and sb
		:(~opcode[0]&opcode[1])? 					//swl and swr
			(opcode[2])?
				rdataB<<{ALUresult[1:0],3'b0}		//swr
			:
				rdataB>>{~ALUresult[1:0],3'b0}		//swl
		:
			rdataB									//sw
		;
		//======================================================================
		//Write_strb
		assign sh_sb_strb = 	{2'b0,opcode[0],1'b1}<<ALUresult[1:0];
		assign Write_strb =
		(~opcode[1])?													//sh and sb
			sh_sb_strb
		:(~opcode[0]&opcode[1])? 										//swl and swr
			(opcode[2])?
				{1'b1,~(|ALUresult[1:0]),~ALUresult[1],!ALUresult[1:0]}	//swr
			:
				{&ALUresult[1:0],ALUresult[1],|ALUresult[1:0],1'b1}		//swl
		:
			4'b1111;													//sw
	//======================================================================
	//WB
	//======================================================================
		//load data
		wire [4:0] 	lhb_lshift,lhb_rshift;
		wire [31:0] sign,lRead_data;
		assign lhb_lshift = {~ALUresult[1],~(opcode[0]|ALUresult[0]),3'b0};
		assign lhb_rshift = {1'b1,~opcode[0],3'b0};
		assign lRead_data = Read_data<<lhb_lshift;
		assign sign       = {{16{~opcode[2]}},{8{~(opcode[0]|opcode[2])}},8'h00}
							&{32{lRead_data[31]}};
		assign load_data  = 
		(opcode[1])?
			(opcode[0])?
				Read_data													//lw
			:(opcode[2])?													
				(Read_data&mask)>>shiftime | (rdataB&(~(mask>>shiftime)))	//lwr
			:																
				(Read_data&mask)<<shiftime | (rdataB&(~(mask<<shiftime)))	//lwl
		:
			((lRead_data)>>lhb_rshift)|sign			//lh,lhu,lb,lbu
		;
		//======================================================================
		//load mask
		// opcode[2] 指示lwl和lwr
		assign shiftime = {{2{~opcode[2]}}^ALUresult[1:0],3'b0};
		assign mask = (ONE<<shiftime)>>(shiftime&{32{~opcode[2]}});
		//======================================================================
		//Register write
		wire   shift_lui;	
		assign RF_wen   = 	RegWrite&(~Mov|(Zero^func[0]));
		assign RF_waddr = 	(Jump_Reg)?5'd31:(RegDst)?rd:rt;
		assign shift_lui=	RType&(!func[5:3])|(&opcode[3:0]);
		assign RF_wdata = 	(MemToReg)?
			load_data		//从Mem取数
		:(shift_lui)?
			ShiftResult		//移位操作
		:(Jump_Reg)?
			(PC+8)			//jal,jalr
		:(Mov)?
			rdataA			//movn,movz
		:
			ALUresult		//default
		;
endmodule


module reg_file(
	input clk,
	input rst,
	input [`ADDR_WIDTH - 1:0] waddr,
	input [`ADDR_WIDTH - 1:0] raddr1,
	input [`ADDR_WIDTH - 1:0] raddr2,
	input wen,
	input [`DATA_WIDTH - 1:0] wdata,
	output [`DATA_WIDTH - 1:0] rdata1,
	output [`DATA_WIDTH - 1:0] rdata2
);

	// TODO: Please add your logic code here
	reg [`DATA_WIDTH-1:0] regM [0:31];
	integer i;
	always @(posedge clk or posedge rst) begin
		regM[0]=32'b0;
		if(wen && waddr!=5'b0) begin
			regM[waddr]<=wdata;
		end
	end

	assign rdata1=regM[raddr1];
	assign rdata2=regM[raddr2];

endmodule


module alu(
	input [`DATA_WIDTH - 1:0] A,
	input [`DATA_WIDTH - 1:0] B,
	input [2:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output [`DATA_WIDTH - 1:0] Result
);
	wire [`DATA_WIDTH - 1:0]   SUB;
	wire [`DATA_WIDTH - 1:0]   result_AND_OR;
	wire [`DATA_WIDTH - 1:0]   result_add;
	wire [`DATA_WIDTH - 1:0]   result_xor_nor;
	wire 					   temp_carry;
	wire 					   result_compare;
	wire 					   result_u_compare;
	wire 					   Asign,Bsign,R_sign;
	wire                       XNOR,AND,ANDOR,CMP,UCMP;
	/* ALUop:
	* 000 AND 001 OR
	* 010 ADD 110 SUB
	* 111 SLT 011 SLTU
	* 100 XOR 101 NOR
	*/
	//加减法辅助信号
	assign SUB = 	{32{(&ALUop[2:1])|(&ALUop[1:0])}}; //是否执行减法
	assign Asign= 	A[`DATA_WIDTH - 1];				   //A的符号位
	assign Bsign= 	B[`DATA_WIDTH - 1]^ ALUop[2];	   //B的符号位
	assign Rsign= 	result_add[`DATA_WIDTH - 1];	   //结果的符号位
	//=============================================================================
	//判断送出哪种结果的信号
	assign CMP= 	&ALUop; 						   //有符号比较
	assign AND= 	~ALUop[0]; 						   //与
	assign XNOR= 	ALUop[2]&(~ALUop[1]);			   //异或，同或
	assign ANDOR= 	~(ALUop[2]|ALUop[1]);              //与，或
	assign UCMP= 	(~ALUop[2])&(&ALUop[1:0]);         //无符号比较
	//=============================================================================
	//ALUop[0] 分辨是与还是或
	assign result_AND_OR= (AND)? (A&B) : (A|B);
	//=============================================================================
	//在最高位添加一个0来判断无符号运算是否溢出
	assign {temp_carry,result_add}= {1'b0,A} + {1'b0,B ^ SUB} + {32'b0,SUB[0]};
	//=============================================================================
	//无符号进借位
	assign CarryOut= (!B)?
			0
		:(!SUB)?
			temp_carry
		:
			~temp_carry;

	//=============================================================================
	//有符号溢出
	assign Overflow= ( Asign&Bsign&(~Rsign)) | ((~Asign)&(~Bsign)&Rsign);
	//=============================================================================
	//根据结果的最高位来进行大小比较
	assign result_compare= 	 	Rsign ^ Overflow;
	assign result_u_compare= 	CarryOut;
	//=============================================================================
	//异或和同或
	assign result_xor_nor= 		(ALUop[0])? ~(A^B) : (A^B);
	//=============================================================================
	//送出结果
	assign Result= (XNOR)?
			result_xor_nor
		:(ANDOR)?
			result_AND_OR
		:(UCMP)?
			result_u_compare
		:(CMP)?
			result_compare
		:
			result_add;
	//=============================================================================
	//结果逻辑非判断是否为0
	assign Zero= !Result;
	//=============================================================================
endmodule



module shifter (
	input [`DATA_WIDTH - 1:0] A,
	input [`ADDR_WIDTH - 1:0] B,
	input [1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);

	// TODO: Please add your logic code here
	/* Shiftop
	* 00 SLL
	* 10 SRL 11 SRA
	*/
	wire [`DATA_WIDTH-1:0] result_sll;
	wire [`DATA_WIDTH-1:0] result_sra;
	wire [`DATA_WIDTH-1:0] result_srl;
	//wire [`DATA_WIDTH-1:0] arithmetic;

	//assign arithmetic=     ~((`DATA_WIDTH'b1<<B)>>B);
	assign result_sll=     	A<<B;
	assign result_sra=      ($signed(A))>>>B;//({32{Shiftop[0]}}&arithmetic) || (A>>B); //也可以直接用 ($signed(A))>>>B
	assign result_srl=		A>>B;
	assign Result = 	   (Shiftop[1])?(Shiftop[0])?result_sra:result_srl:result_sll;//(Shiftop[1])?result_sra:result_sll;//(Shiftop[1])?(Shiftop[0])?result_sra:result_srl:result_sll;
endmodule
