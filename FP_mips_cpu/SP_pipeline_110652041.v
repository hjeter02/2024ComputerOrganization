module SP_pipeline(
	// INPUT SIGNAL
	clk,
	rst_n,
	in_valid,
	inst,
	mem_dout,
	// OUTPUT SIGNAL
	out_valid,
	inst_addr,
	mem_wen,
	mem_addr,
	mem_din
);



//------------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION                         
//------------------------------------------------------------------------

input                    clk, rst_n, in_valid;
input             [31:0] inst;
input  signed     [31:0] mem_dout;
output reg               out_valid;
output reg        [31:0] inst_addr;
output reg               mem_wen;
output reg        [11:0] mem_addr;
output reg signed [31:0] mem_din;

//------------------------------------------------------------------------
//   DECLARATION
//------------------------------------------------------------------------
//Instruction opcodes

//Memories

//pipeline registers
reg[31:0] IDEX, EXMEM, MEMWB;
reg[4:0] IDEXrs, IDEXrt, IDEXrd;
// REGISTER FILE, DO NOT EDIT THE NAME.
reg	signed [31:0] r [0:31]; 
reg	signed [31:0] r_ID [0:31]; 
reg	signed [31:0] r_WB [0:31]; 

reg[5:0] opcode, func;
reg[4:0] rs, rt, rd, shmt;
reg[15:0] imm;
//------------------------------------------------------------------------
//   DESIGN
//------------------------------------------------------------------------

//ID stage
//------------------------------------------------------------------------
always @(*) begin
	opcode = inst[31:26];
	rs = inst[25:21];
	rt = inst[20:16];
	rd = inst[15:11];
	shmt = inst[10:6];
	func = inst[5:0];
	imm = inst[15:0];
	
end


//------------------------------------------------------------------------
reg[3:0] ALUop_EX, ALUop_ID;
//ALU inputs
reg signed [31:0] Ain, Bin, se_EX, ze_EX, read_data_2_EX;
wire signed [31:0] se_ID, ze_ID;
reg[1:0] inst_type_EX, inst_type_ID; 

reg[4:0] write_reg_EX, write_reg_MEM, write_reg_WB;
wire[4:0] write_reg_ID;

reg reg_write_ID; 
reg reg_write_EX, reg_write_MEM, reg_write_WB;

reg mem_read_ID;
reg mem_read_EX, mem_read_MEM, mem_read_WB;

reg mem_write_ID;
reg mem_write_EX, mem_write_MEM;

reg jal_flag_ID, jal_flag_EX, jal_flag_MEM, jal_flag_WB;

reg [31:0] pc_for_jal_EX, pc_for_jal_MEM, pc_for_jal_WB; 

reg [31:0] inst_addr_ID;

reg [4:0] shmt_EX;

assign se_ID = {{16{inst[15]}}, imm};
assign ze_ID = {16'b0, imm};
assign write_reg_ID = (!opcode)? rd : rt;

//PC
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		inst_addr <= 0;
	end
	else begin
		inst_addr <= inst_addr_ID;
	end
	
end

//ID to EX pipline registers
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin
	//
	if (!rst_n)begin
		ALUop_EX <= 0;
		Ain <= 0;
		read_data_2_EX <= 0;
		se_EX <= 0;
		ze_EX <= 0;
		inst_type_EX <= 0;
		write_reg_EX <= 0;
		reg_write_EX <= 0;
		mem_read_EX <= 0;
		mem_write_EX <= 0;
		jal_flag_EX <= 0;
		pc_for_jal_EX <= 0;
		shmt_EX <= 0;

	end
	else begin
		ALUop_EX <= ALUop_ID;
		Ain <= r_WB[rs];
		read_data_2_EX <= r_WB[rt];
		se_EX <= se_ID;
		ze_EX <= ze_ID;
		inst_type_EX <= inst_type_ID;
		write_reg_EX <= write_reg_ID;
		reg_write_EX <= reg_write_ID;
		mem_read_EX <= mem_read_ID;
		mem_write_EX <= mem_write_ID;
		jal_flag_EX <= jal_flag_ID;
		pc_for_jal_EX <= inst_addr + 4;
		shmt_EX <= (opcode == 9)? 16 : shmt; 

	end

	
end


//ALUop decode
//------------------------------------------------------------------------
parameter AND = 0, OR = 1, ADD = 2, SUB = 3, SLT = 4, SLL = 5, NOR = 6, LUI = 7;
always @(*) begin
	ALUop_ID = 0;
	inst_type_ID = 2;
	inst_addr_ID = inst_addr;
	reg_write_ID = 0;
	mem_write_ID = 0;
	mem_read_ID = 0;
	jal_flag_ID = 0;
	if (in_valid) begin
		reg_write_ID = 1;
		inst_addr_ID = inst_addr + 4;
		case(opcode)
		0 : begin//R type
			inst_type_ID = 0;
			case(func)
			0 : ALUop_ID = AND;
			1 : ALUop_ID = OR;
			2 : ALUop_ID = ADD;
			3 : ALUop_ID = SUB;
			4 : ALUop_ID = SLT;
			5 : ALUop_ID = SLL;
			6 : ALUop_ID = NOR;
			7 : begin
				inst_addr_ID = r[rs];//jr
				reg_write_ID = 0;
			end 
			default : begin
				ALUop_ID = 0;
				reg_write_ID = 0;
			end
			

			endcase
		
		end
		1 : begin//andi
			ALUop_ID = AND;
			inst_type_ID = 1;
		end 

		2 : begin//ori
			ALUop_ID = OR;
			inst_type_ID = 1;
		end 

		3 : ALUop_ID = ADD;//addi
		4 : ALUop_ID = SUB;//subi
		5 : begin//lw
			ALUop_ID = ADD;
			mem_read_ID = 1;
		end
	
		6 : begin//sw
			ALUop_ID = ADD;
			reg_write_ID = 0;
			mem_write_ID = 1;
		end
		7 : begin//beq
			reg_write_ID = 0;
			if (r[rs] == r[rt]) begin
				inst_addr_ID = inst_addr + 4 + {{14{inst[15]}}, imm, 2'b0};//se_ID<<2/////
			end
			else begin
				inst_addr_ID = inst_addr + 4;
			end
		end
		8 : begin//bne
			reg_write_ID = 0;
			if (r[rs] != r[rt]) begin
				inst_addr_ID = inst_addr + 4 + {{14{inst[15]}}, imm, 2'b0};///////
			end
			else begin
				inst_addr_ID = inst_addr + 4;
			end
		end
		9 : ALUop_ID = LUI;//lui
		10 : begin
			reg_write_ID = 0;
			inst_addr_ID = {inst_addr[31:28], inst[25:0], 2'b0};//j
		end 
		11 : begin//jal
			reg_write_ID = 0;
			inst_addr_ID = {inst_addr[31:28], inst[25:0], 2'b0};
			jal_flag_ID = 1;
		end
		
		default : begin
			ALUop_ID = 0;
			reg_write_ID = 0;
		end
		

		endcase
	end
	
end


//EX stage
//------------------------------------------------------------------------

reg signed [31:0] ALUrst_EX, ALUrst_MEM, ALUrst_WB, mem_addr_EX, mem_din_EX;
reg mem_wen_EX;

//EX to MEM pipline registers
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		ALUrst_MEM <= 0;
		write_reg_MEM <= 0;
		reg_write_MEM <= 0;
		mem_read_MEM <= 0;
		mem_wen <= 1;///////////////
		mem_addr <= 0;
		mem_din <= 0;
		jal_flag_MEM <= 0;
		pc_for_jal_MEM <= 0;
	end
	else begin
		ALUrst_MEM <= ALUrst_EX;
		write_reg_MEM <= write_reg_EX;
		reg_write_MEM <= reg_write_EX;
		mem_read_MEM <= mem_read_EX;
		mem_wen <= mem_wen_EX;
		mem_addr <= mem_addr_EX;
		mem_din <= mem_din_EX;
		jal_flag_MEM <= jal_flag_EX;
		pc_for_jal_MEM <= pc_for_jal_EX;
		
	end
end

reg addr_src;

//ALU
//------------------------------------------------------------------------
always @(*) begin

	Bin = 0;
	case(inst_type_EX)
	0 : Bin = read_data_2_EX;
	1 : Bin = ze_EX;
	2 : Bin = se_EX;
	default : Bin = 0;
	endcase

	case (ALUop_EX)
	AND : ALUrst_EX = Ain & Bin;
	OR : ALUrst_EX = Ain | Bin;
	ADD : ALUrst_EX = Ain + Bin;
	SUB : ALUrst_EX = Ain - Bin;
	SLT : begin
		if (Ain < Bin) begin
			ALUrst_EX = 1;
		end
		else begin
			ALUrst_EX = 0;
		end
	end
	SLL : ALUrst_EX = Ain << shmt_EX;
	NOR : ALUrst_EX = ~(Ain | Bin);
	LUI : ALUrst_EX = {se_EX[15:0], 16'b0};
	default : ALUrst_EX = 0;
	endcase

	//mem_wen_EX = ~ mem_write_EX;


	addr_src = mem_read_EX | mem_write_EX;

	if (addr_src) begin
		mem_addr_EX = ALUrst_EX;
	end
	else begin
		mem_addr_EX = 'bx;
	end

	if (mem_write_EX) begin
		mem_din_EX = read_data_2_EX;
		mem_wen_EX = 0;
	end
	else begin
		mem_din_EX = 0;
		mem_wen_EX = 1;
	end


end


//MEM stage
//reg signed [31:0] mem_dout_WB;
//MEM to WB pipline registers
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		ALUrst_WB <= 0;
		write_reg_WB <= 0;
		reg_write_WB <= 0;
		mem_read_WB <= 0;
		//mem_dout_WB <= 0;
		jal_flag_MEM <= 0;
		pc_for_jal_MEM <= 0;

		
	end
	else begin
		ALUrst_WB <= ALUrst_MEM;
		write_reg_WB <= write_reg_MEM;
		reg_write_WB <= reg_write_MEM;
		mem_read_WB <= mem_read_MEM;
		//mem_dout_WB <= mem_dout;
		jal_flag_WB <= jal_flag_MEM;
		pc_for_jal_WB <= pc_for_jal_MEM;
	end
end


//WB stage

reg signed [31:0] wb;

//WB to output registers
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin

	if (!rst_n) begin
		for (integer i = 0; i < 32; i = i + 1)
		r[i] <= 0;
	end
	else begin
		for (integer i = 0; i < 32; i = i + 1)
		r[i] <= r_WB[i];
	end

end 

//WB and jump
//------------------------------------------------------------------------
always @(*) begin
	for (integer i = 0; i < 32; i = i + 1)
	r_WB[i] = r[i];

	if (jal_flag_WB) begin
		r_WB[31] = pc_for_jal_WB;
	end
	else begin
		r_WB[31] = r[31];
	end

	if (mem_read_WB) begin
		wb = mem_dout;
	end
	else begin
		wb = ALUrst_WB;
	end
	
	// else begin
	// 	r_WB[write_reg_WB] = r[write_reg_WB];
	// end

	if (jal_flag_WB) begin
		r_WB[31] = pc_for_jal_WB;
	end
	else if (reg_write_WB) begin
		r_WB[write_reg_WB] = wb; 
	end

end
reg [1:0] cnt;

//out_valid
//------------------------------------------------------------------------
always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		cnt <= 0;
		out_valid <=0;
	end
	else begin
		cnt <= (cnt !=0 || in_valid != out_valid)? cnt + 1 : 0 ;
		out_valid <= (cnt == 3)? out_valid + 1 : out_valid;
	end

	
end

endmodule