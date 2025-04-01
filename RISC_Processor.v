/*
Student1 : Osaid Nur	 1210733
Student2 : Moath Wajeeh	 1210125
Student3 : Obayda Sarraj 1211128
*/

parameter

    // R-Type Instructions Project
    AND = 4'b0000, // Reg(Rd) = Reg(Rs1) & Reg(Rs2) 
    ADD = 4'b0001, // Reg(Rd) = Reg(Rs1) + Reg(Rs2) 
    SUB = 4'b0010, // Reg(Rd) = Reg(Rs1) - Reg(Rs2) 

    // I-Type Instructions
    ADDI = 4'b0011, // Reg(Rd) = Reg(Rs1) & Immediate5->16
    ANDI = 4'b0100, // Reg(Rd) = Reg(Rs1) + Immediate5->16
    LW   = 4'b0101, // Reg(Rd) = Mem(Reg(Rs1) + Imm_5)
    LBu  = 4'b0110, // Reg(Rd) = Mem(Reg(Rs1) + Imm_unsigned) 
    LBs  = 4'b0110, // Reg(Rd) = Mem(Reg(Rs1) + Imm_signed) 
    SW   = 4'b0111, // Mem(Reg(Rs1) + Imm_5) = Reg(Rd)	 
  BGT	 = 4'b1000,	 //if (Reg(Rd) > Reg(Rs1))
                     // Next PC = PC + sign_extended (Imm)
                   // else PC = PC + 2 

  BGTZ = 4'b1000,	//if (Reg(Rd) > Reg(0))	
                // Next PC = PC + sign_extended (Imm)
                    // Next PC = PC +2

  BLT	 = 4'b1001,	//if (Reg(Rd) < Reg(Rs1))
          //Next PC = PC + sign_extended (Imm)
          //else PC = PC + 2

  BLTZ = 4'b1001,	// if (Reg(Rd) < Reg(R0))
          //Next PC = PC + sign_extended (Imm)
          //else PC = PC + 2

    BEQ  = 4'b1010, //if (Reg(Rd) == Reg(Rs1))
          //Next PC = PC + sign_extended (Imm)
          //else PC = PC + 2
  BEQZ = 4'b1010,	 //if (Reg(Rd) == Reg(R0))
           //Next PC = PC + sign_extended (Imm)
           //else PC = PC + 2

  BNE	 = 4'b1011,	  //if (Reg(Rd) != Reg(Rs1))
           //Next PC = PC + sign_extended (Imm)	 
                   //else PC = PC + 2	

  BNEZ = 4'b1011, //if (Reg(Rd) != Reg(Rs1))
          //Next PC = PC + sign_extended (Imm)
          //else PC = PC + 2

    // J-Type Instructions
   JMP  = 4'b1100, // Next PC = {PC[15:12], Immediate}
   CALL = 4'b1101, // Next PC = {PC[15:12], Immediate} PC + 2 is saved on r15
   RET  = 4'b1110, //Next PC = r7	

    // S-Type Instructions
    Sv  = 4'b1111, // M[rs] = imm

// ALU function code signal
// 2-bit chip-select for ALU


  ALU_Add = 4'b0000, // used in ADD, ADDI, LW, SW, JAL
  ALU_Sub = 4'b0001, // used in SUB
ALU_And = 4'b0010, // used in AND, ANDI 

  ALU_BGT  = 4'b0011,
  ALU_BGTZ = 4'b0100,
  ALU_BLT  = 4'b0101,
  ALU_BLTZ = 4'b0110,
  ALU_BEQ  = 4'b0111,
  ALU_BEQZ = 4'b1000,
  ALU_BNE = 4'b1001,
  ALU_BNEZ = 4'b1010, 
  ALU_LBs= 4'b1011,

PC_Src_Dft = 2'b00, // PC = PC + 2
PC_Src_Ra  = 2'b01, // return address from stack	
PC_Src_Jmp = 2'b11, // jump address
PC_Src_BTA = 2'b10, // branch target address

  
    ALUSrc = 1'b0, // Else If S-Type || R-Type instruction, RB is used as operand2
   
    ALU_Src_UIm = 1'b1, // Else If I-Type instruction && Unsigned Immediate14 is used as operand2

// 8 registers -16-bit
    R0 = 3'd0, 
    R1 = 3'd1, 
    R2 = 3'd2,
    R3 = 3'd3, 
    R4 = 3'd4, 
    R5 = 3'd5, 
    R6 = 3'd6, 
    R7 = 3'd7; 


module ClockGenerator (
    clk
);

output reg clk=0;

always #5 begin
    clk=~clk;
end


endmodule



module dataMemory(
    input wire clk,
     input wire [15:0] Address,
    input wire [15:0] InputBus,
    output reg [15:0] OutputBus,
    input wire sig_enable_write,
    input wire [1:0] sig_enable_read
);
    // memory
    reg [15:0] memory [0:255];

    // Read/write instruction at positive edge of clk
    always @(posedge sig_enable_read or posedge sig_enable_write or clk) begin
        if (sig_enable_read == 2'b01) begin
            // Read word from memory
            OutputBus <= memory[Address];
        end
        else if (sig_enable_read == 2'b10) begin
            // Read byte from memory and zero-extend
            OutputBus <= {8'b0, memory[Address][7:0]};
        end
        else if (sig_enable_read == 2'b11) begin
            // Read byte from memory and sign-extend
            OutputBus <= {{8{memory[Address][15]}}, memory[Address][7:0]};
        end
        else if (sig_enable_write) begin
            // Write word to memory
            memory[Address] <= InputBus;
        end
    end

    // ----------------- INITIALIZATION -----------------

    initial begin
    // Store some initial data
     memory[0] = 16'd9;
     memory[1] = 16'd4;
     memory[2] = 16'd2;
     memory[3] = 16'd3;
     memory[5] = 16'd5;
     memory[6]=	16'd6;
     memory[30] = 16'd4;
     memory[31] = 16'hF004;
    end

endmodule

module controlUnit(
    clk,

   
    sig_alu_op,	//here this signal to choice op code for instruction if and or add or sub or branch
    NextPC, 
    sig_rb_src, // this signal to choice rd or rs2 in bus2	

  sig_rsORpc_src,//this signal to choice rs1 or pc value in I type
  //sig_r0ORrs_src, //this signal to choice 
  sig_rs1_src, //this signal to choice output depend on op code
  sig_rs2_src,
    ALUSrc,  // this signal to choice bus2 or 4 type output extender
    sig_rf_enable_write, // this signal on rigster file to wite result
    MemWrite, //data mem
    MemRead,  //data mem 
    //MemRead_byte, //data mem
    sig_write_back_data_select,	   // signal to choice between output alu or data memory
    sig_address_src,
  sig_data_src,

    // stage enable outputs
    en_instruction_fetch,
    en_instruction_decode,
    en_execute,

    // inputs
  //   S ,R,J,I
    OpCode,//op code for our project 
    ZFlag, 
  NFlag,
    VFlag,

  m,
  sig_BZ

    );

    // ----------------- INPUTS -----------------

    // clk
    input wire clk;

    // op code
    input wire [3:0] OpCode;

    // zero flag
    input wire ZFlag,NFlag,VFlag;

  //input m for lws and branch
    input wire m;

    // ----------------- OUTPUTS -----------------

    // Mux signals
    output reg [3:0] sig_alu_op;
    output reg [1:0] NextPC = PC_Src_Dft;
    output reg ALUSrc;

    output reg sig_write_back_data_select,
                sig_rb_src,sig_rsORpc_src,sig_rs1_src,sig_rs2_src,sig_address_src,sig_BZ,sig_data_src;

    // operation enable signals
    output reg sig_rf_enable_write = 1'b0,MemWrite = 1'b0;

    output reg [1:0] MemRead = 2'b00;		

    // stage enable signals ( used as clk for each stage )
    output reg en_execute = 1'b0,
                en_instruction_fetch = 1'b0,
                en_instruction_decode = 1'b0;

    // ----------------- INTERNALS -----------------

  // binary codes for stages
    `define Fetch 0
    `define Decode 1
    `define Execution 2
    `define Memory 3
    `define WriteBack 4
    `define Init 5 // STAGE Initial


    reg [2:0] current_stage = `Init;
    reg [2:0] next_stage = `Fetch; 

      always@(posedge clk) begin

        current_stage = next_stage;

    end

  ///////////////////////////////////////////////////////////////////

      always@(posedge clk) begin 

        case (current_stage)

            `Init: begin                                    
                en_instruction_fetch = 1'b0; 
                next_stage <= `Fetch;
            end

            `Fetch: begin 
                // disable previous stage
                en_instruction_decode = 1'b0;
                en_execute = 1'b0;

                // disable signals
                sig_rf_enable_write = 1'b0;
                MemWrite = 1'b0;
                MemRead = 2'b00;


                // enable current stage
                en_instruction_fetch = 1'b1; 

                // next stage
                next_stage <= `Decode;

            

                // here the inputs are the ones from the previous stage
               if (OpCode == RET ) begin

                   NextPC = PC_Src_Ra; // use return address as PC

             end  else if ( OpCode == JMP ||  OpCode == CALL ) begin

                 NextPC = PC_Src_Jmp; // use JTA as PC	


      end else if (OpCode == BEQZ && ZFlag == 1'b1 && m ==1'b1) begin
          NextPC = PC_Src_BTA;
        end else if (OpCode == BEQ && ZFlag == 1'b1 && m ==1'b0) begin
            NextPC = PC_Src_BTA;								  
        end else if (OpCode == BNEZ && ZFlag == 1'b0 && m ==1'b1) begin
            NextPC = PC_Src_BTA;
        end else if (OpCode == BNE && ZFlag == 1'b0 && m ==1'b0) begin
            NextPC = PC_Src_BTA;	 
        end else if (OpCode == BLT && NFlag == 1'b1 && m ==1'b0) begin
            NextPC = PC_Src_BTA;	
        end else if (OpCode == BLTZ && NFlag == 1'b1 && m ==1'b1) begin
            NextPC = PC_Src_BTA;
        end else if (OpCode == BGT && NFlag == 1'b0 && ZFlag == 1'b0  && m ==1'b0) begin
            NextPC = PC_Src_BTA;
        end else if (OpCode == BGTZ && NFlag == 1'b0 && ZFlag == 1'b0  && m ==1'b1) begin
            NextPC = PC_Src_BTA;
    end	else begin

        NextPC = PC_Src_Dft; // use next instruction address as PC

          end


            end

            `Decode: begin 


              // disable previous stage
                en_instruction_fetch = 1'b0;

                // enable current stage
                en_instruction_decode = 1'b1;

                 // next stage
                next_stage <= ( OpCode == JMP || OpCode == CALL || OpCode == RET ) ? `Fetch : `Execution;

                // ---- RS1 Source ----	

                sig_rs1_src = (OpCode > SUB) ? 1'b1 : 1'b0; 	//I-type 1 or R-type  0

         // ---- RS2 Source ----
                sig_rs2_src = (OpCode > SUB && OpCode!=Sv ) ? 1'b1 : 1'b0; //rd or rs2

         // ---- RD Source ----
                sig_rb_src = (OpCode > SUB) ? 1'b1 : 1'b0; //I-type-1 or R-type-0 
        sig_BZ=  ( (OpCode == BEQZ || OpCode == BNEZ  || 
        OpCode == BLTZ ||  OpCode == BGTZ ) && m ===1'b1)   ?1'b1 : 1'b0;	



        sig_rf_enable_write = (OpCode == JMP || OpCode == CALL) ? 1'b1:1'b0;


          // ---- ALU Source ----
    if  (OpCode < SUB || OpCode == SUB || ( OpCode >SW && OpCode < JMP ) )  begin
        // R-Type that uses register 
        ALUSrc = ALUSrc; // use Rb as operand
    end else if  (OpCode > SUB )  begin
        // ANDI instruction uses unsigned immediate
        ALUSrc = ALU_Src_UIm; // use unsigned immediate as operand
    end 
    //else if (InstructionType == I_Type && OpCode != ANDI) begin
    // I_Type instruction other than ANDI uses signed immediate
    //  ALUSrc = ALU_Src_SIm; // use signed immediate as operand
    //end
end	  

            `Execution: begin 


      // disable previous stage
                en_instruction_decode = 1'b0;

        // enable current stage
                en_execute = 1'b1;

                // next stage
                if  ( 
                    ( OpCode inside { CALL,RET} ) || 
                    ( OpCode inside {LW, SW,LBs,LBu,Sv} ) ) begin 

                    next_stage <= `Memory;

                end else if ( 
                    (OpCode inside {BEQZ,BEQ,BNEZ,BNE,BLT,BLTZ,BGT,BGTZ} )) begin 

                    next_stage <= `Fetch;

                end else begin

                    next_stage <= `WriteBack;

                end

                // ---- ALU Operation ----


       if ( 
                    (  OpCode == ANDI ) || 
                    (  OpCode == AND ) ) begin 

                    sig_alu_op = ALU_And; // bitwise AND

                end else if ( OpCode == SUB ) begin

                    sig_alu_op = ALU_Sub; // subtract

                end else    if (OpCode == ADD || OpCode == ADDI || OpCode == LW || OpCode == SW) begin
                    sig_alu_op = ALU_Add; // Set ALU operation to add


         ////////////////////////////
         //m	  Lood
         end else  if (OpCode == LBs && m==1'b1 )begin 

                    sig_alu_op = ALU_LBs; //

                end		
       else if  (OpCode == LBu && m==1'b0 )begin 

                    sig_alu_op = ALU_Add; //

                end
      ////////////////////////////////////   

      //Branch
        else if ( OpCode == BEQ && m==1'b0) begin

        sig_alu_op =ALU_BEQ ; // add	

                end	 

        else if ( OpCode == BEQZ && m==1'b1) begin

        sig_alu_op =ALU_BEQZ ; // add	

                end

        else if ( OpCode == BGT && m ==1'b0 ) begin

        sig_alu_op = ALU_BGT; // add	

                end	

        else if ( OpCode == BGTZ && m ==1'b1 ) begin

        sig_alu_op = ALU_BGTZ; // add	

                end	

        else if ( OpCode == BLT && m ==1'b0 ) begin

        sig_alu_op = ALU_BLT; // add	

                end

        else if ( OpCode == BLTZ && m ==1'b1 ) begin

        sig_alu_op = ALU_BLTZ; 	

                end


          else if ( OpCode == BNEZ && m ==1'b1 ) begin

        sig_alu_op = ALU_BNEZ;	

                end

        else if ( OpCode == BNE && m ==1'b0 ) begin

        sig_alu_op = ALU_BNE; 	
        end


        sig_data_src= (OpCode==Sv)? 1'b1:1'b0;
        sig_address_src= (OpCode == Sv) ? 1'b1 : 1'b0;


            end

      `Memory: begin   	

           // disable previous stage
                en_execute = 1'b0;

                 // next stage
                next_stage <= ( OpCode == LW || OpCode == LBs) ? `WriteBack : `Fetch;

                // ---------------------------------------------
                // ------------ set control signals ------------
                // ---------------------------------------------

                // ---- Memory Write ----

                MemWrite = (OpCode == SW || OpCode == Sv ) ? 1'b1 : 1'b0;

                // ---- Memory Read ----

        if (OpCode == LW)begin
            MemRead = 2'b01;
      end else if  (OpCode == LBs && m == 1'b1)begin
          MemRead = 2'b11;

      end	else if  (OpCode == LBu && m == 1'b0)begin 
          MemRead = 2'b10;
      end else begin
          MemRead=2'b00;

          end
            end

            `WriteBack: begin  

        // disable previous stages
                MemWrite = 1'b0;
                MemRead = 2'b00;
                //  MemRead_byte =1'b0;
                en_execute = 1'b0;                

                 // next stage
                next_stage <= `Fetch;

                // register file write enable
                sig_rf_enable_write = 1'b1;

                // write back data source
                sig_write_back_data_select = ( OpCode == LW || OpCode == LBs) ? 1'b1 : 1'b0;

            end
      endcase

    end



endmodule  


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module ALU(
    input wire clk,
    input wire [3:0] sig_alu_op,
    input wire [15:0] A, B,
    output reg [15:0] Output,
    output reg ZFlag,
    output reg NFlag,
    output reg VFlag
);		 


        assign ZFlag = (Output==0);
        assign NFlag = (Output[15] == 1); // if 2s complement number is negative, MSB is 1


    always @(posedge clk) begin
        case (sig_alu_op)
            ALU_Add, ALU_LBs: begin
                {VFlag, Output} <= {A[15], A} + {B[15], B};
            end
            ALU_Sub: begin
                {VFlag, Output} <= {A[15], A} - {B[15], B};
            end
            ALU_And: begin
                Output <= A & B;
                VFlag <= 0;
            end
            ALU_BGT, ALU_BGTZ, ALU_BLT, ALU_BLTZ, ALU_BEQ, ALU_BEQZ, ALU_BNEZ, ALU_BNE: begin
                {VFlag, Output} <= {B[15], B} - {A[15], A} ;
            end
            default: begin
                Output <= 0;
                VFlag <= 0;

            end
        endcase

    end

endmodule



 


module dataMemory_tb ();

    // ----------------- CLOCK -----------------

    // clk generator wires/registers
    reg clk;
    initial clk = 0;
    always #5 clk = ~clk; // generates clk square wave with 10ns period

    // ----------------- DATA MEMORY -----------------

    reg [15:0] Address;
    reg [15:0] InputBus;
    wire [15:0] OutputBus;
    reg sig_enable_write;
    reg [1:0] sig_enable_read;


    // Instantiate the data memory module
    dataMemory data_mem(
        .clk(clk),
        .Address(Address),
        .InputBus(InputBus),
        .OutputBus(OutputBus),
        .sig_enable_write(sig_enable_write),
        .sig_enable_read(sig_enable_read)

    );


    initial begin
        // Initial values
        Address = 16'b0;
        InputBus = 16'b0;
        sig_enable_write = 0;
        sig_enable_read = 0;


        // Write 9 to address 0
        #10;
        Address = 16'd0;
        InputBus = 16'd9;
        sig_enable_write = 1;
        sig_enable_read = 0;

        #100;

        // Write 4 to address 1
        #10;
        Address = 16'd1;
        InputBus = 16'h0FF4;
        sig_enable_write = 1;
        sig_enable_read = 0;

        #100;

        // Read from address 0
        #10;
        Address = 16'd0;
        sig_enable_write = 0;
        sig_enable_read =  2'b01;
        #100;


        // Read from address 1
        #10;
        Address = 16'd1;
        sig_enable_write = 0;
        sig_enable_read =  2'b01;

        #100;

        // Read sign extention byte from address 0
        #10;
        Address = 16'd0;
        sig_enable_write = 0;
        sig_enable_read =  2'b10;

        #100;

        $finish;
    end

endmodule 


module registerFile (
    input wire clk,
    input wire sig_enable_write,
  input wire  sig_BZ,
    input wire [2:0] RA, RB, RW,
    output reg [15:0] BusA, BusB,
    input wire [15:0] BusW
);

    reg [15:0] RegF [0:15];

    always @(posedge clk) begin 
    if (sig_BZ==1'b1)begin
    BusA=RegF[0];
    BusB = RegF[RB];
    end 
    else begin
        BusA = RegF[RA];
        BusB = RegF[RB];	
    end 
    end

    // write to register on positive edge of sig_enable_write
    always @(posedge clk) begin
        if (sig_enable_write) begin
            if (RW != 3'b0) begin // write register is not R0
                RegF[RW] = BusW;
            end
        end
    end

    // initialize registers to 0
    initial begin
    RegF[0] <= 16'h0000;
    RegF[1] <= 16'h0001;
    RegF[2] <= 16'h0002;
    RegF[3] <= 16'h0003;
    RegF[4] <= 16'h0004;
    RegF[5] <= 16'h0005;
    RegF[6] <= 16'h0006;
    RegF[7] <= -16'h00AB;
    RegF[8] <= 16'h0000;
    RegF[9] <= 16'h0000;
    RegF[10] <= 16'h0000;
    RegF[11] <= 16'h0000;
    RegF[12] <= 16'h0000;
    RegF[13] <= 16'h0000;
    RegF[14] <= 16'h0000;
    RegF[15] <= 16'h0000;


    end

endmodule  

module registerFile_tb();

    // ----------------- CLOCK -----------------

    // Clock signal
    reg clk;

    // Generate clk signal
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Toggle clk every 5 time units
    end

    // ----------------- REGISTER FILE -----------------

    // Inputs
    reg sig_enable_write;
    reg [3:0] RA, RB, RW;
    reg [15:0] BusW;

    // Outputs
    wire [15:0] BusA, BusB;

    // Instantiate the register file
    registerFile uut (
        .clk(clk),
        .sig_enable_write(sig_enable_write), 
    .sig_BZ(sig_BZ),
        .RA(RA),
        .RB(RB),
        .RW(RW),
        .BusW(BusW),
        .BusA(BusA),
        .BusB(BusB)
    );

    // ----------------- TEST SEQUENCE -----------------

    initial begin
        // Initialize signals
        sig_enable_write = 0;
        RA = 3'd0;
        RB = 3'd0;
        RW = 3'd0;
        BusW = 16'd0;

        // Wait for reset
        #10;

        // Write 15 to register 1
        RW = 3'd1;
        BusW = 16'd15;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Write 30 to register 2
        RW = 3'd2;
        BusW = 16'd30;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Read from register 1 and 2
        RA = 3'd1;
        RB = 3'd2;
        #10;

        // Display the results
        $display("Register 1: %d", BusA);
        $display("Register 2: %d", BusB);

        // Write 45 to register 3
        RW = 3'd3;
        BusW = 16'd45;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Read from register 3
        RA = 3'd3;
        #10;

        // Display the result
        $display("Register 3: %d", BusA);

        // End the simulation
        #10;
        $finish;
    end

endmodule	


module instructionMemory(clk, Address, Instruction);


    // clk
    input wire clk;

    // address bus
    input wire [15:0] Address;


    // instruction register
    output reg [0:15] Instruction;


    // instruction memory
    reg [15:0] instruction_memory [0:127];


    // ----------------- LOGIC -----------------

    assign Instruction = instruction_memory[Address[15:0]]; 


    initial begin
 

      // initial
        instruction_memory[0] = 16'b0;

 
  //--------------------R-type -test --------------------------------//

        instruction_memory[1] = { ADD, R1, R3, R2,3'b001};
        instruction_memory[2] = { SUB, R4, R3, R2,3'b001};
        instruction_memory[3] = { AND, R5, R3, R2,3'b001};


    /*instruction_memory[4] = { LW,1'b1 ,R1, R2, 5'b00001 };
    instruction_memory[6]=  { SW,1'b1 ,R7, R2, 5'b0000}; 
    instruction_memory[8]=  { LBs,1'b1 ,R1, R2, 5'b00010 };
    instruction_memory[6]=  { SW,1'b1 ,R0, R2, 5'b00010 }; 
    instruction_memory[10] = { LW,1'b1 ,R0, R1, 5'b00000 };	 */	

    // --------Branch Test-------------------//	


    //BGT,BGTZ  ZERO & NEGATVE =0
    //BLT,BLTZ NEGATIVE =1
    //BEQ,BEQZ ZERO =1
    //BNE.BNEZ ZERO=0
    instruction_memory[4] = { BLT,1'b0 ,R2, R3, 5'b00001 };//Negative flag = 1
    // instruction_memory[5] = { BLTZ,1'b1 ,R7, R0, 5'b0001 };//Negative flag = 1
    // instruction_memory[6] = { BGT,1'b0 ,R2, R1, 5'b00001 };	//Negative flag = 0 zero flag =0 
    // instruction_memory[7] = { BGTZ,1'b1 ,R6, R0, 5'b00010 }; //Negative flag = 0 zero flag =0 
    // instruction_memory[9] = { BEQ,1'b0 ,R1, R1, 5'b00011 }; 
    // instruction_memory[12] = { BEQZ,1'b1 ,R5, R1, 5'b00001 };
    // instruction_memory[13] = { BNE,1'b0 ,R1, R1, 5'b00001 };
    // instruction_memory[15] = { BNEZ,1'b1 ,R1, R0, 5'b11110 };  


  //instruction_memory[4]=  { Sv ,R5, 9'b000000011};
  //instruction_memory[6] = { LW,1'b0 ,R1, R4, 5'b00001 }; // check	value BUSW Or value Data output must be 5

    // instruction_memory[16]= { JMP ,12'b000001100};  
    // instruction_memory[17]= { CALL ,12'b000001111};	
    // instruction_memory[18] = { LW,1'b0 ,R1, R4, 5'b00001 };
    // instruction_memory[19]= { RET ,12'b000001111};




    end
endmodule




// Pc Source can be one of these:
// 1- PC + 2 (normal one)
// 2- BranchPc with Rules suffeneicet.
// 3- JumpPc pc[15:12]||offset*2
// 4- Return Pc

//****************************************************************************************************************
// PC module 
//****************************************************************************************************************
module PC(clk, PC, Imm_Itype, Imm_Jtype, ReturnAddress,NextPC);

    // Clock signal
    input wire clk;

    // PC source control signal
    input wire [1:0] NextPC; 

    // Return address from the stack
    input wire [15:0] ReturnAddress;

    // Immediate value for I-Type instructions
    input wire [15:0] Imm_Itype;

    // Immediate value for J-Type instructions
    input wire signed [11:0] Imm_Jtype;

    // Program Counter
    output reg [15:0] PC;


    // Incremented PC by 2
    wire [15:0] pc_plus_2;

    // Jump target address
    wire [15:0] jump_target_address;

    // Branch target address
    wire [15:0] brach_target_address;

    // Sign-extended immediate
    wire [15:0] sign_extended_imm;

    // Calculate PC + 2
    assign pc_plus_2 = PC + 16'd2;

    // Calculate jump target address
    assign jump_target_address = {{PC[15:12]},Imm_Jtype};

    // Calculate branch target address
    assign brach_target_address =PC +Imm_Itype;

    // Calculate sign-extended immediate target address
    assign sign_extended_imm = PC + Imm_Itype;

    // Initialize PC to 0 at the beginning
    initial begin
        PC <= 16'd0;
    end

    // Update PC on the positive edge of the clk
    always @(posedge clk) begin
        case (NextPC)
            2'b00: begin
                // Default: PC = PC + 2
                PC = pc_plus_2;      
            end  
            2'b01: begin
                // Return address: PC = ReturnAddress
                PC = ReturnAddress;  
            end  
            2'b10: begin
                // Branch target address: PC = PC + Imm_Itype
                PC = brach_target_address ;
            end  
            2'b11: begin
                // Jump target address: PC = PC + Imm_Jtype
                PC = jump_target_address;
            end  

        endcase
    end

endmodule 

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 module TopDesign();

    initial begin		 
    #0
        $display("(%0t) > initializing processor ...", $time);

        #400 $finish;
    end


     wire clk;

    wire [3:0] sig_alu_op;
    wire [1:0] NextPC;
    wire  ALUSrc;

    // single bit mux control signals
    wire sig_write_back_data_select,
            sig_rb_src,sig_rs1_src, sig_rs2_src,sig_rsORpc_src,sig_BZ;

    // operation enable signals
    wire sig_rf_enable_write,
            MemWrite;
     wire [1:0] MemRead;       

    // stage enable signals ( used as clk for each stage )
    wire en_instruction_fetch,
            en_instruction_decode,
            en_execute;

    // ----------------- Instrution Memory -----------------

    // instruction memory wires/registers		
    wire [15:0] PC; // output of PC Module input to instruction memory
     wire [15:0] Instruction; // output if instruction memory, input to other modules
   // Immediate value to be sign-extended
     wire signed [4:0] Imm;
    // Instruction Parts
    wire [3:0] OpCode; // function code

    // R-Type
    wire [2:0] Rs1_Rtype, Rd_Rtype, Rs2; // register selection	
  //I-type
  wire [2:0] Rs1_Itype, Rd_Itype; // register selection
    wire [1:0] InstructionType; // instruction type [ R, S, I, J ]
    wire m; // m bit signed or unsignd extends	

  //Memory Write
  wire sig_address_src,
       sig_data_src;

    // ----------------- Assignment -----------------

    // Function Code
     assign OpCode = Instruction[15:12];

    // R-Type
     assign Rs1_Rtype = Instruction[8:6];
     assign Rd_Rtype = Instruction[11:9];
     assign Rs2 = Instruction[5:3];


      // I-Type
     assign Rs1_Itype = Instruction[7:5];
     assign Rd_Itype = Instruction[10:8];
     assign m = Instruction[11]; 
     wire signed [4:0] Imm_Itype;
     assign Imm_Itype = Instruction[4:0]; 

    // J-Type
     wire signed [11:0] Imm_Jtype;
     assign Imm_Jtype = Instruction[11:0];

    wire signed [8:0] S_TypeImmediate;
     assign S_TypeImmediate = Instruction[8:0];


    // S-Type
    wire [0:4] SA;

     // assign SA = Instruction[20:24];

    // Instruction Type and Stop Bit
     // assign InstructionType = Instruction[29:30];
     //  assign StopBit = Instruction[31];


    // ----------------- PC Modules -----------------

    // register file wires/registers
    reg [15:0] ReturnAddress;  // input to PC Module from TODO
     wire signed [15:0] Sign_Extended_Imm_Jtype, Sign_Extended_Imm_Itype ;  // input to PC Module from decode stage
     wire [15:0] Unsigned_Extended_Imm_Itype, Unsigned_Extended_SA,Unsigned_Extended_S_TypeImmediate; // input to ALU Module from decode stage

    // signed extender for J-Type instructions immediate ( 24 bit to 32 )
     assign Sign_Extended_Imm_Jtype = { {8{Imm_Jtype[0]}}, Imm_Jtype }; 

  assign Unsigned_Extended_S_TypeImmediate = { {7{1'b0}}, S_TypeImmediate };

    // signed extender for I-Type instructions immediate ( 5 bit to 16 )
     assign Sign_Extended_Imm_Itype = { {11{Imm_Itype[4]}}, Imm_Itype };

    // unsigned extender for I-Type instructions immediate ( 5 bit to 16 )
     assign Unsigned_Extended_Imm_Itype = { {11{1'b0}}, Imm_Itype };

    // unsigned extender for S-Type instructions immediate ( 5 bit to 32 )
  //  assign Unsigned_Extended_SA = { {27{1'b0}}, SA };


    // ----------------- Register File -----------------

    reg [15:0] BusW; // TODO
    wire [15:0] BusA, BusB;

    wire [2:0] RA, RB, RW;
  ////////////////////////
    assign RA = (sig_rs1_src==1'b1 && OpCode ==RET) ? R7:  
     (sig_rs1_src==1'b0) ? Rs1_Rtype:Rs1_Itype;
    assign RB = (sig_rs2_src == 1'b0 && OpCode == Sv) ? Rd_Rtype : 
  (sig_rs2_src == 1'b0) ? Rs2 : Rd_Itype; 


    assign RW = ( OpCode == CALL) ? R7:
              (sig_rb_src==1'b0) ? Rd_Rtype: Rd_Itype;

  //  assign BusW = (sig_rf_enable_write == 1'b1 && OpCode == CALL) ? (PC + 16'd2) : BusW ; // other value or previous value;

  ////////////////////////

    // ----------------- ALU -----------------

    reg [15:0] ALU_A, ALU_B; // operands
    wire [15:0] ALU_Output;
    wire ZFlag;
    wire NFlag;
     wire VFlag;

    assign ALU_A = BusA; 
     assign ALU_B =  (ALUSrc==1'b0) ? BusB:Unsigned_Extended_Imm_Itype;	



    // ----------------- Data Memory -----------------

    // data memory wires/registers/signals
     wire [15:0] Address;
    wire [15:0] DataMemoryInputBus;

    wire [15:0] DataMemoryOutputBus;

     assign Address = (sig_address_src==1'b1)? RB:ALU_Output;
    assign DataMemoryInputBus = (sig_data_src==1'b1)? Unsigned_Extended_S_TypeImmediate:BusB;


  // generates clk square wave with 10ns period
  ClockGenerator clk_generator(clk);


    controlUnit control_unit (
        .clk(clk),

        // signal outputs
        .sig_alu_op(sig_alu_op),
        .NextPC(NextPC),
        .sig_rb_src(sig_rb_src),
        .sig_rsORpc_src(sig_rsORpc_src),
        .sig_rs1_src(sig_rs1_src),
        .sig_rs2_src(sig_rs2_src),
        .ALUSrc(ALUSrc),
        .sig_rf_enable_write(sig_rf_enable_write),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .sig_write_back_data_select(sig_write_back_data_select),
    .sig_address_src(sig_address_src),
    .sig_data_src(sig_data_src),

        // stage enable outputs
        .en_instruction_fetch(en_instruction_fetch),
        .en_instruction_decode(en_instruction_decode),
        .en_execute(en_execute),

        // inputs
        .OpCode(OpCode),
        .ZFlag(ZFlag),
    .NFlag(NFlag),
        .VFlag (VFlag ),
        .m(m),	
    .sig_BZ(sig_BZ)
    );





     instructionMemory instruction_memory(clk, PC, Instruction);
     PC pc_module(en_instruction_fetch, PC, Sign_Extended_Imm_Itype,Imm_Jtype, BusA,NextPC);
    registerFile register_file(en_instruction_decode,sig_rf_enable_write,sig_BZ,RA, RB, RW, BusA, BusB,BusW);
     ALU alu(en_execute,sig_alu_op ,ALU_A, ALU_B,ALU_Output,ZFlag,NFlag,VFlag);
     dataMemory data_memory(clk, Address, DataMemoryInputBus, DataMemoryOutputBus, MemWrite, MemRead);
    assign BusW = (sig_write_back_data_select == 0 && OpCode == CALL) ?(PC + 16'd2) : 
       (sig_write_back_data_select == 0 ) ? ALU_Output : DataMemoryOutputBus;


endmodule		 





