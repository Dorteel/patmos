/*
 * Decode stage of Patmos.
 *
 * Authors: Martin Schoeberl (martin@jopdesign.com)
 *          Wolfgang Puffitsch (wpuffitsch@gmail.com)
 */

package patmos

import Chisel._

import Constants._

class Decode() extends Module {
  val io = IO(new DecodeIO())
  // Itermidiate solution copying opcodes from Constants.scala as all chiseltype constants are giving runtime errors when importet from the constants file
  val OPCODE_ALUI = UInt("b00")
  val OPCODE_ALU = UInt("b01000")
  val OPCODE_SPC = UInt("b01001")
  val OPCODE_LDT = UInt("b01010")
  val OPCODE_STT = UInt("b01011")

  val OPCODE_STC = UInt("b01100")

  val OPCODE_CFL_CALLND = UInt("b10000")
  val OPCODE_CFL_BRND   = UInt("b10010")
  val OPCODE_CFL_BRCFND = UInt("b10100")
  val OPCODE_CFL_TRAP   = UInt("b10110")

  val OPCODE_CFL_CALL   = UInt("b10001")
  val OPCODE_CFL_BR     = UInt("b10011")
  val OPCODE_CFL_BRCF   = UInt("b10101")

  val OPCODE_CFL_CFLRND = UInt("b11000")
  val OPCODE_CFL_CFLR   = UInt("b11001")

  val OPCODE_ALUL = UInt("b11111")

  val OPC_ALUR  = UInt("b000")
  val OPC_ALUU  = UInt("b001")
  val OPC_ALUM  = UInt("b010")
  val OPC_ALUC  = UInt("b011")
  val OPC_ALUP  = UInt("b100")
  val OPC_ALUB  = UInt("b101")
  val OPC_ALUCI = UInt("b110")

  val OPC_MTS = UInt("b010")
  val OPC_MFS = UInt("b011")

  val MSIZE_W = UInt("b000")
  val MSIZE_H = UInt("b001")
  val MSIZE_B = UInt("b010")
  val MSIZE_HU = UInt("b011")
  val MSIZE_BU = UInt("b100")

  val MTYPE_S = UInt("b00")
  val MTYPE_L = UInt("b01")
  val MTYPE_C = UInt("b10")
  val MTYPE_M = UInt("b11")

  val FUNC_ADD = UInt("b0000")
  val FUNC_SUB = UInt("b0001")
  val FUNC_XOR = UInt("b0010")
  val FUNC_SL = UInt("b0011")
  val FUNC_SR = UInt("b0100")
  val FUNC_SRA = UInt("b0101")
  val FUNC_OR = UInt("b0110")
  val FUNC_AND = UInt("b0111")
  val FUNC_NOR = UInt("b1011")
  val FUNC_SHADD = UInt("b1100")
  val FUNC_SHADD2 = UInt("b1101")

  val MFUNC_MUL = UInt("b0000")
  val MFUNC_MULU = UInt("b0001")

  val CFUNC_EQ = UInt("b0000")
  val CFUNC_NEQ = UInt("b0001")
  val CFUNC_LT = UInt("b0010")
  val CFUNC_LE = UInt("b0011")
  val CFUNC_ULT = UInt("b0100")
  val CFUNC_ULE = UInt("b0101")
  val CFUNC_BTEST = UInt("b0110")

  val PFUNC_OR = UInt("b00")
  val PFUNC_AND = UInt("b01")
  val PFUNC_XOR = UInt("b10")
  val PFUNC_NOR = UInt("b11")

  val JFUNC_RET   = UInt("b0000")
  val JFUNC_XRET  = UInt("b0001")
  val JFUNC_CALL  = UInt("b0100")
  val JFUNC_BR    = UInt("b0101")
  val JFUNC_BRCF  = UInt("b1010")

  val SPEC_FL = UInt("b0000")
  val SPEC_SL = UInt("b0010")
  val SPEC_SH = UInt("b0011")
  val SPEC_SS = UInt("b0101")
  val SPEC_ST = UInt("b0110")

  val SPEC_SRB = UInt("b0111")
  val SPEC_SRO = UInt("b1000")
  val SPEC_SXB = UInt("b1001")
  val SPEC_SXO = UInt("b1010")

  val STC_SRES   = UInt("b0000")
  val STC_SENS   = UInt("b0100")
  val STC_SFREE  = UInt("b1000")
  val STC_SSPILL = UInt("b1100")

  val STC_SENSR   = UInt("b0101")
  val STC_SSPILLR = UInt("b1101")


  val rf = Module(new RegisterFile())
  // register file is connected with unregistered instruction word
  rf.io.rfRead.rsAddr(0) := io.fedec.instr_a(16, 12)
  rf.io.rfRead.rsAddr(1) := io.fedec.instr_a(11, 7)
  if (PIPE_COUNT > 1) {
    rf.io.rfRead.rsAddr(2) := io.fedec.instr_b(16, 12)
    rf.io.rfRead.rsAddr(3) := io.fedec.instr_b(11, 7)
  }
  rf.io.ena := io.ena
  // RF write from write back stage
  rf.io.rfWrite <> io.rfWrite

  // register input from fetch stage
  val decReg = Reg(new FeDec())
  when(io.ena) {
    decReg := io.fedec
    when(io.flush) {
      decReg.flush()
      decReg.relPc := io.fedec.relPc
    }
  }

  // default values
  io.decex.defaults()

  // forward RF addresses and data
  io.decex.rsAddr(0) := decReg.instr_a(16, 12)
  io.decex.rsAddr(1) := decReg.instr_a(11, 7)
  if (PIPE_COUNT > 1) {
    io.decex.rsAddr(2) := decReg.instr_b(16, 12)
    io.decex.rsAddr(3) := decReg.instr_b(11, 7)
  }

  io.decex.rsData(0) := rf.io.rfRead.rsData(0)
  io.decex.rsData(1) := rf.io.rfRead.rsData(1)
  if (PIPE_COUNT > 1) {
    io.decex.rsData(2) := rf.io.rfRead.rsData(2)
    io.decex.rsData(3) := rf.io.rfRead.rsData(3)
  }

  val decoded = Wire(Vec(PIPE_COUNT, Bool()))
  decoded := Vec.fill(PIPE_COUNT) {Bool(false)}

  // Decoding of dual-issue operations
  println("SofarSoGood")
  val dual = decReg.instr_a(INSTR_WIDTH - 1) && decReg.instr_a(26, 22) =/= OPCODE_ALUL;
  for (i <- 0 until PIPE_COUNT) {
    val instr   = if (i == 0) { decReg.instr_a } else { decReg.instr_b }
    val opcode  = instr(26, 22)
    val opc     = instr(6, 4)
    val isValid = if (i == 0) { Bool(true) } else { dual }

    val immVal = Wire(UInt())
    // Default value for immediates
    immVal := Cat(UInt(0), instr(11, 0))

    // ALU register
    io.decex.aluOp(i).func := instr(3, 0)

    // ALU immediate
    when(opcode(4, 3) === OPCODE_ALUI) {
      io.decex.aluOp(i).func := Cat(UInt(0), instr(24, 22))
      io.decex.immOp(i) := isValid
      io.decex.wrRd(i) := isValid
      decoded(i) := Bool(true)
    }
    // Other ALU
    when(opcode === OPCODE_ALU) {
      switch(opc) {
        is(OPC_ALUR) {
          io.decex.wrRd(i) := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_ALUU) {
          io.decex.wrRd(i) := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_ALUM) {
          io.decex.aluOp(i).isMul := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_ALUC) {
          io.decex.aluOp(i).isCmp := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_ALUCI) {
          io.decex.aluOp(i).isCmp := isValid
          io.decex.immOp(i) := isValid
          immVal := Cat(UInt(0), instr(11, 7))
          decoded(i) := Bool(true)
        }
        is(OPC_ALUP) {
          io.decex.aluOp(i).isPred := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_ALUB) {
          io.decex.wrRd(i) := isValid
          io.decex.aluOp(i).isBCpy := isValid
          io.decex.immOp(i) := isValid
          immVal := Cat(UInt(0), instr(11, 7))
          decoded(i) := Bool(true)
        }
      }
    }
    // Special registers
    when(opcode === OPCODE_SPC) {
      switch(opc) {
        is(OPC_MTS) {
          io.decex.aluOp(i).isMTS := isValid
          decoded(i) := Bool(true)
        }
        is(OPC_MFS) {
          io.decex.aluOp(i).isMFS := isValid
          io.decex.wrRd(i) := isValid
          decoded(i) := Bool(true)
        }
      }
    }

    // Default immediate value
    io.decex.immVal(i) := immVal

    // Predicates
    io.decex.predOp(i).func := Cat(instr(3), instr(0))
    io.decex.predOp(i).s1Addr := instr(15, 12)
    io.decex.predOp(i).s2Addr := instr(10, 7)
    io.decex.predOp(i).dest := instr(19, 17)
    io.decex.pred(i) := instr(30, 27)

    // Default destination
    io.decex.rdAddr(i) := instr(21, 17)
  }

  // Decoding of additional operations for first pipeline
  val instr = decReg.instr_a
  val opcode = instr(26, 22)
  val func = instr(3, 0)

  val ldsize = instr(11, 9)
  val ldtype = instr(8, 7)
  val stsize = instr(21, 19)
  val sttype = instr(18, 17)
  val stcfun = instr(21, 18)

  val dest = Wire(UInt(width = REG_BITS))
  val longImm = Wire(Bool())

  val isMem = Wire(Bool())
  val isStack = Wire(Bool())

  val isSTC = Wire(Bool())
  val stcImm = Cat(UInt(0), instr(17, 0), Bits("b00"))

  // Long immediates set this
  longImm := Bool(false)

  // Load/stores and stack control operations set this
  isMem := Bool(false)
  isStack := Bool(false)
  isSTC := Bool(false)

  // Everything except calls uses the default
  dest := instr(21, 17)

  // ALU long immediate (Bit 31 is set as well)
  when(opcode === OPCODE_ALUL && instr(6, 4) === UInt(0)) {
    io.decex.aluOp(0).func := func
    io.decex.immOp(0) := Bool(true)
    longImm := Bool(true)
    io.decex.wrRd(0) := Bool(true)
    decoded(0) := Bool(true)
  }
  // Stack control
  when(opcode === OPCODE_STC) {
    switch(stcfun) {
      is(STC_SRES) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_RES
        io.decex.immOp(0) := Bool(true)
        decoded(0) := Bool(true)
      }
      is(STC_SENS) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_ENS
        io.decex.immOp(0) := Bool(true)
        decoded(0) := Bool(true)
      }
      is(STC_SENSR) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_ENS
        decoded(0) := Bool(true)
      }
      is(STC_SFREE) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_FREE
        io.decex.immOp(0) := Bool(true)
        decoded(0) := Bool(true)
      }
      is(STC_SSPILL) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_SPILL
        io.decex.immOp(0) := Bool(true)
        decoded(0) := Bool(true)
      }
      is(STC_SSPILLR) {
        isSTC := Bool(true)
        io.decex.stackOp := sc_OP_SPILL
        decoded(0) := Bool(true)
      }
    }
  }
  
  // Control-flow operations
  when(opcode === OPCODE_CFL_TRAP) {
    io.decex.trap := Bool(true)
    io.decex.xsrc := instr(EXC_SRC_BITS-1, 0)
    decoded(0) := Bool(true)
  }
  when(opcode === OPCODE_CFL_CALL || opcode === OPCODE_CFL_CALLND) {
    io.decex.immOp(0) := Bool(true)
    io.decex.call := Bool(true)
    io.decex.nonDelayed := opcode === OPCODE_CFL_CALLND
    decoded(0) := Bool(true)
  }
  when(opcode === OPCODE_CFL_BR || opcode === OPCODE_CFL_BRND) {
    io.decex.immOp(0) := Bool(true)
    io.decex.jmpOp.branch := Bool(true)
    io.decex.nonDelayed := opcode === OPCODE_CFL_BRND
    decoded(0) := Bool(true)
  }
  when(opcode === OPCODE_CFL_BRCF || opcode === OPCODE_CFL_BRCFND) {
    io.decex.immOp(0) := Bool(true)
    io.decex.brcf := Bool(true)
    io.decex.nonDelayed := opcode === OPCODE_CFL_BRCFND
    decoded(0) := Bool(true)
  }
  when(opcode === OPCODE_CFL_CFLR || opcode === OPCODE_CFL_CFLRND) {
    switch(func) {
      is(JFUNC_RET) {
        io.decex.ret := Bool(true)
        decoded(0) := Bool(true)
      }
      is(JFUNC_XRET) {
        io.decex.xret := Bool(true)
        decoded(0) := Bool(true)
      }
      is(JFUNC_CALL) {
        io.decex.call := Bool(true)
        decoded(0) := Bool(true)
      }
      is(JFUNC_BR) {
        io.decex.jmpOp.branch := Bool(true)
        decoded(0) := Bool(true)
      }
      is(JFUNC_BRCF) {
        io.decex.brcf := Bool(true)
        decoded(0) := Bool(true)
      }
    }
    io.decex.nonDelayed := opcode === OPCODE_CFL_CFLRND
  }

  val shamt = Wire(UInt())
  shamt := UInt(0)
  // load
  when(opcode === OPCODE_LDT) {
    isMem := Bool(true)
    io.decex.memOp.load := Bool(true)
    io.decex.wrRd(0) := Bool(true)
    switch(ldsize) {
      is(MSIZE_W) {
        shamt := UInt(2)
      }
      is(MSIZE_H) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
      }
      is(MSIZE_B) {
        io.decex.memOp.byte := Bool(true)
      }
      is(MSIZE_HU) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
        io.decex.memOp.zext := Bool(true)
      }
      is(MSIZE_BU) {
        io.decex.memOp.byte := Bool(true)
        io.decex.memOp.zext := Bool(true)
      }
    }
    io.decex.memOp.typ := ldtype;
    when(ldtype === MTYPE_C && io.exc.local) {
      io.decex.memOp.typ := MTYPE_L
    }
    when(ldtype === MTYPE_S) {
      isStack := Bool(true)
    }
    decoded(0) := Bool(true)
  }
  // store
  when(opcode === OPCODE_STT) {
    isMem := Bool(true)
    io.decex.memOp.store := Bool(true)
    switch(stsize) {
      is(MSIZE_W) {
        shamt := UInt(2)
      }
      is(MSIZE_H) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
      }
      is(MSIZE_B) {
        io.decex.memOp.byte := Bool(true)
      }
    }
    io.decex.memOp.typ := sttype;
    when(sttype === MTYPE_C && io.exc.local) {
      io.decex.memOp.typ := MTYPE_L
    }
    when(sttype === MTYPE_S) {
      isStack := Bool(true)
    }
    decoded(0) := Bool(true)
  }

  // Offset for loads/stores
  val addrImm = Wire(UInt())
  addrImm := Cat(UInt(0), instr(6, 0))
  switch(shamt) {
    is(UInt(1)) { addrImm := Cat(UInt(0), instr(6, 0), Bits(0, width = 1)) }
    is(UInt(2)) { addrImm := Cat(UInt(0), instr(6, 0), Bits(0, width = 2)) }
  }

  // Non-default immediate value
  when (isSTC || isStack || isMem || longImm) {
    io.decex.immVal(0) := Mux(isSTC, stcImm,
                              Mux(isStack, addrImm,
                                  Mux(isMem, addrImm,
                                      decReg.instr_b)))
  }
  // we could mux the imm / register here as well

  // Immediate for absolute calls
  io.decex.callAddr := Cat(UInt(0), instr(21, 0), Bits("b00"))

  // Immediate for branch is sign extended, not extended for call
  // PC-relative value is precomputed here
  io.decex.jmpOp.target := decReg.pc + Cat(Fill(PC_SIZE - 22, instr(21)), instr(21, 0))
  io.decex.jmpOp.reloc := decReg.reloc

  // Pass on PC
  io.decex.pc := decReg.pc
  io.decex.base := decReg.base
  io.decex.relPc := decReg.relPc

  // Set destination address
  io.decex.rdAddr(0) := dest

  // Disable register write on register 0
  for (i <- 0 until PIPE_COUNT) {
    when(io.decex.rdAddr(i) === UInt("b00000")) {
      io.decex.wrRd(i) := Bool(false)
    }
  }

  // Illegal operation
  io.decex.illOp := !Mux(dual, decoded.reduce(_&_), decoded(0))

  // Trigger exceptions
  val inDelaySlot = Reg(UInt(width = 2))

  when(io.exc.exc ||
       (io.exc.intr && inDelaySlot === UInt(0))) {
    io.decex.defaults()
    io.decex.pred(0) := UInt(0)
    io.decex.xcall := Bool(true)
    io.decex.xsrc := io.exc.src
    io.decex.callAddr := io.exc.addr
    io.decex.immOp(0) := Bool(true)
    io.decex.base := Mux(io.exc.exc, io.exc.excBase, decReg.base)
    io.decex.relPc := Mux(io.exc.exc, io.exc.excAddr, decReg.relPc)
  }

  // Update delay slot information
  when(io.ena) {
    val decDelaySlot = inDelaySlot - UInt(1)
    inDelaySlot := Mux(io.flush, UInt(1),
                       Mux(io.decex.call || io.decex.ret || io.decex.brcf ||
                           io.decex.xcall || io.decex.xret, UInt(3),
                           Mux(io.decex.jmpOp.branch, UInt(2),
                               Mux(io.decex.aluOp(0).isMul,
                                   Mux(inDelaySlot > UInt(1), decDelaySlot, UInt(1)),
                                   Mux(inDelaySlot =/= UInt(0), decDelaySlot, UInt(0))))))
  }

  // reset at end to override any computations
  when(reset) { decReg.flush() }
}
