ISSUE ?= B
NUM_CORE ?= 2
NUM_TL_UL ?= 0
NUM_M_AGENT ?= 1
NUM_SLICE ?= 4
WITH_CHISELDB ?= 1
WITH_TLLOG ?= 1
WITH_CHILOG ?= 1
BY_ETIME ?= 1
BY_VTIME ?= 0
FPGA ?= 0

init:
	git submodule update --init
	cd rocket-chip && git submodule update --init hardfloat cde

compile:
	mill -i XSCache.compile

CHI_PASS_ARGS = ISSUE=$(ISSUE) NUM_CORE=$(NUM_CORE) NUM_TL_UL=$(NUM_TL_UL) NUM_SLICE=$(NUM_SLICE) \
			    WITH_CHISELDB=$(WITH_CHISELDB) WITH_TLLOG=$(WITH_TLLOG) WITH_CHILOG=$(WITH_CHILOG) \
				BY_ETIME=$(BY_ETIME) BY_VTIME=$(BY_VTIME) \
			    FPGA=$(FPGA)

TOP = TestTop
CHI_TOP_ARGS = --issue $(ISSUE) --core $(NUM_CORE) --tl-ul $(NUM_TL_UL) --bank $(NUM_SLICE) \
		   	   --chiseldb $(WITH_CHISELDB) --tllog $(WITH_TLLOG) --chilog $(WITH_CHILOG) \
			   --etime $(BY_ETIME) --vtime $(BY_VTIME) \
		       --fpga $(FPGA)
MATRIX_TOP_ARGS = --core $(NUM_CORE) --tl-ul $(NUM_TL_UL) --m-agent $(NUM_M_AGENT) --bank $(NUM_SLICE) \
		   	   --chiseldb $(WITH_CHISELDB) --tllog $(WITH_TLLOG) \
		       --fpga $(FPGA)
BUILD_DIR_L2 = ./build/coupledl2
BUILD_DIR_LLC = ./build/openllc
BUILD_DIR_ZJ_SINGLECORE = ./build/zj_singlecore
BUILD_DIR_ZJ_DUALCORE = ./build/zj_dualcore
BUILD_DIR_ZJ_MATRIX = ./build/zj_matrix
TOP_V_L2 = $(BUILD_DIR_L2)/$(TOP).sv
TOP_V_LLC = $(BUILD_DIR_LLC)/$(TOP).sv
MEM_GEN = ./scripts/vlsi_mem_gen
MEM_GEN_SEP = ./scripts/gen_sep_mem.sh

gen-test-top:
	mill -i XSCache.testtop.l2.runMain xscache.coupledL2.$(TOP)_$(SYSTEM) -td $(BUILD_DIR_L2) --target systemverilog --split-verilog
	if [ -f "$(TOP_V_L2).conf" ]; then $(MEM_GEN_SEP) "$(MEM_GEN)" "$(TOP_V_L2).conf" "$(BUILD_DIR_L2)"; fi

gen-test-top-chi:
	mill -i XSCache.testtop.l2.runMain xscache.coupledL2.$(TOP)_$(SYSTEM) -td $(BUILD_DIR_L2) $(CHI_TOP_ARGS) --target systemverilog --split-verilog
	if [ -f "$(TOP_V_L2).conf" ]; then $(MEM_GEN_SEP) "$(MEM_GEN)" "$(TOP_V_L2).conf" "$(BUILD_DIR_L2)"; fi

test-top-chi:
	$(MAKE) gen-test-top-chi SYSTEM=CHIL2 $(CHI_PASS_ARGS)

test-top-chi-dualcore-0ul:
	$(MAKE) gen-test-top-chi SYSTEM=CHIL2 $(CHI_PASS_ARGS) NUM_CORE=2 NUM_TL_UL=0

test-top-chi-dualcore-2ul:
	$(MAKE) gen-test-top-chi SYSTEM=CHIL2 $(CHI_PASS_ARGS) NUM_CORE=2 NUM_TL_UL=2

test-top-chi-quadcore-0ul:
	$(MAKE) gen-test-top-chi SYSTEM=CHIL2 $(CHI_PASS_ARGS) NUM_CORE=4 NUM_TL_UL=0

test-top-chi-quadcore-2ul:
	$(MAKE) gen-test-top-chi SYSTEM=CHIL2 $(CHI_PASS_ARGS) NUM_CORE=4 NUM_TL_UL=2

test-top-l3-openllc:
	mill -i XSCache.testtop.openllc.runMain xscache.openLLC.TestTop_L3 -td build --target systemverilog --split-verilog

test-top-l2l3-openllc:
	mill -i XSCache.testtop.openllc.runMain xscache.openLLC.TestTopOpenLLC_SingleCore -td $(BUILD_DIR_LLC) --target systemverilog --split-verilog

test-top-matrix:
	mill -i XSCache.testtop.openllc.runMain xscache.openLLC.TestTopMatrix -td $(BUILD_DIR_LLC) $(MATRIX_TOP_ARGS) --target systemverilog --split-verilog

test-top-l2l3l2-openllc:
	mill -i XSCache.testtop.openllc.runMain xscache.openLLC.TestTopOpenLLC_DualCore -td $(BUILD_DIR_LLC) --target systemverilog --split-verilog

test-top-zhujiang-singlecore:
	mill -i XSCache.testtop.zhujiang.runMain zhujiang.TestTopZhuJiang_SingleCore -td $(BUILD_DIR_ZJ_SINGLECORE) --target systemverilog --split-verilog

test-top-zhujiang-dualcore:
	mill -i XSCache.testtop.zhujiang.runMain zhujiang.TestTopZhuJiang_DualCore -td $(BUILD_DIR_ZJ_DUALCORE) --target systemverilog --split-verilog

test-top-zhujiang-matrix:
	mill -i XSCache.testtop.zhujiang.runMain zhujiang.TestTopZhuJiang_Matrix -td $(BUILD_DIR_ZJ_MATRIX) --target systemverilog --split-verilog

clean:
	rm -rf ./build

bsp:
	mill -i mill.bsp.BSP/install

idea:
	mill -i mill.scalalib.GenIdea/idea

reformat:
	mill -i __.reformat

checkformat:
	mill -i __.checkFormat

.PHONY: init bsp checkformat clean compile-coupledl2 compile-openllc idea reformat 
