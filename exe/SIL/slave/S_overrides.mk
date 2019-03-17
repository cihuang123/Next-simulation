## Readme First
## You should not modify this file.
## Because, this is the common for master and slave.
## If you necessary to change, please also consider SIL, PIL, HIL both master and slave. 
## Also, please reference the sil_model.mk and cmm_model.mk
##
SHELL=/bin/bash
TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/.git ]; then \
					TOP_DIR=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_DIR)
$(info TOP_DIR = $(TOP_DIR))
include $(TOP_DIR)/exe/xil_common/makefile/cmm_model.mk
include $(TOP_DIR)/exe/xil_common/makefile/sil_model.mk
include $(TOP_DIR)/exe/xil_common/makefile/cmm_flags.mk
include $(TOP_DIR)/exe/xil_common/makefile/ros_model.mk
#MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
#SIM_ROOT = $(patsubst %/exe/SIL/master/S_overrides.mk, %, $(MKFILE_PATH))