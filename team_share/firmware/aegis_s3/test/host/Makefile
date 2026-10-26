# Host build of the portable core + unit tests. Needs only a C compiler.
#   make -C test/host        (from firmware/aegis_s3)
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Werror -Wno-unused-parameter
CORE    := ../../components/aegis_core
QTABLE  := ../../../..          # project/ holds aegis_q_table.h
SRCS    := test_core.c $(CORE)/aegis_dsp.c $(CORE)/aegis_control.c $(CORE)/aegis_policy.c $(CORE)/aegis_mission.c $(CORE)/aegis_reflex.c

test_core: $(SRCS) $(wildcard $(CORE)/include/*.h)
	$(CC) $(CFLAGS) -I$(CORE)/include -I$(QTABLE) $(SRCS) -lm -o $@

.PHONY: run clean
run: test_core
	./test_core

clean:
	rm -f test_core

.DEFAULT_GOAL := run
