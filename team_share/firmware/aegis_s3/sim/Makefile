# Software-in-the-loop simulator: links the firmware's portable core unchanged.
#   make -C sim            build
#   make -C sim batch      run all scenarios (writes results/)
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Werror -Wno-unused-parameter
CORE    := ../components/aegis_core
QTABLE  := ../../..
SRCS    := sim.c $(CORE)/aegis_dsp.c $(CORE)/aegis_control.c $(CORE)/aegis_policy.c $(CORE)/aegis_mission.c
RUNS    ?= 200

sim: $(SRCS) $(wildcard $(CORE)/include/*.h) $(QTABLE)/aegis_q_table.h
	$(CC) $(CFLAGS) -I$(CORE)/include -I$(QTABLE) $(SRCS) -lm -o $@

.PHONY: batch clean
batch: sim
	mkdir -p results
	./sim nominal $(RUNS) 7 results/nominal.csv 23 results/example_nominal & \
	./sim harsh $(RUNS) 7 results/harsh.csv & \
	./sim no_audio $(RUNS) 7 results/no_audio.csv & \
	AEGIS_POLICY=table ./sim nominal $(RUNS) 7 results/qtable_nominal.csv & \
	wait

clean:
	rm -f sim
