# Portable algorithms (no ESP-IDF headers); also built on the host by test/host.
idf_component_register(
    SRCS "aegis_dsp.c" "aegis_control.c" "aegis_policy.c" "aegis_mission.c" "aegis_reflex.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "../../../..")   # project/: aegis_q_table.h from train_rl_policy.py
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
