# Host-built environment (host/fx_dump_env.c) for soc/main_fx_soc.c (FX_ENV_BLOB); path from -DFX_ENV_FILE
    .section .rodata.fx_env_blob, "a"
    .balign 4
    .globl fx_env_blob
    .globl fx_env_blob_end
fx_env_blob:
    .incbin FX_ENV_FILE
fx_env_blob_end:
