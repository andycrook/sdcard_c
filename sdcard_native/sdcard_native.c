// sdcard_native: native read-path helpers for the Python SPI SD block driver.
// Dynruntime module for MicroPython on the project's embedded targets.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "py/dynruntime.h"

#define SDCARD_BLOCK_SIZE 512u
#define SDCARD_BLOCK_SIZE_SHIFT 9u
#define SDCARD_BLOCK_SIZE_MASK (SDCARD_BLOCK_SIZE - 1u)
#define SDCARD_TOKEN_DATA 0xfeu
#define SDCARD_EIO 5

#define DEFAULT_CMD_TIMEOUT 100
#define DEFAULT_DATA_TIMEOUT_MS 100
#define DEFAULT_DATA_TOKEN_POLL_SPINS 16
#define DEFAULT_DATA_TOKEN_DELAY_US 50

typedef struct {
    mp_obj_t spi_write;
    mp_obj_t spi_readinto;
    mp_obj_t spi_write_readinto;
    mp_obj_t cs_obj;
    mp_obj_t cmdbuf_obj;
    mp_obj_t dummybuf_obj;
    mp_obj_t tokenbuf_obj;
    uint8_t *cmdbuf;
    uint8_t *dummybuf;
    uint8_t *tokenbuf;
    mp_obj_t cs_low_obj;
    mp_obj_t cs_high_obj;
    mp_obj_t fill_ff_obj;
} sd_ctx_t;

static inline void call_function_1(mp_obj_t fun, mp_obj_t arg0) {
    mp_call_function_n_kw(fun, 1, 0, &arg0);
}

static inline void call_function_2(mp_obj_t fun, mp_obj_t arg0, mp_obj_t arg1) {
    mp_obj_t args[2] = {arg0, arg1};
    mp_call_function_n_kw(fun, 2, 0, args);
}

static inline void sd_set_cs(const sd_ctx_t *ctx, mp_obj_t value_obj) {
    call_function_1(ctx->cs_obj, value_obj);
}

static inline void sd_spi_write(const sd_ctx_t *ctx, mp_obj_t buf_obj) {
    call_function_1(ctx->spi_write, buf_obj);
}

static inline void sd_spi_clock_ff(const sd_ctx_t *ctx) {
    call_function_2(ctx->spi_readinto, ctx->tokenbuf_obj, ctx->fill_ff_obj);
}

static inline void sd_spi_readinto_ref(const sd_ctx_t *ctx, size_t len, uint8_t *buf) {
    mp_obj_t buf_obj = mp_obj_new_bytearray_by_ref(len, buf);
    call_function_2(ctx->spi_readinto, buf_obj, ctx->fill_ff_obj);
}

static inline void sd_spi_write_readinto(const sd_ctx_t *ctx, mp_obj_t src_obj, mp_obj_t dest_obj) {
    call_function_2(ctx->spi_write_readinto, src_obj, dest_obj);
}

static inline void sd_prepare_cmd(uint8_t *cmdbuf, uint8_t cmd, uint32_t arg, uint8_t crc) {
    cmdbuf[0] = (uint8_t)(0x40u | cmd);
    cmdbuf[1] = (uint8_t)(arg >> 24);
    cmdbuf[2] = (uint8_t)(arg >> 16);
    cmdbuf[3] = (uint8_t)(arg >> 8);
    cmdbuf[4] = (uint8_t)arg;
    cmdbuf[5] = crc;
}

static size_t sd_compute_sleep_loops(mp_int_t data_timeout_ms, mp_int_t delay_us) {
    uint32_t remaining_us = (uint32_t)data_timeout_ms * 1000u;
    uint32_t step_us = (uint32_t)delay_us;

#if defined(__ARM_ARCH_6M__)
    size_t sleep_loops = 0u;

    while (remaining_us >= step_us) {
        remaining_us -= step_us;
        ++sleep_loops;
    }

    if (sleep_loops == 0u) {
        sleep_loops = 1u;
    }
#else
    size_t sleep_loops = 1u;

    if (step_us != 0u) {
        sleep_loops = remaining_us / step_us;
    }
    if (sleep_loops == 0u) {
        sleep_loops = 1u;
    }
#endif

    return sleep_loops;
}

static void sd_raise_eio(const sd_ctx_t *ctx) {
    sd_set_cs(ctx, ctx->cs_high_obj);
    sd_spi_clock_ff(ctx);
    mp_raise_OSError(SDCARD_EIO);
}

static int sd_send_cmd(
    const sd_ctx_t *ctx,
    uint8_t cmd,
    uint32_t arg,
    uint8_t crc,
    int final,
    bool release,
    bool skip1,
    int timeout
) {
    if (timeout <= 0) {
        timeout = DEFAULT_CMD_TIMEOUT;
    }

    sd_set_cs(ctx, ctx->cs_low_obj);
    sd_prepare_cmd(ctx->cmdbuf, cmd, arg, crc);
    sd_spi_write(ctx, ctx->cmdbuf_obj);

    if (skip1) {
        sd_spi_clock_ff(ctx);
    }

    for (int i = 0; i < timeout; ++i) {
        sd_spi_clock_ff(ctx);
        uint8_t response = ctx->tokenbuf[0];
        if ((response & 0x80u) == 0u) {
            if (final < 0) {
                sd_spi_clock_ff(ctx);
                final = -1 - final;
            }
            for (int j = 0; j < final; ++j) {
                sd_spi_clock_ff(ctx);
            }
            if (release) {
                sd_set_cs(ctx, ctx->cs_high_obj);
                sd_spi_clock_ff(ctx);
            }
            return (int)response;
        }
    }

    sd_set_cs(ctx, ctx->cs_high_obj);
    sd_spi_clock_ff(ctx);
    return -1;
}

static void sd_wait_for_data_token(
    const sd_ctx_t *ctx,
    mp_obj_t sleep_us_fun,
    mp_obj_t delay_us_obj,
    mp_int_t delay_us,
    mp_int_t data_timeout_ms,
    mp_int_t poll_spins
) {
    if (poll_spins <= 0) {
        poll_spins = DEFAULT_DATA_TOKEN_POLL_SPINS;
    }
    if (data_timeout_ms <= 0) {
        data_timeout_ms = DEFAULT_DATA_TIMEOUT_MS;
    }
    if (delay_us <= 0) {
        delay_us = DEFAULT_DATA_TOKEN_DELAY_US;
        delay_us_obj = MP_OBJ_NEW_SMALL_INT(delay_us);
    }

    for (mp_int_t i = 0; i < poll_spins; ++i) {
        sd_spi_clock_ff(ctx);
        if (ctx->tokenbuf[0] == SDCARD_TOKEN_DATA) {
            return;
        }
    }

    size_t sleep_loops = sd_compute_sleep_loops(data_timeout_ms, delay_us);

    for (size_t i = 0; i < sleep_loops; ++i) {
        if (sleep_us_fun != mp_const_none) {
            call_function_1(sleep_us_fun, delay_us_obj);
        }
        sd_spi_clock_ff(ctx);
        if (ctx->tokenbuf[0] == SDCARD_TOKEN_DATA) {
            return;
        }
    }

    sd_raise_eio(ctx);
}

static void sd_read_block(
    const sd_ctx_t *ctx,
    mp_obj_t dest_obj,
    size_t dest_len,
    mp_obj_t sleep_us_fun,
    mp_obj_t delay_us_obj,
    mp_int_t delay_us,
    mp_int_t data_timeout_ms,
    mp_int_t poll_spins
) {
    sd_wait_for_data_token(ctx, sleep_us_fun, delay_us_obj, delay_us, data_timeout_ms, poll_spins);

    mp_obj_t dummy_obj = ctx->dummybuf_obj;
    if (dest_len != SDCARD_BLOCK_SIZE) {
        dummy_obj = mp_obj_new_bytearray_by_ref(dest_len, ctx->dummybuf);
    }

    sd_spi_write_readinto(ctx, dummy_obj, dest_obj);
    sd_spi_clock_ff(ctx);
    sd_spi_clock_ff(ctx);
}

static mp_obj_t sdcard_native_readinto(size_t n_args, const mp_obj_t *args) {
    mp_arg_check_num(n_args, 0, 6, 11, false);

    mp_obj_t spi_readinto_obj = args[0];
    mp_obj_t spi_write_readinto_obj = args[1];
    mp_obj_t cs_obj = args[2];
    mp_obj_t dummybuf_obj = args[3];
    mp_obj_t tokenbuf_obj = args[4];
    mp_obj_t dest_obj = args[5];

    mp_buffer_info_t dummybuf_info;
    mp_get_buffer_raise(dummybuf_obj, &dummybuf_info, MP_BUFFER_READ);

    mp_buffer_info_t tokenbuf_info;
    mp_get_buffer_raise(tokenbuf_obj, &tokenbuf_info, MP_BUFFER_RW);
    if (tokenbuf_info.len < 1u) {
        mp_raise_ValueError(MP_ERROR_TEXT("tokenbuf too small"));
    }

    mp_buffer_info_t dest_info;
    mp_get_buffer_raise(dest_obj, &dest_info, MP_BUFFER_RW);
    if (dest_info.len == 0u) {
        mp_raise_ValueError(MP_ERROR_TEXT("dest too small"));
    }
    if (dummybuf_info.len < dest_info.len) {
        mp_raise_ValueError(MP_ERROR_TEXT("dummybuf too small"));
    }

    mp_obj_t sleep_us_fun = n_args >= 7 ? args[6] : mp_const_none;
    mp_int_t data_timeout_ms = n_args >= 8 ? mp_obj_get_int(args[7]) : DEFAULT_DATA_TIMEOUT_MS;
    mp_int_t poll_spins = n_args >= 9 ? mp_obj_get_int(args[8]) : DEFAULT_DATA_TOKEN_POLL_SPINS;
    mp_int_t delay_us = n_args >= 10 ? mp_obj_get_int(args[9]) : DEFAULT_DATA_TOKEN_DELAY_US;
    bool selected = n_args >= 11 ? mp_obj_is_true(args[10]) : false;

    if (data_timeout_ms <= 0) {
        data_timeout_ms = DEFAULT_DATA_TIMEOUT_MS;
    }
    if (poll_spins <= 0) {
        poll_spins = DEFAULT_DATA_TOKEN_POLL_SPINS;
    }
    if (delay_us <= 0) {
        delay_us = DEFAULT_DATA_TOKEN_DELAY_US;
    }

    sd_ctx_t ctx = {
        .spi_write = mp_const_none,
        .spi_readinto = spi_readinto_obj,
        .spi_write_readinto = spi_write_readinto_obj,
        .cs_obj = cs_obj,
        .cmdbuf_obj = mp_const_none,
        .dummybuf_obj = dummybuf_obj,
        .tokenbuf_obj = tokenbuf_obj,
        .cmdbuf = NULL,
        .dummybuf = (uint8_t *)dummybuf_info.buf,
        .tokenbuf = (uint8_t *)tokenbuf_info.buf,
        .cs_low_obj = MP_OBJ_NEW_SMALL_INT(0),
        .cs_high_obj = MP_OBJ_NEW_SMALL_INT(1),
        .fill_ff_obj = MP_OBJ_NEW_SMALL_INT(0xff),
    };

    mp_obj_t delay_us_obj = MP_OBJ_NEW_SMALL_INT(delay_us);

    if (!selected) {
        sd_set_cs(&ctx, ctx.cs_low_obj);
    }
    sd_read_block(&ctx, dest_obj, dest_info.len, sleep_us_fun, delay_us_obj, delay_us, data_timeout_ms, poll_spins);
    if (!selected) {
        sd_set_cs(&ctx, ctx.cs_high_obj);
        sd_spi_clock_ff(&ctx);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sdcard_native_readinto_obj, 6, 11, sdcard_native_readinto);

static mp_obj_t sdcard_native_readblocks(size_t n_args, const mp_obj_t *args) {
    mp_arg_check_num(n_args, 0, 8, 12, false);

    mp_obj_t spi_obj = args[0];
    mp_obj_t cs_obj = args[1];
    mp_obj_t cmdbuf_obj = args[2];
    mp_obj_t tokenbuf_obj = args[3];

    mp_buffer_info_t cmdbuf_info;
    mp_get_buffer_raise(cmdbuf_obj, &cmdbuf_info, MP_BUFFER_RW);
    if (cmdbuf_info.len < 6u) {
        mp_raise_ValueError(MP_ERROR_TEXT("cmdbuf too small"));
    }

    mp_buffer_info_t tokenbuf_info;
    mp_get_buffer_raise(tokenbuf_obj, &tokenbuf_info, MP_BUFFER_RW);
    if (tokenbuf_info.len < 1u) {
        mp_raise_ValueError(MP_ERROR_TEXT("tokenbuf too small"));
    }

    mp_buffer_info_t buf_info;
    mp_get_buffer_raise(args[7], &buf_info, MP_BUFFER_RW);
    if (buf_info.len == 0 || (buf_info.len & SDCARD_BLOCK_SIZE_MASK) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("Buffer length is invalid"));
    }

    mp_int_t block_num = mp_obj_get_int(args[5]);
    mp_int_t cdv = mp_obj_get_int(args[6]);
    if (block_num < 0 || cdv <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("block_num/cdv must be positive"));
    }

    mp_int_t cmd_timeout = n_args >= 9 ? mp_obj_get_int(args[8]) : DEFAULT_CMD_TIMEOUT;
    mp_int_t data_timeout_ms = n_args >= 10 ? mp_obj_get_int(args[9]) : DEFAULT_DATA_TIMEOUT_MS;
    mp_int_t poll_spins = n_args >= 11 ? mp_obj_get_int(args[10]) : DEFAULT_DATA_TOKEN_POLL_SPINS;
    mp_int_t delay_us = n_args >= 12 ? mp_obj_get_int(args[11]) : DEFAULT_DATA_TOKEN_DELAY_US;

    if (cmd_timeout <= 0) {
        cmd_timeout = DEFAULT_CMD_TIMEOUT;
    }
    if (data_timeout_ms <= 0) {
        data_timeout_ms = DEFAULT_DATA_TIMEOUT_MS;
    }
    if (poll_spins <= 0) {
        poll_spins = DEFAULT_DATA_TOKEN_POLL_SPINS;
    }
    if (delay_us <= 0) {
        delay_us = DEFAULT_DATA_TOKEN_DELAY_US;
    }

    sd_ctx_t ctx = {
        .spi_write = mp_load_attr(spi_obj, MP_QSTR_write),
        .spi_readinto = mp_load_attr(spi_obj, MP_QSTR_readinto),
        .spi_write_readinto = mp_load_attr(spi_obj, MP_QSTR_write_readinto),
        .cs_obj = cs_obj,
        .cmdbuf_obj = cmdbuf_obj,
        .dummybuf_obj = args[4],
        .tokenbuf_obj = tokenbuf_obj,
        .cmdbuf = (uint8_t *)cmdbuf_info.buf,
        .dummybuf = NULL,
        .tokenbuf = (uint8_t *)tokenbuf_info.buf,
        .cs_low_obj = MP_OBJ_NEW_SMALL_INT(0),
        .cs_high_obj = MP_OBJ_NEW_SMALL_INT(1),
        .fill_ff_obj = MP_OBJ_NEW_SMALL_INT(0xff),
    };

    mp_obj_t sleep_us_fun = mp_const_none;
    mp_obj_t delay_us_obj = MP_OBJ_NEW_SMALL_INT(delay_us);
    if (delay_us > 0) {
        mp_obj_t time_mod = mp_import_name(MP_QSTR_time, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        sleep_us_fun = mp_load_attr(time_mod, MP_QSTR_sleep_us);
    }

    sd_spi_clock_ff(&ctx);

    size_t nblocks = buf_info.len >> SDCARD_BLOCK_SIZE_SHIFT;
    uint32_t address = (uint32_t)block_num * (uint32_t)cdv;
    uint8_t *dest = (uint8_t *)buf_info.buf;

    if (nblocks == 1u) {
        if (sd_send_cmd(&ctx, 17u, address, 0u, 0, false, false, (int)cmd_timeout) != 0) {
            sd_raise_eio(&ctx);
        }

        mp_obj_t dest_obj = mp_obj_new_bytearray_by_ref(SDCARD_BLOCK_SIZE, dest);
        sd_read_block(&ctx, dest_obj, SDCARD_BLOCK_SIZE, sleep_us_fun, delay_us_obj, delay_us, data_timeout_ms, poll_spins);
        sd_set_cs(&ctx, ctx.cs_high_obj);
        sd_spi_clock_ff(&ctx);
        return mp_const_none;
    }

    if (sd_send_cmd(&ctx, 18u, address, 0u, 0, false, false, (int)cmd_timeout) != 0) {
        sd_raise_eio(&ctx);
    }

    for (size_t block = 0; block < nblocks; ++block) {
        mp_obj_t dest_obj = mp_obj_new_bytearray_by_ref(SDCARD_BLOCK_SIZE, dest + (block * SDCARD_BLOCK_SIZE));
        sd_read_block(
            &ctx,
            dest_obj,
            SDCARD_BLOCK_SIZE,
            sleep_us_fun,
            delay_us_obj,
            delay_us,
            data_timeout_ms,
            poll_spins
        );
    }

    if (sd_send_cmd(&ctx, 12u, 0u, 0xffu, 0, true, true, (int)cmd_timeout) != 0) {
        mp_raise_OSError(SDCARD_EIO);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sdcard_native_readblocks_obj, 8, 12, sdcard_native_readblocks);

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args) {
    MP_DYNRUNTIME_INIT_ENTRY

    mp_store_global(MP_QSTR___name__, MP_OBJ_NEW_QSTR(MP_QSTR_sdcard_native));
    mp_store_global(MP_QSTR_readinto, MP_OBJ_FROM_PTR(&sdcard_native_readinto_obj));
    mp_store_global(MP_QSTR_readblocks, MP_OBJ_FROM_PTR(&sdcard_native_readblocks_obj));
    mp_store_global(MP_QSTR_backend, mp_obj_new_str("sdcard-readinto-v1", 18));

    MP_DYNRUNTIME_INIT_EXIT
}