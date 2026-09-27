/*
 * Profile-aware scaler input processor.
 * Scales matching input events (e.g. REL_WHEEL / REL_HWHEEL) with a
 * different multiplier/divisor per BLE profile (and optionally for USB).
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_profile_scaler

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zmk/endpoints.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct profile_scaler_config {
    uint8_t type;
    size_t codes_len;
    const uint16_t *codes;
    size_t scales_len; /* number of uint32 entries (2 per profile) */
    const uint32_t *scales;
    bool has_usb_scale;
    uint32_t usb_mul;
    uint32_t usb_div;
};

static void get_scale(const struct profile_scaler_config *cfg, uint32_t *mul, uint32_t *div) {
    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    size_t pairs = cfg->scales_len / 2;
    size_t idx = 0;

    if (ep.transport == ZMK_TRANSPORT_USB) {
        if (cfg->has_usb_scale) {
            *mul = cfg->usb_mul;
            *div = cfg->usb_div;
            return;
        }
        idx = 0; /* USB without usb-scale -> first pair */
    } else {
        idx = ep.ble.profile_index < 0 ? 0 : (size_t)ep.ble.profile_index;
    }

    if (idx >= pairs) {
        idx = pairs - 1; /* profiles beyond the list use the last pair */
    }

    *mul = cfg->scales[idx * 2];
    *div = cfg->scales[idx * 2 + 1];
}

static int profile_scaler_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct profile_scaler_config *cfg = dev->config;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool match = false;
    for (size_t i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            match = true;
            break;
        }
    }
    if (!match) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    uint32_t mul, div;
    get_scale(cfg, &mul, &div);
    if (div == 0) {
        div = 1;
    }

    int32_t value_mul = (int32_t)event->value * (int32_t)mul;
    if (state && state->remainder) {
        value_mul += *state->remainder;
    }

    int32_t scaled = value_mul / (int32_t)div;

    if (state && state->remainder) {
        *state->remainder = (int16_t)(value_mul - scaled * (int32_t)div);
    }

    event->value = CLAMP(scaled, INT16_MIN, INT16_MAX);

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api profile_scaler_driver_api = {
    .handle_event = profile_scaler_handle_event,
};

#define PS_INST(n)                                                                                 \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, profile_scales) >= 2 &&                                      \
                     (DT_INST_PROP_LEN(n, profile_scales) % 2) == 0,                               \
                 "profile-scales must be pairs of <multiplier divisor>");                          \
    static const uint16_t ps_codes_##n[] = DT_INST_PROP(n, codes);                                 \
    static const uint32_t ps_scales_##n[] = DT_INST_PROP(n, profile_scales);                       \
    static const struct profile_scaler_config ps_config_##n = {                                    \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = ps_codes_##n,                                                                     \
        .scales_len = DT_INST_PROP_LEN(n, profile_scales),                                         \
        .scales = ps_scales_##n,                                                                   \
        .has_usb_scale = DT_INST_NODE_HAS_PROP(n, usb_scale),                                      \
        .usb_mul = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, usb_scale),                                \
                               (DT_INST_PROP_BY_IDX(n, usb_scale, 0)), (1)),                       \
        .usb_div = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, usb_scale),                                \
                               (DT_INST_PROP_BY_IDX(n, usb_scale, 1)), (1)),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &ps_config_##n, POST_KERNEL,                        \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &profile_scaler_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS_INST)
