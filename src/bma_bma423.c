#include "bma423.h"

#include "mpconfigport.h"
#include "mphalport.h"

#include "py/obj.h"
#include "py/runtime.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

typedef struct bma423_if_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c_obj;
    uint8_t address;
    struct bma4_dev bma423;
    uint8_t remap_axes;
    mp_obj_t int_obj[2];
    struct
    {
        mp_obj_t activity_handler;
        mp_obj_t single_tap_handler;
        mp_obj_t double_tap_handler;
        mp_obj_t wrist_wear_handler;
    } int_handler[2];
    uint16_t int_map;
} bma423_if_obj_t;

const mp_obj_type_t bma423_if_type;

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*!
 * @brief Converts raw sensor values(LSB) to meters per seconds square.
 *
 * @param[in] val       Raw sensor value.
 * @param[in] g_range   Accel Range selected (2G, 4G, 8G, 16G).
 * @param[in] bit_width Resolution of the sensor.
 *
 * @return Accel values in meters per second square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;
    return GRAVITY_EARTH * val * g_range / half_scale;
}


static int8_t i2c_bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
    mp_obj_t args[3];
    mp_obj_t dest[2], ret;
    const char *buf;
    size_t l = 0;
    bma423_if_obj_t *self = (bma423_if_obj_t *)intf_ptr;

    // mp_printf(&mp_plat_print, "i2c read (address=%d, reg=0x%x, len=%d)\r\n", self->address, reg, len);

    args[0] = mp_obj_new_int(self->address);
    args[1] = mp_obj_new_int(reg);
    args[2] = mp_obj_new_int(len);

    mp_load_method_maybe(self->i2c_obj, MP_QSTR_readfrom_mem, dest);
    ret = mp_call_method_self_n_kw(dest[0], dest[1], 3, 0, args);
    buf = mp_obj_str_get_data(ret, &l);
    memcpy(data, buf, l);
    return 0;
}


static int8_t i2c_bus_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
    mp_obj_t dest[2];
    mp_obj_t args[3];
    bma423_if_obj_t *self = (bma423_if_obj_t *)intf_ptr;

    // mp_printf(&mp_plat_print, "i2c write (address=%d, reg=0x%x, len=%d)\r\n", self->address, reg, len);

    args[0] = mp_obj_new_int(self->address);
    args[1] = mp_obj_new_int(reg);
    args[2] = mp_obj_new_bytes(data, len);

    mp_load_method_maybe(self->i2c_obj, MP_QSTR_writeto_mem, dest);
    mp_call_method_self_n_kw(dest[0], dest[1], 3, 0, args);
    return 0;
}


static void bma4_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    mp_hal_delay_us(period);
    return ;
}


static void bma4_error_codes_print_result(const char api_name[], uint16_t rslt) {
    if (rslt != BMA4_OK) {
        mp_printf(&mp_plat_print, "calling %s (error=%d, ", api_name, rslt);
        if (rslt & BMA4_E_NULL_PTR) {
            mp_printf(&mp_plat_print, "Null pointer)\r\n");
        } else if (rslt & BMA4_E_CONFIG_STREAM_ERROR) {
            mp_printf(&mp_plat_print, "Invalid configuration stream)\r\n");
        } else if (rslt & BMA4_E_SELF_TEST_FAIL) {
            mp_printf(&mp_plat_print, "Self test failed)\r\n");
        } else if (rslt & BMA4_E_INVALID_SENSOR) {
            mp_printf(&mp_plat_print, "Device not found)\r\n");
        } else {
            /* For more error codes refer "*_defs.h" */
            mp_printf(&mp_plat_print, "Unknown error code)\r\n");
        }
    }
}

typedef struct bma423_binding_int
{
    mp_obj_t pin_obj;
    bma423_if_obj_t *bma423_obj;
} bma423_binding_int;
static bma423_binding_int *bma423_binding_int_tables;
static int bma423_binding_int_tables_num = 0;

void store_iterm(mp_obj_t pin_obj, bma423_if_obj_t *bma423_obj) {
    for (size_t i = 0; i < bma423_binding_int_tables_num; i++) {
        if (bma423_binding_int_tables[i].pin_obj == MP_OBJ_NULL) {
            // mp_printf(&mp_plat_print, "store_iterm\r\n");
            bma423_binding_int_tables[i].pin_obj = pin_obj;
            bma423_binding_int_tables[i].bma423_obj = bma423_obj;
        }
    }
}

bma423_if_obj_t* load_iterm(mp_obj_t pin_obj) {
    for (size_t i = 0; i < bma423_binding_int_tables_num; i++) {
        if (bma423_binding_int_tables[i].pin_obj == pin_obj) {
            // mp_printf(&mp_plat_print, "load_iterm\r\n");
            return bma423_binding_int_tables[i].bma423_obj;
        }
    }
    return MP_OBJ_NULL;
}


mp_obj_t irq_handler(mp_obj_t self_in) {
    bma423_if_obj_t *bma423_obj = load_iterm(self_in);
    uint16_t int_status;
    uint8_t activity_out = 0;
    uint8_t int_line = 0;

    if (bma423_obj->int_obj[0] == self_in) {
        int_line = 0;
    } else {
        int_line = 1;
    }

    int8_t rslt = bma423_read_int_status(&int_status, &bma423_obj->bma423);
    bma4_error_codes_print_result("bma423_read_int_status", rslt);
    if (BMA4_OK == rslt) {
        if (int_status & BMA423_SINGLE_TAP_INT) {
            // mp_printf(&mp_plat_print, "Single tap received\n");
            if (bma423_obj->int_handler[int_line].single_tap_handler != MP_OBJ_NULL) {
                mp_sched_schedule(bma423_obj->int_handler[int_line].single_tap_handler, mp_const_none);
            }
        } else if (int_status & BMA423_DOUBLE_TAP_INT) {
            // mp_printf(&mp_plat_print, "Double tap received\n");
            if (bma423_obj->int_handler[int_line].double_tap_handler != MP_OBJ_NULL) {
                mp_sched_schedule(bma423_obj->int_handler[int_line].double_tap_handler, mp_const_none);
            }
        } else if (int_status & BMA423_WRIST_WEAR_INT) {
            // mp_printf(&mp_plat_print, "Wrist wear received\n");
            if (bma423_obj->int_handler[int_line].wrist_wear_handler != MP_OBJ_NULL) {
                mp_sched_schedule(bma423_obj->int_handler[int_line].wrist_wear_handler, mp_const_none);
            }
        } else if (int_status & BMA423_ACTIVITY_INT) {
            /* Read activity output register for recognizing specific activity */
            rslt = bma423_activity_output(&activity_out, &bma423_obj->bma423);
            bma4_error_codes_print_result("bma423_activity_output", rslt);
            #if 0
            switch (activity_out) {
                case BMA423_USER_STATIONARY:
                    mp_printf(&mp_plat_print, "User is stationary\n");
                    break;
                case BMA423_USER_WALKING:
                    mp_printf(&mp_plat_print, "User is walking\n");
                    break;
                case BMA423_USER_RUNNING:
                    mp_printf(&mp_plat_print, "User is running\n");
                    break;
                case BMA423_STATE_INVALID:
                    mp_printf(&mp_plat_print, "Invalid activity recognized\n");
                    break;
                default:
                    break;
            }
            #endif
            if (bma423_obj->int_handler[int_line].activity_handler != MP_OBJ_NULL) {
                mp_sched_schedule(bma423_obj->int_handler[int_line].activity_handler, mp_obj_new_int(activity_out));
            }
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(irq_handler_obj, irq_handler);


STATIC mp_obj_t bma423_make_new(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_i2c, ARG_address, ARG_int1, ARG_int2, ARG_debug};
    mp_arg_t make_new_args[] = {
        { MP_QSTR_i2c,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}           },
        { MP_QSTR_address, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = BMA4_I2C_ADDR_PRIMARY} },
        { MP_QSTR_int1,    MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL}           },
        { MP_QSTR_int2,    MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL}           },
        { MP_QSTR_debug,  MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_bool = false}                },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(make_new_args)];
    mp_arg_parse_all(n_args , pos_args, kw_args, MP_ARRAY_SIZE(make_new_args),
                     make_new_args, args);

    bma423_if_obj_t *self = m_new_obj_with_finaliser(bma423_if_obj_t);
    if (!self) {
        mp_raise_TypeError(MP_ERROR_TEXT("malloc fail"));
        return mp_const_none;
    }

    self->base.type = &bma423_if_type;

    self->i2c_obj = MP_OBJ_FROM_PTR(args[ARG_i2c].u_obj);
    if (self->i2c_obj == mp_const_none) {
        mp_raise_ValueError("must assignation i2c port");
        return mp_const_none;
    }
    self->address = args[ARG_address].u_int;
    self->int_obj[0] = args[ARG_int1].u_obj;
    self->int_obj[1] = args[ARG_int2].u_obj;

    self->bma423.intf_ptr = self;
    self->bma423.intf = BMA4_I2C_INTF;
    self->bma423.bus_read = i2c_bus_read;
    self->bma423.bus_write = i2c_bus_write;
    self->bma423.delay_us = bma4_delay_us;
    self->bma423.read_write_len = 8;
    self->bma423.variant = BMA42X_VARIANT;

    int8_t rslt = bma4_soft_reset(&self->bma423);
    bma4_error_codes_print_result("bma4_soft_reset", rslt);

    rslt = bma423_init(&self->bma423);
    bma4_error_codes_print_result("bma423_init", rslt);

    rslt = bma423_write_config_file(&self->bma423);
    bma4_error_codes_print_result("bma423_write_config_file", rslt);

    mp_obj_t dest[2];
    mp_obj_t args1[3];
    if (self->int_obj[0] != MP_OBJ_NULL) {
        // init
        args1[0] = mp_load_attr(self->int_obj[0], MP_QSTR_IN);
        mp_load_method_maybe(self->int_obj[0], MP_QSTR_init, dest);
        mp_call_method_self_n_kw(dest[0], dest[1], 1, 0, args1);

        if (MP_OBJ_NULL == bma423_binding_int_tables) {
            bma423_binding_int_tables = (bma423_binding_int *)m_malloc(sizeof(bma423_binding_int));
            bma423_binding_int_tables_num += 1;
        } else {
            bma423_binding_int_tables = m_realloc(bma423_binding_int_tables, (bma423_binding_int_tables_num + 1) * sizeof(bma423_binding_int));
            bma423_binding_int_tables_num += 1;
        }
        store_iterm(self->int_obj[0], self);

        // irq
        args1[0] = MP_ROM_PTR(&irq_handler_obj);
        args1[1] = mp_load_attr(self->int_obj[0], MP_QSTR_IRQ_RISING);
        mp_load_method_maybe(self->int_obj[0], MP_QSTR_irq, dest);
        mp_call_method_self_n_kw(dest[0], dest[1], 2, 0, args1);

        struct bma4_int_pin_config config ;
        config.edge_ctrl = BMA4_LEVEL_TRIGGER;
        config.lvl = BMA4_ACTIVE_HIGH;
        config.od = BMA4_PUSH_PULL;
        config.output_en = BMA4_OUTPUT_ENABLE;
        config.input_en = BMA4_INPUT_DISABLE;
        rslt = bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &self->bma423);
        bma4_error_codes_print_result("bma4_set_int_pin_config", rslt);
    }

    if (self->int_obj[1] != MP_OBJ_NULL) {
        // init
        args1[0] = mp_load_attr(self->int_obj[1], MP_QSTR_IN);
        mp_load_method_maybe(self->int_obj[1], MP_QSTR_init, dest);
        mp_call_method_self_n_kw(dest[0], dest[1], 1, 0, args1);

        if (MP_OBJ_NULL == bma423_binding_int_tables) {
            bma423_binding_int_tables = (bma423_binding_int *)m_malloc(sizeof(bma423_binding_int));
            bma423_binding_int_tables_num += 1;
        } else {
            bma423_binding_int_tables = m_realloc(bma423_binding_int_tables, (bma423_binding_int_tables_num + 1) * sizeof(bma423_binding_int));
            bma423_binding_int_tables_num += 1;
        }
        store_iterm(self->int_obj[1], self);

        // irq
        args1[0] = MP_ROM_PTR(&irq_handler_obj);
        args1[1] = mp_load_attr(self->int_obj[1], MP_QSTR_IRQ_RISING);
        mp_load_method_maybe(self->int_obj[1], MP_QSTR_irq, dest);
        mp_call_method_self_n_kw(dest[0], dest[1], 2, 0, args1);

        struct bma4_int_pin_config config ;
        config.edge_ctrl = BMA4_LEVEL_TRIGGER;
        config.lvl = BMA4_ACTIVE_HIGH;
        config.od = BMA4_PUSH_PULL;
        config.output_en = BMA4_OUTPUT_ENABLE;
        config.input_en = BMA4_INPUT_DISABLE;
        rslt = bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &self->bma423);
        bma4_error_codes_print_result("bma4_set_int_pin_config", rslt);
    }

    return MP_OBJ_FROM_PTR(self);
}
MP_DEFINE_CONST_FUN_OBJ_KW(bma423_make_new_obj, 1, bma423_make_new);


mp_obj_t delete(mp_obj_t self_in) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    m_del_obj(bma423_if_obj_t, self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(delete_obj, delete);


STATIC mp_obj_t accel_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_enable, ARG_direction, ARG_layer };
    mp_arg_t accel_config_args[] = {
        { MP_QSTR_enable,    MP_ARG_REQUIRED | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_direction, MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = 0     } },
        { MP_QSTR_layer,     MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = 1     } },
    };
    struct bma423_axes_remap remap;
    mp_arg_val_t args[MP_ARRAY_SIZE(accel_config_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
                     MP_ARRAY_SIZE(accel_config_args), accel_config_args, args);

    bma423_if_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    int8_t rslt = bma4_set_accel_enable(args[ARG_enable].u_bool, &self->bma423);
    bma4_error_codes_print_result("bma4_set_accel_enable", rslt);

    if (args[ARG_layer].u_int) {
        switch (args[ARG_direction].u_int)
        {
            case 0:
                remap.x_axis = 0;
                remap.x_axis_sign = 0;
                remap.y_axis = 1;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 0;
                self->remap_axes = 0;
                break;

            case 1:
                remap.x_axis = 0;
                remap.x_axis_sign = 1;
                remap.y_axis = 1;
                remap.y_axis_sign = 1;
                remap.z_axis = 2;
                remap.z_axis_sign = 0;
                self->remap_axes = 1;
                break;

            case 2:
                remap.x_axis = 1;
                remap.x_axis_sign = 1;
                remap.y_axis = 0;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 0;
                self->remap_axes = 2;
                break;

            case 3:
                remap.x_axis = 1;
                remap.x_axis_sign = 0;
                remap.y_axis = 0;
                remap.y_axis_sign = 1;
                remap.z_axis = 2;
                remap.z_axis_sign = 0;
                self->remap_axes = 3;
                break;

            default:
                remap.x_axis = 0;
                remap.x_axis_sign = 0;
                remap.y_axis = 1;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 0;
                self->remap_axes = 0;
                break;
        }
    } else {
        switch (args[ARG_direction].u_int)
        {
            case 0:
                remap.x_axis = 1;
                remap.x_axis_sign = 0;
                remap.y_axis = 0;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 1;
                self->remap_axes = 4;
                break;

            case 1:
                remap.x_axis = 1;
                remap.x_axis_sign = 1;
                remap.y_axis = 0;
                remap.y_axis_sign = 1;
                remap.z_axis = 2;
                remap.z_axis_sign = 1;
                self->remap_axes = 5;
                break;

            case 2:
                remap.x_axis = 0;
                remap.x_axis_sign = 1;
                remap.y_axis = 1;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 1;
                self->remap_axes = 6;
                break;

            case 3:
                remap.x_axis = 0;
                remap.x_axis_sign = 0;
                remap.y_axis = 1;
                remap.y_axis_sign = 1;
                remap.z_axis = 2;
                remap.z_axis_sign = 1;
                self->remap_axes = 7;
                break;

            default:
                remap.x_axis = 1;
                remap.x_axis_sign = 0;
                remap.y_axis = 0;
                remap.y_axis_sign = 0;
                remap.z_axis = 2;
                remap.z_axis_sign = 1;
                self->remap_axes = 4;
                break;
        }
    }

    rslt = bma423_set_remap_axes(&remap, &self->bma423);
    bma4_error_codes_print_result("bma423_set_remap_axes", rslt);

    // todo: bma4_set_accel_config

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(accel_config_obj, 2, accel_config);


STATIC mp_obj_t accel_read(mp_obj_t self_in) {
    mp_obj_t value[3];
    struct bma4_accel sens_data;
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_read_accel_xyz(&sens_data, &self->bma423);
    bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

    switch (self->remap_axes)
    {
        case 0:
            value[0] = mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 1:
            value[0] = mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 2:
            value[0] = mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 3:
            value[0] = mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 4:
            value[0] = mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 5:
            value[0] = mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 6:
            value[0] = mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 7:
            value[0] = mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        default:
            value[0] = mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            value[1] = mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            value[2] = mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;
    }

    return mp_obj_new_tuple(3, value);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(accel_read_obj, accel_read);


STATIC mp_obj_t accel_x_read(mp_obj_t self_in) {
    struct bma4_accel sens_data;
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_read_accel_xyz(&sens_data, &self->bma423);
    bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

    switch (self->remap_axes)
    {
        case 0:
            return mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 1:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 2:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 3:
            return mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 4:
            return mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 5:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 6:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 7:
            return mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        default:
            return mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;
    }

    return mp_obj_new_float(0.0);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(accel_read_x_obj, accel_x_read);


STATIC mp_obj_t accel_y_read(mp_obj_t self_in) {
    struct bma4_accel sens_data;
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_read_accel_xyz(&sens_data, &self->bma423);
    bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

    switch (self->remap_axes)
    {
        case 0:
            return mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 1:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 2:
            return mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 3:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 4:
            return mp_obj_new_float(lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 5:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.x, 2, self->bma423.resolution));
            break;

        case 6:
            return mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        case 7:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;

        default:
            return mp_obj_new_float(lsb_to_ms2(sens_data.y, 2, self->bma423.resolution));
            break;
    }

    return mp_obj_new_float(0.0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(accel_read_y_obj, accel_y_read);


STATIC mp_obj_t accel_z_read(mp_obj_t self_in) {
    struct bma4_accel sens_data;
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_read_accel_xyz(&sens_data, &self->bma423);
    bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

    switch (self->remap_axes)
    {
        case 0:
            return mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 1:
            return mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 2:
            return mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 3:
            return mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 4:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 5:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 6:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        case 7:
            return mp_obj_new_float(-lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;

        default:
            return mp_obj_new_float(lsb_to_ms2(sens_data.z, 2, self->bma423.resolution));
            break;
    }

    return mp_obj_new_float(0.0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(accel_read_z_obj, accel_z_read);


STATIC mp_obj_t temperature_read(mp_obj_t self_in) {
    int32_t temp = 0;
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_get_temperature(&temp, BMA4_DEG, &self->bma423);
    bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

    return mp_obj_new_float((float)(temp / BMA4_SCALE_TEMP));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(temperature_read_obj, temperature_read);


STATIC mp_obj_t reset(mp_obj_t self_in) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t rslt = bma4_soft_reset(&self->bma423);
    bma4_error_codes_print_result("bma4_soft_reset", rslt);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(reset_obj, reset);


STATIC mp_obj_t clear(mp_obj_t self_in) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t rslt;

    // int8_t rslt = bma4_soft_reset(&self->bma423);
    // bma4_error_codes_print_result("bma4_soft_reset", rslt);

    rslt = bma423_reset_step_counter(&self->bma423);
    bma4_error_codes_print_result("bma423_reset_step_counter", rslt);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(clear_obj, clear);


STATIC mp_obj_t step_config(mp_obj_t self_in, mp_obj_t enable_in) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t rslt;
    struct bma4_accel_config accel_conf;

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(1, &self->bma423);
    bma4_error_codes_print_result("bma4_set_accel_enable", rslt);

    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    accel_conf.range = BMA4_ACCEL_RANGE_2G;
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;
    rslt = bma4_set_accel_config(&accel_conf, &self->bma423);
    bma4_error_codes_print_result("bma4_set_accel_config", rslt);

    rslt = bma423_feature_enable(BMA423_STEP_CNTR, mp_obj_is_true(enable_in), &self->bma423);
    bma4_error_codes_print_result("bma423_feature_enable", rslt);

    // rslt = bma423_step_counter_set_watermark(1, &self->bma423);
    // bma4_error_codes_print_result("bma423_step_counter status", rslt);

    // rslt = bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_STEP_CNTR_INT, 1, &self->bma423);
    // bma4_error_codes_print_result("bma423_map_interrupt status", rslt);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(step_config_obj, step_config);


STATIC mp_obj_t step_counter(mp_obj_t self_in) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t step_count = 0;
    int8_t rslt;

    rslt = bma423_step_counter_output(&step_count, &self->bma423);
    bma4_error_codes_print_result("bma423_step_counter_output", rslt);

    return mp_obj_new_int(step_count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(step_counter_obj, step_counter);


enum { ARG_handler, ARG_int_line};
static mp_arg_t handler_args[] = {
    { MP_QSTR_handler,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_int_line, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 1}           },
};

STATIC mp_obj_t activity(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int8_t rslt;
    uint8_t int_line;

    mp_arg_val_t args[MP_ARRAY_SIZE(handler_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(handler_args),
                     handler_args, args);

    int_line = args[ARG_int_line].u_int - 1;

    if (0 == int_line || 1 == int_line) {
        if (self->int_obj[int_line] != MP_OBJ_NULL) {
            /* set activity handler */
            self->int_handler[int_line].activity_handler = args[ARG_handler].u_obj;
            /* Enable activity feature */
            rslt = bma423_feature_enable(BMA423_STEP_ACT, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_feature_enable", rslt);
            self->int_map |= BMA423_ACTIVITY_INT;
            rslt = bma423_map_interrupt(int_line, self->int_map, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_map_interrupt", rslt);
        } else {
            mp_raise_ValueError("Interrupt not enabled");
        }
    } else {
        mp_raise_ValueError("only accepts 1 or 2");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(activity_obj, 2, activity);


STATIC mp_obj_t single_tap(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int8_t rslt;
    uint8_t int_line;

    mp_arg_val_t args[MP_ARRAY_SIZE(handler_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(handler_args),
                     handler_args, args);

    int_line = args[ARG_int_line].u_int - 1;

    if (0 == int_line || 1 == int_line) {
        if (self->int_obj[int_line] != MP_OBJ_NULL) {
            /* set irq handler */
            self->int_handler[int_line].single_tap_handler = args[ARG_handler].u_obj;
            /* Enable single tap feature */
            rslt = bma423_feature_enable(BMA423_SINGLE_TAP, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_feature_enable", rslt);
            self->int_map |= BMA423_SINGLE_TAP_INT;
            rslt = bma423_map_interrupt(int_line, self->int_map, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_map_interrupt", rslt);
        } else {
            mp_raise_ValueError("Interrupt not enabled");
        }
    } else {
        mp_raise_ValueError("only accepts 1 or 2");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(single_tap_obj, 2, single_tap);


STATIC mp_obj_t double_tap(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int8_t rslt;
    uint8_t int_line;

    mp_arg_val_t args[MP_ARRAY_SIZE(handler_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(handler_args),
                     handler_args, args);

    int_line = args[ARG_int_line].u_int - 1;

    if (0 == int_line || 1 == int_line) {
        if (self->int_obj[int_line] != MP_OBJ_NULL) {
            /* set irq handler */
            self->int_handler[int_line].double_tap_handler = args[ARG_handler].u_obj;
            /* Enable double tap feature */
            rslt = bma423_feature_enable(BMA423_DOUBLE_TAP, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_feature_enable", rslt);
            self->int_map |= BMA423_DOUBLE_TAP_INT;
            rslt = bma423_map_interrupt(int_line, self->int_map, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_map_interrupt", rslt);
        } else {
            mp_raise_ValueError("Interrupt not enabled");
        }
    } else {
        mp_raise_ValueError("only accepts 1 or 2");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(double_tap_obj, 2, double_tap);


STATIC mp_obj_t wrist_wear(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    bma423_if_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int8_t rslt;
    uint8_t int_line;

    mp_arg_val_t args[MP_ARRAY_SIZE(handler_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(handler_args),
                     handler_args, args);

    int_line = args[ARG_int_line].u_int - 1;

    if (0 == int_line || 1 == int_line) {
        if (self->int_obj[int_line] != MP_OBJ_NULL) {
            /* set irq handler */
            self->int_handler[int_line].wrist_wear_handler = args[ARG_handler].u_obj;
            /* Enable double tap feature */
            rslt = bma423_feature_enable(BMA423_WRIST_WEAR, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_feature_enable", rslt);
            self->int_map |= BMA423_WRIST_WEAR_INT;
            rslt = bma423_map_interrupt(int_line, self->int_map, BMA4_ENABLE, &self->bma423);
            bma4_error_codes_print_result("bma423_map_interrupt", rslt);
        } else {
            mp_raise_ValueError("Interrupt not enabled");
        }
    } else {
        mp_raise_ValueError("only accepts 1 or 2");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(wrist_wear_obj, 2, wrist_wear);


STATIC const mp_rom_map_elem_t bma423_if_locals_dict_table[] = {
    // method
    { MP_ROM_QSTR(MP_QSTR___del__),      MP_ROM_PTR(&delete_obj)           },
    { MP_ROM_QSTR(MP_QSTR_deinit),       MP_ROM_PTR(&delete_obj)           },
    { MP_ROM_QSTR(MP_QSTR_accel_config), MP_ROM_PTR(&accel_config_obj)     },
    { MP_ROM_QSTR(MP_QSTR_accel),        MP_ROM_PTR(&accel_read_obj)       },
    { MP_ROM_QSTR(MP_QSTR_x),            MP_ROM_PTR(&accel_read_x_obj)     },
    { MP_ROM_QSTR(MP_QSTR_y),            MP_ROM_PTR(&accel_read_y_obj)     },
    { MP_ROM_QSTR(MP_QSTR_z),            MP_ROM_PTR(&accel_read_z_obj)     },
    { MP_ROM_QSTR(MP_QSTR_temperature),  MP_ROM_PTR(&temperature_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),        MP_ROM_PTR(&reset_obj)            },
    { MP_ROM_QSTR(MP_QSTR_clear),        MP_ROM_PTR(&clear_obj)            },
    { MP_ROM_QSTR(MP_QSTR_step_config),  MP_ROM_PTR(&step_config_obj)      },
    { MP_ROM_QSTR(MP_QSTR_step_counter), MP_ROM_PTR(&step_counter_obj)     },
    { MP_ROM_QSTR(MP_QSTR_activity),     MP_ROM_PTR(&activity_obj)         },
    { MP_ROM_QSTR(MP_QSTR_single_tap),   MP_ROM_PTR(&single_tap_obj)       },
    { MP_ROM_QSTR(MP_QSTR_double_tap),   MP_ROM_PTR(&double_tap_obj)       },
    { MP_ROM_QSTR(MP_QSTR_wrist_wear),   MP_ROM_PTR(&wrist_wear_obj)       },

    { MP_ROM_QSTR(MP_QSTR_BOTTOM_LAYER), MP_ROM_INT(0)                     },
    { MP_ROM_QSTR(MP_QSTR_TOP_LAYER),    MP_ROM_INT(1)                     },
    { MP_ROM_QSTR(MP_QSTR_UPPER_RIGHT),  MP_ROM_INT(0)                     },
    { MP_ROM_QSTR(MP_QSTR_LOWER_LEFT),   MP_ROM_INT(1)                     },
    { MP_ROM_QSTR(MP_QSTR_UPPER_LEFT),   MP_ROM_INT(2)                     },
    { MP_ROM_QSTR(MP_QSTR_LOWER_RIGHT),  MP_ROM_INT(3)                     },
    { MP_ROM_QSTR(MP_QSTR_INT1),         MP_ROM_INT(1)                     },
    { MP_ROM_QSTR(MP_QSTR_INT2),         MP_ROM_INT(2)                     },
};
STATIC MP_DEFINE_CONST_DICT(bma423_if_locals_dict, bma423_if_locals_dict_table);


const mp_obj_type_t bma423_if_type = {
    { &mp_type_type },
    .name = MP_QSTR_BMA423,
    // .make_new = bma423_make_new,
    .locals_dict = (mp_obj_dict_t *)&bma423_if_locals_dict,
};
