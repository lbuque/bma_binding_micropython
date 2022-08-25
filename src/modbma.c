/*
 * This file is part of the MicroPython ESP32 project,
 * https://github.com/lewisxhe/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 lewisxhe (https://github.com/lewisxhe)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// #ifdef CONFIG_MICROPY_USE_BMA423

#include "modbma.h"

#include "py/runtime.h"
#include "py/obj.h"


STATIC const mp_rom_map_elem_t bma_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bma) },
    { MP_ROM_QSTR(MP_QSTR_BMA423), MP_ROM_PTR(&bma423_make_new_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bma_module_globals, bma_module_globals_table);


const mp_obj_module_t mp_module_bma = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bma_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bma, mp_module_bma);

// #endif