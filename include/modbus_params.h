/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*=====================================================================================
 * Description:
 *   The Modbus parameter structures used to define Modbus instances that
 *   can be addressed by Modbus protocol. Define these structures per your needs in
 *   your application. Below is just an example of possible parameters.
 *====================================================================================*/
#ifndef MODBUS_PARAMS__H__
#define MODBUS_PARAMS__H__

#include <stdint.h>

#include "esp_err.h"

#include "esp_modbus_slave.h"

// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
// It also has proper access funtions to access those parameters.

typedef enum
{
    ACTUAL_TEMP_DEGC = 0,
    MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT,
} ModbusParams_InReg_Float_t;
typedef enum
{
    PELTIER_DRIVER_STATE = 0,
    MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT,
} ModbusParams_InReg_UInt_t;

typedef enum
{
    REQUESTED_TEMP_DEGC = 0,
    MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT,
} ModbusParams_HoldReg_Float_t;
typedef enum
{
    TEMP_SENSE_PERIOD_MS = 0,
    MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT,
} ModbusParams_HoldReg_UInt_t;

typedef enum
{
    ENABLE_TEMP_CONTROL = 0,
    ENABLE_PELTIER_OVERRIDE,
    OVERRIDE_PELTIER_COOLING,
    OVERRIDE_PELTIER_HEATING,
    MODBUS_PARAMS_COIL_COUNT,
} ModbusParams_Coil_t;

typedef enum
{
    TEMP_CONTROL_ENABLED = 0,
    MODBUS_PARAMS_DISCRETE_INPUT_COUNT,
} ModbusParams_DiscreteInput_t;

#define MODBUS_PARAMS_COIL_PORTS_COUNT           ((uint8_t)(MODBUS_PARAMS_COIL_COUNT / 8) + 1)
#define MODBUS_PARAMS_DISCRETE_INPUT_PORTS_COUNT ((uint8_t)(MODBUS_PARAMS_DISCRETE_INPUT_COUNT / 8) + 1)

esp_err_t modbus_params_init(void *slave_handler);

esp_err_t modbus_params_get_input_register_float_reg_area(ModbusParams_InReg_Float_t           index,
                                                          mb_register_area_descriptor_t *const reg_area);
esp_err_t modbus_params_get_input_register_uint_reg_area(ModbusParams_InReg_UInt_t            index,
                                                         mb_register_area_descriptor_t *const reg_area);
esp_err_t modbus_params_get_holding_register_uint_reg_area(ModbusParams_HoldReg_UInt_t          index,
                                                           mb_register_area_descriptor_t *const reg_area);
esp_err_t modbus_params_get_holding_register_float_reg_area(ModbusParams_HoldReg_Float_t         index,
                                                            mb_register_area_descriptor_t *const reg_area);
esp_err_t modbus_params_get_coil_port_reg_area(uint8_t index, mb_register_area_descriptor_t *const reg_area);
esp_err_t modbus_params_get_discrete_input_port_reg_area(ModbusParams_DiscreteInput_t         index,
                                                         mb_register_area_descriptor_t *const reg_area);

esp_err_t modbus_params_set_input_register_float(ModbusParams_InReg_Float_t index, float value);
esp_err_t modbus_params_set_input_register_uint(ModbusParams_InReg_UInt_t index, uint16_t value);

esp_err_t modbus_params_set_holding_register_uint(ModbusParams_HoldReg_UInt_t index, uint16_t value);
esp_err_t modbus_params_get_holding_register_uint(ModbusParams_HoldReg_UInt_t index, uint16_t *const value);

esp_err_t modbus_params_set_holding_register_float(ModbusParams_HoldReg_Float_t index, float value);
esp_err_t modbus_params_get_holding_register_float(ModbusParams_HoldReg_Float_t index, float *const value);

esp_err_t modbus_params_set_coil_state(ModbusParams_Coil_t index, bool state);
esp_err_t modbus_params_get_coil_state(ModbusParams_Coil_t index, bool *const state);

esp_err_t modbus_params_set_discrete_input_state(ModbusParams_DiscreteInput_t index, bool state);

#endif // MODBUS_PARAMS__H__
