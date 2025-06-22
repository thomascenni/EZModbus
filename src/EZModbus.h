/**
 * @file EZModbus.h
 * @brief Main include file for the EZModbus library
 */

#pragma once

// Core components
#include "core/ModbusCore.h"
#include "core/ModbusCodec.hpp"
#include "interfaces/ModbusInterface.hpp"

// Drivers
#include "drivers/ModbusHAL_UART.h"
#include "drivers/ModbusHAL_TCP.h"

// Interfaces
#include "interfaces/ModbusRTU.h"
#include "interfaces/ModbusTCP.h"

// Application components
#include "apps/ModbusClient.h"
#include "apps/ModbusServer.h"
#include "apps/ModbusBridge.hpp"