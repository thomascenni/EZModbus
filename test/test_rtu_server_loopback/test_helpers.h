#pragma once

#include <atomic>
#include <vector>
#include <unity.h>
#include "EZModbus.h"
#include <utils/ModbusLogger.h>

// Give some time for the application logs to be printed before asserting
#ifdef EZMODBUS_DEBUG
    #define TEST_ASSERT_START() { Modbus::Logger::waitQueueFlushed(); }
#else
    #define TEST_ASSERT_START() { vTaskDelay(pdMS_TO_TICKS(50)); }
#endif

// Pour simplifier les noms
using ServerReg = Modbus::Server::Register;
using RegType = Modbus::RegisterType;

// 1) L’objet TestRegister garde sa valeur en vie et fournit le ModbusRegister
struct DummyRegister {
    uint32_t value;
    ServerReg reg;

    DummyRegister(RegType RT, uint16_t addr) {
        // Define the value as address of the register (or 1 if it's a coil or discrete input)
        switch (RT) {
            case Modbus::DISCRETE_INPUT:
            case Modbus::COIL:
                value = 1;
                break;
            case Modbus::HOLDING_REGISTER:
            case Modbus::INPUT_REGISTER:
                value = addr;
                break;
        }
        reg.type    = RT;
        reg.address = addr;
        reg.readCb  = [this](const ServerReg& reg) { return static_cast<uint16_t>(value); };
        if (RT == Modbus::HOLDING_REGISTER || RT == Modbus::COIL) {
            reg.writeCb = [this](uint16_t v, const ServerReg& reg) -> bool {
                value = v;
                return true;
            };
        } else {
            reg.writeCb = nullptr;
        }
        reg.name = nullptr;
    }
};

// 2) TestRegisterSuite stocke les TestRegister et expose leur ModbusRegister
template<RegType RT>
struct DummyRegSuite {
    std::vector<DummyRegister> storage;      // garde en vie tous les TestRegister
    std::vector<ServerReg>     handle;         // copie des ModbusRegister pour addRegisters()

    DummyRegSuite(uint16_t start, size_t N) {
        storage.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            storage.emplace_back(RT, uint16_t(start + i));
        }
        update();
    }

    void update() {
        handle.clear();
        handle.reserve(storage.size());
        for (auto& tr : storage) {
            handle.push_back(tr.reg);
        }
    }
};

// 3) Helper générique pour valider addRegisters()
inline void do_test_add_register(Modbus::Server& server,
                                  const ServerReg& reg,
                                  bool expectSuccess,
                                  const char* desc)
{
    auto res = server.addRegister(reg);
    TEST_ASSERT_START();
    if (expectSuccess)
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
    else
        TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
}

// 3) Helper générique pour valider addRegisters()
inline void do_test_add_registers(Modbus::Server& server,
                                  const std::vector<ServerReg>& regs,
                                  bool expectSuccess,
                                  const char* desc)
{
    auto res = server.addRegisters(regs);
    TEST_ASSERT_START();
    if (expectSuccess)
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
    else
        TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
}
