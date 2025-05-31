#include <Arduino.h>
#include <unity.h>
#include <EZModbus.h>
#include "SafeLogSink.h"
#include "test_helpers.h"
#include <ModbusTestAgent.h>
#include <ModbusTestClient.h>
#include <utils/ModbusLogger.h>

// Pin definitions
#define MBT_RX D7 // ModbusTest RX
#define MBT_TX D8 // ModbusTest TX
#define EZM_RX D5 // EZModbus RX
#define EZM_TX D6 // EZModbus TX

// Serial ports definitions
#define EZM_SERIAL Serial1
#define MBT_SERIAL Serial2

// Create our test agent and client
SafeLogStream _agentLogStream;
ModbusTestAgent agent(MBT_SERIAL, 20000, &_agentLogStream);
ModbusTestClient testClient(agent); 

#define HOLDING_REGISTERS 3
#define INPUT_REGISTERS 4
#define COILS 0
#define DISCRETE_INPUTS 1

// EZModbus RTU server
ModbusHAL::UART ezm_uart(UART_NUM_1, 9600, ModbusHAL::UART::CONFIG_8N1, EZM_RX, EZM_TX);
ModbusInterface::RTU rtu(ezm_uart, Modbus::SLAVE);
Modbus::Server server(rtu, 1, 1);

// Tasks
TaskHandle_t ezmTaskHandle = NULL;
bool ezmTaskInitialized = false;

void flushSerialBuffer(HardwareSerial& port) {
    while (port.available()) {
        port.read();
    }
}

// EZModbus server task
void EZModbusServerTask(void* pModbusServer) {
    Modbus::Server* srv = (Modbus::Server*)pModbusServer;
    auto srvInitRes = srv->begin();
    if (srvInitRes != Modbus::Server::SUCCESS) {
        Modbus::Logger::logln("[EZModbusServerTask] EZModbus Server initialization failed");
        vTaskDelete(NULL);
        return;
    }
    Modbus::Logger::logln("[EZModbusServerTask] EZModbus Server (specific SlaveID) initialized.");

    while (true) {
        if (pModbusServer == &server) {
             ezmTaskInitialized = true;
        }
        srv->poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setUp() {
    // Clear both RX buffers before each test
    vTaskDelay(pdMS_TO_TICKS(10));
    ezm_uart.flush_input();
    flushSerialBuffer(MBT_SERIAL);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void tearDown() {
    server.clearAllRegisters();
}

// ===================================================================================
// REGISTER STORE TESTS
// ===================================================================================

void test_add_single_registers() {
    Modbus::Logger::logln("\nTEST_ADD_SINGLE_REGISTERS");
    // Define 1 register of each type
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    // Add registers with single add command
    do_test_add_register(server, h.reg, true, "holding add failed");
    do_test_add_register(server, i.reg, true, "input add failed");
    do_test_add_register(server, c.reg, true, "coil add failed");
    do_test_add_register(server, d.reg, true, "discrete input add failed");
}

void test_add_multiple_registers() {
    Modbus::Logger::logln("\nTEST_ADD_MULTIPLE_REGISTERS");
    // Define 5 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER>   h(10,5);
    DummyRegSuite<Modbus::INPUT_REGISTER>     i(20,5);
    DummyRegSuite<Modbus::COIL>               c(30,5);
    DummyRegSuite<Modbus::DISCRETE_INPUT>     d(40,5);
    // Add registers with multiple add command
    do_test_add_registers(server, h.handle, true, "5 holding add failed");
    do_test_add_registers(server, i.handle, true, "5 input add failed");
    do_test_add_registers(server, c.handle, true, "5 coil add failed");
    do_test_add_registers(server, d.handle, true, "5 discrete input add failed");
}

// void test_add_overflow_registers() {
//     Modbus::Logger::logln("\nTEST_ADD_OVERFLOW_REGISTERS");
//     // Define max number of registers per type
//     DummyRegSuite<Modbus::HOLDING_REGISTER> h(1, Modbus::Server::MAX_REGISTERS);
//     DummyRegSuite<Modbus::INPUT_REGISTER> i(1, Modbus::Server::MAX_REGISTERS);
//     DummyRegSuite<Modbus::COIL> c(1, Modbus::Server::MAX_REGISTERS);
//     DummyRegSuite<Modbus::DISCRETE_INPUT> d(1, Modbus::Server::MAX_REGISTERS);
//     // Define one more register to overflow
//     DummyRegister h2(Modbus::HOLDING_REGISTER, Modbus::Server::MAX_REGISTERS+1);
//     DummyRegister c2(Modbus::INPUT_REGISTER, Modbus::Server::MAX_REGISTERS+1);
//     DummyRegister i2(Modbus::COIL, Modbus::Server::MAX_REGISTERS+1);
//     DummyRegister d2(Modbus::DISCRETE_INPUT, Modbus::Server::MAX_REGISTERS+1);
//     // Add all of them, first add should succeed, second add should fail
//     do_test_add_registers(server, h.handle, true, "fill with holding failed");
//     do_test_add_registers(server, i.handle, true, "fill with holding failed");
//     do_test_add_registers(server, c.handle, true, "fill with holding failed");
//     do_test_add_registers(server, d.handle, true, "fill with holding add failed");
//     do_test_add_register(server, h2.reg, false, "overflow holding add failed");
//     do_test_add_register(server, i2.reg, false, "overflow holding add failed");
//     do_test_add_register(server, c2.reg, false, "overflow holding add failed");
//     do_test_add_register(server, d2.reg, false, "overflow holding add failed");
// }

void test_add_no_read_callback() {
    Modbus::Logger::logln("\nTEST_ADD_NO_READ_CALLBACK");
    // Define a register with no read callback
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    h.reg.readCb = nullptr;
    // Add the register, this should fail
    do_test_add_register(server, h.reg, false, "no readCb add failed");
}

void test_add_readonly_with_write_callback() {
    Modbus::Logger::logln("\nTEST_ADD_READONLY_WITH_WRITE_CALLBACK");
    // Define one readonly register of each type & add a write callback
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    i.reg.writeCb = [](uint16_t v, const Modbus::Server::Register& r) -> bool { return true; };
    d.reg.writeCb = [](uint16_t v, const Modbus::Server::Register& r) -> bool { return true; };
    // Add the registers, should fail
    do_test_add_register(server, i.reg, false, "readonly with writeCb add failed");
    do_test_add_register(server, d.reg, false, "readonly with writeCb add failed");
}

void test_add_write_without_write_callback() {
    Modbus::Logger::logln("\nTEST_ADD_WRITE_WITHOUT_WRITE_CALLBACK");
    // Define one write register of each typewith no write callback
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    h.reg.writeCb = nullptr;
    c.reg.writeCb = nullptr;
    // Add the registers, should fail
    do_test_add_register(server, h.reg, false, "write without writeCb add failed");
    do_test_add_register(server, c.reg, false, "write without writeCb add failed");
}

void test_register_bad_type() {
    Modbus::Logger::logln("\nTEST_REGISTER_BAD_TYPE");
    // Define a register with a bad type (cast a random value to the RegisterType enum)
    const Modbus::RegisterType badType = static_cast<Modbus::RegisterType>(42);
    DummyRegister b(badType, 1);
    // Add the register, should fail
    do_test_add_register(server, b.reg, false, "bad type add failed");
}

void test_get_register() {
    Modbus::Logger::logln("\nTEST_GET_REGISTER");
    // Add registers
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    do_test_add_register(server, h.reg, true, "holding add failed");
    do_test_add_register(server, i.reg, true, "input add failed");
    do_test_add_register(server, c.reg, true, "coil add failed");
    do_test_add_register(server, d.reg, true, "discrete input add failed");
    // Test get register
    TEST_ASSERT_TRUE(server.getRegister(Modbus::HOLDING_REGISTER, 1));
    TEST_ASSERT_TRUE(server.getRegister(Modbus::INPUT_REGISTER, 1));
    TEST_ASSERT_TRUE(server.getRegister(Modbus::COIL, 1));
    TEST_ASSERT_TRUE(server.getRegister(Modbus::DISCRETE_INPUT, 1));
    // Test get non-existing register
    TEST_ASSERT_FALSE(server.getRegister(Modbus::HOLDING_REGISTER, 2));
    TEST_ASSERT_FALSE(server.getRegister(Modbus::INPUT_REGISTER, 2));
    TEST_ASSERT_FALSE(server.getRegister(Modbus::COIL, 2));
    TEST_ASSERT_FALSE(server.getRegister(Modbus::DISCRETE_INPUT, 2));
}

// void test_clear_registers() {
//     Modbus::Logger::logln("\nTEST_CLEAR_REGISTERS");
//     // Add registers
//     DummyRegSuite<Modbus::HOLDING_REGISTER> h(10,5);
//     DummyRegSuite<Modbus::INPUT_REGISTER> i(20,5);
//     DummyRegSuite<Modbus::COIL> c(30,5);
//     DummyRegSuite<Modbus::DISCRETE_INPUT> d(40,5);
//     do_test_add_registers(server, h.handle, true, "5 holding add failed");
//     do_test_add_registers(server, i.handle, true, "5 input add failed");
//     do_test_add_registers(server, c.handle, true, "5 coil add failed");
//     do_test_add_registers(server, d.handle, true, "5 discrete input add failed");
//     // Clear registers and test if all of them are cleared
//     server.clearRegisters(Modbus::HOLDING_REGISTER);
//     server.clearRegisters(Modbus::INPUT_REGISTER);
//     server.clearRegisters(Modbus::COIL);
//     server.clearRegisters(Modbus::DISCRETE_INPUT);
//     for (auto& reg : h.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
//     for (auto& reg : i.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
//     for (auto& reg : c.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
//     for (auto& reg : d.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
// }

void test_clear_all_registers() {
    Modbus::Logger::logln("\nTEST_CLEAR_ALL_REGISTERS");
    // Add registers
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(10,5);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(20,5);
    DummyRegSuite<Modbus::COIL> c(30,5);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(40,5);
    // Clear all registers and test if all of them are cleared
    server.clearAllRegisters();
    for (auto& reg : h.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : i.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : c.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : d.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
}

void test_add_registers_atomicity() {
    Modbus::Logger::logln("\nTEST_ADD_REGISTERS_ATOMICITY");
    // Define registers
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(10,5);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(20,5);
    DummyRegSuite<Modbus::COIL> c(30,5);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(40,5);
    // Introduce error : clear readCb from a random register of each type
    h.handle[2].readCb = nullptr;
    i.handle[2].readCb = nullptr;
    c.handle[2].readCb = nullptr;
    d.handle[2].readCb = nullptr;
    // Try to add registers, this must fail
    do_test_add_registers(server, h.handle, false, "5 holding with 1 wrong add failed");
    do_test_add_registers(server, i.handle, false, "5 input with 1 wrong add failed");
    do_test_add_registers(server, c.handle, false, "5 coil with 1 wrong add failed");
    do_test_add_registers(server, d.handle, false, "5 discrete input with 1 wrong add failed");
    // Check that NO register was added
    for (auto& reg : h.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : i.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : c.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
    for (auto& reg : d.handle) TEST_ASSERT_FALSE(server.getRegister(reg.type, reg.address));
}

void test_register_overwrite() {
    Modbus::Logger::logln("\nTEST_REGISTER_OVERWRITE");
    // Define a set of registers
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    // Define another set of registers with a different read callback returning "42"
    DummyRegister h2(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i2(Modbus::INPUT_REGISTER, 1);
    DummyRegister c2(Modbus::COIL, 1);
    DummyRegister d2(Modbus::DISCRETE_INPUT, 1);
    h2.reg.readCb = [](const Modbus::Server::Register& r) -> uint16_t { return (uint16_t)42; };
    i2.reg.readCb = [](const Modbus::Server::Register& r) -> uint16_t { return (uint16_t)42; };
    c2.reg.readCb = [](const Modbus::Server::Register& r) -> uint16_t { return (uint16_t)42; };
    d2.reg.readCb = [](const Modbus::Server::Register& r) -> uint16_t { return (uint16_t)42; };
    // Add the first set of registers
    do_test_add_register(server, h.reg, true, "holding add failed");
    do_test_add_register(server, i.reg, true, "input add failed");
    do_test_add_register(server, c.reg, true, "coil add failed");
    do_test_add_register(server, d.reg, true, "discrete input add failed");
    // Add the second set of registers
    do_test_add_register(server, h2.reg, true, "overwrite holding add failed");
    do_test_add_register(server, i2.reg, true, "overwrite input add failed");
    do_test_add_register(server, c2.reg, true, "overwrite coil add failed");
    do_test_add_register(server, d2.reg, true, "overwrite discrete input add failed");
    // Check that the second set of registers overwrote the first set
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(42, server.getRegister(Modbus::HOLDING_REGISTER, 1).readCb(h2.reg));
    TEST_ASSERT_EQUAL(42, server.getRegister(Modbus::INPUT_REGISTER, 1).readCb(i2.reg));
    TEST_ASSERT_EQUAL(42, server.getRegister(Modbus::COIL, 1).readCb(c2.reg));
    TEST_ASSERT_EQUAL(42, server.getRegister(Modbus::DISCRETE_INPUT, 1).readCb(d2.reg));
}

// DEFINES FOR TEST_CONCURRENT_ADD_REGISTERS

// Semaphores for barrier synchronization
static SemaphoreHandle_t readySem;
static SemaphoreHandle_t startSem;
static SemaphoreHandle_t doneSem;
// Results of the concurrent add
static int addResults[2];
// Register to add concurrently
static DummyRegister concurrentReg(Modbus::HOLDING_REGISTER, 42);

void addRegisterTask(void* pvParameters) {
    uintptr_t idx = (uintptr_t)pvParameters;
    // 1) Signal that we are ready
    xSemaphoreGive(readySem);
    // 2) Wait for the start
    xSemaphoreTake(startSem, portMAX_DELAY);
    // 3) Do the concurrent add
    addResults[idx] = server.addRegister(concurrentReg.reg);
    // 4) Signal that we are done
    xSemaphoreGive(doneSem);
    vTaskDelete(NULL);
}

void test_concurrent_add_registers() {
    Modbus::Logger::logln("\nTEST_CONCURRENT_ADD_REGISTERS");

    // 0) Suspend the server task to release the cores
    vTaskSuspend(ezmTaskHandle);

    // 1) (Re)create the semaphores
    readySem = xSemaphoreCreateCounting(2, 0);
    startSem = xSemaphoreCreateCounting(2, 0);
    doneSem  = xSemaphoreCreateCounting(2, 0);
    TEST_ASSERT_NOT_NULL(readySem);
    TEST_ASSERT_NOT_NULL(startSem);
    TEST_ASSERT_NOT_NULL(doneSem);

    // 2) Launch 2 tasks, idx=0 ➞ core0, idx=1 ➞ core1, priority > poll()
    const UBaseType_t prio = uxTaskPriorityGet(NULL) + 1; // just above the runner's priority
    xTaskCreatePinnedToCore(addRegisterTask, "AddReg0", 4096, (void*)0, prio, NULL, 0);
    xTaskCreatePinnedToCore(addRegisterTask, "AddReg1", 4096, (void*)1, prio, NULL, 1);

    // 3) Wait for them to be ready
    xSemaphoreTake(readySem, portMAX_DELAY);
    xSemaphoreTake(readySem, portMAX_DELAY);

    // 4) Start the tasks
    xSemaphoreGive(startSem);
    xSemaphoreGive(startSem);

    // 5) Wait for them to finish addRegister()
    xSemaphoreTake(doneSem, portMAX_DELAY);
    xSemaphoreTake(doneSem, portMAX_DELAY);

    // 6) Check that only one succeeded and the other is BUSY
    bool s0 = (addResults[0] == Modbus::Server::SUCCESS);
    bool s1 = (addResults[1] == Modbus::Server::SUCCESS);
    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(s0 ^ s1, "Only one of the two tasks must succeed");
    if (s0)
      TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_REG_BUSY, addResults[1], "2nd task must return ERR_REG_BUSY");
    else
      TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_REG_BUSY, addResults[0], "1st task must return ERR_REG_BUSY");

    // 7) Check that only one register was added
    auto r = server.getRegister(Modbus::HOLDING_REGISTER, concurrentReg.reg.address);
    TEST_ASSERT_TRUE(r);
    TEST_ASSERT_EQUAL_MESSAGE(concurrentReg.reg.readCb(concurrentReg.reg), r.readCb(r), "readCb must be the same");
    // 8) Cleanup
    server.clearAllRegisters();
    vSemaphoreDelete(readySem);
    vSemaphoreDelete(startSem);
    vSemaphoreDelete(doneSem);
    vTaskResume(ezmTaskHandle);
}

// ===================================================================================
// READ/WRITE REQUESTS TESTS
// ===================================================================================

void test_single_read_request() {
    Modbus::Logger::logln("\nTEST_SINGLE_READ_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, i.handle, true, "20 input add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");
    do_test_add_registers(server, d.handle, true, "20 discrete input add failed");

    // Perform a single read request for each register & register type

    for (auto& reg : h.handle) {
        auto val = testClient.holdingRegisterRead(1, reg.address);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(reg.readCb(reg), val); // Check that the read callback returns the correct value
    }

    for (auto& reg : i.handle) {
        auto val = testClient.inputRegisterRead(1, reg.address);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(reg.readCb(reg), val); // Check that the read callback returns the correct value
    }

    for (auto& reg : c.handle) {
        auto val = testClient.coilRead(1, reg.address);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(reg.readCb(reg), val); // Check that the read callback returns the correct value
    }

    for (auto& reg : d.handle) {
        auto val = testClient.discreteInputRead(1, reg.address);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(reg.readCb(reg), val); // Check that the read callback returns the correct value
    }
}

void test_single_read_requests_undefined_registers() {
    Modbus::Logger::logln("\nTEST_SINGLE_READ_REQUESTS_UNDEFINED_REGISTERS");
    // Perform a single read request for 1 register of each type (should fail because of rejectUndefined)
    int val;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    val = testClient.holdingRegisterRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.inputRegisterRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.coilRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.discreteInputRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value    
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_read_request() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, i.handle, true, "20 input add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");
    do_test_add_registers(server, d.handle, true, "20 discrete input add failed");

    // Perform a multiple read request for 5 registers of each type

    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 10;
    int numRead;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of holding registers");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(startAddr + k, testClient.read(), "Incorrect holding register value");
    }

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of input registers");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(startAddr + k, testClient.read(), "Incorrect input register value");
    }

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of coils");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.read(), "Incorrect coil value");
    }

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of discrete inputs");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.read(), "Incorrect discrete input value");
    }
}

void test_multiple_read_request_undefined_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST_UNDEFINED_REGISTERS");
    // Perform a multiple read request for 5 register of each type (should fail because of rejectUndefined)
    int numRead;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";
    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 5;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of coils");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_read_request_overlapping_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST_OVERLAPPING_REGISTERS");
    // Create 4 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,4);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,4);
    DummyRegSuite<Modbus::COIL> c(1,4);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,4);
    // Add them
    do_test_add_registers(server, h.handle, true, "4 holding add failed");
    do_test_add_registers(server, i.handle, true, "4 input add failed");
    do_test_add_registers(server, c.handle, true, "4 coil add failed");
    do_test_add_registers(server, d.handle, true, "4 discrete input add failed");

    // Perform a multiple read request for 5 registers of each type, should fail TOTALLY because
    // of rejectUndefined & last register is undefined (atomicity)

    int numRead;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";
    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 5;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of coils");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_single_write_request() {
    Modbus::Logger::logln("\nTEST_SINGLE_WRITE_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");

    // Perform a single write request for each register & register type

    int coilVal = 0;
    int holdingVal = 42;

    for (auto& reg : h.handle) {
        auto ret = testClient.holdingRegisterWrite(1, reg.address, holdingVal);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(1, ret); // Check that the write suceeded
        TEST_ASSERT_EQUAL(holdingVal, (int)reg.readCb(reg));
    }

    for (auto& reg : c.handle) {
        auto ret = testClient.coilWrite(1, reg.address, coilVal);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(1, ret); // Check that the write suceeded
        TEST_ASSERT_EQUAL(coilVal, (int)reg.readCb(reg));
    }
}

void test_single_write_requests_undefined_registers() {
    Modbus::Logger::logln("\nTEST_SINGLE_WRITE_REQUESTS_UNDEFINED_REGISTERS");
    // Perform a single write request for 1 register of each type (should fail because of rejectUndefined)
    int writeVal = 1;
    int val;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    val = testClient.holdingRegisterWrite(1, 1, writeVal);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(0, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.coilWrite(1, 1, writeVal);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(0, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_write_request() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");

    // Perform a multiple write request for 5 registers of each type

    int coilVal = 0;
    int holdingVal = 42;

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 10;

    // Write holding registers
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(), 
        "endTransmission failed for holding registers");

    // Verify the values were written correctly
    for (int i = 0; i < numRegsToWrite; i++) {
        auto reg = server.getRegister(Modbus::HOLDING_REGISTER, startAddr + i);
        TEST_ASSERT_START();
        TEST_ASSERT_TRUE(reg);
        TEST_ASSERT_EQUAL(holdingVal, (int)reg.readCb(reg));
    }

    // Write coils
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(),
        "endTransmission failed for coils");

    // Verify the values were written correctly
    for (int i = 0; i < numRegsToWrite; i++) {
        auto reg = server.getRegister(Modbus::COIL, startAddr + i);
        TEST_ASSERT_START();
        TEST_ASSERT_TRUE(reg);
        TEST_ASSERT_EQUAL(coilVal, (int)reg.readCb(reg));
    }
}

void test_multiple_write_request_undefined_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST_UNDEFINED_REGISTERS");
    // Perform a multiple write request for 5 registers of each type

    int coilVal = 0;
    int holdingVal = 42;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 5;

    // Write holding registers
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(), 
        "endTransmission must fail for holding registers");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
    // Write coils
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(),
        "endTransmission must fail for coils");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_write_request_overlapping_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST_OVERLAPPING_REGISTERS");
    // Create 4 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,4);
    DummyRegSuite<Modbus::COIL> c(1,4);
    // Add them
    do_test_add_registers(server, h.handle, true, "4 holding add failed");
    do_test_add_registers(server, c.handle, true, "4 coil add failed");

    // Perform a multiple write request for 5 registers of each type, should fail TOTALLY because
    // of rejectUndefined & last register is undefined (atomicity)

    int coilVal = 0;
    int holdingVal = 42;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 5;

    // Write holding registers
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(), 
        "endTransmission must fail for holding registers");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    // Write coils
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(),
        "endTransmission must fail for coils");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_second_server_on_same_interface() {
    Modbus::Logger::logln("\nTEST_SECOND_SERVER_ON_SAME_INTERFACE");

    Modbus::Server server2(rtu, 2, 2);
    TaskHandle_t ezmTaskHandle2 = NULL;
    xTaskCreatePinnedToCore(EZModbusServerTask, "EZModbusServerTask2", 16384, &server2, 5, &ezmTaskHandle2, 0);

    Modbus::Logger::logln("[test_second_server] Waiting for Server 2 to initialize...");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Register 1 holding register on each server
    DummyRegister h1(Modbus::HOLDING_REGISTER, 1);
    DummyRegister h2(Modbus::HOLDING_REGISTER, 1);
    do_test_add_register(server, h1.reg, true, "add holding register failed");
    do_test_add_register(server2, h2.reg, true, "add holding register failed");

    // Write the register on each server with different values
    uint16_t valSrv1 = 3;
    uint16_t valSrv2 = 4;

    // Write to server 1
    auto ret = testClient.holdingRegisterWrite(1, h1.reg.address, valSrv1);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(1, ret); // Check that the write succeeded
    TEST_ASSERT_EQUAL(valSrv1, (int)h1.reg.readCb(h1.reg));

    // Write to server 2
    ret = testClient.holdingRegisterWrite(2, h2.reg.address, valSrv2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(1, ret); // Check that the write succeeded
    TEST_ASSERT_EQUAL(valSrv2, (int)h2.reg.readCb(h2.reg));

    // Write to both servers in broadcast mode
    uint16_t valBroadcast = 5;
    ret = testClient.holdingRegisterWrite(0, h1.reg.address, valBroadcast); // SlaveID 0 for broadcast

    // Process for one second to see if we get responses
    uint32_t processTime = 1000;
    while (processTime > 0) {
        agent.poll();
        vTaskDelay(pdMS_TO_TICKS(1));
        processTime--;
    }

    // Check the writes succeeded & we have not received any answer
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(valBroadcast, (int)h1.reg.readCb(h1.reg));
    TEST_ASSERT_EQUAL(valBroadcast, (int)h2.reg.readCb(h2.reg));
    TEST_ASSERT_EQUAL(false, agent.hasData());
    
    if(ezmTaskHandle2) {
        vTaskDelete(ezmTaskHandle2);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ModbusTestClient init
    MBT_SERIAL.begin(9600, SERIAL_8N1, MBT_RX, MBT_TX);
    vTaskDelay(pdMS_TO_TICKS(50));
    flushSerialBuffer(MBT_SERIAL);

    Modbus::Logger::logln("[setup] Initializing EzmUart...");
    esp_err_t uart_init_res = ezm_uart.begin();
    if (uart_init_res != ESP_OK) {
        Modbus::Logger::logln("[setup] EzmUart initialization failed)");
        return;
    }
    Modbus::Logger::logln("[setup] EzmUart initialized.");

    Modbus::Logger::logln("[setup] Initializing RTU interface...");
    auto ifcInitRes = rtu.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus RTU interface initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] EZModbus RTU interface initialized.");

    Modbus::Logger::logln("[setup] Creating EZModbusServerTask for main server...");
    xTaskCreatePinnedToCore(EZModbusServerTask, "EZModbusServerTask", 16384, &server, 5, &ezmTaskHandle, 0);
    while (!ezmTaskInitialized) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    Modbus::Logger::logln("[setup] EZModbus server task initialized, starting tests...");

    // Run tests
    UNITY_BEGIN();
 
    // Register store tests
    RUN_TEST(test_add_single_registers);
    RUN_TEST(test_add_multiple_registers);
    // RUN_TEST(test_add_overflow_registers); // Commented out because max # = MAX_UINT16 now
    RUN_TEST(test_add_no_read_callback);
    RUN_TEST(test_add_readonly_with_write_callback);
    RUN_TEST(test_add_write_without_write_callback);
    RUN_TEST(test_register_bad_type);
    RUN_TEST(test_get_register);
    // RUN_TEST(test_clear_registers); // Commented out because deprecated (use clear_all only)
    RUN_TEST(test_clear_all_registers);
    RUN_TEST(test_add_registers_atomicity);
    RUN_TEST(test_register_overwrite);
    // RUN_TEST(test_concurrent_add_registers); // TODO: fix test code

    // Read/write requests tests
    RUN_TEST(test_single_read_request);
    RUN_TEST(test_single_read_requests_undefined_registers);
    RUN_TEST(test_multiple_read_request);
    RUN_TEST(test_multiple_read_request_undefined_registers);
    RUN_TEST(test_multiple_read_request_overlapping_registers);
    RUN_TEST(test_single_write_request);
    RUN_TEST(test_single_write_requests_undefined_registers);
    RUN_TEST(test_multiple_write_request);
    RUN_TEST(test_multiple_write_request_undefined_registers);
    RUN_TEST(test_multiple_write_request_overlapping_registers);

    RUN_TEST(test_second_server_on_same_interface);

    UNITY_END();
}

void loop() {
    Modbus::Logger::logln("Idling in loop()...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}