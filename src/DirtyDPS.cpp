#include "DirtyDPS.h"

using namespace dps;
using namespace dps3xx;

const int32_t DirtyDPS::scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

////////  Constructor, Destructor, begin, end  ////////

DirtyDPS::DirtyDPS(void)
{
    // assume that initialization has failed before it has been done
    m_initFail = 1U;
    m_threeWire = 0U;
}

DirtyDPS::~DirtyDPS(void)
{
    end();
}

void DirtyDPS::begin(TwoWire &bus)
{
    begin(bus, DPS__STD_SLAVE_ADDRESS);
}

void DirtyDPS::begin(TwoWire &bus, uint8_t slaveAddress)
{
    // this flag will show if the initialization was successful
    m_initFail = 0U;

    // Set I2C bus connection
    m_SpiI2c = 1U;
    m_i2cbus = &bus;
    m_slaveAddress = slaveAddress;

    // Init bus
    m_i2cbus->begin();

    delay(50); // startup time of Dps3xx

    init();
}

void DirtyDPS::begin(SPIClass &bus, int32_t chipSelect)
{
    begin(bus, chipSelect, 0U);
}

void DirtyDPS::begin(SPIClass &bus, int32_t chipSelect, uint8_t threeWire)
{
    // this flag will show if the initialization was successful
    m_initFail = 0U;

    // Set SPI bus connection
    m_SpiI2c = 0U;
    m_spibus = &bus;
    m_chipSelect = chipSelect;

    // Init bus
    m_spibus->begin();
    m_spibus->setDataMode(SPI_MODE3);

    pinMode(m_chipSelect, OUTPUT);
    digitalWrite(m_chipSelect, HIGH);

    delay(50); // startup time of Dps3xx

    // switch to 3-wire mode if necessary
    // do not use writeByteBitfield or check option to set SPI mode!
    // Reading is not possible until SPI-mode is valid
    if (threeWire)
    {
        m_threeWire = 1U;
        if (writeByte(DPS3xx__REG_ADR_SPI3W, DPS3xx__REG_CONTENT_SPI3W))
        {
            m_initFail = 1U;
            return;
        }
    }

    init();
}

void DirtyDPS::end(void)
{
    standby();
}

////////  Public helpers  ////////

uint8_t DirtyDPS::getProductId(void)
{
    return m_productID;
}

uint8_t DirtyDPS::getRevisionId(void)
{
    return m_revisionID;
}

int16_t DirtyDPS::getContResults(float *tempBuffer,
                                 uint8_t &tempCount,
                                 float *prsBuffer,
                                 uint8_t &prsCount)
{
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in background mode
    if (!(m_opMode & 0x04))
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (!tempBuffer || !prsBuffer)
    {
        return DPS__FAIL_UNKNOWN;
    }
    tempCount = 0U;
    prsCount = 0U;

    // while FIFO is not empty
    while (readByteBitfield(registers[FIFO_EMPTY]) == 0)
    {
        int32_t raw_result;
        float result;
        // read next result from FIFO
        int16_t type = getFIFOvalue(&raw_result);
        switch (type)
        {
        case 0: // temperature
            if (tempCount < DPS__FIFO_SIZE)
            {
                result = calcTemp(raw_result);
                tempBuffer[tempCount++] = result;
            }
            break;
        case 1: // pressure
            if (prsCount < DPS__FIFO_SIZE)
            {
                result = calcPressure(raw_result);
                prsBuffer[prsCount++] = result;
            }
            break;
        case -1: // read failed
            break;
        }
    }
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::getSingleResult(float &result)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    // read finished bit for current opMode
    int16_t rdy;
    switch (m_opMode)
    {
    case CMD_TEMP: // temperature
        rdy = readByteBitfield(config_registers[TEMP_RDY]);
        break;
    case CMD_PRS: // pressure
        rdy = readByteBitfield(config_registers[PRS_RDY]);
        break;
    default: // not in command mode
        return DPS__FAIL_TOOBUSY;
    }
    // read new measurement result
    switch (rdy)
    {
    case DPS__FAIL_UNKNOWN: // could not read ready flag
        return DPS__FAIL_UNKNOWN;
    case 0: // ready flag not set, measurement still in progress
        return DPS__FAIL_UNFINISHED;
    case 1: // measurement ready, expected case
        Mode oldMode = m_opMode;
        m_opMode = IDLE; // opcode was automatically reset by DPS3xx
        int32_t raw_val;
        switch (oldMode)
        {
        case CMD_TEMP: // temperature
            getRawResult(&raw_val, registerBlocks[TEMP]);
            result = calcTemp(raw_val);
            return DPS__SUCCEEDED;
        case CMD_PRS: // pressure
            getRawResult(&raw_val, registerBlocks[PRS]);
            result = calcPressure(raw_val);
            return DPS__SUCCEEDED;
        default:
            return DPS__FAIL_UNKNOWN; // should already be filtered above
        }
    }
    return DPS__FAIL_UNKNOWN;
}

int16_t DirtyDPS::getSingleResult(float &result, std::function<float(float)> transform)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    // read finished bit for current opMode
    int16_t rdy;
    switch (m_opMode)
    {
    case CMD_TEMP: // temperature
        rdy = readByteBitfield(config_registers[TEMP_RDY]);
        break;
    case CMD_PRS: // pressure
        rdy = readByteBitfield(config_registers[PRS_RDY]);
        break;
    default: // not in command mode
        return DPS__FAIL_TOOBUSY;
    }
    // read new measurement result
    switch (rdy)
    {
    case DPS__FAIL_UNKNOWN: // could not read ready flag
        return DPS__FAIL_UNKNOWN;
    case 0: // ready flag not set, measurement still in progress
        return DPS__FAIL_UNFINISHED;
    case 1: // measurement ready, expected case
        Mode oldMode = m_opMode;
        m_opMode = IDLE; // opcode was automatically reset by DPS3xx
        int32_t raw_val;
        switch (oldMode)
        {
        case CMD_TEMP: // temperature
            getRawResult(&raw_val, registerBlocks[TEMP]);
            result = calcTemp(raw_val, transform);
            return DPS__SUCCEEDED;
        case CMD_PRS: // pressure
            getRawResult(&raw_val, registerBlocks[PRS]);
            result = calcPressure(raw_val, transform);
            return DPS__SUCCEEDED;
        default:
            return DPS__FAIL_UNKNOWN; // should already be filtered above
        }
    }
    return DPS__FAIL_UNKNOWN;
}

int16_t DirtyDPS::measureTempOnce(float &result)
{
    return measureTempOnce(result, m_tempOsr);
}

int16_t DirtyDPS::measureTempOnce(float &result, uint8_t oversamplingRate)
{
    // Start measurement
    int16_t ret = startMeasureTempOnce(oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    delay(calcBusyTime(0U, m_tempOsr) / DPS__BUSYTIME_SCALING);
    delay(DPS3xx__BUSYTIME_FAILSAFE);

    ret = getSingleResult(result);
    if (ret != DPS__SUCCEEDED)
    {
        standby();
    }
    return ret;
}

int16_t DirtyDPS::measureTempOnce(float &result, uint8_t oversamplingRate, std::function<float(float)> transform)
{
    // Start measurement
    int16_t ret = startMeasureTempOnce(oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    delay(calcBusyTime(0U, m_tempOsr) / DPS__BUSYTIME_SCALING);
    delay(DPS3xx__BUSYTIME_FAILSAFE);

    ret = getSingleResult(result, transform);
    if (ret != DPS__SUCCEEDED)
    {
        standby();
    }
    return ret;
}

int16_t DirtyDPS::startMeasureTempOnce(void)
{
    return startMeasureTempOnce(m_tempOsr);
}

int16_t DirtyDPS::startMeasureTempOnce(uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (oversamplingRate != m_tempOsr)
    {
        // configuration of oversampling rate
        if (configTemp(0U, oversamplingRate) != DPS__SUCCEEDED)
        {
            return DPS__FAIL_UNKNOWN;
        }
    }

    // set device to temperature measuring mode
    return setOpMode(CMD_TEMP);
}

int16_t DirtyDPS::measurePressureOnce(float &result)
{
    return measurePressureOnce(result, m_prsOsr);
}

int16_t DirtyDPS::measurePressureOnce(float &result, uint8_t oversamplingRate)
{
    // start the measurement
    int16_t ret = startMeasurePressureOnce(oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    delay(calcBusyTime(0U, m_prsOsr) / DPS__BUSYTIME_SCALING);
    delay(DPS3xx__BUSYTIME_FAILSAFE);

    ret = getSingleResult(result);
    if (ret != DPS__SUCCEEDED)
    {
        standby();
    }
    return ret;
}

int16_t DirtyDPS::measurePressureOnce(float &result, uint8_t oversamplingRate, std::function<float(float)> transform)
{
    // start the measurement
    int16_t ret = startMeasurePressureOnce(oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    delay(calcBusyTime(0U, m_prsOsr) / DPS__BUSYTIME_SCALING);
    delay(DPS3xx__BUSYTIME_FAILSAFE);

    ret = getSingleResult(result, transform);
    if (ret != DPS__SUCCEEDED)
    {
        standby();
    }
    return ret;
}

int16_t DirtyDPS::startMeasurePressureOnce(void)
{
    return startMeasurePressureOnce(m_prsOsr);
}

int16_t DirtyDPS::startMeasurePressureOnce(uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // configuration of oversampling rate, lowest measure rate to avoid conflicts
    if (oversamplingRate != m_prsOsr)
    {
        if (configPressure(0U, oversamplingRate))
        {
            return DPS__FAIL_UNKNOWN;
        }
    }
    // set device to pressure measuring mode
    return setOpMode(CMD_PRS);
}

int16_t DirtyDPS::startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (calcBusyTime(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (configTemp(measureRate, oversamplingRate))
    {
        return DPS__FAIL_UNKNOWN;
    }

    if (enableFIFO())
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (setOpMode(CONT_TMP))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (calcBusyTime(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (configPressure(measureRate, oversamplingRate))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (enableFIFO())
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (setOpMode(CONT_PRS))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::startMeasureBothCont(uint8_t tempMr,
                                       uint8_t tempOsr,
                                       uint8_t prsMr,
                                       uint8_t prsOsr)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (calcBusyTime(tempMr, tempOsr) + calcBusyTime(prsMr, prsOsr) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (configTemp(tempMr, tempOsr))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // update precision and measuring rate
    if (configPressure(prsMr, prsOsr))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (enableFIFO())
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (setOpMode(CONT_BOTH))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::standby(void)
{
    // abort if initialization failed
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // set device to idling mode
    int16_t ret = setOpMode(IDLE);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }
    ret = disableFIFO();
    return ret;
}

int16_t DirtyDPS::correctTemp(void)
{
    if (m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    writeByte(0x0E, 0xA5);
    writeByte(0x0F, 0x96);
    writeByte(0x62, 0x02);
    writeByte(0x0E, 0x00);
    writeByte(0x0F, 0x00);

    // perform a first temperature measurement (again)
    // the most recent temperature will be saved internally
    // and used for compensation when calculating pressure
    float trash;
    measureTempOnce(trash);

    return DPS__SUCCEEDED;
}

DirtyDPS::TemperatureCoefficients DirtyDPS::getTemperatureCoefficients() const
{
    TemperatureCoefficients t{};
    t.c0_half = static_cast<float>(m_c0Half);
    t.c1 = static_cast<float>(m_c1);
    t.oversampling = m_tempOsr;
    t.scaling_divisor = scaling_facts[m_tempOsr];
    return t;
}

DirtyDPS::PressureCoefficients DirtyDPS::getPressureCoefficients() const
{
    PressureCoefficients p{};
    p.c00 = static_cast<float>(m_c00);
    p.c10 = static_cast<float>(m_c10);
    p.c01 = static_cast<float>(m_c01);
    p.c11 = static_cast<float>(m_c11);
    p.c20 = static_cast<float>(m_c20);
    p.c21 = static_cast<float>(m_c21);
    p.c30 = static_cast<float>(m_c30);
    p.oversampling = m_prsOsr;
    p.scaling_divisor = scaling_facts[m_prsOsr];
    return p;
}

////////  Private/protected helpers  ////////

int16_t DirtyDPS::setOpMode(uint8_t opMode)
{
    if (writeByteBitfield(opMode, config_registers[MSR_CTRL]) == -1)
    {
        return DPS__FAIL_UNKNOWN;
    }
    m_opMode = (Mode)opMode;
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::getIntStatusFifoFull(void)
{
    return readByteBitfield(config_registers[INT_FLAG_FIFO]);
}

int16_t DirtyDPS::getIntStatusTempReady(void)
{
    return readByteBitfield(config_registers[INT_FLAG_TEMP]);
}

int16_t DirtyDPS::getIntStatusPrsReady(void)
{
    return readByteBitfield(config_registers[INT_FLAG_PRS]);
}

int16_t DirtyDPS::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
    tempMr &= 0x07;
    tempOsr &= 0x07;
    // two accesses to the same register; for readability
    int16_t ret = writeByteBitfield(tempMr, config_registers[TEMP_MR]);
    ret = writeByteBitfield(tempOsr, config_registers[TEMP_OSR]);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    m_tempMr = tempMr;
    m_tempOsr = tempOsr;
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
    prsMr &= 0x07;
    prsOsr &= 0x07;
    int16_t ret = writeByteBitfield(prsMr, config_registers[PRS_MR]);
    ret = writeByteBitfield(prsOsr, config_registers[PRS_OSR]);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    m_prsMr = prsMr;
    m_prsOsr = prsOsr;
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::flushFIFO()
{
    return writeByteBitfield(1U, registers[FIFO_FL]);
}

int16_t DirtyDPS::enableFIFO()
{
    return writeByteBitfield(1U, config_registers[FIFO_EN]);
}

int16_t DirtyDPS::disableFIFO()
{
    int16_t ret = flushFIFO();
    ret = writeByteBitfield(0U, config_registers[FIFO_EN]);
    return ret;
}

uint16_t DirtyDPS::calcBusyTime(uint16_t mr, uint16_t osr)
{
    // formula from datasheet (optimized)
    return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

int16_t DirtyDPS::getFIFOvalue(int32_t *value)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};

    // abort on invalid argument or failed block reading
    if (value == NULL || readBlock(registerBlocks[PRS], buffer) != DPS__RESULT_BLOCK_LENGTH)
        return DPS__FAIL_UNKNOWN;
    *value = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    getTwosComplement(value, 24);
    return buffer[2] & 0x01;
}

int16_t DirtyDPS::readByte(uint8_t regAddress)
{
    // delegate to specialized function if connected via SPI
    if (m_SpiI2c == 0)
    {
        return readByteSPI(regAddress);
    }

    m_i2cbus->beginTransmission(m_slaveAddress);
    m_i2cbus->write(regAddress);
    m_i2cbus->endTransmission(false);
    // request 1 byte from slave
    if (m_i2cbus->requestFrom(m_slaveAddress, (uint8_t)1, (uint8_t)1) > 0)
    {
        return m_i2cbus->read(); // return this byte on success
    }
    else
    {
        return DPS__FAIL_UNKNOWN; // if 0 bytes were read successfully
    }
}

int16_t DirtyDPS::readByteSPI(uint8_t regAddress)
{
    // this function is only made for communication via SPI
    if (m_SpiI2c != 0)
    {
        return DPS__FAIL_UNKNOWN;
    }
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // reserve and initialize bus
    m_spibus->beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                           MSBFIRST,
                                           SPI_MODE3));
    // enable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, LOW);
    // send address with read command to Dps3xx
    m_spibus->transfer(regAddress | DPS3xx__SPI_READ_CMD);
    // receive register content from Dps3xx
    uint8_t ret = m_spibus->transfer(0xFF); // send a dummy byte while receiving
    // disable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, HIGH);
    // close current SPI transaction
    m_spibus->endTransaction();
    // return received data
    return ret;
}

int16_t DirtyDPS::readBlockSPI(RegBlock_t regBlock, uint8_t *buffer)
{
    // this function is only made for communication via SPI
    if (m_SpiI2c != 0)
    {
        return DPS__FAIL_UNKNOWN;
    }
    // do not read if there is no buffer
    if (buffer == NULL)
    {
        return 0; // 0 bytes were read successfully
    }
    // mask regAddress
    regBlock.regAddress &= ~DPS3xx__SPI_RW_MASK;
    // reserve and initialize bus
    m_spibus->beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                           MSBFIRST,
                                           SPI_MODE3));
    // enable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, LOW);
    // send address with read command to Dps3xx
    m_spibus->transfer(regBlock.regAddress | DPS3xx__SPI_READ_CMD);

    // receive register contents from Dps3xx
    for (uint8_t count = 0; count < regBlock.length; count++)
    {
        buffer[count] = m_spibus->transfer(0xFF); // send a dummy byte while receiving
    }

    // disable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, HIGH);
    // close current SPI transaction
    m_spibus->endTransaction();
    // return received data
    return regBlock.length;
}

int16_t DirtyDPS::writeByte(uint8_t regAddress, uint8_t data)
{
    return writeByte(regAddress, data, 0U);
}

int16_t DirtyDPS::writeByte(uint8_t regAddress, uint8_t data, uint8_t check)
{
    // delegate to specialized function if connected via SPI
    if (m_SpiI2c == 0)
    {
        return writeByteSpi(regAddress, data, check);
    }
    m_i2cbus->beginTransmission(m_slaveAddress);
    m_i2cbus->write(regAddress);          // Write Register number to buffer
    m_i2cbus->write(data);                // Write data to buffer
    if (m_i2cbus->endTransmission() != 0) // Send buffer content to slave
    {
        return DPS__FAIL_UNKNOWN;
    }
    else
    {
        if (check == 0)
            return 0;                     // no checking
        if (readByte(regAddress) == data) // check if desired by calling function
        {
            return DPS__SUCCEEDED;
        }
        else
        {
            return DPS__FAIL_UNKNOWN;
        }
    }
}

int16_t DirtyDPS::writeByteSpi(uint8_t regAddress, uint8_t data, uint8_t check)
{
    // this function is only made for communication via SPI
    if (m_SpiI2c != 0)
    {
        return DPS__FAIL_UNKNOWN;
    }
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // reserve and initialize bus
    m_spibus->beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                           MSBFIRST,
                                           SPI_MODE3));
    // enable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, LOW);
    // send address with read command to Dps3xx
    m_spibus->transfer(regAddress | DPS3xx__SPI_WRITE_CMD);

    // write register content to Dps3xx
    m_spibus->transfer(data);

    // disable ChipSelect for Dps3xx
    digitalWrite(m_chipSelect, HIGH);
    // close current SPI transaction
    m_spibus->endTransaction();

    // check if necessary
    if (check == 0)
    {
        // no checking necessary
        return DPS__SUCCEEDED;
    }
    // checking necessary
    if (readByte(regAddress) == data)
    {
        // check passed
        return DPS__SUCCEEDED;
    }
    else
    {
        // check failed
        return DPS__FAIL_UNKNOWN;
    }
}

int16_t DirtyDPS::writeByteBitfield(uint8_t data, RegMask_t regMask)
{
    return writeByteBitfield(data, regMask.regAddress, regMask.mask, regMask.shift, 0U);
}

int16_t DirtyDPS::writeByteBitfield(uint8_t data,
                                    uint8_t regAddress,
                                    uint8_t mask,
                                    uint8_t shift,
                                    uint8_t check)
{
    int16_t old = readByte(regAddress);
    if (old < 0)
    {
        // fail while reading
        return old;
    }
    return writeByte(regAddress, ((uint8_t)old & ~mask) | ((data << shift) & mask), check);
}

int16_t DirtyDPS::readByteBitfield(RegMask_t regMask)
{
    int16_t ret = readByte(regMask.regAddress);
    if (ret < 0)
    {
        return ret;
    }
    return (((uint8_t)ret) & regMask.mask) >> regMask.shift;
}

int16_t DirtyDPS::readBlock(RegBlock_t regBlock, uint8_t *buffer)
{
    // delegate to specialized function if connected via SPI
    if (m_SpiI2c == 0)
    {
        return readBlockSPI(regBlock, buffer);
    }
    // do not read if there is no buffer
    if (buffer == NULL)
    {
        return 0; // 0 bytes read successfully
    }

    m_i2cbus->beginTransmission(m_slaveAddress);
    m_i2cbus->write(regBlock.regAddress);
    m_i2cbus->endTransmission(false);
    // request length bytes from slave
    int16_t ret = m_i2cbus->requestFrom(m_slaveAddress, (uint8_t)regBlock.length, (uint8_t)1);
    // read all received bytes to buffer
    for (int16_t count = 0; count < ret; count++)
    {
        buffer[count] = m_i2cbus->read();
    }
    return ret;
}

void DirtyDPS::getTwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((uint32_t)1 << (length - 1)))
    {
        *raw -= (uint32_t)1 << length;
    }
}

int16_t DirtyDPS::getRawResult(int32_t *raw, RegBlock_t reg)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};
    if (readBlock(reg, buffer) != DPS__RESULT_BLOCK_LENGTH)
        return DPS__FAIL_UNKNOWN;

    *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    getTwosComplement(raw, 24);
    return DPS__SUCCEEDED;
}

////////  DPS3xx-specific  ////////

void DirtyDPS::init(void)
{
    int16_t prodId = readByteBitfield(registers[PROD_ID]);
    if (prodId < 0)
    {
        // Connected device is not a Dps3xx
        m_initFail = 1U;
        return;
    }
    m_productID = prodId;

    int16_t revId = readByteBitfield(registers[REV_ID]);
    if (revId < 0)
    {
        m_initFail = 1U;
        return;
    }
    m_revisionID = revId;

    // find out which temperature sensor is calibrated with coefficients...
    int16_t sensor = readByteBitfield(registers[TEMP_SENSORREC]);
    if (sensor < 0)
    {
        m_initFail = 1U;
        return;
    }

    //...and use this sensor for temperature measurement
    m_tempSensor = sensor;
    if (writeByteBitfield((uint8_t)sensor, registers[TEMP_SENSOR]) < 0)
    {
        m_initFail = 1U;
        return;
    }

    // read coefficients
    if (readcoeffs() < 0)
    {
        m_initFail = 1U;
        return;
    }

    // set to standby for further configuration
    standby();

    // set measurement precision and rate to standard values;
    configTemp(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);
    configPressure(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);

    // perform a first temperature measurement
    // the most recent temperature will be saved internally
    // and used for compensation when calculating pressure
    float trash;
    measureTempOnce(trash);

    // make sure the Dps3xx is in standby after initialization
    standby();

    // Fix IC with a fuse bit problem, which lead to a wrong temperature
    // Should not affect ICs without this problem
    correctTemp();
}

int16_t DirtyDPS::readcoeffs(void)
{
    // TODO: remove magic number
    uint8_t buffer[18];
    // read COEF registers to buffer
    int16_t ret = readBlock(coeffBlock, buffer);
    if (!ret)
        return DPS__FAIL_INIT_FAILED;

    // compose coefficients from buffer content
    m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&m_c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    m_c0Half = m_c0Half / 2U;

    // now do the same thing for all other coefficients
    m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&m_c1, 12);
    m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&m_c00, 20);
    m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    getTwosComplement(&m_c10, 20);

    m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&m_c01, 16);

    m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&m_c11, 16);
    m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&m_c20, 16);
    m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&m_c21, 16);
    m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&m_c30, 16);
    return DPS__SUCCEEDED;
}

int16_t DirtyDPS::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
    // generic config
    tempMr &= 0x07;
    tempOsr &= 0x07;
    int16_t ret = writeByteBitfield(tempMr, config_registers[TEMP_MR]);
    ret = writeByteBitfield(tempOsr, config_registers[TEMP_OSR]);
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    m_tempMr = tempMr;
    m_tempOsr = tempOsr;

    // DPS3xx-specific bits
    writeByteBitfield(m_tempSensor, registers[TEMP_SENSOR]);
    // set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3)
    if (tempOsr > DPS3xx__OSR_SE)
    {
        ret = writeByteBitfield(1U, registers[TEMP_SE]);
    }
    else
    {
        ret = writeByteBitfield(0U, registers[TEMP_SE]);
    }
    return ret;
}

int16_t DirtyDPS::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
    // generic config
    prsMr &= 0x07;
    prsOsr &= 0x07;
    int16_t ret = writeByteBitfield(prsMr, config_registers[PRS_MR]);
    ret = writeByteBitfield(prsOsr, config_registers[PRS_OSR]);
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    m_prsMr = prsMr;
    m_prsOsr = prsOsr;

    // DPS3xx-specific bits
    if (prsOsr > DPS3xx__OSR_SE)
    {
        ret = writeByteBitfield(1U, registers[PRS_SE]);
    }
    else
    {
        ret = writeByteBitfield(0U, registers[PRS_SE]);
    }
    return ret;
}

float DirtyDPS::calcTemp(int32_t raw)
{
    float temp = raw;

    // scale temperature according to scaling table and oversampling
    temp /= scaling_facts[m_tempOsr];

    // update last measured temperature
    // it will be used for pressure compensation
    m_lastTempScal = temp;

    // Calculate compensated temperature
    temp = m_c0Half + m_c1 * temp;

    return temp;
}

float DirtyDPS::calcTemp(int32_t raw, std::function<float(float)> transform)
{
    float temp = raw;

    // scale temperature according to scaling table and oversampling
    temp /= scaling_facts[m_tempOsr];

    // apply the transformation function
    temp = transform(temp);

    // update last measured temperature
    // it will be used for pressure compensation
    m_lastTempScal = temp;

    // Calculate compensated temperature
    temp = m_c0Half + m_c1 * temp;

    return temp;
}

float DirtyDPS::calcPressure(int32_t raw)
{
    float prs = raw;

    // scale pressure according to scaling table and oversampling
    prs /= scaling_facts[m_prsOsr];

    // Calculate compensated pressure
    prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

    // return pressure
    return prs;
}

float DirtyDPS::calcPressure(int32_t raw, std::function<float(float)> transform)
{
    float prs = raw;

    // scale pressure according to scaling table and oversampling
    prs /= scaling_facts[m_prsOsr];

    // apply the transformation function
    prs = transform(prs);

    // Calculate compensated pressure
    prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

    // return pressure
    return prs;
}

int16_t DirtyDPS::setInterruptSources(uint8_t intr_source, uint8_t polarity)
{
    // Interrupts are not supported with 4 Wire SPI
    if (!m_SpiI2c & !m_threeWire)
    {
        return DPS__FAIL_UNKNOWN;
    }
    return writeByteBitfield(intr_source, registers[INT_SEL]) || writeByteBitfield(polarity, registers[INT_HL]);
}
