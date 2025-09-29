#ifndef DIRTY_DPS_H_INCLUDED
#define DIRTY_DPS_H_INCLUDED

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <functional>

#include "util/dps_config.h"

class DirtyDPS
{
public:
    // Returned data structures
    struct TemperatureCoefficients {
        float c0_half;
        float c1;
        uint8_t oversampling;      // current OSR exponent (0..7)
        int32_t scaling_divisor;   // scaling_facts[oversampling]
    };

    struct PressureCoefficients {
        float c00;
        float c10;
        float c01;
        float c11;
        float c20;
        float c21;
        float c30;
        uint8_t oversampling;      // current OSR exponent (0..7)
        int32_t scaling_divisor;   // scaling_facts[oversampling]
    };

    // constructor/destructor
    DirtyDPS(void);
    ~DirtyDPS(void);

    // I2C begin functions
    void begin(TwoWire &bus);
    void begin(TwoWire &bus, uint8_t slaveAddress);

    // SPI begin functions
    void begin(SPIClass &bus, int32_t chipSelect);
    void begin(SPIClass &bus, int32_t chipSelect, uint8_t threeWire);

    // End function (sets sensor to idle)
    void end(void);

    // Basic info
    uint8_t getProductId(void);
    uint8_t getRevisionId(void);

    // Mode control
    int16_t standby(void);

    // One-shot temperature
    int16_t measureTempOnce(float &result);
    int16_t measureTempOnce(float &result, uint8_t oversamplingRate);
    int16_t measureTempOnce(float &result, uint8_t oversamplingRate, std::function<float(float)> transform);
    int16_t startMeasureTempOnce(void);
    int16_t startMeasureTempOnce(uint8_t oversamplingRate);

    // One-shot pressure
    int16_t measurePressureOnce(float &result);
    int16_t measurePressureOnce(float &result, uint8_t oversamplingRate);
    int16_t measurePressureOnce(float &result, uint8_t oversamplingRate, std::function<float(float)> transform);
    int16_t startMeasurePressureOnce(void);
    int16_t startMeasurePressureOnce(uint8_t oversamplingRate);

    // Get single result (based on last command)
    int16_t getSingleResult(float &result);
    int16_t getSingleResult(float &result, std::function<float(float)> transform);

    // Continuous modes
    int16_t startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);
    int16_t startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);
    int16_t startMeasureBothCont(uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);

    // FIFO/continuous readout
    int16_t getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount);

    // Interrupt status helpers (common)
    int16_t getIntStatusFifoFull(void);
    int16_t getIntStatusTempReady(void);
    int16_t getIntStatusPrsReady(void);

    // Device-specific helpers
    int16_t correctTemp(void);
    int16_t setInterruptSources(uint8_t intr_source, uint8_t polarity = 1);

    // Coefficient accessors
    TemperatureCoefficients getTemperatureCoefficients() const;
    PressureCoefficients getPressureCoefficients() const;

protected:
    // scaling factor table
    static const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS];

    dps::Mode m_opMode;

    // flags
    uint8_t m_initFail;

    // identification
    uint8_t m_productID;
    uint8_t m_revisionID;

    // settings
    uint8_t m_tempMr;
    uint8_t m_tempOsr;
    uint8_t m_prsMr;
    uint8_t m_prsOsr;

    // compensation coefficients for DPS3xx
    int32_t m_c00;
    int32_t m_c10;
    int32_t m_c01;
    int32_t m_c11;
    int32_t m_c20;
    int32_t m_c21;
    int32_t m_c30;
    uint8_t m_tempSensor;
    int32_t m_c0Half;
    int32_t m_c1;

    // last measured scaled temperature (necessary for pressure compensation)
    float m_lastTempScal;

    // bus specific
    uint8_t m_SpiI2c; // 0=SPI, 1=I2C

    // I2C
    TwoWire *m_i2cbus;
    uint8_t m_slaveAddress;

    // SPI
    SPIClass *m_spibus;
    int32_t m_chipSelect;
    uint8_t m_threeWire;

    // Initialization
    void init(void);
    int16_t readcoeffs(void);

    // Mode/config
    int16_t setOpMode(uint8_t opMode);
    int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);
    int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);
    int16_t flushFIFO();

    // Calculations
    float calcTemp(int32_t raw);
    float calcTemp(int32_t raw, std::function<float(float)> transform);
    float calcPressure(int32_t raw);
    float calcPressure(int32_t raw, std::function<float(float)> transform);

    // Helpers
    int16_t enableFIFO();
    int16_t disableFIFO();
    uint16_t calcBusyTime(uint16_t temp_rate, uint16_t temp_osr);
    int16_t getFIFOvalue(int32_t *value);
    void getTwosComplement(int32_t *raw, uint8_t length);
    int16_t getRawResult(int32_t *raw, RegBlock_t reg);

    // Low-level IO
    int16_t readByte(uint8_t regAddress);
    int16_t readByteSPI(uint8_t regAddress);
    int16_t readBlock(RegBlock_t regBlock, uint8_t *buffer);
    int16_t readBlockSPI(RegBlock_t regBlock, uint8_t *buffer);

    int16_t writeByte(uint8_t regAddress, uint8_t data);
    int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check);
    int16_t writeByteSpi(uint8_t regAddress, uint8_t data, uint8_t check);
    int16_t writeByteBitfield(uint8_t data, RegMask_t regMask);
    int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, uint8_t check);
    int16_t readByteBitfield(RegMask_t regMask);
};

#endif // DIRTY_DPS_H_INCLUDED
