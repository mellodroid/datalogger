#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

#include "MMA8451.h"

namespace mma8451 {
    
    Mma8451::Mma8451() : i2cName("/dev/i2c-1"), commsOpen(false)
    {
        if ((i2cComms = open(i2cName.c_str(), O_RDWR)) >= 0) {
            if (ioctl(i2cComms, I2C_SLAVE, (int)mma8451::ADDRESS) >= 0) {
                commsOpen = true;
            }
        }
        
        setActive(false);
        
        result.resize(3);
        
        scale = getFullScale();
        
        for (auto i = 0; i < MAX_DATA_COUNT; ++i) {
            readBuffer[i] = 255;
            writeBuffer[i] = 255;
        }
    }
    
    Mma8451::Mma8451(std::string filename) : i2cName(filename), commsOpen(false)
    {
        if ((i2cComms = open(i2cName.c_str(), O_RDWR)) >= 0) {
            if (ioctl(i2cComms, I2C_SLAVE, mma8451::ADDRESS) >= 0) {
                commsOpen = true;
            }
        }
        
        result.resize(3);
        
        scale = getFullScale();
    }
    
    Mma8451::~Mma8451(void)
    {
        if (commsOpen) {
            setActive(false);
            
            close(i2cComms);
            commsOpen = false;
        }
    }
    
    bool Mma8451::writeRegister(Registers reg, uint8_t *data)
    {
        if (commsOpen) {
            if (i2c_smbus_write_byte_data(i2cComms, static_cast<uint8_t>(reg), data[0]) >= 0) 
                return true;
        }
        return false;
    }
    
    bool Mma8451::readRegister(Registers reg, uint8_t *data)
    {
        if (commsOpen) {
            int myData;
            myData = i2c_smbus_read_byte_data(i2cComms, static_cast<uint8_t>(reg));
            if (myData >= 0) {
                data[0] = static_cast<uint8_t>(myData & 0x000000FF);
                return true;
            }
        }
        return false;
    }
    
    template<typename T>
    bool Mma8451::readMultipleData(Registers startReg, uint8_t count, T** data)
    {
        if ((startReg + count + 1) > MAX_DATA_COUNT)
            return false;
            
        if (commsOpen) {
            if (i2c_smbus_read_i2c_block_data(i2cComms, static_cast<uint8_t>(startReg), count, readBuffer) >= 0) {
                *data = reinterpret_cast<T*>(readBuffer);
                return true;
            }
        }
        return false;
    }
    
    bool Mma8451::getActive(void)
    {
        uint8_t rx_data;
        return ((readRegister(Registers::CTRL_REG1, &rx_data) & 0x01) == 1);
    }
    
    bool Mma8451::setActive(bool enable)
    {
        uint8_t rx_data, tx_data;
        
        tx_data = enable ? 1 : 0;
        
        if (readRegister(Registers::CTRL_REG1, &rx_data)) {
            tx_data = tx_data | (rx_data & 0xFE);
            if (writeRegister(Registers::CTRL_REG1, &tx_data)) {
                return true;
            }
        }
        return false;
    }
    
    bool Mma8451::isCommsOpen(void)
    {
        return commsOpen;
    }
    
    bool Mma8451::setCommsFile(std::string filename)
    {
        i2cName = filename;
        
        if (commsOpen) {
            close(i2cComms);
            commsOpen = false;
        }
        
        if ((i2cComms = open(i2cName.c_str(), O_RDWR)) >= 0) {
            if (ioctl(i2cComms, I2C_SLAVE, mma8451::ADDRESS) >= 0) {
                commsOpen = true;
            }
        }
        
        return commsOpen;
    }
    
    std::string Mma8451::getCommsFile(void) 
    {
        return i2cName;
    }
        
    std::vector<float> Mma8451::getAccelerations(void)
    {
        int16_t* data = nullptr;
        int16_t ix, iy, iz;
        float divisor;
        
        if (readMultipleData(Registers::OUT_X_MSB, 6, &data))
        {
            switch (scale) {
                case FullScale::FULL_SCALE_2G:
                    divisor = 4096.0f * 4;
                    break;
                case FullScale::FULL_SCALE_4G:
                    divisor = 2048.0f * 4;
                    break;
                case FullScale::FULL_SCALE_8G:
                    divisor = 1024.0f * 4;
                    break;
            }
            
            ix = (data[0] & 0xFF00) >> 8 | (data[0] & 0x00FF) << 8;
            iy = (data[1] & 0xFF00) >> 8 | (data[1] & 0x00FF) << 8;
            iz = (data[2] & 0xFF00) >> 8 | (data[2] & 0x00FF) << 8;
            
            result[0] = static_cast<float>(ix) / divisor;
            result[1] = static_cast<float>(iy) / divisor;
            result[2] = static_cast<float>(iz) / divisor;
        }
        return result;
    }
    
    bool Mma8451::setFullScale(FullScale value)
    {
        if (setActive(false))
        {
            uint8_t rx_data, tx_data;
            
            if (readRegister(Registers::XYZ_DATA_CFG, &rx_data))
            {
                switch (value) {
                    case FullScale::FULL_SCALE_2G:
                        tx_data = 0;
                        break;
                    case FullScale::FULL_SCALE_4G:
                        tx_data = 1;
                        break;
                    case FullScale::FULL_SCALE_8G:
                        tx_data = 2;
                        break;
                }
                tx_data = tx_data | (rx_data & 0x10);
                
                if (writeRegister(Registers::XYZ_DATA_CFG, &tx_data)) {
                    if (setActive(true)) {
                        scale = value;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setFifoMode(FifoMode mode, uint8_t triggerCount)
    {
        if (setActive(false)) {
            uint8_t tx_data = (triggerCount & 0x3F) | (static_cast<uint8_t>(mode) << 6);
            if (writeRegister(Registers::F_SETUP, &tx_data)) {
                if (setActive(true)) {
                return true;
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setTriggerMode(TriggerMode triggers)
    {
        if (setActive(false)) {
            uint8_t tx_data = triggers.transientInterrupt << 5 | \
                              triggers.orientationInterrupt << 4 | \
                              triggers.pulseInterrupt << 3 | \
                              triggers.freefallInterrupt << 2;
            if (writeRegister(Registers::TRIG_CFG, &tx_data)) {
                if (setActive(true)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setHighPassOutput(bool enable)
    {
        if (setActive(false))
        {
            uint8_t rx_data, tx_data;
            
            if (readRegister(Registers::XYZ_DATA_CFG, &rx_data))
            {
                tx_data = (enable << 4) | (rx_data & 0x03);
                
                if (writeRegister(Registers::XYZ_DATA_CFG, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    FullScale Mma8451::getFullScale(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::XYZ_DATA_CFG, &rx_data)) {
            switch (rx_data & 0x03) {
                case 0:
                    scale = FullScale::FULL_SCALE_2G;
                    break;
                case 1:
                    scale = FullScale::FULL_SCALE_4G;
                    break;
                case 2:
                    scale = FullScale::FULL_SCALE_8G;
                    break;
                default:
                    scale = FullScale::RESERVED;
                    break;
            }
        }
        return scale;
    }
    
    FifoMode Mma8451::getFifoMode(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::F_SETUP, &rx_data)) {
            return static_cast<FifoMode>((rx_data & 0xC0) >> 6);
        }
        return FifoMode::DISABLED;
    }
    
    TriggerMode Mma8451::getTriggerMode(void)
    {
        uint8_t rx_data;
        TriggerMode value;
        if (readRegister(Registers::TRIG_CFG, &rx_data)) {
            value.transientInterrupt   = (rx_data & 0x20) == 0x20;
            value.orientationInterrupt = (rx_data & 0x10) == 0x10;
            value.pulseInterrupt       = (rx_data & 0x08) == 0x08;
            value.freefallInterrupt    = (rx_data & 0x04) == 0x04;
        }
        return value;
    }
    
    bool Mma8451::getFifoGateError(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::SYSMOD, &rx_data)) {
            return (rx_data & 0x80) == 0x80;
        }
        return false;
    }
    
    uint8_t Mma8451::getElapsedFifoTime(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::SYSMOD, &rx_data)) {
            return (rx_data & 0x7C) >> 2;
        }
        return 0;
    }
    
    SystemMode Mma8451::getSystemMode(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::SYSMOD, &rx_data)) {
            return static_cast<SystemMode>(rx_data & 0x03);
        }
        return SystemMode::UNKNOWN;
    }
    
    InterruptSource Mma8451::getInterrupts(void)
    {
        uint8_t rx_data;
        InterruptSource value;
        if (readRegister(Registers::INT_SOURCE, &rx_data)) {
            value.autoSleep = (rx_data & 0x80) == 0x80;
            value.fifo      = (rx_data & 0x40) == 0x40;
            value.dataReady = (rx_data & 0x01) == 0x01;
            value.triggers.transientInterrupt   = (rx_data & 0x20) == 0x20;
            value.triggers.orientationInterrupt = (rx_data & 0x10) == 0x10;
            value.triggers.pulseInterrupt       = (rx_data & 0x08) == 0x08;
            value.triggers.freefallInterrupt    = (rx_data & 0x04) == 0x04;
        }
        return value;
    }
    
    bool Mma8451::isHighPassOutputEnbabled(void)
    {
        uint8_t rx_data;
        if (setActive(false)) {
            if (readRegister(Registers::XYZ_DATA_CFG, &rx_data)) {
                if (setActive(true)) {
                    return (rx_data & 0x10) == 0x10;
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setAutoSleepSampleFreq(SleepRate rate)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG1, &rx_data)) {
                tx_data = (rx_data & 0x3F) | (static_cast<uint8_t>(rate) << 6);
                if (writeRegister(Registers::CTRL_REG1, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setDataRate(DataRate rate)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG1, &rx_data)) {
                tx_data = (rx_data & 0xC7) | (static_cast<uint8_t>(rate) << 3);
                if (writeRegister(Registers::CTRL_REG1, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool Mma8451::enableLowNoiseMode(bool enable)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG1, &rx_data)) {
                tx_data = (rx_data & 0xFB) | (enable << 2);
                if (writeRegister(Registers::CTRL_REG1, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::enableFastReadMode(bool enable)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG1, &rx_data)) {
                tx_data = (rx_data & 0xFD) | (enable << 1);
                if (writeRegister(Registers::CTRL_REG1, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    SleepRate Mma8451::getAutoSleepSampleFreq(void)
    {
        uint8_t rx_data;
        SleepRate value = SleepRate::SLEEP_RATE_50HZ;
        if (readRegister(Registers::CTRL_REG1, &rx_data)) {
            value = static_cast<SleepRate>(rx_data >> 6);
        }
        return value;
    }
    
    DataRate Mma8451::getDataRate(void)
    {
        uint8_t rx_data;
        DataRate value = DataRate::RATE_800HZ;
        if (readRegister(Registers::CTRL_REG1, &rx_data)) {
            value = static_cast<DataRate>((rx_data >> 3) & 0x03);
        }
        return value;
    }
    
    bool Mma8451::isLowNoiseModeEnabled(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::CTRL_REG1, &rx_data)) {
            return (rx_data & 0x04) == 0x04;
        }
        return false;
    }
    
    bool Mma8451::isFastReadModeEnabled(void)
    {
        uint8_t rx_data;
        if (readRegister(Registers::CTRL_REG1, &rx_data)) {
            return (rx_data & 0x02) == 0x02;
        }
        return false;
    }
    
    bool Mma8451::runSelfTest(bool enable)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG2, &rx_data)) {
                tx_data = (rx_data & 0x7F) | (enable << 7);
                if (writeRegister(Registers::CTRL_REG2, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::resetDevice(bool enable)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG2, &rx_data)) {
                tx_data = (rx_data & 0xBF) | (enable << 6);
                if (writeRegister(Registers::CTRL_REG2, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setSleepPowerMode(PowerMode mode)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG2, &rx_data)) {
                tx_data = (rx_data & 0xE7) | (static_cast<uint8_t>(mode) << 3);
                if (writeRegister(Registers::CTRL_REG2, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::enableAutoSleep(bool enable)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG2, &rx_data)) {
                tx_data = (rx_data & 0xFB) | (enable << 2);
                if (writeRegister(Registers::CTRL_REG2, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setActivePowerMode(PowerMode mode)
    {
        uint8_t rx_data, tx_data;
        if (setActive(false)) {
            if (readRegister(Registers::CTRL_REG2, &rx_data)) {
                tx_data = (rx_data & 0xFC) | (static_cast<uint8_t>(mode));
                if (writeRegister(Registers::CTRL_REG2, &tx_data)) {
                    if (setActive(true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool Mma8451::setOffsetCorrections(float xOffset, float yOffset, float zOffset)
    {
        int8_t xyz[3];
        uint8_t* ptrToXyz = reinterpret_cast<uint8_t*>(xyz);
        
        xyz[0] = static_cast<int8_t>(std::min(std::max(xOffset, -0.255f), 0.255f) / 0.002f);
        xyz[1] = static_cast<int8_t>(std::min(std::max(yOffset, -0.255f), 0.255f) / 0.002f);
        xyz[2] = static_cast<int8_t>(std::min(std::max(zOffset, -0.255f), 0.255f) / 0.002f);
            
        if (setActive(false)) {
            writeRegister(Registers::OFF_X, &(ptrToXyz[0]));
            writeRegister(Registers::OFF_Y, &(ptrToXyz[1]));
            writeRegister(Registers::OFF_Z, &(ptrToXyz[2]));
            if (setActive(true)) {
                return true;
            }
        }
        return false;
    }
}