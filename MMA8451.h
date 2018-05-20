#ifndef __DATALOGGER_MMA8451_H
#define __DATALOGGER_MMA8451_H

#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace mma8451 
{
    
    constexpr uint8_t MAX_DATA_COUNT = 64;
    
    constexpr uint8_t ID_MMA8451 = 0x1A;

    constexpr uint8_t ADDRESS = 0x1D;

    enum FullScale {
        FULL_SCALE_2G = 4096,
        FULL_SCALE_4G = 2048,
        FULL_SCALE_8G = 1024,
        RESERVED = 3
    };
    
    enum FifoMode {
        DISABLED         = 0x00, // Only most recent value is returned.
        CIRCULAR_BUFFER  = 0x01, // Contains most recent samples only.
        STOP_ON_OVERFLOW = 0x02, // Stops accepting new values until old values are read.
        TRIGGER_MODE     = 0x03
    };
    
    enum SystemMode {
        STANDBY = 0x00,
        ACTIVE  = 0x01,
        SLEEP   = 0x02,
        UNKNOWN = 0x03        
    };
    
    struct TriggerMode {
        bool transientInterrupt;
        bool orientationInterrupt;
        bool pulseInterrupt;
        bool freefallInterrupt;
    };
    
    struct InterruptSource {
        bool autoSleep;
        bool fifo;
        TriggerMode triggers;
        bool dataReady;
    };
    
    enum PowerMode {
        NORMAL              = 0x00,
        LOW_NOISE_LOW_POWER = 0x01,
        HIGH_RESOLUTION     = 0x02,
        LOW_POWER           = 0x03
    };
    
    enum SleepRate {
        SLEEP_RATE_50HZ   = 0x00,
        SLEEP_RATE_12p5HZ = 0x01,
        SLEEP_RATE_6p25HZ = 0x02,
        SLEEP_RATE_1p56HZ = 0x03
    };
    
    enum DataRate {
        RATE_800HZ  = 0x00,
        RATE_400HZ  = 0x01,
        RATE_200HZ  = 0x02,
        RATE_100HZ  = 0x03,
        RATE_50HZ   = 0x04,
        RATE_12p5HZ = 0x05,
        RATE_6p25HZ = 0x06,
        RATE_1p56HZ = 0x07
    };
    
    enum Registers {
        STATUS = 0x00,
        OUT_X_MSB,
        OUT_X_LSB,
        OUT_Y_MSB,
        OUT_Y_LSB,
        OUT_Z_MSB,
        OUT_Z_LSB,
        RSVD1,
        RSVD2,
        F_SETUP,
        TRIG_CFG,
        SYSMOD,
        INT_SOURCE,
        WHO_AM_I,
        XYZ_DATA_CFG,
        HP_FILTER_CUTOFF,
        PL_STATUS, // 0x10
        PL_CFG,
        PL_COUNT,
        PL_BF_ZCOMP,
        PL_THS_REG,
        FF_MT_CFG,
        FF_MT_SRC,
        FF_MT_THS,
        FF_MT_COUNT,
        RSVD3,
        RSVD4,
        RSVD5,
        RSVD6,
        TRANSIENT_CFG,
        TRANSIENT_SRC,
        TRANSIENT_THS,
        TRANSIENT_COUNT, // 0x20
        PULSE_CFG,
        PULSE_SRC,
        PULSE_THSX,
        PULSE_THSY,
        PULSE_THSZ,
        PULSE_TMLT,
        PULSE_LTCY,
        PULSE_WIND,
        ASLP_COUNT,
        CTRL_REG1,
        CTRL_REG2,
        CTRL_REG3,
        CTRL_REG4,
        CTRL_REG5,
        OFF_X,
        OFF_Y, // 0x30
        OFF_Z
    };

    
    class Mma8451 
    {
        public:
            Mma8451();  //Constructor
            Mma8451(std::string filename);
            ~Mma8451(); //Destructor

            bool isCommsOpen(void);
            
            bool setCommsFile(std::string filename);
            std::string getCommsFile(void);
            
            std::vector<float> getAccelerations(void);
            
            // Misc Registers
            bool setFullScale(FullScale scale);
            bool setFifoMode(FifoMode mode, uint8_t triggerCount);
            bool setTriggerMode(TriggerMode triggers);
            bool setHighPassOutput(bool enable);
            FullScale getFullScale(void);        
            FifoMode getFifoMode(void);        
            TriggerMode getTriggerMode(void);
            bool getFifoGateError(void);
            uint8_t getElapsedFifoTime(void);
            SystemMode getSystemMode(void);
            InterruptSource getInterrupts(void);
            bool isHighPassOutputEnbabled(void);
            
            // Control Register 1
            bool setAutoSleepSampleFreq(SleepRate rate);
            bool setDataRate(DataRate rate);
            bool enableLowNoiseMode(bool enable);
            bool enableFastReadMode(bool enable);
            SleepRate getAutoSleepSampleFreq(void);
            DataRate getDataRate(void);
            bool isLowNoiseModeEnabled(void);
            bool isFastReadModeEnabled(void);
            
            // Control Register 2
            bool runSelfTest(bool enable);
            bool resetDevice(bool enable);
            bool setSleepPowerMode(PowerMode mode);
            bool enableAutoSleep(bool enable);
            bool setActivePowerMode(PowerMode mode);
            
            // Offset Correction
            bool setOffsetCorrections(float xOffset, float yOffset, float zOffset);
        private:
            bool writeRegister(Registers reg, uint8_t *data);
            bool readRegister(Registers reg,  uint8_t *data);
            
            template<typename T>
            bool readMultipleData(Registers startReg, uint8_t count, T** data);
            
            bool setActive(bool enable);
            bool getActive(void);
            std::string i2cName;
            int i2cComms;
            bool commsOpen;
            
            uint8_t writeBuffer[MAX_DATA_COUNT];
            uint8_t readBuffer[MAX_DATA_COUNT];
            float div2G = 4096;
            float div4G = 2048;
            float div8G = 1024;
            std::vector<float> result;
            
            FullScale scale;
    };
    
}

#endif // __DATALOGGER_MMA8451_H