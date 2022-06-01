#include "sys.h"
#if defined(__CC_ARM)
#pragma anon_unions /*??????????*/
#endif

typedef __packed struct motionBurst_s
{
    __packed union
    {
        uint8_t motion;
        __packed struct
        {
            uint8_t frameFrom0 : 1;
            uint8_t runMode : 2;
            uint8_t reserved1 : 1;
            uint8_t rawFrom0 : 1;
            uint8_t reserved2 : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} motionBurst_t;

u8 getandsend(void);
void registerWrite(uint8_t reg, uint8_t value);
void readMotion(motionBurst_t *motion);
static uint8_t registerRead(uint8_t reg);
static void InitRegisters(void);
u8 opticalFlowInit(void);