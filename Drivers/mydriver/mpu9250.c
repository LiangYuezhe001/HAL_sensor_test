#include "mpu6050.h"
#include "ak8963.h"
#include "bmp280.h"
#include "mpu9250.h"

//u8 I2Cx;

void sensorsSetupSlaveRead(void)
{   u8 I2Cx;
	//set 6500
    i2cdevWriteBits(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_MST_DLY_BIT,
                    MPU6500_I2C_SLV4_MST_DLY_LENGTH, 19);// slave read speed: 100Hz = (1000Hz / (1 + 9))
    i2cdevWriteBit(0, MPU_ADDR, MPU6500_RA_INT_PIN_CFG, 1, 0);//disable 6500 bypass mode
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_CTRL, MPU6500_WAIT_FOR_ES_BIT, 1);//wait for slave data loaded
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_P_NSR_BIT, 0); //Set slave read/write transition disable
    i2cdevWriteBits(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_CLK_BIT,
                    MPU6500_I2C_MST_CLK_LENGTH, 13);//set iic 400hz

    //set slave 0
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV0_ADDR,0x80|AK8963_ADDRESS_00);//set 8963 as slave 0
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV0_REG , AK8963_RA_ST1);// set reg to read
    i2cdevWriteBits(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV0_CTRL, MPU6500_I2C_SLV_LEN_BIT,
                    MPU6500_I2C_SLV_LEN_LENGTH, 1);// set num of reg to read
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_DELAY_CTRL, 0, 1);
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV0_CTRL, MPU6500_I2C_SLV_EN_BIT,1);//enable slave 0

    //set slave 1
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV1_ADDR,0x80|BMP280_I2C_ADDR);//set 280 as slave 0
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV1_REG , BMP280_STAT_REG);// set reg to read
    i2cdevWriteBits(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV1_CTRL, MPU6500_I2C_SLV_LEN_BIT,
                    MPU6500_I2C_SLV_LEN_LENGTH, 1);// set num of reg to read
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_DELAY_CTRL, 1, 1);
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV1_CTRL, MPU6500_I2C_SLV_EN_BIT,1);//enable slave 1

    //set slave 2
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV2_ADDR,0x80|BMP280_I2C_ADDR);//set 8963 as slave 2
    i2cdevWriteByte(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV2_REG , BMP280_PRESSURE_MSB_REG);// set reg to read
    i2cdevWriteBits(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV2_CTRL, MPU6500_I2C_SLV_LEN_BIT,
                    MPU6500_I2C_SLV_LEN_LENGTH, 6);// set num of reg to read
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_MST_DELAY_CTRL, 2, 1);
    i2cdevWriteBit(I2Cx, MPU_ADDR, MPU6500_RA_I2C_SLV2_CTRL, MPU6500_I2C_SLV_EN_BIT,1);//enable slave 2
}

void Mpu9250Init(void)
{
	
				
	iicsearch();
	i2cdevWriteBit(0, MPU_ADDR, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, 1);
    if (MPU_Init() == 0)
    {		
				i2cdevWriteByte(0, MPU_ADDR, MPU6500_RA_INT_ENABLE, 0);
				HAL_Delay(1);
        i2cdevWriteBit(0, MPU_ADDR, MPU6500_RA_INT_PIN_CFG, 1, 1);//set 6500 bypass mode
				HAL_Delay(1);
        // allow iic to r/w other divice
        printf("6500 I2C connection [OK].\r\n");
			 iicsearch();
    }
    else
    {
        printf("6500 I2C connection [FAIL].\r\n");
    }

   // MPU_Init(); //6500 basic init 

    if (ak8963TestConnection() == true)
    {
        ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
        printf("AK8963 I2C connection [OK].\r\n");
    }
    else
    {
        printf("AK8963 I2C connection [FAIL].\r\n");
    }

    if (bmp280Init() == true)
    {

        printf("AK8963 I2C connection [OK].\r\n");
    }
    else
    {
        printf("AK8963 I2C connection [FAIL].\r\n");
    }
   // void sensorsSetupSlaveRead(void);

}

void get_all_rawdata(int16_t* acc,int16_t* gyro,int16_t* mag,int16_t* baro)
{
	u8 I2Cx;
    u8 buff[30];
    i2cdevRead(I2Cx, MPU_ADDR, MPU6500_RA_ACCEL_XOUT_H, 29, buff);
    //acc&gyro
    acc[1] = (((int16_t) buff[0]) << 8) | buff[1];
    acc[0] = ((((int16_t) buff[2]) << 8) | buff[3]);
    acc[2] = (((int16_t) buff[4]) << 8) | buff[5];
    gyro[1] = (((int16_t) buff[8]) << 8) | buff[9];
    gyro[0] = (((int16_t) buff[10]) << 8) | buff[11];
    gyro[2] = (((int16_t) buff[12]) << 8) | buff[13];
		//mag
    mag[0] = (((int16_t) buff[15]) << 8) | buff[14];
    mag[1] = (((int16_t) buff[17]) << 8) | buff[16];
    mag[2] = (((int16_t) buff[19]) << 8) | buff[18];
		//baro
	
//	if ((buff[19] & 0x08)) /*bit3=1 ????*/
//		{
//			s32 rawPressure = (s32)((((u32)(buffer[1])) << 12) | (((u32)(buffer[2])) << 4) | ((u32)buffer[3] >> 4));
//			s32 rawTemp = (s32)((((u32)(buffer[4])) << 12) | (((u32)(buffer[5])) << 4) | ((u32)buffer[6] >> 4));
//			temp = bmp280CompensateT(rawTemp)/100.0f;		
//			pressure = bmp280CompensateP(rawPressure)/25600.0f;			

////			pressureFilter(&pressure, &sensors.baro.pressure);	
//			sensors.baro.pressure = pressure;
//			sensors.baro.temperature = (float)temp;	/*???*/
//			sensors.baro.asl = bmp280PressureToAltitude(&pressure) * 100.f;	/*?????*/
//		}

}
