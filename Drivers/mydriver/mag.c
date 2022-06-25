#include "MAG_IIC.h"
#include "sys.h"
#include "usart.h"
#include <math.h>
#include "tim.h"
#include "mag.h"
#include "string.h"

// u8 mag_set_register_bit(u8 input_register, u8 *register_data, u8 *register_bit, u8 *bit_val)
//{

//	u8 u8_reg;
//	register_data = MAG_Read_Byte(input_register);
//	register_bit = bit_val;
//	MAG_Write_Byte(input_register, register_data);
//	return 1;
//}

// soft reset
u8 mag_soft_reset(void)
{
	HSCDTD_CTRL3_t reg;
	u8 u8_reg;

	///////reset mag sensor//////////////
	reg.SRST = 1;
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL3, u8_reg); //重置
	delay_ms(5);
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&reg, &u8_reg, 1);
	if (reg.SRST == 1)
	{
		//	printf("reset fail");
		return 0;
	}
	else
		return 1;
}
// who am i
//我是sei
u8 who_am_i(void)
{
//	HSCDTD_CTRL3_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_WIA);
	if (u8_reg == 0x49)
		return 0;
	else
		return 1;
}
// set_state
//
u8 mag_set_state(u8 state)
{
	HSCDTD_CTRL1_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL1);
	memcpy(&reg, &u8_reg, 1);
	reg.FS = state;
	/*
	HSCDTD_STATE_NORMAL = 0b00,
	HSCDTD_STATE_FORCE = 0b01,
	*/
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL1, u8_reg);
	return 1;
}
// set_resolution
//设置分辨率耶
u8 mag_set_resolution(void)
{
	HSCDTD_CTRL4_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL4);
	memcpy(&reg, &u8_reg, 1);

	reg.RS = 1; // 15 bit
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL4, u8_reg);
	return 1;
}
// set_mode
//
u8 mag_set_mode(void)
{
	HSCDTD_CTRL1_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL1);
	memcpy(&reg, &u8_reg, 1);
	reg.PC = 1; // ACTIVE MODE
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL1, u8_reg);
	return 1;
}
// self_test
//自测
u8 mag_self_test(void)
{
	HSCDTD_CTRL3_t reg;
	u8 u8_reg;
	//重启
	//
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&reg, &u8_reg, 1);
	reg.STC = 1;
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL3, u8_reg);
	delay_ms(5);
	//

	u8_reg = MAG_Read_Byte(HSCDTD_REG_SELFTEST_RESP);
	if (u8_reg != 0xAA)
		return 0;

	u8_reg = MAG_Read_Byte(HSCDTD_REG_SELFTEST_RESP);
	if (u8_reg != 0x55)
		return 0;

	return 1;
}
u8 mag_set_output_data_rate(void)
{
	HSCDTD_CTRL1_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL1);
	memcpy(&reg, &u8_reg, 1);

	reg.ODR = 3;
	/*
	HSCDTD_ODR_0_5HZ = 0b00,
	HSCDTD_ODR_10HZ = 0b01,
	HSCDTD_ODR_20HZ = 0b10,
	HSCDTD_ODR_100HZ = 0b11,
	*/
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL1, u8_reg);
	return 1;
}

/**
 * @brief Set the fifo data storage method.
 *
 * Set the method for storing data in FIFO. There are 2
 * valid modes:
 *  - Direct (Default)
 *  - Comparision.
 *
 * This functionality is only available if FIFO is enabled.
 *
 * If storage method is set to 'Comparision' refer to page 11
 * of the datasheet for more information.
 *
 * @param p_dev Pointer to device struct.
 * @param fco Storage method
 * @return hscdtd_status.
 */
u8 mag_set_fifo_data_storage_method(void)
{
	HSCDTD_CTRL2_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL2);
	memcpy(&reg, &u8_reg, 1);

	reg.FCO = 0;
	/*
	HSCDTD_FCO_DIRECT = 0b00,
	HSCDTD_FCO_COMP = 0b01,
	*/
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL2, u8_reg);
	return 1;
}

u8 mag_set_fifo_comparision_method(void)
{

	HSCDTD_CTRL2_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL2);
	memcpy(&reg, &u8_reg, 1);

	reg.AOR = 0;
	/*
	HSCDTD_AOR_OR = 0b00, (Default)
	HSCDTD_AOR_AND = 0b01,
	*/
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL2, u8_reg);
	return 1;
}

u8 mag_set_fifo_enable(void)
{

	HSCDTD_CTRL2_t reg;
	u8 u8_reg;
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL2);
	memcpy(&reg, &u8_reg, 1);

	reg.FF = 0;
	/*
	HSCDTD_FF_DISABLE = 0b00,
	HSCDTD_FF_ENABLE = 0b01,
	*/
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL2, u8_reg);
	return 1;
}
u8 mag_read_temp(void)
{
	u8 u8_reg;
	// We can safely cast the uint8_t to a int8_t as the the value of the
	// value is formatted as int8_t.
	u8_reg = MAG_Read_Byte(HSCDTD_REG_TEMP);
	// Ignore read register status.
	return u8_reg;
}

u8 mag_offset_calibration(void)
{

	HSCDTD_CTRL3_t reg;
	u8 u8_reg;

	// Set the state to the force state.
	mag_set_state(1);

	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&reg, &u8_reg, 1);

	reg.OCL = 1;

	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL3, u8_reg);

	mag_set_state(0);
	return 1;
}

u8 mag_temperature_compensation(void)
{
	u8 status;
	int8_t i;
	HSCDTD_CTRL3_t reg;
	u8 u8_reg;
	HSCDTD_STAT_t stat;

	// Set the state to the force state.
	mag_set_state(1);
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&reg, &u8_reg, 1);
	reg.TCS = 1;
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL3, u8_reg);

	status = 0;
	// Attempt to check status for ~50ms (Duration does not really matter).
	// If no temperature after that, something has gone wrong.
	for (i = 0; i < 50; i++)
	{
		delay_ms(1);

		// Read status register to check if temp data is ready.
		u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
		memcpy(&stat, &u8_reg, 1);
		if (stat.TRDY == 1)
		{
			// The datasheet specifies that the bit is cleared after
			// reading the TEMP register.
			// We don't need the value here, so we don't need the return
			// value.
			mag_read_temp();
			status = 1;
			break;
		}
	}
	// Set old state back.
	mag_set_state(0);
	return status;
}

u8 mag_read_data(hscdtd_mag_t *p_mag_data)
{
	//	u8 status;
	int8_t i;
	uint8_t buf[6];
	int16_t tmp;
	float *mag_data;

	mag_data = &p_mag_data->mag_x;

	// Read all mag data registers in one go.
	for (i = 0; i < 6; i++)
	{
		buf[i] = MAG_Read_Byte(HSCDTD_REG_XOUT_L + i);
	}

	for (i = 0; i < 3; i++)
	{
		// Each axis is formatted little endian, flip it and make it signed.
		tmp = (int16_t)((uint16_t)((buf[2 * i + 1] << 8) | (buf[2 * i])));
		mag_data[i] = tmp * 0.150f; // Assumes 15 bit value.
	}

	return 1;
}

u8 mag_force_mode_read_data(hscdtd_mag_t *p_mag_data)
{
	u8 status;
	u8 u8_reg;
	HSCDTD_STAT_t stat;
	HSCDTD_CTRL3_t reg;
	int8_t i;

	// Read the status register to clear any status bits.
	u8_reg = MAG_Read_Byte(HSCDTD_REG_STATUS);

	// Start measurement
	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&reg, &u8_reg, 1);
	reg.FRC = 1;
	memcpy(&u8_reg, &reg, 1);
	MAG_Write_Byte(HSCDTD_REG_CTRL3, u8_reg);

	// Wait until data is ready.
	for (i = 0; i < 10; i++)
	{
		u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
		memcpy(&stat, &u8_reg, 1);
		if (stat.DRDY == 1)
			break;
		delay_ms(1);
	}

	if (stat.DRDY != 1)
		return 0;

	// Use magneto read function to read the data into the pointer.
	status = mag_read_data(p_mag_data);
	return status;
}

u8 mag_data_ready()
{
	//	u8 status;
	HSCDTD_STAT_t stat;
	u8 u8_reg;

	u8_reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	memcpy(&stat, &u8_reg, 1);
	return stat.DRDY;
}

u8 mag_set_offset(float x_off, float y_off, float z_off)
{
	uint16_t tmp_off;
	uint8_t offset_map[6];
	int8_t i;

	// Use a variable to make it easier to support 14bit in the future.
	float max_offset = 2457.6;

	// The sensor substracts the offset from the sensor value.
	// This doesn't really make sense from a user perspective. So the
	// negative version of the user supplied offset is applied.
	// Put those in a array to simplify conversion.
	float offsets[] = {-x_off, -y_off, -z_off};

	// Loop over the values.
	for (i = 0; i < 3; i++)
	{
		// Check if the offset value is valid.
		if (offsets[i] > max_offset || offsets[i] < -max_offset)
		{
			// If the value is larger than the max value the behavior is
			// undefined. Best to avoid it.
			return 0;
		}
		// Convert to uin16_t
		tmp_off = (uint16_t)(offsets[i] / 0.15f);
		// Write the values to the offset map
		offset_map[2 * i] = (uint8_t)(tmp_off & 0xFF);
		offset_map[2 * i + 1] = (uint8_t)((tmp_off >> 8) & 0xFF);
	}

	// Write the offset map to the sensor.
	for (i = 0; i < 6; i++)
	{
		MAG_Write_Byte(HSCDTD_REG_OFFSET_X_L + i, offset_map[i]);
	}
	return 1;
}

//
//
//初始化
u8 MAG_Init(void)
{
	mag_soft_reset();
	who_am_i();
	mag_set_state(0);
	mag_set_resolution();
	mag_set_mode();
	mag_self_test();
	mag_set_output_data_rate();
	mag_set_fifo_enable();
	mag_temperature_compensation();
	return 1;
}
