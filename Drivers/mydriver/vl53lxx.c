#include "sys.h"
#include "tim.h"
#include "vl53lxx_i2c.h"
#include "vl53lxx.h"
#include "vl53l1_api.h"

//VL53L1_Dev_t	dev;


u16 vl53lxxId = 0;	/*vl53芯片ID*/
//bool isEnableVl53lxx = true;		/*是否使能激光*/

//static bool isInitvl53l0x = false;	/*初始化vl53l0x*/
//static bool isInitvl53l1x = false;	/*初始化vl53l1x*/
//static bool reInitvl53l0x = false;	/*再次初始化vl53l0x*/
//static bool reInitvl53l1x = false;	/*再次初始化vl53l1x*/

static u8 count = 0;
static u8 validCnt = 0;
static u8 inValidCnt = 0;

static u16 range_last = 0;
float quality = 1.0f;

//zRange_t vl53lxx;


void vl53l0xTask(void* arg);
void vl53l1xTask(void* arg);
	
void vl53lxxInit(void)
{
	vl53IICInit();	
	delay_ms(10);
	vl53lxxId = vl53l0xGetModelID();
	VL53L1_RdWord(&dev, 0x010F, &vl53lxxId);
	if(vl53lxxId == VL53L1X_ID)
	{
		vl53l1xSetParam();
	}	
	else
	{
		
		
	}
	vl53lxxId = 0;
}



void vl53l1x(void)
{
	int status;
	u8 isDataReady = 0;
	//TickType_t xLastWakeTime = xTaskGetTickCount();;
	static VL53L1_RangingMeasurementData_t rangingData;
	vl53l1xSetParam();	/*设置vl53l1x 参数*/
	
	while(1) 
	{
			status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);	
			if(isDataReady)
			{
				status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
				if(status==0)
				{
					range_last = rangingData.RangeMilliMeter * 0.1f;	/*单位cm*/				
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
			}	
			
			if(range_last < VL53L1X_MAX_RANGE)			
				validCnt++;			
			else 			
				inValidCnt++;			
			
			if(inValidCnt + validCnt == 10)
			{
				quality += (validCnt/10.f - quality) * 0.1f;	/*低通*/
				validCnt = 0;
				inValidCnt = 0;
			}

				
	}
}


u16 LaserGetHeight(void)
{
	int status;
	u8 isDataReady = 0;
	status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
	static VL53L1_RangingMeasurementData_t rangingData;	
			if(isDataReady)
			{
				status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
				if(status==0)
				{
					range_last = rangingData.RangeMilliMeter * 0.1f;	/*单位cm*/				
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
			}	
			else
				return 600;
			if(range_last < VL53L1X_MAX_RANGE)			
				validCnt++;			
			else 			
				inValidCnt++;			
			
			if(inValidCnt + validCnt == 10)
			{
				quality += (validCnt/10.f - quality) * 0.1f;	/*低通*/
				validCnt = 0;
				inValidCnt = 0;
			}
			return range_last;
}


//bool vl53lxxReadRange(zRange_t* zrange)
//{
//	if(vl53lxxId == VL53L0X_ID) 
//	{
//		zrange->quality = quality;		//可信度
//		vl53lxx.quality = quality;
//		
//		if (range_last != 0 && range_last < VL53L0X_MAX_RANGE) 
//		{			
//			zrange->distance = (float)range_last;	//单位[cm]
//			vl53lxx.distance = 	zrange->distance;		
//			return true;
//		}
//	}
//	else if(vl53lxxId == VL53L1X_ID) 
//	{
//		zrange->quality = quality;		//可信度
//		vl53lxx.quality = quality;
//		
//		if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) 
//		{			
//			zrange->distance = (float)range_last;	//单位[cm]	
//			vl53lxx.distance = 	zrange->distance;
//			return true;
//		}
//	}
//	
//	return false;
//}



