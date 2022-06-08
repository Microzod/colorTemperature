#include <Arduino.h>
#include "colorTemperature.h"
#include <LTC2633Library.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp32-hal-timer.h"
#include "freertos/xtensa_api.h"
#include "rom/ets_sys.h"
#include "soc/timer_group_struct.h"
#include "soc/dport_reg.h"
#include "esp_attr.h"
#include "esp_intr.h"

/*
TaskHandle_t colorTemperature::rampUpTaskHandle = NULL;
hw_timer_t * colorTemperature::rampUpTimer = NULL;
portMUX_TYPE colorTemperature::rampUpTimerMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t colorTemperature::rampDownTaskHandle = NULL;
hw_timer_t * colorTemperature::rampDownTimer = NULL;
portMUX_TYPE colorTemperature::rampDownTimerMux = portMUX_INITIALIZER_UNLOCKED;
*/


TaskHandle_t colorTemperature::rampUpTaskHandle = NULL;
hw_timer_t * colorTemperature::rampUpTimer = NULL;
portMUX_TYPE colorTemperature::rampUpTimerMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t colorTemperature::rampDownTaskHandle = NULL;
hw_timer_t * colorTemperature::rampDownTimer = NULL;
portMUX_TYPE colorTemperature::rampDownTimerMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t 			colorTemperature::triggerSunriseAlarm_id;
uint8_t 			colorTemperature::triggerSunsetAlarm_id;
DAC_coefficients 	colorTemperature::coefficients;
DAC_coefficients 	colorTemperature::errorCoefficients;
DAC_artificial_data colorTemperature::artificial;
DAC_writting_data 	colorTemperature::writeValues;
ISR_DATA 			colorTemperature::isrRampValues;

//_percent_adjustmentCoefficients_DAC_A {0.0   , 0.0414, 0.0710, 0.1041, 0.1385, 0.1704, 0.2012, 0.2343, 0.2651, 0.2982, 0.3314, 0.3669, 0.4024, 0.4379, 0.4793, 0.5231, 0.5704, 0.6225, 0.6817, 0.7408, 0.8166, 0.8852, 0.9538, 1.0000},
//_percent_adjustmentCoefficients_DAC_B {1.0000, 0.9586, 0.9290, 0.8959, 0.8615, 0.8296, 0.7988, 0.7657, 0.7349, 0.7018, 0.6686, 0.6331, 0.5976, 0.5621, 0.5207, 0.4769, 0.4296, 0.3775, 0.3183, 0.2592, 0.1834, 0.1148, 0.0462, 0.0}

colorTemperature::colorTemperature( uint8_t ssrA5000k_pin = 26,
                                    uint8_t ssrB2700k_pin = 27,
                                    LTC2633_I2C_ADDRESS I2C_address = LTC2633_CA0_GLOBAL,
                                    TwoWire *theWire = &Wire):
_CCT {5000, 4900, 4800, 4700, 4600, 4500, 4400, 4300, 4200, 4100, 4000, 3900, 3800, 3700, 3600, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700},
_adjustmentCoefficients_DAC_A {0,170,291,426,567,698,824,959,1086,1221,1357,1502,1648,1793,1963,2142,2336,2549,2792,3034,3344,3625,3906,4095},
_adjustmentCoefficients_DAC_B {4095,3925,3804,3669,3528,3397,3271,3136,3009,2874,2738,2593,2447,2302,2132,1953,1759,1546,1303,1061,751,470,189,0},
LTC2633(LTC2633_12BIT, I2C_address, theWire)
{
    _SSR_A_5000k_pin = ssrA5000k_pin;
    _SSR_B_2700k_pin = ssrB2700k_pin;
}

void colorTemperature::begin(uint8_t timerForRampUp = 1, uint8_t timerForRampDown = 2)
{
    _timerUsedFor_RampUp = timerForRampUp;
    _timerUsedFor_RampDown = timerForRampDown;
    
    // Calculate the length of the CCT array:
    _arrayLength = sizeof(_CCT)/sizeof(_CCT[0]);
    
    // Set a default value for the used color temperature:
    setColorTemperature(3900);
    
    // Set a default value for the light intensity:
    coefficients.Intensity = 1023;
	
	turnOff();
    
    // Start the LTC2633 I2C communication and set it to use 400kHz:
    LTC2633.begin(1);
    LTC2633.setI2CFastClockSpeed();
	LTC2633.powerDownChip();
    
    // Set the relay control pins to it's 'OFF' state:
    pinMode(_SSR_A_5000k_pin, OUTPUT);
        digitalWrite(_SSR_A_5000k_pin, LOW);
    pinMode(_SSR_B_2700k_pin, OUTPUT);
        digitalWrite(_SSR_B_2700k_pin, LOW);
    
    // Initilize the struct used as a indicator that an error has occured:
    errorCoefficients.A = 4.0;
    errorCoefficients.B = 4.0;
    errorCoefficients.Intensity = 4.0;
    errorCoefficients.raw_A = 5000;
    errorCoefficients.raw_B = 5000;
    errorCoefficients.valueMatchFlag = 4;

    //setupTimeAlarms();
    // Prepare the TimeAlarms alarm used to trigger a Sunrise:
    triggerSunriseAlarm_id = Alarm.alarmRepeat(1500, triggerSunriseFunction);
    Alarm.disable(triggerSunriseAlarm_id);
    
    // Prepare the TimeAlarms alarm used to trigger a Sunset:
    triggerSunsetAlarm_id = Alarm.alarmRepeat(1000, triggerSunsetFunction);
    Alarm.disable(triggerSunsetAlarm_id);
    
    // Start the rampUpTask Task:
    xTaskCreatePinnedToCore(&rampUpTask,
                            "rampUpTask",
                            2048,                           					// BaseType_t xTaskCreatePinnedToCore
                            NULL,                           					// (
                            2,                              					//     TaskFunction_t      pvTaskCode,
                            &colorTemperature::rampUpTaskHandle,              	//     const char *const   pcName,
                            1);                             					//     const uint32_t      usStackDepth,
																				//     void *const         pvParameters,
																				//     UBaseType_t         uxPriority,
	// Start the rampDownTask Task:                         					//     TaskHandle_t *const pvCreatedTask,
    xTaskCreatePinnedToCore(&rampDownTask,                  					//     const BaseType_t    xCoreID
                            "rampDownTask",                 					// )
                            2048,                           					
                            NULL,
                            2,
                            &rampDownTaskHandle,
                            1);  
    
    // Prepare the HW Timer used for rampUp/Sunrise:
    rampUpTimer = timerBegin(_timerUsedFor_RampUp, 80, true);
    timerAttachInterrupt(rampUpTimer, &rampUp_ISR, true);
    timerAlarmWrite(rampUpTimer, 500000, true);
    timerAlarmEnable(rampUpTimer);
    timerStop(rampUpTimer);
    timerAlarmDisable(rampUpTimer);
    
    // Prepare the HW Timer used for rampDown/Sunset:
    rampDownTimer = timerBegin(_timerUsedFor_RampDown, 80, true);
    timerAttachInterrupt(rampDownTimer, &rampDown_ISR, true);
    timerAlarmWrite(rampDownTimer, 500000, true);
    timerAlarmEnable(rampDownTimer);
    timerStop(rampDownTimer);
    timerAlarmDisable(rampDownTimer);
    
}

DAC_coefficients colorTemperature::setColorTemperature(uint16_t value)
{
    coefficients.valueMatchFlag = 0;
    for (uint8_t i = 0; i < _arrayLength; i++)
    {
        if (value == _CCT[i])
        {
            coefficients.valueMatchFlag = 1;
            coefficients.A = _adjustmentCoefficients_DAC_A[i];
            coefficients.B = _adjustmentCoefficients_DAC_B[i];
            return coefficients;
        }
    }

    return errorCoefficients;
}

DAC_coefficients colorTemperature::writeColorTemperature(uint16_t value)
{
    coefficients.valueMatchFlag = 0;
    for (uint8_t i = 0; i < _arrayLength; i++)
    {
        if (value == _CCT[i])
        {
            coefficients.valueMatchFlag = 1;
            coefficients.A = _adjustmentCoefficients_DAC_A[i];
            coefficients.B = _adjustmentCoefficients_DAC_B[i];
            calculateInternalValues();
            writeToDAC();
            return coefficients;
        }
    }

    return errorCoefficients;
}

DAC_coefficients colorTemperature::setIntensity(uint16_t intensity)
{
    coefficients.Intensity = intensity;
    return coefficients;
}

DAC_coefficients colorTemperature::writeIntensity(uint16_t intensity)
{
    coefficients.Intensity = intensity;
    calculateInternalValues();
    writeToDAC();
    return coefficients;
}

void colorTemperature::setPowerState(uint8_t onOrOff)
{
	_onOrOff = onOrOff;
}

void colorTemperature::turnOn()
{
	_onOrOff = 1;
}

void colorTemperature::turnOff()
{
	_onOrOff = 0;
}

DAC_coefficients colorTemperature::writeToLed()
{
    if (coefficients.valueMatchFlag)
    {
        calculateInternalValues();
        
		if (_onOrOff)
		{
			digitalWrite(_SSR_A_5000k_pin, HIGH);
			digitalWrite(_SSR_B_2700k_pin, HIGH);
			writeToDAC();
		}
		else
		{
			LTC2633.powerDownChip();
			digitalWrite(_SSR_A_5000k_pin, LOW);
			digitalWrite(_SSR_B_2700k_pin, LOW);
		}
        return coefficients;
    }
    
    return errorCoefficients;
}

void colorTemperature::calculateInternalValues()
{
    writeValues.artificialMaxValue = coefficients.Intensity;
    
    double resultA = (coefficients.Intensity * (coefficients.A / 4095)) + 0.5;
    double resultB = (coefficients.Intensity * (coefficients.B / 4095)) + 0.5;
    
    writeValues.A = (uint16_t)resultA;
    writeValues.B = (uint16_t)resultB;
    
    writeValues.written_A_plus_B = writeValues.A + writeValues.B;
}

void colorTemperature::writeToDAC()
{
    LTC2633.A.write(writeValues.A);
    LTC2633.B.write(writeValues.B);
}

int8_t colorTemperature::writeRawA(uint16_t valueA)
{
    writeValues.A = valueA;
    return LTC2633.A.write(valueA);
}

int8_t colorTemperature::writeRawB(uint16_t valueB)
{
    writeValues.B = valueB;
    return LTC2633.B.write(valueB);
}

int8_t colorTemperature::writeRaw(uint16_t valueA, uint16_t valueB)
{
    writeValues.A = valueA;
    writeValues.B = valueB;
    LTC2633.A.write(valueA);
    return LTC2633.B.write(valueB);
}

uint16_t colorTemperature::getValueA()
{
    return writeValues.A;
}

uint16_t colorTemperature::getValueB()
{
    return writeValues.B;
}

void colorTemperature::enableLED(char led)
{
	if (led == 'A')
		digitalWrite(_SSR_A_5000k_pin, HIGH);
	else if (led == 'B')
		digitalWrite(_SSR_B_2700k_pin, HIGH);
}

void colorTemperature::disableLED(char led)
{
	if (led == 'A')
		digitalWrite(_SSR_A_5000k_pin, LOW);
	else if (led == 'B')
		digitalWrite(_SSR_B_2700k_pin, LOW);
}

void colorTemperature::enableLEDs()
{
    digitalWrite(_SSR_B_2700k_pin, HIGH);
    digitalWrite(_SSR_A_5000k_pin, HIGH);
}

void colorTemperature::enableLED2700k()
{
    digitalWrite(_SSR_B_2700k_pin, HIGH);
}

void colorTemperature::enableLED5000k()
{
    digitalWrite(_SSR_A_5000k_pin, HIGH);
}

void colorTemperature::disableLEDs()
{
    digitalWrite(_SSR_B_2700k_pin, LOW);
    digitalWrite(_SSR_A_5000k_pin, LOW);
}

void colorTemperature::disableLED2700k()
{
    digitalWrite(_SSR_B_2700k_pin, LOW);
}

void colorTemperature::disableLED5000k()
{
    digitalWrite(_SSR_A_5000k_pin, LOW);
}

/*
void colorTemperature::calculateInternalValuesPercent()
{
    artificial.maxValue_double = (4095.0 * coefficients.IntensityPercent) + 0.5;
    double maxResult = artificial.maxValue_double;
    artificial.maxValue_uint = (uint16_t)maxResult;
    writeValues.artificialMaxValue = (uint16_t)maxResult;
    
    double resultA = (artificial.maxValue_double * coefficients.A) + 0.5;
    double resultB = (artificial.maxValue_double * coefficients.B) + 0.5;
    
    writeValues.A = (uint16_t)resultA;
    writeValues.B = (uint16_t)resultB;
    
    //artificial.bothValuesCombined = writeValues.A + writeValues.B;
}
*/

uint32_t colorTemperature::calculateRampUpValues()
{
    calculateInternalValues();
    // Calaculate the timeInterval needed in between each increament by 1 in order to complete during rampUpDuration, result in µS:
    double temp = ((isrRampValues.sunriseLength / coefficients.Intensity) * 1000000) + 0.5;
    isrRampValues.rampUpUpdateInterval_uS = (uint32_t)temp;
    
    // Load the appropiate values, which will be used to fill the tempStorage for the current value during ramping.
    // This is for rampUp hence the DAC write process should begin with 0 and then ramp uppwards:
    isrRampValues.rampUpStartValueA = 0;
    isrRampValues.rampUpStartValueB = 0;
    
    // These values will be used to keep track of when the 2 above values has reached there target.
    isrRampValues.rampUpTargetValueA = writeValues.A;
    isrRampValues.rampUpTargetValueB = writeValues.B;
    
    // This is effectivly A + B, the total number of increamnets need to finish the rampUp:
    isrRampValues.maxValueForRampUpCounter = writeValues.written_A_plus_B;
    
    isrRampValues.rampUpDataCalculated = 1;
    
    return isrRampValues.rampUpUpdateInterval_uS;
}

uint32_t colorTemperature::calculateRampDownValues()
{
    calculateInternalValues();
    // Calaculate the timeInterval needed in between each increament by 1 in order to complete during rampUpDuration, result in µS:
    // temp = (( timeDurationOfSunriseInSeconds / DACValue) * 1000000 ) + 0.5 = numberOf_µS_toWriteToTimer
    double temp = ((isrRampValues.sunsetLength / coefficients.Intensity) * 1000000) + 0.5;
    isrRampValues.rampDownUpdateInterval_uS = (uint32_t)temp;
    
    // Load the appropiate values, which will be used to fill the tempStorage for the current value during ramping.
    // This is for rampDown hence the DAC write process should begin with the maximum value and then ramp downwards:
    isrRampValues.rampDownStartValueA = writeValues.A;
    isrRampValues.rampDownStartValueB = writeValues.B;
    
    // These values will be used to keep track of when the 2 above values has reached there target.
    isrRampValues.rampDownTargetValueA = 0;
    isrRampValues.rampDownTargetValueB = 0;
    
    // This is effectivly A + B, the total number of increamnets need to finish the rampUp:
    isrRampValues.maxValueForRampDownCounter = writeValues.written_A_plus_B;
    
    isrRampValues.rampDownDataCalculated = 1;
    
    return isrRampValues.rampDownUpdateInterval_uS;
}

void colorTemperature::triggerSunriseFunction()
{
    calculateRampUpValues();
    isrRampValues.isRampingUp = 1;
    
    uint32_t tempRampValue_uS = isrRampValues.rampUpUpdateInterval_uS;
    
    xTaskNotify(colorTemperature::rampUpTaskHandle,   // The handle of the port being notified.
                0,                  // Data that can be sent with the notification. How the data is used depends on the value of the eAction parameter.
                 eNoAction);            // eIncrement/eDecrement?/eSetValueWithOverwrite/eSetValueWithoutOverwrite/eNoAction
    
    timerAlarmWrite(rampUpTimer, tempRampValue_uS, true);
    timerStart(rampUpTimer);
    timerAlarmEnable(rampUpTimer);
}

void colorTemperature::triggerSunsetFunction()
{
    calculateRampDownValues();
    isrRampValues.isRampingDown = 1;
    
    uint32_t tempRampValue_uS = isrRampValues.rampDownUpdateInterval_uS;
    
    xTaskNotify(rampDownTaskHandle, // The handle of the port being notified.
                0,                  // Data that can be sent with the notification. How the data is used depends on the value of the eAction parameter.
                 eNoAction);            // eIncrement/eDecrement?/eSetValueWithOverwrite/eSetValueWithoutOverwrite/eNoAction
    
    timerAlarmWrite(rampDownTimer, tempRampValue_uS, true);
    timerStart(rampDownTimer);
    timerAlarmEnable(rampDownTimer);
}

void colorTemperature::setSunriseAlarmTime(time_t turnOnTime)
{
    isrRampValues.sunriseTriggerTime = turnOnTime;
    Alarm.write(triggerSunriseAlarm_id, isrRampValues.sunriseTriggerTime);
    isrRampValues.rampUpIsEnabled = 0;
    Alarm.disable(triggerSunriseAlarm_id);
}

void colorTemperature::setSunsetAlarmTime(time_t turnOffTime)
{
    isrRampValues.sunsetTriggerTime = turnOffTime;
    Alarm.write(triggerSunsetAlarm_id, isrRampValues.sunsetTriggerTime);
    isrRampValues.rampDownIsEnabled = 0;
    Alarm.disable(triggerSunsetAlarm_id);
}

void colorTemperature::setSunriseDuration(time_t sunriseLength_seconds)
{
    
    isrRampValues.sunriseLength = sunriseLength_seconds;

}

void colorTemperature::setSunsetDuration(time_t sunsetLength_seconds)
{
    
    isrRampValues.sunsetLength = sunsetLength_seconds;
}

void colorTemperature::enableSunriseAlarm()
{
    isrRampValues.rampUpIsEnabled = 1;
    Alarm.enable(triggerSunriseAlarm_id);
}

void colorTemperature::enableSunsetAlarm()
{
    isrRampValues.rampDownIsEnabled = 1;
    Alarm.enable(triggerSunsetAlarm_id);
}

void colorTemperature::disableSunriseAlarm()
{
    isrRampValues.rampUpIsEnabled = 0;
    Alarm.disable(triggerSunriseAlarm_id);
}

void colorTemperature::disableSunsetAlarm()
{
    isrRampValues.rampDownIsEnabled = 0;
    Alarm.disable(triggerSunsetAlarm_id);
}
 
bool colorTemperature::isSunriseAlarmEnabled()
{
    return isrRampValues.rampUpIsEnabled;
}
 
bool colorTemperature::isSunsetAlarmEnabled()
{
    return isrRampValues.rampDownIsEnabled;
}


colorTemperature LED = colorTemperature(26, 27, LTC2633_CA0_VCC, &Wire);

// Timer Interrupt related functions:
void IRAM_ATTR rampUp_ISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    portENTER_CRITICAL_ISR(&colorTemperature::rampUpTimerMux);  // Preemptive context switches cannot occur when in between portENTER_CRITICAL_ISR and portEXIT_CRITICAL_ISR.
    xTaskNotifyFromISR( colorTemperature::rampUpTaskHandle,        // The handle of the port being notified.
                        1,                                      // Data that can be sent with the notification. How the data is used depends on the value of the eAction parameter.
                        eNoAction,                              // eIncrement/eDecrement?/eSetValueWithOverwrite/eSetValueWithoutOverwrite/eNoAction
                        &xHigherPriorityTaskWoken);             // will get set to pdTRUE if a context switch should be performed before exiting the ISR.
    portEXIT_CRITICAL_ISR(&colorTemperature::rampUpTimerMux);
    
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

void rampUpTask(void *pvParameters)
{
    volatile bool restartThisTask;
    volatile uint8_t twoFlags;
    volatile uint16_t valueOfDAC_A;
    volatile uint16_t valueOfDAC_B;
    volatile uint16_t targetValueOfDAC_A;
    volatile uint16_t targetValueOfDAC_B;
    volatile uint16_t maxValueRampCounter;
    volatile uint16_t rampCounter;
    //topOfRampUp:
    
    for (;;)
    {
        // Waiting for the very first notification to start the loop:
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Set the used variables to there original state:        
        valueOfDAC_A        = colorTemperature::isrRampValues.rampUpStartValueA;
        valueOfDAC_B        = colorTemperature::isrRampValues.rampUpStartValueB;
        targetValueOfDAC_A  = colorTemperature::isrRampValues.rampUpTargetValueA;
        targetValueOfDAC_B  = colorTemperature::isrRampValues.rampUpTargetValueB;
        maxValueRampCounter = colorTemperature::isrRampValues.maxValueForRampUpCounter;
        rampCounter = 0;
        twoFlags = 0;
        restartThisTask = 0;
        
        while (!restartThisTask)
        {
            
            
            if (valueOfDAC_A <= targetValueOfDAC_A)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                
                portENTER_CRITICAL(&colorTemperature::rampUpTimerMux);
                LED.LTC2633.A.write(valueOfDAC_A++);    // Write to DAC the value of rampUp_valueOfDAC_A++.
                rampCounter++;
                portEXIT_CRITICAL(&colorTemperature::rampUpTimerMux);
            }
            
            
            
            if (valueOfDAC_B <= targetValueOfDAC_B)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                
                portENTER_CRITICAL(&colorTemperature::rampUpTimerMux);
                LED.LTC2633.B.write(valueOfDAC_B++);    // Write to DAC the value of rampUp_valueOfDAC_B++.
                rampCounter++;
                portEXIT_CRITICAL(&colorTemperature::rampUpTimerMux);
            }
            
            //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            if (rampCounter >= maxValueRampCounter)
            {
                restartThisTask = 1;
                colorTemperature::isrRampValues.isRampingUp = 0;
            }
            
        }
        //goto topOfRampUp;
        timerAlarmDisable(colorTemperature::rampUpTimer);
        timerStop(colorTemperature::rampUpTimer);
    }
    
    // Insurance preventing port returning never-ever:
    vTaskDelete( NULL );
}
    
void IRAM_ATTR rampDown_ISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    portENTER_CRITICAL_ISR(&colorTemperature::rampDownTimerMux);// Preemptive context switches cannot occur when in between portENTER_CRITICAL_ISR and portEXIT_CRITICAL_ISR.
    xTaskNotifyFromISR( colorTemperature::rampDownTaskHandle, // The handle of the port being notified.
                        0,                                      // Data that can be sent with the notification. How the data is used depends on the value of the eAction parameter.
                        eNoAction,                              // eIncrement/eDecrement?/eSetValueWithOverwrite/eSetValueWithoutOverwrite/eNoAction
                        &xHigherPriorityTaskWoken);             // will get set to pdTRUE if a context switch should be performed before exiting the ISR.
    portEXIT_CRITICAL_ISR(&colorTemperature::rampDownTimerMux);
    
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

void rampDownTask(void *pvParameters)
{
    volatile bool restartThisTask;
    volatile uint8_t twoFlags;
    volatile uint16_t valueOfDAC_A;
    volatile uint16_t valueOfDAC_B;
    volatile uint16_t targetValueOfDAC_A;
    volatile uint16_t targetValueOfDAC_B;
    volatile uint16_t maxValueRampCounter;
    volatile uint16_t rampCounter;
    //topOfRampUp:
    
    for (;;)
    {
        // Waiting for the very first notification to start the loop:
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Set the used variables to there original state:        
        valueOfDAC_A        = colorTemperature::isrRampValues.rampDownStartValueA;
        valueOfDAC_B        = colorTemperature::isrRampValues.rampDownStartValueB;
        targetValueOfDAC_A  = colorTemperature::isrRampValues.rampDownTargetValueA;
        targetValueOfDAC_B  = colorTemperature::isrRampValues.rampDownTargetValueB;
        maxValueRampCounter = 0;
        rampCounter = colorTemperature::isrRampValues.maxValueForRampDownCounter;;
        twoFlags = 0;
        restartThisTask = 0;
        
        while (!restartThisTask)
        {
            
            
            if (valueOfDAC_A)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                
                portENTER_CRITICAL(&colorTemperature::rampDownTimerMux);
                LED.LTC2633.A.write(valueOfDAC_A--);    // Write to DAC the value of rampUp_valueOfDAC_A++.
                rampCounter--;
                portEXIT_CRITICAL(&colorTemperature::rampDownTimerMux);
            }
            
            
            
            if (valueOfDAC_B)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                
                portENTER_CRITICAL(&colorTemperature::rampDownTimerMux);
                LED.LTC2633.B.write(valueOfDAC_B--);    // Write to DAC the value of rampUp_valueOfDAC_B++.
                rampCounter--;
                portEXIT_CRITICAL(&colorTemperature::rampDownTimerMux);
            }
            
            //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            if (!rampCounter)
            {
                restartThisTask = 1;
                colorTemperature::isrRampValues.isRampingDown = 0;
            }
            
        }
        //goto topOfRampUp;
        timerAlarmDisable(colorTemperature::rampDownTimer);
        timerStop(colorTemperature::rampDownTimer);
    }
    
    // Insurance preventing port returning never-ever:
    vTaskDelete( NULL );
}

/*
class rampTask : public CppTaskPinnedToCore
{
public:
    rampTask(unsigned portSHORT _usStackDepth,
             UBaseType_t _priority,
             const char* _name, // base class arguments
             uint8_t _port,
             uint8_t _pin,
             uint32_t _ticks ) : CppTaskPinnedToCore{_usStackDepth, _priority, _name},
                                 port{_port},
                                 pin{_pin},
                                 ticks{_ticks}
    {
        //Chip_GPIO_SetPinDIROutput( LPC_GPIO, this->port, this->pin );
        
        // rampTask Init here:
    }

    virtual void externalTask() override
    {
        for (;;)
        {
            // Waiting for the very first notification to start the loop:
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            cct.calculateRampUpValues();
            cct.rampValues.isRampingUp = 1;
            restartThisTask = 0;
            
            //?? Store the handle of the calling port.
            //??xRampUpTaskNotify_handle = xTaskGetCurrentTaskHandle();
            
            // Set the used variables to there original state:        
            valueOfDAC_A        = cct.rampValues.rampStartValueA;
            valueOfDAC_B        = cct.rampValues.rampStartValueB;
            targetValueA        = cct.rampValues.A_target;
            targetValueB        = cct.rampValues.B_target;
            maxValueRampCounter = cct.rampValues.maxValueForRampCounter;
            rampCounter = 0;
            
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            while (!restartThisTask)
            {
                rampCounter = 0;
                
                while (valueOfDAC_A <= rampUp_targetValueA)
                {
                    // Write to DAC the value of rampUp_valueOfDAC_A++.
                    
                    cct.LTC2633.A.write(valueOfDAC_A++); 
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                }
                
                rampCounter += valueOfDAC_A;
                
                while (valueOfDAC_B <= targetValueB)
                {
                    
                    rampCounter++;
                    // Write to DAC the value of rampUp_valueOfDAC_B++.
                    cct.LTC2633.B.write(valueOfDAC_B++);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                }
                
                rampCounter += valueOfDAC_B;
                
                restartThisTask = 1;
                
            }
            
            //goto topOfRampUp;
        }
    
        // Insurance preventing port returning never-ever:
        vTaskDelete( NULL );
    }

//private:
    bool restartThisTask;
    volatile uint16_t valueOfDAC_A;
    volatile uint16_t valueOfDAC_B;
    volatile uint16_t targetValueA;
    volatile uint16_t targetValueB;
    volatile uint16_t maxValueRampCounter;
    volatile uint16_t rampCounter;
    
};
*/