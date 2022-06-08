#ifndef _CORRELATED_COLOR_TEMPERATURE_H_
#define _CORRELATED_COLOR_TEMPERATURE_H_

#include <Arduino.h>
#include <LTC2633Library.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

#include "driver/timer.h"
#include "esp_timer.h"

#include "esp32-hal-timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "soc/timer_group_struct.h"
#include "soc/dport_reg.h"
#include "esp_attr.h"
#include "esp_intr.h"


struct DAC_coefficients
{
    uint16_t A = 0;
    uint16_t B = 0;
    uint16_t raw_A = 0;
    uint16_t raw_B = 0;
    uint16_t Intensity = 0; 
    uint8_t valueMatchFlag = 0;
};

struct DAC_artificial_data
{
    const double IntensityPerCount = 0.00024421;
    double maxValue_double = 0.0;
    uint16_t maxValue = 0;
    uint16_t bothValuesCombined = 0;
};

struct DAC_writting_data
{
    uint16_t artificialMaxValue = 0;
    uint16_t written_A_plus_B = 0;
    
    uint16_t A = 0;
    uint16_t B = 0;
};

struct ISR_DATA
{
    uint16_t usedIntensitySetting = 0;
    
    time_t sunriseTriggerTime = 0;  // Fetch this number from esp server
    time_t sunsetTriggerTime = 0;   // Fetch this number from esp server
    
    time_t sunriseLength = 0;  // Fetch this number from esp server
    time_t sunsetLength = 0;   // Fetch this number from esp server
    
    uint32_t rampUpUpdateInterval_uS = 0;
    uint16_t rampUpStartValueA = 0;
    uint16_t rampUpStartValueB = 0;
    uint16_t rampUpTargetValueA = 0;
    uint16_t rampUpTargetValueB = 0;
    uint16_t maxValueForRampUpCounter = 0;
    bool rampUpDataCalculated = 0;
    bool rampUpIsEnabled = 0;
    volatile bool isRampingUp = 0;
    
    uint32_t rampDownUpdateInterval_uS = 0;
    uint16_t rampDownStartValueA = 0;
    uint16_t rampDownStartValueB = 0;
    uint16_t rampDownTargetValueA = 0;
    uint16_t rampDownTargetValueB = 0;
    uint16_t maxValueForRampDownCounter = 0;
    bool rampDownDataCalculated = 0;
    bool rampDownIsEnabled = 0;
    volatile bool isRampingDown = 0;
};

// Timer Interrupt related functions:
void IRAM_ATTR rampUp_ISR();
void rampUpTask(void *pvParameters);
    
void IRAM_ATTR rampDown_ISR();
void rampDownTask(void *pvParameters);



class colorTemperature /*correlatedColorTemperature*/
{
    public:
        
        const uint16_t _CCT[24]; // Array with _CCT values to be compared with input for match, from myIndex 0 - 24 = 5000k - 2700k.
		// enum {CCT5000, degree4900, DEGREE4800, deg4700, DEG4600, D4500, F4400, CCT4300, CCT4200, CCT4100, CCT4000, CCT3900, CCT3800, CCT3700, CCT3600, CCT3500, CCT3400, CCT3300, CCT3200, CCT3100, CCT3000, CCT2900, CCT2800, CCT2700, CCTmaxIndex};
        const uint16_t _adjustmentCoefficients_DAC_A[24];   // DAC_A = 2700 °K - coefficients corresponding to _CCT[].
        const uint16_t _adjustmentCoefficients_DAC_B[24];   // DAC_B = 5000 °K - coefficients corresponding to _CCT[].
		//const double _percent_adjustmentCoefficients_DAC_A[24];
        //const double _percent_adjustmentCoefficients_DAC_B[24];
        
        static uint8_t triggerSunriseAlarm_id;
        static uint8_t triggerSunsetAlarm_id;
        
        LTC2633Library LTC2633;
        static DAC_coefficients coefficients;
        static DAC_coefficients errorCoefficients;
        static DAC_artificial_data artificial;
        static DAC_writting_data writeValues;
        static ISR_DATA isrRampValues;

        uint8_t _SSR_B_2700k_pin;
        uint8_t _SSR_A_5000k_pin;
        uint8_t _arrayLength;
		
        static TaskHandle_t rampUpTaskHandle;
        static hw_timer_t * rampUpTimer;
        static portMUX_TYPE rampUpTimerMux;
        
        static TaskHandle_t rampDownTaskHandle;
        static hw_timer_t * rampDownTimer;
        static portMUX_TYPE rampDownTimerMux;
		
		uint8_t _timerUsedFor_RampUp;
        uint8_t _timerUsedFor_RampDown;
		uint8_t _onOrOff;
		uint8_t _needToCalculate;
        
    //protected:
        
        
        colorTemperature(uint8_t ssr2700k_pin, uint8_t ssr5000k_pin, LTC2633_I2C_ADDRESS I2C_address, TwoWire *theWire);
        
        void begin(uint8_t timerForRampUp, uint8_t rimerForRampDown);
        
        DAC_coefficients setColorTemperature(uint16_t value);
        DAC_coefficients writeColorTemperature(uint16_t value);
        
        DAC_coefficients setIntensity(uint16_t intensity);
        DAC_coefficients writeIntensity(uint16_t intensity);
		
		void setPowerState(uint8_t onOrOff);
		void turnOn();
		void turnOff();
        
        DAC_coefficients writeToLed(); // write to the DAC's using the values set with setColorTemperature() & setIntensity().
		static void calculateInternalValues();
        void writeToDAC();
        
        int8_t writeRawA(uint16_t valueA);
        int8_t writeRawB(uint16_t valueB);
        int8_t writeRaw(uint16_t valueA, uint16_t valueB);
        
        uint16_t getValueA();
        uint16_t getValueB();
        
		void enableLED(char led);
		void disableLED(char led);
		
        void enableLEDs();
        void enableLED2700k();
        void enableLED5000k();
        void disableLEDs();
        void disableLED2700k();
        void disableLED5000k();
        
        // Timer related functions:
        static uint32_t calculateRampUpValues();
        static uint32_t calculateRampDownValues();
        
        static void triggerSunriseFunction();
        static void triggerSunsetFunction();
        
        void setSunriseAlarmTime(time_t turnOnTime);
        void setSunriseDuration(time_t sunriseLength_seconds);
        
        void setSunsetAlarmTime(time_t turnOffTime);
        void setSunsetDuration(time_t sunsetLength_seconds);
        
        void enableSunriseAlarm();
		void enableSunsetAlarm();
		
        void disableSunriseAlarm();
		void disableSunsetAlarm();
		
        bool isSunriseAlarmEnabled();
		bool isSunsetAlarmEnabled();
};

extern colorTemperature LED;


#endif  //  _COLOR_TEMPERATURE_H_


// const double percentPerCount = 0.00024421;
//
// 1 / 4095 = 0.00024420024420024420024420024420024
//            0.0002442002442002442
//          = 0.0002443
// 0.8 / 0.0002443  = 3274.6623004502660663119115841179
// 0.8 / 0.00024421 = 3275.8691290282953196019819008231
// 0.8 * 4095       = 3276
// (1800 / 3276) * 1000               = 549.45054945054945054945054945055
// (1800 / (0.8 / 0.00024421)) * 1000 = 549.47249999999999999999999999999
// double temp = (rampUpDuration / (rampUpTargetIntensity / percentPerCount)) * 1000;
// rampUpdateInterval = (unsigned long)temp;
// 
// 80% over 30 minutes(1800 seconds) = 0.00044444444444444444444444444444444 0.044444444444444444444444444444444 percentPerSecond
// 0.8 / 1800 = 4.4444444444444444444444444444444e-4
// 50% over 30 minutes(1800 seconds) = 0.00027777777777777777777777777777778 0.027777777777777777777777777777778 percentPerSecond
// 
// calculateRampUpValues_theory()
// {
            // Divide the total rampUpDuration with the number of counts the sum of which is the rampUpTargetIntensity,
            // multplied by 1000 to represent mS instead if Seconds:
            //      In other words, find out how much time in between each increase by 1 written to the DAC is needed
            //      in order to go from 0 to the target intensity during the total ram-up time.
            //      Example rampUpDuration = 1800 seconds, targetIntensity = 80%(e.i. 0.8), percentPerCount = 0.00024421, = 549 mS
            //      result = (1800 / (0.8 / 0.00024421)) * 1000 = 549.4725 = 549;
            //      double temp = (rampUpDuration / (rampUpTargetIntensity / percentPerCount)) * 1000;
            //      rampUpdateInterval = (unsigned long)temp;
            //      Not included: Adding 0.5 in order to round decimals < .5 downwards and > .5 upwards.
            // 
            // RE-DESIGN USING FREERTOS, using µS instead of mS: 1000 * 1 mS = 1 S |/| 1000000 * 1 uS = 1 S. 549 mS = 549000 µS
            // 
            // (1800 / (0.8 / 0.00024421)) * 1000000 = 549472.5 = 549;
            // 
            // (seconds / (fullscale * intensityPercent)) * 1000000 = 549472.5 = 549;
            // (1800 / (4096 * 0.8)) * 1000000 = 549472.5 = 549;
            // 
            // double temp = ((rampUpDuration / (coefficients.maxValueScaledToIntensity_uint)) * 1000000) + 0.5;
// };
// 
// ¤> 
// ¤> DAC_A = 5000 K
// ¤> DAC_B = 2700 K
// ¤> double CCT_5000k_DAC_A = 0;
// ¤> double CCT_4900k_DAC_A = 0.0414;
// ¤> double CCT_4800k_DAC_A = 0.0710;
// ¤> double CCT_4700k_DAC_A = 0.1041;
// ¤> double CCT_4600k_DAC_A = 0.1385;
// ¤> double CCT_4500k_DAC_A = 0.1704;
// ¤> double CCT_4400k_DAC_A = 0.2012;
// ¤> double CCT_4300k_DAC_A = 0.2343;
// ¤> double CCT_4200k_DAC_A = 0.2651;
// ¤> double CCT_4100k_DAC_A = 0.2982;
// ¤> double CCT_4000k_DAC_A = 0.3314;
// ¤> double CCT_3900k_DAC_A = 0.3669;
// ¤> double CCT_3800k_DAC_A = 0.4024;
// ¤> double CCT_3700k_DAC_A = 0.4379;
// ¤> double CCT_3600k_DAC_A = 0.4793;
// ¤> double CCT_3500k_DAC_A = 0.5231;
// ¤> double CCT_3400k_DAC_A = 0.5704;
// ¤> double CCT_3300k_DAC_A = 0.6225;
// ¤> double CCT_3200k_DAC_A = 0.6817;
// ¤> double CCT_3100k_DAC_A = 0.7408;
// ¤> double CCT_3000k_DAC_A = 0.8166;
// ¤> double CCT_2900k_DAC_A = 0.8852;
// ¤> double CCT_2800k_DAC_A = 0.9538;
// ¤> double CCT_2700k_DAC_A = 1.0000;
// ¤> 
// ¤> double CCT_5000k_DAC_B = 1.0000;
// ¤> double CCT_4900k_DAC_B = 0.9586;
// ¤> double CCT_4800k_DAC_B = 0.9290;
// ¤> double CCT_4700k_DAC_B = 0.8959;
// ¤> double CCT_4600k_DAC_B = 0.8615;
// ¤> double CCT_4500k_DAC_B = 0.8296;
// ¤> double CCT_4400k_DAC_B = 0.7988;
// ¤> double CCT_4300k_DAC_B = 0.7657;
// ¤> double CCT_4200k_DAC_B = 0.7349;
// ¤> double CCT_4100k_DAC_B = 0.7018;
// ¤> double CCT_4000k_DAC_B = 0.6686;
// ¤> double CCT_3900k_DAC_B = 0.6331;
// ¤> double CCT_3800k_DAC_B = 0.5976;
// ¤> double CCT_3700k_DAC_B = 0.5621;
// ¤> double CCT_3600k_DAC_B = 0.5207;
// ¤> double CCT_3500k_DAC_B = 0.4769;
// ¤> double CCT_3400k_DAC_B = 0.4296;
// ¤> double CCT_3300k_DAC_B = 0.3775;
// ¤> double CCT_3200k_DAC_B = 0.3183;
// ¤> double CCT_3100k_DAC_B = 0.2592;
// ¤> double CCT_3000k_DAC_B = 0.1834;
// ¤> double CCT_2900k_DAC_B = 0.1148;
// ¤> double CCT_2800k_DAC_B = 0.0462;
// ¤> double CCT_2700k_DAC_B = 0;
// ¤> 
// ¤> 
// ¤> TO-DO LIST:
// ¤> Skriv koden som kan kontroller lampans AV/PÅ läge enligt dataStruct information.
// ¤> Gradvis tända/släcka lampan, enligt timeFrame(ifrån helt AV till helt PÅ), intensityLevel(nivån som är "helt PÅ"), DACStepCount = intensityLevel / timeFrame.
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// ¤> 
// 
// 1.0 / 4096 = 0.000244140625
// 100 / 4096 = 0.0244140625
// 
// 
// 1.0 / 4095 = 0.00024420024420024420024420024420024
// 100 / 4095 = 0.02442002442002442002442002442002
//              
/*
#include "PinDefinitionsAndMore.h"

#include <IRremote.hpp>

        void setupIrSensor()
        {
            // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
            IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
        
            Serial.print(F("Ready to receive IR signals of protocols: "));
            printActiveIRProtocols(&Serial);
            Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
        }





        void readIrSensor()
        {
            if (IrReceiver.decode())
            {
                
                IrReceiver.printIRResultShort(&Serial);                     // Print a short summary of received data
                if (IrReceiver.decodedIRData.protocol == UNKNOWN)
                    IrReceiver.printIRResultRawFormatted(&Serial, true);    // We have an unknown protocol here, print more info
                
                Serial.println();
                
                IrReceiver.resume();                                        // Enable receiving of the next value
                 
                //check the received data and perform actions according to the received command
                if (IrReceiver.decodedIRData.command == 0x10)
                {
                    // do something
                }
                else if (IrReceiver.decodedIRData.command == 0x11)
                {
                    // do something else
                }
            }
        }
*/

/*
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    
    xSemaphoreGive();
    
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
    timer = timerBegin(0, 80, true);                // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp.
    timerAttachInterrupt(timer, &onTimer, true);    // edge (not level) triggered.
    timerAlarmWrite(timer, TIMER_TIME, true);       // 1000000 * 1 us = 1 s, autoreload true.
    timerAlarmEnable(timer);                        // enable
}

void task()
{
    timerAlarmDisable(timer);                   // Disable the ISR.
    timerAlarmWrite(timer, TIMER_TIME, true);   // ISR trigger interval, autoreload true.
    timerAlarmEnable(timer);                    // Re-enable the ISR again.
}
*/
