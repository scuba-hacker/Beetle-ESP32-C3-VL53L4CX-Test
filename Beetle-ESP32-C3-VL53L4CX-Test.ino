// repurposed from the examples for VL53L4CX, tweeked for the Beetle ESP32-C3 from DFRobot
// Test the Time of Flight sensor

/**
 ******************************************************************************
 * @file    VL53L4CX_Sat_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    16 March 2022
 * @brief   Arduino test application for the STMicrolectronics VL53L4CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use this sketch you need to connect the VL53L4CD satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (GND) of the VL53L4CD satellite connected to GND of the Nucleo board
 * pin 2 (VDD) of the VL53L4CD satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (SCL) of the VL53L4CD satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 4 (SDA) of the VL53L4CD satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 5 (GPIO1) of the VL53L4CD satellite connected to pin A2 of the Nucleo board
 * pin 6 (XSHUT) of the VL53L4CD satellite connected to pin A1 of the Nucleo board
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);

const int beetleLed = 10;

bool timeOfFlightSensorAvailable = false;

void setup()
{
  pinMode(beetleLed, OUTPUT);

  // LED flash - we're alive!
  Serial.begin(115200);
  int warmUp=10;
  
  while (warmUp--)
  {
    digitalWrite(beetleLed,HIGH);
    delay(250);
    digitalWrite(beetleLed,LOW);
    delay(250);
   
    Serial.println("Warming up...");
  }
  Serial.println("\nHere we go...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  VL53L4CX_Error initError = sensor_vl53l4cx_sat.InitSensor(0x12);

  char TOFErrorBuffer[] = "(err)";

  if (initError == VL53L4CX_ERROR_NONE)
  {
    Serial.println("Adafruit VL53L4CX Time Of Flight Sensor Initialise ok");

    VL53L4CX_Error measureError = sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();  

    if (measureError == VL53L4CX_ERROR_NONE)
    {
      Serial.println("Adafruit VL53L4CX Time Of Flight Sensor Started Measurement ok");
      timeOfFlightSensorAvailable = true;
    }
    else
    {
     const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(measureError);
     Serial.printf("Error: Adafruit VL53L4CX Time Of Flight Sensor startup measurement error: %i %s\n",measureError, TOFErrorBuffer);
     timeOfFlightSensorAvailable = false;
    }
  }
  else
  {
    const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(initError);
    Serial.printf("Error: Adafruit VL53L4CX Time Of     Flight Sensor initialisation error: %i %s\n",initError, TOFErrorBuffer); 
    timeOfFlightSensorAvailable = false;
  }
}

void loop()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(beetleLed, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    SerialPort.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        SerialPort.print("\r\n                               ");
      }
      SerialPort.print("status=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
      SerialPort.print(", D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
      SerialPort.print(", Signal=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps, Ambient=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  digitalWrite(beetleLed, LOW);
}

char* getTimeOfFlightSensorErrorString(VL53L4CX_Error e)
{
  switch (e)
  {
    case VL53L4CX_ERROR_NONE: return "VL53L4CX_ERROR_NONE";
    case VL53L4CX_ERROR_CALIBRATION_WARNING: return "VL53L4CX_ERROR_CALIBRATION_WARNING";
    case VL53L4CX_ERROR_MIN_CLIPPED: return "VL53L4CX_ERROR_MIN_CLIPPED";
    case VL53L4CX_ERROR_UNDEFINED: return "VL53L4CX_ERROR_UNDEFINED";
    case VL53L4CX_ERROR_INVALID_PARAMS: return "VL53L4CX_ERROR_INVALID_PARAMS";
    case VL53L4CX_ERROR_NOT_SUPPORTED: return "VL53L4CX_ERROR_NOT_SUPPORTED";
    case VL53L4CX_ERROR_RANGE_ERROR: return "VL53L4CX_ERROR_RANGE_ERROR";
    case VL53L4CX_ERROR_TIME_OUT: return "VL53L4CX_ERROR_TIME_OUT";
    case VL53L4CX_ERROR_MODE_NOT_SUPPORTED: return "VL53L4CX_ERROR_MODE_NOT_SUPPORTED";
    case VL53L4CX_ERROR_BUFFER_TOO_SMALL: return "VL53L4CX_ERROR_BUFFER_TOO_SMALL";
    case VL53L4CX_ERROR_COMMS_BUFFER_TOO_SMALL: return "VL53L4CX_ERROR_COMMS_BUFFER_TOO_SMALL";
    case VL53L4CX_ERROR_GPIO_NOT_EXISTING: return "VL53L4CX_ERROR_GPIO_NOT_EXIS";
    case VL53L4CX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED: return "VL53L4CX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED";
    case VL53L4CX_ERROR_CONTROL_INTERFACE: return "VL53L4CX_ERROR_CONTROL_INTERFACE";
    case VL53L4CX_ERROR_INVALID_COMMAND: return "VL53L4CX_ERROR_INVALID_COMMAND";
    case VL53L4CX_ERROR_DIVISION_BY_ZERO : return "VL53L4CX_ERROR_DIVISION_BY_ZERO";
    case VL53L4CX_ERROR_REF_SPAD_INIT: return "VL53L4CX_ERROR_REF_SPAD_INIT";
    case VL53L4CX_ERROR_GPH_SYNC_CHECK_FAIL: return "VL53L4CX_ERROR_GPH_SYNC_CHECK_FAIL";
    case VL53L4CX_ERROR_STREAM_COUNT_CHECK_FAIL: return "VL53L4CX_ERROR_STREAM_COUNT_CHECK_FAIL";
    case VL53L4CX_ERROR_GPH_ID_CHECK_FAIL: return "VL53L4CX_ERROR_GPH_ID_CHECK_FAIL";
    case VL53L4CX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL: return "VL53L4CX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL";
    case VL53L4CX_ERROR_ZONE_GPH_ID_CHECK_FAIL: return "VL53L4CX_ERROR_ZONE_GPH_ID_CHECK_FAIL";
    case VL53L4CX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_F";
    case VL53L4CX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL: return "VL53L4CX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_";
    case VL53L4CX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL";
    case VL53L4CX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL: return "VL53L4CX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL";
    case VL53L4CX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL";
    case VL53L4CX_ERROR_TUNING_PARM_KEY_MISMATCH: return "VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS: return "VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH: return "VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW: return "VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW";
    case VL53L4CX_WARNING_OFFSET_CAL_MISSING_SAMPLES: return "VL53L4CX_WARNING_OFFSET_CAL_MISSING_SAMPLES";
    case VL53L4CX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH: return "VL53L4CX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH";
    case VL53L4CX_WARNING_OFFSET_CAL_RATE_TOO_HIGH: return "VL53L4CX_WARNING_OFFSET_CAL_RATE_TOO_HI";
    case VL53L4CX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW: return "VL53L4CX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW";
    case VL53L4CX_WARNING_ZONE_CAL_MISSING_SAMPLES: return "VL53L4CX_WARNING_ZONE_CAL_MISSING_SAMPLES";
    case VL53L4CX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH: return "VL53L4CX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH";
    case VL53L4CX_WARNING_ZONE_CAL_RATE_TOO_HIGH: return "VL53L4CX_WARNING_ZONE_CAL_RATE_TOO_HIGH";
    case VL53L4CX_WARNING_XTALK_MISSING_SAMPLES: return "VL53L4CX_WARNING_XTALK_MISSING_SAMPLES";
    case VL53L4CX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT: return "VL53L4CX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT";
    case VL53L4CX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT: return "VL53L4CX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT";
    case VL53L4CX_ERROR_NOT_IMPLEMENTED: return "VL53L4CX_ERROR_NOT_IMPLEMENTED";
    case VL53L4CX_ERROR_PLATFORM_SPECIFIC_START: return "VL53L4CX_ERROR_PLATFORM_SPECIFIC_START";
    default: return "Undefined enum";
  }
}
