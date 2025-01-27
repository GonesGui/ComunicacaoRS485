/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017 
   -------------------------------------------------------------------------------------
*/

#ifndef HX711_ADC_h
#define HX711_ADC_h

#include <Arduino.h>

/** ----------------- Defines da Lib HX711_ADC --------------- */

// number of samples in moving average dataset, value must be 1, 2, 4, 8, 16, 32, 64 or 128.
#define HX711_SAMPLES 8  // default value: 16

// adds extra sample(s) to the dataset and ignore peak high/low sample, value must be 0 or 1.
#define IGN_HIGH_SAMPLE 1  // default value: 1
#define IGN_LOW_SAMPLE 1   // default value: 1

// microsecond delay after writing sck pin high or low. This delay could be required for faster mcu's.
// So far the only mcu reported to need this delay is the ESP32 (issue #35), both the Arduino Due and ESP8266
// seems to run fine without it. Change the value to '1' to enable the delay.
#define HX711_SCK_DELAY 1  // default value: 0

// if you have some other time consuming (>60μs) interrupt routines that trigger while the sck pin is high,
// this could unintentionally set the HX711 into "power down" mode if required you can change the value to '1'
// to disable interrupts when writing to the sck pin.
#define SCK_DISABLE_INTERRUPTS 0  // default value: 0
/** ----------------- FIM Defines da Lib HX711_ADC --------------- */

/*
Note: HX711_ADC configuration values has been moved to file config.h
*/

#define DATA_SET 	HX711_SAMPLES + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE // total samples in memory

#if (HX711_SAMPLES  != 1) & (HX711_SAMPLES  != 2) & (HX711_SAMPLES  != 4) & (HX711_SAMPLES  != 8) & (HX711_SAMPLES  != 16) & (HX711_SAMPLES  != 32) & (HX711_SAMPLES  != 64) & (HX711_SAMPLES  != 128)
	#error "number of HX711_SAMPLES not valid!"
#endif

#if (HX711_SAMPLES  == 1) & ((IGN_HIGH_SAMPLE  != 0) | (IGN_LOW_SAMPLE  != 0))
	#error "number of HX711_SAMPLES not valid!"
#endif

#if 		(HX711_SAMPLES == 1)
#define 	DIVB 0
#elif 		(HX711_SAMPLES == 2)
#define 	DIVB 1
#elif 		(HX711_SAMPLES == 4)
#define 	DIVB 2
#elif  		(HX711_SAMPLES == 8)
#define 	DIVB 3
#elif  		(HX711_SAMPLES == 16)
#define 	DIVB 4
#elif  		(HX711_SAMPLES == 32)
#define 	DIVB 5
#elif  		(HX711_SAMPLES == 64)
#define 	DIVB 6
#elif  		(HX711_SAMPLES == 128)
#define 	DIVB 7
#endif

#define SIGNAL_TIMEOUT	100

class HX711_ADC
{	
		
	public:
		bool smphrHX711 = false;
		HX711_ADC(uint8_t dout, uint8_t sck); 		//constructor
		void setGain(uint8_t gain = 128); 			//value must be 32, 64 or 128*
		void begin();								//set pinMode, HX711 gain and power up the HX711
		void begin(uint8_t gain);					//set pinMode, HX711 selected gain and power up the HX711
		void start(unsigned long t); 					//start HX711 and do tare 
		void start(unsigned long t, bool dotare);		//start HX711, do tare if selected
		int startMultiple(unsigned long t); 			//start and do tare, multiple HX711 simultaniously
		int startMultiple(unsigned long t, bool dotare);	//start and do tare if selected, multiple HX711 simultaniously
		void tare(); 								//zero the scale, wait for tare to finnish (blocking)
		void tareNoDelay(); 						//zero the scale, initiate the tare operation to run in the background (non-blocking)
		bool getTareStatus();						//returns 'true' if tareNoDelay() operation is complete
		void setCalFactor(float cal); 				//set new calibration factor, raw data is divided by this value to convert to readable data
		float getCalFactor(); 						//returns the current calibration factor
		float getData(); 							//returns data from the moving average dataset 

		int getReadIndex(); 						//for testing and debugging
		float getConversionTime(); 					//for testing and debugging
		float getSPS();								//for testing and debugging
		bool getTareTimeoutFlag();					//for testing and debugging
		void disableTareTimeout();					//for testing and debugging
		long getSettlingTime();						//for testing and debugging
		void powerDown(); 							//power down the HX711
		void powerUp(); 							//power up the HX711
		long getTareOffset();						//get the tare offset (raw data value output without the scale "calFactor")
		void setTareOffset(long newoffset);			//set new tare offset (raw data value input without the scale "calFactor")
		uint8_t update(); 							//if conversion is ready; read out 24 bit data and add to dataset
		bool dataWaitingAsync(); 					//checks if data is available to read (no conversion yet)
		bool updateAsync(); 						//read available data and add to dataset 
		void setSamplesInUse(int samples);			//overide number of samples in use
		int getSamplesInUse();						//returns current number of samples in use
		void resetSamplesIndex();					//resets index for dataset
		bool refreshDataSet();						//Fill the whole dataset up with new conversions, i.e. after a reset/restart (this function is blocking once started)
		bool getDataSetStatus();					//returns 'true' when the whole dataset has been filled up with conversions, i.e. after a reset/restart
		float getNewCalibration(float known_mass);	//returns and sets a new calibration value (calFactor) based on a known mass input
		bool getSignalTimeoutFlag();				//returns 'true' if it takes longer time then 'SIGNAL_TIMEOUT' for the dout pin to go low after a new conversion is started
		void setReverseOutput();					//reverse the output value

	protected:
		void conversion24bit(); 					//if conversion is ready: returns 24 bit data and starts the next conversion
		long smoothedData();						//returns the smoothed data value calculated from the dataset
		uint8_t sckPin; 							//HX711 pd_sck pin
		uint8_t doutPin; 							//HX711 dout pin
		uint8_t GAIN;								//HX711 GAIN
		float calFactor = 1.0;						//calibration factor as given in function setCalFactor(float cal)
		float calFactorRecip = 1.0;					//reciprocal calibration factor (1/calFactor), the HX711 raw data is multiplied by this value
		volatile long dataSampleSet[DATA_SET + 1];	// dataset, make voltile if interrupt is used 
		long tareOffset = 0;
		int readIndex = 0;
		unsigned long conversionStartTime = 0;
		unsigned long conversionTime = 0;
		uint8_t isFirst = 1;
		uint8_t tareTimes = 0;
		uint8_t divBit = DIVB;
		const uint8_t divBitCompiled = DIVB;
		bool doTare = 0;
		bool startStatus = 0;
		unsigned long startMultipleTimeStamp = 0;
		unsigned long startMultipleWaitTime = 0;
		uint8_t convRslt = 0;
		bool tareStatus = 0;
		unsigned int tareTimeOut = (HX711_SAMPLES + IGN_HIGH_SAMPLE + IGN_HIGH_SAMPLE) * 150; // tare timeout time in ms, no of samples * 150ms (10SPS + 50% margin)
		bool tareTimeoutFlag = 0;
		bool tareTimeoutDisable = 0;
		int samplesInUse = HX711_SAMPLES;
		long lastSmoothedData = 0;
		bool dataOutOfRange = 0;
		unsigned long lastDoutLowTime = 0;
		bool signalTimeoutFlag = 0;
		bool reverseVal = 0;
		bool dataWaiting = 0;
};	

#endif
   