//#include <Event.h>
//#include <Timer.h>

//#include <EveryTimer.h>
//#include <OneShotTimer.h>
//#include "Timer.h"

#include <LiquidCrystal.h>

#define RELAY 12
#define START_UP_DELAY 3000


const float lUpCurrent[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
const int opCurrentAdcValue[] = { 3, 7, 10, 14, 17, 20, 24, 27, 31, 34, 68, 102, 136, 170, 341, 512, 683, 853, 1024 };


typedef struct
{
  unsigned int voltInput;
  unsigned int voltOutput;
  unsigned int currentInput;
  unsigned int currentOutput;
}adc_t;

typedef struct
{
    unsigned int peakMax;
    unsigned int lowMin;
    unsigned int meanValue;    
    float realWorldValue;    
}data_t;

void Init_LCD( void );
void timerISR( void );

adc_t adc;
data_t outputVolt, inputVolt, outputCurrent, inputCurrent;
unsigned int load;

unsigned char opLowValue = 0;
unsigned char opHighValue = 0;

bool oneSec_f = false;

const unsigned int ADC_OUTPUT_VOLT = A1;
const unsigned int ADC_INPUT_VOLT  = A0;
const unsigned int ADC_OUTPUT_CURRENT  = A3;
const unsigned int ADC_INPUT_CURRENT = A4;

const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
unsigned int looptime = 0;
void setup() 
{
  Init_LCD();
  Serial.begin(9600);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
  delay( 3000);
  delay(START_UP_DELAY*2);
  digitalWrite(RELAY, HIGH);
}

void loop() 
{

  for( looptime=0;looptime<=1000; looptime++ )
  {
    
//!< OUTPUTVOLTAGE READING
		adc.voltOutput = analogRead(ADC_OUTPUT_VOLT); 
		if( adc.voltOutput > outputVolt.peakMax )
		{
		outputVolt.peakMax = adc.voltOutput;
		}
		if( adc.voltOutput < outputVolt.lowMin )
		{
		  outputVolt.lowMin = adc.voltOutput;
		}

//!< INPUTVOLTAGE READING
		adc.voltInput = analogRead(ADC_INPUT_VOLT); 
		if( adc.voltInput > inputVolt.peakMax )
		{
			inputVolt.peakMax = adc.voltInput;
		}
		if( adc.voltInput < inputVolt.lowMin )
		{
			inputVolt.lowMin = adc.voltInput;
		}
    
//===============!< OUTPUTCURRENT READING
		adc.currentOutput = analogRead(ADC_OUTPUT_CURRENT); 
		if( adc.currentOutput > outputCurrent.peakMax )
		{
		outputCurrent.peakMax = adc.currentOutput;
		}
		if( adc.currentOutput < outputCurrent.lowMin )
		{
		  outputCurrent.lowMin = adc.currentOutput;
		}

//!< INPUTCURRENT READING  --- currently no need of input current....
/*		adc.currentInput = analogRead(ADC_INPUT_CURRENT); 
		if( adc.currentInput > inputCurrent.peakMax )
		{
			inputCurrent.peakMax = adc.currentInput;
		}
		if( adc.currentInput < inputCurrent.lowMin )
		{
			inputCurrent.lowMin = adc.currentInput;
		}*/
    
   
  }
         
	//oneSec_f = false;
	outputVolt.meanValue = outputVolt.peakMax - outputVolt.lowMin;
	outputVolt.realWorldValue = outputVolt.meanValue * 0.449657;			   	//  n=(311/1023)*m;0.30400782
	outputVolt.realWorldValue = (outputVolt.realWorldValue/1.390);   
	
	outputCurrent.meanValue = outputCurrent.peakMax - outputCurrent.lowMin;
   for ( char i = 0; i<19; i++)
    {
      if( opCurrentAdcValue[i] > inputVolt.meanValue )
      {
        opLowValue = i-1;
        break;
      }
    }
    for ( char i = 0; i<19; i++)
    {
      if( opCurrentAdcValue[i] > inputVolt.meanValue )
      {
        opHighValue = i;
        break;
      }
    }

    outputCurrent.realWorldValue = lUpCurrent[opLowValue];
	//outputCurrent.realWorldValue = outputCurrent.meanValue * 0.449657;			//  n=(311/1023)*m;0.30400782
	//outputCurrent.realWorldValue = (outputCurrent.realWorldValue/1.390);  
	
	inputVolt.meanValue = inputVolt.peakMax - inputVolt.lowMin;
	inputVolt.realWorldValue = inputVolt.meanValue * 0.449657;					//  n=(311/1023)*m;0.30400782
	inputVolt.realWorldValue = (inputVolt.realWorldValue/1.390);  

  load = outputVolt.realWorldValue/outputCurrent.realWorldValue;
/*	inputCurrent.meanValue = inputCurrent.peakMax - inputCurrent.lowMin;
	inputCurrent.realWorldValue = inputCurrent.meanValue * 0.449657;			//  n=(311/1023)*m;0.30400782
	inputCurrent.realWorldValue = (inputCurrent.realWorldValue/1.390);        
*/
	updateLcd();

    outputVolt.peakMax = 0;
    outputVolt.lowMin = 0x3FF;
    inputVolt.peakMax = 0;
    inputVolt.lowMin = 0x3FF;
    outputCurrent.peakMax = 0;
    outputCurrent.lowMin = 0x3FF;
    inputCurrent.peakMax = 0;
    inputCurrent.lowMin = 0x3FF;
    load = 0;
}

void Init_LCD( void )
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("--- WELCOME ---");
  lcd.setCursor(0,1);
  lcd.print("**** AiMAC **** ");
  delay(2000);
  
  lcd.clear();
  lcd.print("Initializing...");
  lcd.setCursor(0,1);
  lcd.print("Safe 1K5 ");

}

void updateLcd( void )
{
    lcd.clear();
    lcd.print( "IPV:" );
    lcd.print( (int)inputVolt.realWorldValue, DEC );
    lcd.print( " OPV:" );
    lcd.print( (int)outputVolt.realWorldValue, DEC );
    
    lcd.setCursor(0,1);
    lcd.print( "OPC:" );
    lcd.print( (int)outputCurrent.realWorldValue, DEC );
    lcd.print( " LOD:" );
    lcd.print( (int)load, DEC );
    lcd.setCursor(15,1);
    lcd.print( "%" );
   
}
void timerISR( void )
{
  oneSec_f = true;
}
void takeReading()
{
  oneSec_f = true;
}
