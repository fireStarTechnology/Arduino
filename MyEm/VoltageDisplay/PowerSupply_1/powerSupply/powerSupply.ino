//#include <Event.h>
//#include <Timer.h>

//#include <EveryTimer.h>
//#include <OneShotTimer.h>
//#include "Timer.h"

#include <LiquidCrystal.h>

#define RELAY 12
#define START_UP_DELAY 3000

#define TOTAL_POWER 1464

float voltage[] = { 4,6,8,10,12,14,16 };
float current[] = { 0.6, 2.6, 4.55, 6.57, 8.6, 10.6, 12.6 };

uint16_t outCurrentAdcLUT[] = {0, 3, 7, 10, 14, 17, 20, 24, 27, 31, 34, 68, 102, 136, 170, 341, 512, 683, 853, 1024 };
float outCurrentCalLUT[] = { 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 10, 15, 20, 25, 30 };
uint8_t sizeAdcLUT = sizeof(outCurrentAdcLUT)/ sizeof(uint16_t);
uint8_t sizeCalCurLUT = sizeof(outCurrentCalLUT)/ sizeof(float);

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

/********************************************************************************
Name        : setup
para 1      : N/A
return      : N/A
Discription : System Initialization code.
**********************************************************************************/
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
/********************************************************************************
Name        : loop
para 1      : N/A
return      : N/A
Discription : This is main while loop run continueslly 
**********************************************************************************/
void loop() 
{
  float power = 0;


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
    
        //==!< OUTPUTCURRENT READING
        adc.currentOutput = analogRead(ADC_OUTPUT_CURRENT); 
        if( adc.currentOutput > outputCurrent.peakMax )
        {
        outputCurrent.peakMax = adc.currentOutput;
        }
        if( adc.currentOutput < outputCurrent.lowMin )
        {
          outputCurrent.lowMin = adc.currentOutput;
        }
  }
    //XXXXXXXXXXXXXXXXXXXXX  END OF 1000 SAMPLES  XXXXXXXXXXXXXXXXXXXXXX
  
    //oneSec_f = false;
    outputVolt.meanValue = outputVolt.peakMax - outputVolt.lowMin;
    outputVolt.realWorldValue = outputVolt.meanValue * 0.449657;                   //  n=(311/1023)*m;0.30400782
    outputVolt.realWorldValue = (outputVolt.realWorldValue/1.390);   
            
    inputVolt.meanValue = inputVolt.peakMax - inputVolt.lowMin;
    inputVolt.realWorldValue = inputVolt.meanValue * 0.449657;                    //  n=(311/1023)*m;0.30400782
    inputVolt.realWorldValue = (inputVolt.realWorldValue/1.390);  

    outputCurrent.meanValue = outputCurrent.peakMax - outputCurrent.lowMin;    
    outputCurrent.realWorldValue = findyValue(outputCurrent.meanValue);
    power = outputVolt.realWorldValue*outputCurrent.realWorldValue;

    
    load = (power/TOTAL_POWER)*100;

  
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

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX END OF MAIN LOOP  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/********************************************************************************
Name        : updateLcd
para 1      : N/A
return      : N/A
Discription : The LCD display will update with the real world value.
**********************************************************************************/
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
  lcd.print("*** Safe 1K5 ***");

}
/********************************************************************************
Name        : updateLcd
para 1      : N/A
return      : N/A
Discription : The LCD display will update with the real world value.
**********************************************************************************/
void updateLcd( void )
{
    lcd.clear();
    lcd.print( "IPV:" );
    lcd.print( (int)inputVolt.realWorldValue, DEC );
    lcd.print( " OPV:" );
    lcd.print( (int)outputVolt.realWorldValue, DEC );
    
    lcd.setCursor(0,1);
    lcd.print( "OPC:" );
    lcd.print( outputCurrent.realWorldValue, DEC );
    lcd.setCursor(8,1);
    lcd.print( " LOD:" );
    lcd.print( (int)load, DEC );
    lcd.setCursor(15,1);
    lcd.print( "%" );   
}
/********************************************************************************
Name        : updateLcd
para 1      : N/A
return      : N/A
Discription : The LCD display will update with the real world value.
**********************************************************************************/
void timerISR( void )
{
  oneSec_f = true;
}
/********************************************************************************
Name        : updateLcd
para 1      : N/A
return      : N/A
Discription : The LCD display will update with the real world value.
**********************************************************************************/
void takeReading()
{
  oneSec_f = true;
}
/********************************************************************************
Name        : updateLcd
para 1      : N/A
return      : N/A
Discription : The LCD display will update with the real world value.
**********************************************************************************/
float findyValue( int x3 )
{
    uint8_t x1p = 0, x2p = 0;
    uint8_t loop = 0;
    float m = 0;
    float y3 = 0;

    x1p = 0;
    x2p = sizeAdcLUT - 1;
    for(loop = 0;loop<sizeAdcLUT-1;loop++)
    {
        if( x3 > outCurrentAdcLUT[x1p] )
            x1p = loop;
        if( x3 < voltage[x2p] )
            x2p = outCurrentAdcLUT-(loop+1);
    }
    if(x1p != 0)
    x1p--;
    x2p++;

    //outputCurrent.realWorldValue = x1p;
   // load = x2p;

    m = ( outCurrentCalLUT[x2p] - outCurrentCalLUT[x1p] )/ ( outCurrentAdcLUT[x2p] - outCurrentAdcLUT[x1p] );
    y3 = ((m*(x3-outCurrentAdcLUT[x1p]))+ outCurrentCalLUT[x1p]);

    return( y3 );
}
