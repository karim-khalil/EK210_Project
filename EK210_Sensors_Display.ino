           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          // Sources: Example testing sketch for various DHT humidity/temperature sensors, Written by ladyada, public domain  //
         // CO2 Sensor code Author:  Tiequan Shao: tiequan.shao@sandboxelectronics.com                                       //
        //                          Peng Wei:     peng.wei@sandboxelectronics.com                                           //
       //Thermistor: http://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/                          //
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/************************Hardware Related Macros************************************/
#define         MG_PIN                       (A0)     //define which analog input channel you are going to use CO2 Sensor
#define         BOOL_PIN                     (2)      //digital pinout CO2 Sensor
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define DHTPIN 2    //  Digital pin connected to the DHT sensor
//#define SERIESRESISTOR 10000    
//#define THERMISTORPIN A1

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321                                                     //normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.35) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2



/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};


        //////////////////////////////////Temperature//////////////////////////////////
            int ThermistorPin = 0;                                                  //
            int Vo;                                                                //
            float R1 = 10000;                                                     //
            float logR2, R2, T, Tc, Tf;                                          //
            float c1=1.009249522e-03, c2=2.378405444e-04, c3 = 2.019202697e-07; //
  ///////////////////////////////////////////////////////////////////////////////

  //Relay
  int in1 = 8;

#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 



// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27,20,4);

void setup() 
{
  Serial.begin(9600);

  //CO2
  pinMode(BOOL_PIN, INPUT);                        //set pin to input
  digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors

 
  dht.begin();
  
  lcd.init();       //initialize the lcd
  lcd.backlight();  //open the backlight 
  lcd.setCursor(1,0);
}  

void loop() 
{
  
  // Wait a few seconds between measurements.
  delay(2000);  //humidity sensor measures results every 2 seconds (its a very slow sensor
                // Reading temperature or humidity takes about 250 milliseconds!
  float h = dht.readHumidity();

  // Check if read failed and say so (to try again).
  if (isnan(h)) 
  {
    lcd.print(F("Failed to read from DHT sensor!"));
    lcd.clear();
    lcd.setCursor(1,0); 
  }
  
  
  //Temperature
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  
  
  
  int percentage;
  float volts;
  volts = MGRead(MG_PIN);
  percentage = MGGetPercentage(volts,CO2Curve);
  
  
  
 //Printing to LCD
  lcd.print("Humidity: "); //print results to lcd 
  lcd.print(h);
  lcd.print(F("%"));
  
  lcd.setCursor(1,1);
  lcd.print("CO2: ");
 
  if (percentage == -1) 
  {
   lcd.print( "<400" );
  } 
   else 
  {
  lcd.print(percentage);
  }
 
  lcd.print( " ppm" );
  
  lcd.setCursor(1,3);
  lcd.print("Temperature: ");
  lcd.print(Tc);*/
 
  //Heating
  
  if (Tc > 6100.00)
  {
    digitalWrite(in1, LOW);
    lcd.setCursor(1,4);
    lcd.print("Heater on!");
  }
  else
  {
    digitalWrite(in1, HIGH);
    lcd.setCursor(1,4);
    lcd.print("Heater off!");
  }
  
  lcd.setCursor(1,0);
 }

 /*****************************  MGRead *********************************************
Input:   mg_pin - analog channel
Output:  output of SEN-000007
Remarks: This function reads the output of SEN-000007
************************************************************************************/
float MGRead(int mg_pin)

{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;
}


/*****************************  MQGetPercentage **********************************
Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(MG-811 output) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
