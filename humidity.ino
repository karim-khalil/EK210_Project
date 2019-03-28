#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define dataPin 8 // Defines pin number to which the sensor is connected
dht DHT; // Creats a DHT object
LiquidCrystal_I2C lcd(0x27,20,4); //Creates a LiquidCrystal_I2C object
void setup() {
  Serial.begin(9600);
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
}
void loop() {
  int readData = DHT.read22(dataPin); // Reads the data from the sensor
  float h = DHT.humidity; // Gets the values of the humidity
  
  // Printing the results on the serial monitor
  
  Serial.print(" Humidity = ");
  Serial.print(h);
  Serial.println(" % ");
  lcd.setCursor(0,0);
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print(" %");
 
  
  delay(2000); // Delays 2 secods, as the DHT22 sampling rate is 0.5Hz
}

