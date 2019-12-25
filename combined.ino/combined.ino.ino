
// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

// rolling average library, form https://github.com/MajenkoLibraries/Average, forked to https://github.com/paulrogers/Average
#include <Average.h>


#define DHTPIN1 2     // Digital pin connected to the DHT sensor
#define DHTPIN2 3     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);


// Values for thermostat control

float default_temp = 18.00;
float  default_swing = 2.0;

//   the heater on/off button 
bool heater_on = false;

// is the heater /actually/ on
bool heating = false;

const int heaterOnOffPin = 8;
const int heatUpPin = 9;
const int heatDownPin = 10;
const int heaterRelayPin = 11;

float setTemperature;
float tempIncrement = 0.5;


// rolling average to help stop spikes in current temp causing heater to cycle quickl
Average<float> ave(20);


// avoid using the delay)( function so we get fast feedback on the switches
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 5000;         


void setHeaterState( bool onOrOff){

  if (onOrOff){
    heating = true;
    digitalWrite( heaterRelayPin , HIGH);
  } else {
    heating = true;
    digitalWrite( heaterRelayPin, LOW );
  }
}


void writeSetTempToLCD( float setTemp){
  lcd.setCursor(5,3);
  lcd.write( setTemp);
  
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));

  dht1.begin();
  dht2.begin();
  lcd.begin(20, 4);

  pinMode(heaterOnOffPin, INPUT_PULLUP);
  pinMode(heatUpPin, INPUT_PULLUP);
  pinMode(heatDownPin, INPUT_PULLUP);
  
  
  pinMode(heaterRelayPin, OUTPUT);
  digitalWrite( heaterRelayPin, LOW);
  setTemperature = default_temp;
}

void loop() {
  // Wait a few seconds between measurements.
  //delay(2000);


   // set the heater on or off
   /*
    *  if HEATER_OFF   - off
    *  if HEATER_ON and set_temp > t1  OFF
    *  if HEATER_ON and (set_temp+ ( -1*default_swing) ) <= t1
    *  
    *  we also want to average the last XX readings beore deciding...phase 2
    *  
    */

    // read state of on/off button
    bool heaterOnOffButtonState = digitalRead(heaterOnOffPin);
    if (!heaterOnOffButtonState ){
      heater_on  = !heater_on;
      
      lcd.setCursor(17,3);
      lcd.print(heaterOnOffButtonState);
      
    }

    bool heaterUpButtonPressed = digitalRead(heatUpPin);
    if ( !heaterUpButtonPressed){
      lcd.setCursor(18,3);
      lcd.print(heaterUpButtonPressed);
      setTemperature = setTemperature + tempIncrement;
      writeSetTempToLCD( setTemperature );
    }

    bool heaterDownButtonPressed = digitalRead(heatDownPin);
    if ( !heaterDownButtonPressed){
      lcd.setCursor(19,3);
      lcd.print(heaterDownButtonPressed);
      setTemperature = setTemperature - tempIncrement;
      writeSetTempToLCD( setTemperature );
    }
    if ( heaterOnOffButtonState && !heater_on  ){
      
    }
      
  

    


    // read higher/lower buttons

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h1 = dht1.readHumidity();
    // Read temperature as Celsius (the default)
    float t1 = dht1.readTemperature();
    float h2 = dht2.readHumidity();
    // Read temperature as Celsius (the default)
    float t2 = dht2.readTemperature();
  

    // record the temp to get our rolling average
    ave.push( t1 );
  
  
  
    
    // Check if any reads failed and exit early (to try again).
    /*
    if (isnan(h1) || isnan(t1) ) {
      //Serial.println(F("Failed to read from DHT sensor!"));
      //return;
      h1  ="?";
      t1 = "?";
    }
    */
  
    Serial.print(F("Humidity: "));
    Serial.print(h1);
    Serial.print(F("%  Temperature: "));
    Serial.print(t1);
    Serial.print(F("°C "));

    Serial.print(" set: ");
    Serial.print( setTemperature );
    Serial.print(" enabled: ");
    Serial.print( heater_on );
    
    Serial.print(" heating: ");
    Serial.println( heating );
    
    
    lcd.setCursor(0, 0);
    lcd.print("In:  ");
    lcd.print( t1 );
    lcd.print(" ");
    lcd.print(h1);
  
    lcd.setCursor(0, 1);
    lcd.print("Out: ");
    lcd.print( t2 );
    lcd.print(" ");
    lcd.print(h2);
  
    // verages, for debug
    lcd.setCursor(0,2);
    lcd.print("av:");
    lcd.print(ave.mean() );
  
    lcd.setCursor(0,3);
    lcd.print("Set: ");
    lcd.print( setTemperature);
    if (heater_on)
      lcd.print(" ON ");
    else
      lcd.print(" OFF");  
  }    
}
