
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
// the default set temp when we start up
float default_temp = 18.00;

// the value where we ignore temp changes after reaching our set point
float  default_swing = 2.0;

//   the heater on/off button
bool heater_enabled = false;

// is the heater /actually/ on
bool heating = false;

// pins for the buttons
const int heaterOnOffPin = 9;
const int heatUpPin = 10;
const int heatDownPin = 8;

// the relay pin
const int heaterRelayPin = 11;

// the temperature we want to aim for
float setTemperature;
// the increment in temperature between button presses
float tempIncrement = 0.5;


// rolling average to help stop spikes in current temp causing heater to cycle quickl
Average<float> ave(20);


// avoid using the delay() function so we get fast feedback on the switches
unsigned long previousMillis = 0;        // will store last time loop was executed
const long sensorReadInterval = 5000;

bool heaterOnOffButtonState_LAST = true;
bool heaterUpButtonState_LAST = true;
bool heaterDownButtonState_LAST = true;

float t1;
float h1;
float t2;
float h2;


// constant for interval before allowing heater to come on again
const  long five_minutes =  300000 ;
unsigned long lastHeaterStoppedAt = 0 ;


void setHeaterState( bool onOrOff) {

  if (onOrOff) {
    heating = true;
    digitalWrite( heaterRelayPin , LOW);
    writeHeaterStateToLCD(" on");
  } else {
    heating = false;
    digitalWrite( heaterRelayPin, HIGH );
    writeHeaterStateToLCD(" off");
  }
}


void writeSetTempToLCD( float setTemp) {
  lcd.setCursor(5, 3);
  lcd.print( "     " );
  lcd.setCursor(5, 3);
  lcd.print( setTemp);

}

void writeHeaterEnabledToLCD( bool heatEnabled) {
  lcd.setCursor(11, 3);
  lcd.print( "     " );
  lcd.setCursor(11, 3);
  if (heatEnabled)
    lcd.print( " ON");
  else
    lcd.print( " OFF");
}

void writeHeaterStateToLCD(String wait) {
  lcd.setCursor(15, 3);
  while (wait.length() < 4 )
    wait = wait + " ";
  lcd.print( wait );
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
  digitalWrite( heaterRelayPin, HIGH);
  setTemperature = default_temp;
}


void loop() {



  // read state of on/off button
  bool heaterOnOffButtonState = digitalRead(heaterOnOffPin);
  if (!heaterOnOffButtonState ) {
    if ( heaterOnOffButtonState_LAST ) {
      heater_enabled  = !heater_enabled;
      Serial.println("on/off button pressed");
      writeHeaterEnabledToLCD(heater_enabled);
    }
  }
  heaterOnOffButtonState_LAST = heaterOnOffButtonState;
  delay(50);

  bool heaterUpButtonPressed = digitalRead(heatUpPin);
  if ( !heaterUpButtonPressed) {
    if ( heaterOnOffButtonState_LAST ) {
      setTemperature = setTemperature + tempIncrement;
      writeSetTempToLCD( setTemperature );
      lcd.print(heaterUpButtonPressed);
      Serial.println("up button pressed");
    }
  }
  heaterOnOffButtonState_LAST = heaterUpButtonPressed;
  delay(50);

  bool heaterDownButtonPressed = digitalRead(heatDownPin);
  if ( !heaterDownButtonPressed) {
    if ( heaterDownButtonState_LAST ) {
      setTemperature = setTemperature - tempIncrement;
      writeSetTempToLCD( setTemperature );
      Serial.println("down button pressed");
    }
  }
  heaterDownButtonState_LAST = heaterUpButtonPressed;
  delay(50);

  if ( heater_enabled  ) {

    if ( (setTemperature > ( ave.mean() ))  && !heating  ) {
      Serial.println("set temp is more than actual, enabling heater, if it hasnt ben used recently");
      // dont start heater within 5 minuts of last heat cycle
      unsigned long currentHeaterMillis = millis();
      Serial.println( currentHeaterMillis );
      Serial.println( five_minutes );
      Serial.println( lastHeaterStoppedAt );

      // if we cycle the heater, with in 5 minutes, we get a -ve, which is > five_minutes
      // so we need to check for > 0 too, but then we get cast issues....
      // As long as we dont cycle in the first 5 minutes, I think its all ok....
      Serial.println(currentHeaterMillis - five_minutes );
      if ( (currentHeaterMillis - five_minutes)  > 0 ) {
        if ( ( currentHeaterMillis - five_minutes ) > lastHeaterStoppedAt  ) {
          setHeaterState( true );
        }  else {
          Serial.println("Want to heat, but need to wait..");
          writeHeaterStateToLCD("wait");
        }
      } else {
        Serial.println("Cant Heat cycle in first 5 minutes. Need to wait..");
        writeHeaterStateToLCD("init");
      }
    }
  } else {
      // likley the on/off button has been set to off
      if ( heating) {
        setHeaterState( false );
        lastHeaterStoppedAt = millis();
        Serial.println(" heater set to off ");
      }    
  }
  if ( (setTemperature <= ( ave.mean()   ) ) && heating) {
    Serial.println("Desired Temperature reached. Heater being switched off!");
    setHeaterState( false );
    lastHeaterStoppedAt = millis();
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorReadInterval) {
    previousMillis = currentMillis;

    // Reading temperature or humidity takes about 250 milliseconds!

    h1 = dht1.readHumidity();
    t1 = dht1.readTemperature();

    h2 = dht2.readHumidity();
    t2 = dht2.readTemperature();


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
    Serial.print( heater_enabled );

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

    // averages, for debug
    lcd.setCursor(0, 2);
    lcd.print("av: ");
    lcd.print(ave.mean() );

    lcd.setCursor(0, 3);
    lcd.print("Set: ");
    writeSetTempToLCD( setTemperature );
    writeHeaterEnabledToLCD( heater_enabled);
  }
}
