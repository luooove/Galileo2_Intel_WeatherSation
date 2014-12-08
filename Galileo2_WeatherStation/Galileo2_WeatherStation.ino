// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain
#include <dht.h>

#define DHTIN 2 // what pin we're connected to
#define DHTOUT 4

// Uncomment whatever type you're using!
#define DHTTYPE DHT11 // DHT 11 
//#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE DHT21 // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTIN,DHTOUT, DHTTYPE);



int UV_PIN_Analog = A0;



/*********Variables Updata**************/
float Humidity_Update = 0;      
float Temperature_Update = 0;  
float PM25_Update = 0;
float UV_Update = 0;
float CO2_Update = 0;


void setup() {
   Serial.begin(9600);
   Serial1.begin(9600);  
   Serial.println("DHTxx test!");
   
   dht.begin();
}

void loop() {
   // Wait a few seconds between measurements.
   delay(2000);
   Sensor_All(); // 
   Update();    //
}



void Update()
{
    Serial1.print(Temperature_Update);Serial1.print("|");
    Serial1.print(Humidity_Update);Serial1.print("|");
    Serial1.print(PM25_Update);Serial1.print("|");
    Serial1.print(UV_Update);Serial1.print("|");
    Serial1.println(CO2_Update);Serial1.print("|");
}

void Sensor_All()
{
   Sensor_TemperatureHumidity();
   Sensor_PM25();
   Sensor_UV();
   Sensor_CO2();
  
}

void Sensor_TemperatureHumidity()
{
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    Serial.println(h);
    
    // Read temperature as Celsius
    float t = dht.readTemperature();
    Serial.println(t);
    
    // Read temperature as Fahrenheit
    float f = dht.readTemperature(true);
     
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
    }
    
    // Compute heat index
    // Must send in temp in Fahrenheit!
    float hi = dht.computeHeatIndex(f, h);
}

void Sensor_PM25()
{
    
}

void Sensor_UV()
{
  i=analogRead(UV_PIN_Analog);
  if(i<46)UV_Update=0;
  else if(i>=46&&i<65)UV_Update=1;
  else if(i>=65&&i<83)UV_Update=2;
  else if(i>=83&&i<103)UV_Update=3;
  else if(i>=103&&i<124)UV_Update=4;
  else if(i>=124&&i<142)UV_Update=5;
  else if(i>=124&&i<142)UV_Update=5;
  else if(i>=142&&i<162)UV_Update=6;
  else if(i>=162&&i<180)UV_Update=7;
  else if(i>=180&&i<199)UV_Update=8;
  else if(i>=199&&i<220)UV_Update=9;
  else if(i>=220&&i<239)UV_Update=10;
  else if(i>=239)UV_Update=11;
}

void Sensor_CO2()
{
  
}

