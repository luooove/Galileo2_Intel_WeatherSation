// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain
#include <dht.h>

/*********PIN Definition**************/
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



/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.324) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.020) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2


/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};   
                                                     //two points are taken from the curve. 
                                                     //with these two points, a line is formed which is
                                                     //"approximately equivalent" to the original curve.
                                                     //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280) 
                                                     //slope = ( reaction voltage ) / (log400 –log1000) 




int UV_PIN_Analog = A0;
int dustPin = A0;
int ledPower = 13;

#define         MG_PIN                       (0)     //define which analog input channel you are going to use
#define         BOOL_PIN                     (2)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier





/*********Variables Updata**************/
float Humidity_Update = 0;      
float Temperature_Update = 0;  
float PM25_Update = 0;
float UV_Update = 0;
float CO2_Update = 0;

int dustVal = 0;
int delayTime=280;
int delayTime2=40;
float offTime=9680;



void setup() {
   Serial.begin(9600);
   Serial1.begin(9600);
   pinMode(BOOL_PIN, INPUT);                        //set pin to input
   digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
  
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
    // ledPower is any digital pin on the arduino connected to Pin 3 on the sensor
    digitalWrite(ledPower,LOW); 
    delayMicroseconds(delayTime);
    dustVal=analogRead(dustPin); 
    delayMicroseconds(delayTime2);
    digitalWrite(ledPower,HIGH); 
    delayMicroseconds(offTime);
   // delay(100);
   // Serial.print("PM2.5:  ");
    //Serial.println(dustVal,2);
   // Serial.println((float(dustVal/1024)-0.0356)*120000*0.035,2);
    PM25_Update = (float(dustVal/1024)-0.0356)*120000*0.035;
}

void Sensor_UV()
{
  int i = 0;
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

void Sensor_CO2()  //not tested 
{
    int percentage;
    float volts;
    
   
    volts = MGRead(MG_PIN);
    Serial.print( "SEN0159:" );
    Serial.print(volts); 
    Serial.print( "V           " );
    
    percentage = MGGetPercentage(volts,CO2Curve);
    Serial.print("CO2:");
    if (percentage == -1) {
        Serial.print( "<400" );
    } else {
        Serial.print(percentage);
    }
    Serial.print( "ppm" );  
    Serial.print( "       Time point:" );
    Serial.print(millis());
    Serial.print("\n");
    
    if (digitalRead(BOOL_PIN) ){
        Serial.print( "=====BOOL is HIGH======" );
    } else {
        Serial.print( "=====BOOL is LOW======" );
    }
      
    Serial.print("\n");
    
    delay(200);
}


/*****************************  MGRead *********************************************
Input:   mg_pin - analog channel
Output:  output of SEN0159
Remarks: This function reads the output of SEN0159
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

