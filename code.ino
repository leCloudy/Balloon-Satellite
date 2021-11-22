
#include <Arduino.h>
#include <Wire.h>
#include <BMx280I2C.h>
#define I2C_ADDRESS 0x76
#include <Wire.h> // Used to establied serial communication on the I2C bus
#include <SparkFunTMP102.h> // Used to send and recieve specific information from our sensor
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
BMx280I2C bmx280(I2C_ADDRESS);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

  #include <SimpleZigBeeRadio.h>
  #include <SoftwareSerial.h>

  // Create the XBee object ...
  SimpleZigBeeRadio xbee = SimpleZigBeeRadio();
  // ... and the software serial port. Note: Only one
  // SoftwareSerial object can receive data at a time.
  SoftwareSerial xbeeSerial(10, 11); // (RX=>DOUT, TX=>DIN)
  
  unsigned long time = 0;
  unsigned long last_sent = 0;
  int sum = 0;
  int check = 0;
  
void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

const int ALERT_PIN = A3;

TMP102 sensor0;


uint16_t pressure;
int temperature;
int accelx; int accely; int accelz;
int magx; int magy; int magz;
int gyrox; int gyroy; int gyroz;


void setup() {
  

  Serial.begin(115200);
  Serial.println("running");

  while( !Serial ){;// Wait for serial port (for Leonardo only). 
    }
    xbeeSerial.begin( 115200 );
    // ... and set the serial port for the XBee radio.
    xbee.setSerial( xbeeSerial );
    // Receive TX Status packets
    xbee.setAcknowledgement(true);
    
    // To ensure that the radio is in API Mode 2 and is
    // operating on the correct PAN ID, you can use the 
    // AT Commands AP and ID. Note: These changes will
    // be stored in volatile memory and will not persist
    // if power is lost.
    xbee.prepareATCommand('AP',2);
    xbee.send();
    delay(200);
    uint8_t panID[] = {0x12,0x34}; // Max: 64-bit
    xbee.prepareATCommand('ID',panID,sizeof(panID));
    xbee.send();
    
  

  while (!Serial);
  Wire.begin();
  if (!bmx280.begin())
  {
    Serial.println("Cannot connect to BME280. ");
    while (1);
  }

  if (bmx280.isBME280())
    Serial.print("BME280 working, ");
  else
    Serial.print("BMP280 working, ");

  bmx280.resetToDefaults();
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  if (bmx280.isBME280())
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);



   // tmp102

      Wire.begin(); //Join I2C Bus
  
  pinMode(ALERT_PIN,INPUT);  // Declare alertPin as an input

  if(!sensor0.begin())
  {
    Serial.println("Cannot connect to TMP102. ");
  }
  
  Serial.print("TMP102 working, ");
  delay(100);

  sensor0.setFault(0);  // Trigger alarm immediately
  sensor0.setAlertPolarity(1); // Active HIGH
  sensor0.setAlertMode(0); // Comparator Mode.
  sensor0.setConversionRate(2);
  sensor0.setExtendedMode(0);
  sensor0.setHighTempF(85.0);  // set T_HIGH in F
  sensor0.setLowTempF(84.0);  // set T_LOW in F

  //gyro
 
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
    
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Cannot connect to LSM9DS1. ");
    while (1);
  }
  Serial.println("LSM9DS1 working.");
  Serial.println("---------------------------------------------");

  setupSensor();
    


    //temperature



}

void loop() {

  

  byte pressurehigh = highByte(pressure); byte pressurelow = lowByte(pressure);
  
  byte temphigh = highByte(temperature); byte templow = lowByte(temperature);
  
  byte accelxhigh = highByte(accelx); byte accelxlow = lowByte(accelx);
  byte accelyhigh = highByte(accely); byte accelylow = lowByte(accely);
  byte accelzhigh = highByte(accelz); byte accelzlow = lowByte(accelz);
  
  byte magxhigh = highByte(magx); byte magxlow = lowByte(magx);
  byte magyhigh = highByte(magy); byte magylow = lowByte(magy);
  byte magzhigh = highByte(magz); byte magzlow = lowByte(magz);
  
  byte gyroxhigh = highByte(gyrox); byte gyroxlow = lowByte(gyrox);
  byte gyroyhigh = highByte(gyroy); byte gyroylow = lowByte(gyroy);
  byte gyrozhigh = highByte(gyroz); byte gyrozlow = lowByte(gyroz);




  delay(1000);

  if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
    return;
  }
 
  do
  {
    delay(100);
  } while (!bmx280.hasValue());



float f_pressure = bmx280.getPressure();
pressure = (round(f_pressure)/10);


  Serial.print("(Pa / 10) Pressure: "); Serial.println(pressure);


  //temp
  
  temperature = ((sensor0.readTempC())*100);
  boolean alertPinState, alertRegisterState;
  sensor0.wakeup();
  alertPinState = digitalRead(ALERT_PIN); // read the Alert from pin
  alertRegisterState = sensor0.alert();   // read the Alert from register
  sensor0.sleep();
  
  Serial.print("(x 10^-2) Temperature: ");
  Serial.println(temperature);

 
  delay(1000);  // Wait 1000ms

  //gyro

  lsm.read();  /* ask it to read in the data */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  accelx = a.acceleration.x * 100; accely = a.acceleration.y *100; accelz = a.acceleration.z *100;
  magx = m.magnetic.x * 100; magy = m.magnetic.y * 100; magz = m.magnetic.z *100;
  gyrox = g.gyro.x *100; gyroy = g.gyro.x*100 ; gyroz = g.gyro.z *100;


  Serial.print("(x 10^-2 ,m/s^2) Accel X: "); Serial.print(accelx); 
  Serial.print("\tY: "); Serial.print(accely);
  Serial.print("\tZ: "); Serial.println(accelz);

  Serial.print("(x 10^-2, uT) Mag X: "); Serial.print(magx);
  Serial.print("\tY: "); Serial.print(magy);
  Serial.print("\tZ: "); Serial.println(magz); 

  Serial.print("(x 10^-2, rad/s) Gyro X: "); Serial.print(gyrox);
  Serial.print("\tY: "); Serial.print(gyroy);
  Serial.print("\tZ: "); Serial.println(gyroz); 



  delay(200);

   time = millis();
    if( time > (last_sent+5000) ){
      last_sent = time; // Update the last_sent variable
      
      // Send 3 random numbers (between 0 and 254) to the
      // coordinator.
      uint8_t payload[13];
      payload[0] = pressurehigh & 0xff;
      payload[1] = pressurelow & 0xff;
      
      payload[2] = temphigh & 0xff;
      payload[3] = templow & 0xff;      
      
      payload[4] = accelx & 0xff;
      payload[5] = accely & 0xff;
      payload[6] = accelz & 0xff;

      payload[7] = magx & 0xff;
      payload[8] = magy & 0xff;
      payload[9] = magz & 0xff;

      payload[10] = gyrox & 0xff;
      payload[11] = gyroy & 0xff;
      payload[12] = gyroz & 0xff;

      
      sum = payload[0] + payload[1] + payload[2] + payload[3] + payload[4] + 
            payload[5] + payload[6] + payload[7] + payload[8] + payload[9] + 
            payload[10] + payload[11] + payload[12];


     for(int i = 0; i < sizeof(payload); i++)
{
  Serial.println(payload[i], HEX);
}

      xbee.prepareTXRequestToCoordinator( payload, sizeof(payload) );
      xbee.send();
      Serial.println();
      Serial.print( "Send: " );
      printPacket( xbee.getOutgoingPacketObject() );

      Serial.println();
      Serial.println();
      Serial.println();
    }
    
    delay(10); // Small delay for stability
    delay(5000);


}


  /////////////////////////////////////////////////////////////
  // Function for printing the complete contents of a packet //
  /////////////////////////////////////////////////////////////
  void printPacket(SimpleZigBeePacket & p){
    Serial.print( START, HEX );
    Serial.print(' ');
    Serial.print( p.getLengthMSB(), HEX );
    Serial.print(' ');
    Serial.print( p.getLengthLSB(), HEX );
    Serial.print(' ');
    // Frame Type and Frame ID are stored in Frame Data
    uint8_t checksum = 0;
    for( int i=0; i<p.getFrameLength(); i++){
      Serial.print( p.getFrameData(i), HEX );
      Serial.print(' ');
      checksum += p.getFrameData(i); 
    }
    // Calculate checksum based on summation of frame bytes
    checksum = 0xff - checksum;
    Serial.print(checksum, HEX );
    Serial.println();
  }
