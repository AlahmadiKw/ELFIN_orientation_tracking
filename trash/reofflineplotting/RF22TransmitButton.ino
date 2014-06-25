#include <Adafruit_LSM303.h>
#include <RF22.h>
#include <SPI.h>
#include <Wire.h> 

RF22 rf22; 
unsigned long previousMillis = 0;   //will be changed periodically for interval purposes 
Adafruit_LSM303 lsm303; 

//Configuration Options--------------------------------------------------------------//
uint8_t callsign[] = "W6YRA"; 
char callsignRep[] = "W6YRA";    //cant cast uint8_t
const unsigned long interval = 20000;    //should be roughly 20 seconds

float frequency = 431.300; 
float ppm = -8;   //transmitter correction offset 
                
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  if (! rf22.init()) 
    Serial.println("RF22 init failed"); 
  if (!lsm303.begin())
  {
    Serial.println("unable to initialize the LSM303. Check your wiring!");
  }  
  rf22.setFrequency(frequency * ((1000000 + ppm)/1000000)); 
  rf22.setModemConfig(RF22::GFSK_Rb2Fd5);  
}

void loop()
{  
  rf22.waitAvailable(); 
  uint8_t buf[RF22_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if(rf22.recv(buf, &len))
  {
    if(buf[0] == 't' && buf[1] == 'x') 
    { 
      previousMillis = millis(); 
      while(true)
      {
        //callsign at beginning and end of packet
        uint8_t msg[25]; 
        msg[0] = 'W'; msg[1] = '6'; msg[2] = 'Y'; msg[3] = 'R'; msg[4] = 'A'; 
        msg[21] = 'W'; msg[22] = '6'; msg[23] = 'Y'; msg[24] = 'R'; msg[25] = 'A';    
        	
        //Extract data from Magnetometer and Accelerometer, these are float values (change to real things later) 
        lsm303.read(); 	
        float ax = lsm303.accelData.x; 
        int temp = (int) ax; 
        uint16_t acc_x = (uint16_t) temp; 
        msg[7] = acc_x & 0xff;  msg[8] = (acc_x >> 8);
      
        float ay = lsm303.accelData.y; 
        temp = (int) ay; 
        uint16_t acc_y = (uint16_t) temp; 
        msg[9] = acc_y & 0xff;  msg[10] = (acc_y >> 8); 
      	
        float az = lsm303.accelData.z; 
        temp = (int) az;
        uint16_t acc_z = (uint16_t) temp; 
        msg[11] = acc_z & 0xff;  msg[12] = (acc_z >> 8);
      
        float mx = lsm303.magData.x; 
        temp = (int) mx; 
        uint16_t mag_x = (uint16_t) temp; 
        msg[13] = mag_x & 0xff;  msg[14] = (mag_x >> 8);
      
        float my = lsm303.magData.y;
        temp = (int) my;  
        uint16_t mag_y = (uint16_t) temp; 
        msg[15] = mag_y & 0xff;  msg[16] = (mag_y >> 8);
      	
        float mz = lsm303.magData.z;
        temp = (int) mz;  
        uint16_t mag_z = (uint16_t) temp; 
        msg[17] = mag_z & 0xff;  msg[18] = (mag_z >> 8);
      	
        //get time of transmission 
        unsigned long ttime = millis();
        uint32_t tCast = (uint32_t) ttime; 
        msg[5] = tCast & 0xff;  msg[6] = (tCast >> 8) & 0xff; 
        msg[19] = (tCast >> 16) & 0xff;   msg[20] = (tCast >> 24);  
        
        //For testing purposes, output the values that are sent
        Serial.print(ttime); 
        Serial.print(",");
        Serial.print(ax, 7); 
        Serial.print(",");
        Serial.print(ay, 7); 
        Serial.print(",");
        Serial.print(az, 7); 
        Serial.print(",");
        Serial.print(mx, 7);
        Serial.print(",");
        Serial.print(my, 7); 
        Serial.print(",");
        Serial.println(mz, 7);
        
        rf22.send(msg, sizeof(msg)); 
        rf22.waitPacketSent();
        unsigned long currentMillis = millis(); 
        if (currentMillis - previousMillis > interval)   //check if the allowed interval is up
          break;    //get out of infinite loop after ___ seconds of running
      }//end of loop
    }//end of if statement checking for tx     
  }//end of if statement checking for received msg
}


