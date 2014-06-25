
#include <SPI.h>
#include <RF22.h>

RF22 rf22;
float frequency = 431.300; 
float ppm = -8;   //transmitter correction offset 

void setup() 
{
  Serial.begin(9600);
  if (!rf22.init())
    Serial.println("RF22 init failed");
  rf22.setFrequency(frequency * ((1000000 + ppm)/1000000)); 
  rf22.setModemConfig(RF22::GFSK_Rb2Fd5);  
}

float convert(uint8_t& small, uint8_t& large)
{
  uint16_t lsmall = (uint16_t) small; 
  uint16_t llarge = (((uint16_t) large) << 8);
  int isum = (int)(lsmall + llarge);
  float fsum = (float) isum; 
  return fsum;  
}

unsigned long timeConvert(uint8_t& small, uint8_t& med, uint8_t& large, uint8_t& huge)
{
  uint32_t lsmall = (uint32_t) small; 
  uint32_t lmed = (((uint32_t) med) << 8); 
  uint32_t llarge = (((uint32_t) large) << 16);
  uint32_t lhuge = (((uint32_t) huge) << 24);
  unsigned long sum = (unsigned long)(lsmall + lmed + llarge +lhuge); 
  return sum; 
} 

void loop()
{
  while (1)
  {
    if (rf22.waitAvailableTimeout(1000) == true)  
    {
      uint8_t rssi = rf22.rssiRead();     //have to take this value first otherwise returns junk
      Serial.print(rssi, DEC); 
      Serial.print(","); 
      uint8_t buf[RF22_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf22.recv(buf, &len)) 
      {
        unsigned long rtime = millis(); 
        Serial.print(rtime, DEC);      //print receiver time
        Serial.print(',');  
        unsigned long ttime = timeConvert(buf[5], buf[6], buf[19], buf[20]); 
        Serial.print(ttime, DEC);    //print transmit time  
        Serial.print(',');    
        
        int incrm = 7;     
        while (incrm <= 18)      //the last useful values are stored in indexes 17 and 18    
        {
          float dataOutput = convert(buf[incrm], buf[incrm + 1]); 
          Serial.print(dataOutput);   //print out the accelerometer and magnetometer values in order listed
          if (incrm == 17)
          {
            Serial.print('\n');    //we've already outputted the last piece of useful data
            break;
          }
          Serial.print(','); 
          incrm += 2; 
        }   
      }      
    }
    else
    {
      while(true) 
      {
        String content = "";
        char character;
        while(Serial.available()) 
        {
          character = Serial.read();
          content.concat(character);
          delay(10); 
        }
        if ((content == "tx") || (content == "TX") || (content == "Tx")) 
        {
          Serial.println(content);
          uint8_t msg[2]; 
          msg[0] = 't'; msg[1] = 'x'; 
          rf22.send(msg, sizeof(msg)); 
          rf22.waitPacketSent();
          break;  
        }
      }
    } //end of if statement
  } //end of infinite while loop 
}

