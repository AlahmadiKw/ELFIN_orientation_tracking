#include "mbed.h"
#include "MMA8451Q.h"
#include "MAG3110.h"


// This project has been created to bring together the libraries required to support
// the hardware and sensors found on the Freescale FRDM-KL46Z board.  The following
// libraries are included and exercised in this project:
//
//  mbed (source: official mbed library)
//      Serial:
//          Serial console routed through the mbed interface
//              PTA2 / UART0_TX = TX Signal - also on Arduino pin D1
//              PTA1 / UART0_RX = RX Signal - also on Arduino pin D0
//
//      DigitalOut:
//          GPIO to drive onboard LEDs
//              PTD5 / GPIO = LED1 - drive low to turn on LED - also on Arduino pin D13
//              PTE29 / GPIO = LED2 - drive low to turn on LED
//
//      DigitalIn:
//          GPIO to monitor the two onboard push buttons
//              PTC3 / GPIO = SW1 - low input = button pressed
//              PTC12 / GPIO = SW3 - low input = button pressed
// 
//      AnalogIn:
//          ADC channel to monitor ambient light sensor
//              PTE22 / ADC = Light Sensor - higher value = darker
//
//  TSI (source: http://mbed.org/users/emilmont/code/TSI/ )
//      Capacitive Touch library to support the onboard Touch-Slider
//
//  FRDM_MMA8451Q (source: http://mbed.org/users/clemente/code/FRDM_MMA8451Q/ )
//      Freescale MMA8451 Accelerometer connected on I2C0
//          PTE24 / I2C0_SCL = I2C bus for communication (shared with MAG3110)
//          PTE25 / I2C0_SDA =  I2C bus for communication (shared with MAG3110)
//          PTC5 / INT1_ACCEL = INT1 output of MMA8451Q
//          PTD1 / INT2_ACCEL = INT2 output of MMA8451Q (shared with MAG3110)
//
//  MAG3110 (source: http://mbed.org/users/mmaas/code/MAG3110/)
//            (based on: http://mbed.org/users/SomeRandomBloke/code/MAG3110/)
//      Freescale MAG3110 Magnetomoter connected on I2C0
//          PTE24 / I2C0_SCL = I2C bus for communication (shared with MMA8451)
//          PTE25 / I2C0_SDA =  I2C bus for communication (shared with MMA8451)
//          PTD1 / INT1_MAG / INT2_ACCEL = INT1 output of MAG3110 (shared with MMA8451)
//



//////////////////////////////////////////////////////////////////////
// Include support for USB Serial console
Serial pc(USBTX, USBRX);


//////////////////////////////////////////////////////////////////////
// Include support for on-board green and red LEDs
#define LED_ON  0
#define LED_OFF 1
DigitalOut greenLED(LED_GREEN);
DigitalOut redLED(LED_RED);


//////////////////////////////////////////////////////////////////////
// Include support for onboard pushbuttons (value = 0 when pressed)
DigitalIn  sw1(PTC3);
DigitalIn  sw3(PTC12);


/////////////////////////////////////////////////////////////////////
// Include support for MMA8451Q Acceleromoter
#define MMA8451_I2C_ADDRESS (0x1d<<1)
MMA8451Q acc(PTE25, PTE24, MMA8451_I2C_ADDRESS);


/////////////////////////////////////////////////////////////////////
// Include support for MAG3110 Magnetometer
MAG3110 mag(PTE25, PTE24);
int xMagRaw;
int yMagRaw;
int zMagRaw;

int minXmag;
int maxXmag;
int minYmag;
int maxYmag;
int minZmag;
int maxZmag;

float xMagMap;
float yMagMap;
float zMagMap;

float magNor;

float xMagNor;
float yMagNor;
float zMagNor;

int tempXmax, tempXmin, tempYmax, tempYmin, tempZmax, tempZmin, newX, newY, newZ;

// moved calibration from class library to here because for some reason the class method is not functioning
void calibrateXYZ(PinName pin, int activeValue, int *minX, int *maxX, int *minY, int *maxY, int *minZ, int *maxZ);



/////////////////////////////////////////////////////////////////////
// Include a 1 second ticker as a heartbeat
Ticker heartBeat;



/////////////////////////////////////////////////////////////////////
// Structure to hold FRDM-KL46Z sensor and input data
struct KL46_SENSOR_DATA {
    float     magXVal;
    float     magYVal;
    float     magZVal;
    
    float   accXVal;
    float   accYVal;
    float   accZVal;
} sensorData;
    

/////////////////////////////////////////////////////////////////////
// Prototype for routine to send all sensor data to serial port
void serialSendSensorData(void);    


/////////////////////////////////////////////////////////////////////
// Prototype for LED flash routine
void ledFlashTick(void);

/////////////////////////////////////////////////////////////////////
// Map function (same one as arduino)
float mapData(int val, int in_min, int in_max, float out_min, float out_max);


// function [ output ] = mapData( data, in_min, in_max, out_min, out_max)
// %UNTITLED Summary of this function goes here
// %   Detailed explanation goes here
//  output = (data-in_min).* (out_max - out_min) / (in_max - in_min) + out_min;

// end


/////////////////////////////////////////////////////////////////////
// main application
int main()
{
    // Ensure LEDs are off
    greenLED = LED_OFF;
    redLED = LED_OFF;
    
    // Set up heartBeat Ticker to flash an LED
    heartBeat.attach(&ledFlashTick, 1.0); 


    // Set Serial Port data rate and say Hello
    pc.baud( 115200 );
    // pc.printf("Hello World\r\n");

    // Turn on pull up resistors on pushbutton inputs
    sw1.mode(PullUp);
    sw3.mode(PullUp);

    // Calibrate Magnetometer
    pc.printf("Press and release SW1, rotate the board 360 degrees.\r\n");
    pc.printf("Then press and release SW1 to complete the calibration process.\r\n");

    calibrateXYZ(PTC3, 0, &minXmag, &maxXmag, &minYmag, &maxYmag, &minZmag, &maxZmag);

    pc.printf("Calibration complete.\r\n");
    pc.printf("%10d %10d %10d %10d %10d %10d\r\n", minXmag, maxXmag, minYmag, maxYmag, minZmag, maxZmag);


    // Loop forever - read and update sensor data and print to console.    
    while(1)
    {   
        sensorData.accXVal = acc.getAccX();
        sensorData.accYVal = acc.getAccY();
        sensorData.accZVal = acc.getAccZ();

        xMagRaw = mag.readVal(MAG_OUT_X_MSB);
        yMagRaw = mag.readVal(MAG_OUT_Y_MSB);
        zMagRaw = mag.readVal(MAG_OUT_Z_MSB);

        xMagMap = mapData(xMagRaw, minXmag, maxXmag, -1, 1)   /1.0;
        yMagMap = mapData(yMagRaw, minYmag, maxYmag, -1, 1)   /1.0;
        zMagMap = mapData(zMagRaw, minZmag, maxZmag, -1, 1)   /1.0;

        magNor = sqrt( (xMagMap*xMagMap) + (yMagMap*yMagMap) + (zMagMap*zMagMap));

        xMagMap /= magNor;
        yMagMap /= magNor;
        zMagMap /= magNor;

        sensorData.magXVal = xMagMap;
        sensorData.magYVal = xMagMap;
        sensorData.magZVal = xMagMap;

        serialSendSensorData();
        
        // Blink red LED (loop running)
        redLED = !redLED;

        wait(0.01);
    }  
}


void serialSendSensorData(void)
{
    printf("%10f  %10f  %10f      ", sensorData.accXVal, sensorData.accYVal, sensorData.accZVal);
    printf("%10f  %10f  %10f\r\n", sensorData.magXVal, sensorData.magYVal, sensorData.magZVal);
    // printf("%10d  %10d  %10d\r\n", xMagRaw, yMagRaw, zMagRaw);
}    


void ledFlashTick(void)
{
    greenLED = !greenLED;
}

float mapData(int val, int in_min, int in_max, float out_min, float out_max){
    float result = (float)(val-in_min) * (out_max-out_min) / (float)(in_max-in_min) + out_min;
    return result;
}

void calibrateXYZ(PinName pin, int activeValue, int *minX, int *maxX, int *minY, int *maxY, int *minZ, int *maxZ)
{
    DigitalIn calPin(pin);
    
    // Wait for Button Press and Release before beginning calibration
    while(calPin != activeValue) {}
    while(calPin == activeValue) {}


    // Read initial values of magnetomoter - read it here to create a slight delay for calPin to settle
    tempXmax = mag.readVal(MAG_OUT_X_MSB);
    tempXmin = tempXmax;
    tempYmax = tempYmin = mag.readVal(MAG_OUT_Y_MSB);
    tempYmax = tempYmax;
    tempZmax = tempZmin = mag.readVal(MAG_OUT_Z_MSB);
    tempZmax = tempZmax;

    // Update min and max values until calPin asserted again
    while(calPin != activeValue) {
        newX = mag.readVal(MAG_OUT_X_MSB);
        newY = mag.readVal(MAG_OUT_Y_MSB);
        newZ = mag.readVal(MAG_OUT_Z_MSB);
        // printf("test %d \r\n", tempXmax);
        if (newX > tempXmax) tempXmax = newX;
        if (newX < tempXmin) tempXmin = newX;
        if (newY > tempYmax) tempYmax = newY;
        if (newY < tempYmin) tempYmin = newY;
        if (newZ > tempZmax) tempZmax = newZ;
        if (newZ < tempZmin) tempZmin = newZ;
    }

    *minX = tempXmin;
    *maxX = tempXmax;
    *minY = tempYmin;
    *maxY = tempYmax;
    *minZ = tempZmin;
    *maxZ = tempZmax;
}
