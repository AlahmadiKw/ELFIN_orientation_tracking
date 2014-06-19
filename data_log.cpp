#include "mbed.h"
#include "MMA8451Q.h"
#include "MAG3110.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

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
float xAccRaw = 0 ;
float yAccRaw = 0;
float zAccRaw = 0;

float xAccFiltered = 0;;
float yAccFiltered = 0;
float zAccFiltered = 0;

float xAccFilteredOld = 0;
float yAccFilteredOld = 0;
float zAccFilteredOld = 0;

float xAcc, yAcc, zAcc;


/////////////////////////////////////////////////////////////////////
// Include support for MAG3110 Magnetometer
MAG3110 mag(PTE25, PTE24);
int xMagRaw =0;
int yMagRaw =0;
int zMagRaw =0;

float xMagFiltered = 0;
float yMagFiltered = 0;
float zMagFiltered = 0;

float xMagFilteredOld = 0;
float yMagFilteredOld = 0;
float zMagFilteredOld = 0;

int minXmag, maxXmag, minYmag, maxYmag, minZmag, maxZmag;
float xMagMap, yMagMap, zMagMap;
float magNor;
float xMag, yMag, zMag;


int tempXmax, tempXmin, tempYmax, tempYmin, tempZmax, tempZmin, newX, newY, newZ;


// moved calibration from class library to here because for some reason the class method is not functioning
void calibrateXYZ(PinName pin, int activeValue, int *minX, int *maxX, int *minY, int *maxY, int *minZ, int *maxZ);

void processAcc();
void processMag();


/////////////////////////////////////////////////////////////////////
// low pass filter
float alpha = 0.4; 


/////////////////////////////////////////////////////////////////////
// Include a 1 second ticker as a heartbeat
Ticker heartBeat;



/////////////////////////////////////////////////////////////////////
// Structure to hold FRDM-KL46Z sensor and input data
float     pitch, roll, yaw;

    

/////////////////////////////////////////////////////////////////////
// Prototype for routine to send all sensor data to serial port
void serialSendSensorData(void);    


/////////////////////////////////////////////////////////////////////
// Prototype for LED flash routine
void ledFlashTick(void);

/////////////////////////////////////////////////////////////////////
// Map function (same one as arduino)
float mapData(float val, int in_min, int in_max, float out_min, float out_max);


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
        processAcc();
        processMag();
        pitch = atan2(  -xAcc ,   sqrt(pow(yAcc,2)+pow(zAcc,2))) *180/M_PI;
        roll  = atan2(   yAcc ,   sqrt(pow(xAcc,2)+pow(zAcc,2))) *180/M_PI; 
        // theta roll , phi pitch 
        yaw= atan2( (-yMag*cos(roll) + zMag*sin(roll) ) , 
                    xMag*cos(pitch) + yMag*sin(pitch)*sin(roll) - zMag*cos(roll)*sin(pitch) )
             *180/M_PI;
        // yaw = atan2((double)(yMag),(double)(xMag))*180/M_PI;

        serialSendSensorData();
        
        // Blink red LED (loop running)
        redLED = !redLED;
        wait(0.05);
    }  
}


void serialSendSensorData(void)
{
    // printf("%10f  %10f\n", roll, pitch);
    printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 1,2,3,xAccRaw, );
}    


void ledFlashTick(void)
{
    greenLED = !greenLED;
}

float mapData(float val, int in_min, int in_max, float out_min, float out_max){
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

void processAcc(){
    // Process acc data 
    xAccRaw = acc.getAccX();
    yAccRaw = acc.getAccY();
    zAccRaw = acc.getAccZ();

    xAccFiltered = xAccFilteredOld + alpha * (xAccRaw - xAccFilteredOld);
    yAccFiltered = yAccFilteredOld + alpha * (yAccRaw - yAccFilteredOld);
    zAccFiltered = zAccFilteredOld + alpha * (zAccRaw - zAccFilteredOld);

    xAccFilteredOld = xAccFiltered;
    yAccFilteredOld = yAccFiltered;
    zAccFilteredOld = zAccFiltered;

    xAcc = xAccFiltered;
    yAcc = yAccFiltered;
    zAcc = zAccFiltered;
}

void processMag(){
    // Process mag data 
    xMagRaw = mag.readVal(MAG_OUT_X_MSB);
    yMagRaw = mag.readVal(MAG_OUT_Y_MSB);
    zMagRaw = mag.readVal(MAG_OUT_Z_MSB);

    // xMagFiltered = xMagFilteredOld + alpha * (xMagRaw - xMagFilteredOld);
    // yMagFiltered = yMagFilteredOld + alpha * (yMagRaw - yMagFilteredOld);
    // zMagFiltered = zMagFilteredOld + alpha * (zMagRaw - zMagFilteredOld);
    xMagFiltered = xMagRaw;
    yMagFiltered = yMagRaw;
    zMagFiltered = zMagRaw;

    xMagFilteredOld = xMagFiltered;
    yMagFilteredOld = yMagFiltered;
    zMagFilteredOld = zMagFiltered;

    xMagFiltered -= (maxXmag-minXmag)/2;
    yMagFiltered -= (maxYmag-minYmag)/2;
    zMagFiltered -= (maxZmag-minZmag)/2;

    magNor = sqrt(pow(xMagFiltered,2)+pow(yMagFiltered,2)+pow(zMagFiltered,2));


    xMag = xMagFiltered/magNor;
    yMag = yMagFiltered/magNor;
    zMag = zMagFiltered/magNor;

        // recalibrate in case 
    // if (xMagFiltered>maxXmag) {maxXmag = xMagFiltered;}
    // if (yMagFiltered>maxYmag) {maxYmag = yMagFiltered;}
    // if (zMagFiltered>maxZmag) {maxZmag = zMagFiltered;}

    // if (xMagFiltered<minXmag) {minXmag = xMagFiltered;}
    // if (yMagFiltered<minYmag) {minYmag = yMagFiltered;}
    // if (zMagFiltered<minZmag) {minZmag = zMagFiltered;}
}
