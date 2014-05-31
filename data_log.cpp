#include "mbed.h"
#include "TSISensor.h"
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


//////////////////////////////////////////////////////////////////////
// Include support for onboard Capacitive Touch Slider
TSISensor slider;


//////////////////////////////////////////////////////////////////////
// Include support for analog inputs
AnalogIn  lightSense(PTE22);


/////////////////////////////////////////////////////////////////////
// Include support for MMA8451Q Acceleromoter
#define MMA8451_I2C_ADDRESS (0x1d<<1)
MMA8451Q acc(PTE25, PTE24, MMA8451_I2C_ADDRESS);


/////////////////////////////////////////////////////////////////////
// Include support for MAG3110 Magnetometer
MAG3110 mag(PTE25, PTE24);


/////////////////////////////////////////////////////////////////////
// Include a 1 second ticker as a heartbeat
Ticker heartBeat;



/////////////////////////////////////////////////////////////////////
// Structure to hold FRDM-KL46Z sensor and input data
struct KL46_SENSOR_DATA {
    int     magXVal;
    int     magYVal;
    float   magHeading;
    
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
    pc.printf("Hello World\r\n");

    // Turn on pull up resistors on pushbutton inputs
    sw1.mode(PullUp);
    sw3.mode(PullUp);

    // Calibrate Magnetometer
    pc.printf("Press and release SW1, rotate the board 360 degrees.\r\n");
    pc.printf("Then press and release SW1 to complete the calibration process.\r\n");

    mag.calXY(PTC3, 0);

    pc.printf("Calibration complete.\r\n");


    // Loop forever - read and update sensor data and print to console.    
    while(1)
    {   
        sensorData.accXVal = acc.getAccX();
        sensorData.accYVal = acc.getAccY();
        sensorData.accZVal = acc.getAccZ();

        sensorData.magXVal = mag.readVal(MAG_OUT_X_MSB);
        sensorData.magYVal = mag.readVal(MAG_OUT_Y_MSB);
        sensorData.magHeading = mag.getHeading();
                
        serialSendSensorData();
        
        // Blink red LED (loop running)
        redLED = !redLED;


        wait(0.01);
    }  
}


void serialSendSensorData(void)
{
    printf("Accelerometer:  %1.3f  %1.3f  %1.3f    ", sensorData.accXVal, sensorData.accYVal, sensorData.accZVal);
    printf("Magnetometer:   %d     %d     %.2f \r\n", sensorData.magXVal, sensorData.magYVal, sensorData.magHeading);
}    


void ledFlashTick(void)
{
    greenLED = !greenLED;
}

