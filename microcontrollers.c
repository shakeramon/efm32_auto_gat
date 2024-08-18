#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_cmu.h"
#include "segmentlcd.h"


//MOTOR
#define MOTOR_IN1_PORT   gpioPortC
#define MOTOR_IN1_PIN    0
#define MOTOR_IN2_PORT   gpioPortC
#define MOTOR_IN2_PIN    1
#define MOTOR_PWM_PORT   gpioPortD
#define MOTOR_PWM_PIN    1
#define BUTTON1_PORT     gpioPortB
#define BUTTON1_PIN      9
#define BUTTON2_PORT     gpioPortB
#define BUTTON2_PIN      10

//ULTRASONIC

#define TRIG_PORT   gpioPortD    // Port where the Trigger pin is connected
#define TRIG_PIN    0            // Pin number for Trigger
#define ECHO_PORT   gpioPortD    // Port where the Echo pin is connected
#define ECHO_PIN    2            // Pin number for Echo
#define TIMER_FREQ 14000000      // Timer frequency in Hz
int del =30;

void simpleDelay(int milliseconds) {
    volatile int count;
    while (milliseconds > 0) {
        count = 5000;  // Adjust this value based on your clock speed
        while (count > 0) {
            count--;
        }
        milliseconds--;
    }
}

void initGPIO(void) {
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Set motor control pins as output
    GPIO_PinModeSet(MOTOR_IN1_PORT, MOTOR_IN1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_IN2_PORT, MOTOR_IN2_PIN, gpioModePushPull, 0);

    // Set PWM pin as output
    GPIO_PinModeSet(MOTOR_PWM_PORT, MOTOR_PWM_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BUTTON1_PORT, BUTTON1_PIN, gpioModeInputPull, 1);
    GPIO_PinModeSet(BUTTON2_PORT, BUTTON2_PIN, gpioModeInputPull, 1);
}
void initPWM(uint32_t frequency, uint32_t dutyCycle) {
    CMU_ClockEnable(cmuClock_TIMER0, true);

    // Set up TIMER0 for PWM
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale1;
    TIMER_Init(TIMER0, &timerInit);

    // Calculate the timer top value based on desired frequency
    uint32_t timerTop = CMU_ClockFreqGet(cmuClock_HFPER) / frequency - 1;
    TIMER_TopSet(TIMER0, timerTop);

    // Set the duty cycle (compare value)
    uint32_t compareValue = (timerTop * dutyCycle) / 100;
    TIMER_CompareSet(TIMER0, 0, compareValue);

    // Route TIMER0 CC0 output to the correct location for MOTOR_PWM_PIN
    TIMER0->ROUTE = TIMER_ROUTE_LOCATION_LOC3 | TIMER_ROUTE_CC0PEN;

    // Configure TIMER0 CC0 for PWM mode
    TIMER0->CC[0].CTRL = TIMER_CC_CTRL_MODE_PWM | TIMER_CC_CTRL_CUFOA_SET;

    // Start TIMER0
    TIMER_Enable(TIMER0, true);
}

void startLeftMotor(void) {
    GPIO_PinOutSet(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    GPIO_PinOutClear(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
}

void stopMotor(void) {
    GPIO_PinOutClear(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    GPIO_PinOutClear(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
}



void startRightMotor(void) {
    GPIO_PinOutSet(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    GPIO_PinOutClear(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
}







void delayUs(int us)
{
    uint32_t endValue = us * (TIMER_FREQ / 1000000);
    TIMER2->CNT = 0;
    TIMER2->CMD = TIMER_CMD_START;
    while (TIMER2->CNT < endValue);
    TIMER2->CMD = TIMER_CMD_STOP;
}

void initDelay(void)
{
    CMU_ClockEnable(cmuClock_TIMER2, true); // Enable clock for TIMER2

    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.enable = false;
    timerInit.prescale = timerPrescale1;
    TIMER_Init(TIMER2, &timerInit);
}

uint32_t measureDistance(void) {
    // Send a 10us pulse to the TRIG_PIN
    GPIO_PinOutSet(TRIG_PORT, TRIG_PIN);
    delayUs(10);
    GPIO_PinOutClear(TRIG_PORT, TRIG_PIN);

    // Wait for ECHO_PIN to go high
    while(!(GPIO_PinInGet(ECHO_PORT, ECHO_PIN)));

    // Start timing
    TIMER1->CNT = 0;
    TIMER1->CMD = TIMER_CMD_START;

    // Wait for ECHO_PIN to go low
    while(GPIO_PinInGet(ECHO_PORT, ECHO_PIN));

    // Stop timing
    TIMER1->CMD = TIMER_CMD_STOP;

    uint32_t duration = TIMER1->CNT;

    // Calculate the distance (duration in microseconds, speed of sound is 343 m/s)
    uint32_t distance = (((duration * 343) / 2) / 10000);  // Convert to cm

    return distance;
}
int distance_correct(void){
	int dist1=0;
	for (int i=0; i<10;i++){
		dist1 += measureDistance();
	}
	dist1 /=10;
	return dist1;


}

void setupUltrasonic(void) {
    //Enable clock for GPIO
    CMU_ClockEnable(cmuClock_GPIO, true);   // Enable clock for GPIO
    CMU_ClockEnable(cmuClock_TIMER1, true); // Enable clock for TIMER1

    // Configure Trigger and Echo pins
    GPIO_PinModeSet(TRIG_PORT, TRIG_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(ECHO_PORT, ECHO_PIN, gpioModeInput, 0);

    // Timer initialization structure
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale1;   // Set prescaler to 1
    TIMER_Init(TIMER1, &timerInit);        // Initialize TIMER2 with the specified settings
}


//COMBINITION



int main(void)
{
    CHIP_Init();
    initDelay();
    initGPIO();
    initPWM(940, 33);
    setupUltrasonic();
    SegmentLCD_Init(false); // Initialize the LCD (false: no voltage boost)
    int doorState = 0;  // 0 - closed, 1 - open
    while (1)
    {
    	uint32_t distance = distance_correct();

        // Display the distance on the LCD
        SegmentLCD_Number(distance);

        delayUs(100);

        if (!GPIO_PinInGet(BUTTON1_PORT, BUTTON1_PIN)) {
        	initPWM(940, 35);
        	del=100;
                }

                if (!GPIO_PinInGet(BUTTON2_PORT, BUTTON2_PIN)) {
                	initPWM(740, 60);
                	del=8;
                }

        if (distance >= 11 && distance <= 500 && doorState == 0) {
                           // Object detected and door is closed, open the door
               	startRightMotor();  // Rotate motor to open the door
                           simpleDelay(del);  // Run the motor for 3 seconds
                           stopMotor();  // Stop the motor
                           doorState = 1;  // Update the door state to open
                           simpleDelay(650);  // Run the motor for 3 seconds
                       } else if ((distance < 10 || distance > 500) && doorState == 1)
                    	   {
                           // Object not detected and door is open, close the door
                       	startLeftMotor();  // Rotate motor to close the door
                           simpleDelay(del);  // Run the motor for 3 seconds
                           stopMotor();  // Stop the motor
                           doorState = 0;  // Update the door state to closed
                           simpleDelay(550);  // Run the motor for 3 seconds
                       }else {
                    	   stopMotor();
                    	   //doorState = 0;  // Update the door state to closed
                    	   simpleDelay(400);  // Run the motor for 3 seconds
                       }



        // Delay 100 microseconds before the next measurement

    }
}
//ultrasonicandlcd
//final code with buttons