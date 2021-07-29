#include "msp.h"
#include <driverlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * Srijan Duggal & Kyle Heiss
 * 5/5/2021
 * ME 4405 - Final Project
 * Dr. Hammond
 */

// Data collection array size
#define dataSize 1000

// Volatile setup for reading cube angle
volatile char temp; // holds current UART receive character
volatile char buffer[13] = ""; // holds angle from Arduino's UART comm
volatile int bufferLocation = 0; // current location in UART angle buffer
volatile float angle; // Current angle
volatile float newAngle; // Next angle value
volatile int flag; // Flag indicating whether zero-order hold was used or not
int angleInt = 180; //Holder variable for sscanf

// Controller items
float pComponent;
float iComponent;
float dComponent;
int iterCounter = 0; // Counter for how many iterations have passed
float rpm_per_rads = 1/(2*M_PI);
float volts_per_rpm = 24.0/5250.0;

// Controller specific information
float output; // desired output duty cycle (signed)
float magnitude; // desired duty cycle
float desired_Ang = 0.0; // target angle (radians)
float Kp = 5; // proportional gain
float Kd = 0.3; // derivative gain
float Ki = 0.15; // integral gain
float dt = 1.0/250.0; // seconds

//// Initialize error terms
float error = 0;// this is the error in the current time step
float prev_error = 0; // error in the previous time step (this is needed to calculate the derivative of the error)
float integral = 0; // keeps track of the integral of the error
float derivative = 0; // is used to hold the derivative of the error


// Configure Timer A in Up Mode for 100 Hz (control loop)
const Timer_A_UpModeConfig upConfig_0 =
{    TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_1,
     30000,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
     TIMER_A_DO_CLEAR
};

// Configure Timer A in Up Mode for 500 Hz (motor PWM)
const Timer_A_UpModeConfig upConfig_1 =
{    TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_1,
     6000,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
     TIMER_A_DO_CLEAR
};

// Configure UART for 19200 bps (read angle from Arduino)
const eUSCI_UART_Config UART_init =
{
 EUSCI_A_UART_CLOCKSOURCE_SMCLK,
 9,
 12,
 34,
 EUSCI_A_UART_NO_PARITY,
 EUSCI_A_UART_LSB_FIRST,
 EUSCI_A_UART_ONE_STOP_BIT,
 EUSCI_A_UART_MODE,
 EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

// Saving data
int dataCounter = 0; // Counter for how many datapoints have been saved
float outputArr[dataSize]; // Control output
float angleArr[dataSize]; // Angle (degrees)
float errorArr[dataSize]; // Angle error (radians)
float pArr[dataSize]; // Proportional gain contribution
float iArr[dataSize]; // Integral gain contribution
float dArr[dataSize]; // Derivative gain contribution

// Function declarations
int sign(float);
void initialize_UART();
void initialize_MOTOR();

void main(void)
{

   // Halt Watchdog timer
   WDT_A_holdTimer();

   // System clock Configuration
   CS_setDCOFrequency(3E+6);
   CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

   // Initialize comms and motor
   initialize_UART();
   initialize_MOTOR();

   // Setup timers
   Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_1); // Motor PWM
   Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig_0); // Control loop
   Interrupt_enableInterrupt(INT_TA0_0); // Motor PWM
   Interrupt_enableInterrupt(INT_TA2_0); // Control loop
   Interrupt_setPriority(INT_TA0_0, 1); // Motor PWM
   Interrupt_setPriority(INT_TA2_0, 0); // Control loop (higher priority than PWM)

   // Enable all interrupts
   Interrupt_enableMaster();

   // Start Timers
   Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
   Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);    // Start Timer A

   ///// PWM SETUP /////
   // Set output mode to Reset/Set
   TA0CCTL1 = OUTMOD_7 ;    // TA0.1 PWM output
   P2SEL0 |= 0xF0 ;    // Enable TA0 functionality on P2.4-2.7
   P2SEL1 &= ~0xF0 ;   // Enable TA0 functionality on P2.4-2.7
   P2DIR |= 0xF0 ;     // Set pin 2.4-2.7 as an output pins

   // Set initial PWM outputs to zero
   TA0CCR1 = 0;

   // Do nothing
   while(1) {}
}

// Empty handler for PWM timer
void TA0_0_IRQHandler(void) {}

// The Interrupt Service Routine (ISR) for Timer A
void TA2_0_IRQHandler(void)
{
    // Disable so that PWM interrupt does not occur here
    Interrupt_disableMaster();

    // Increment number of iterations
    iterCounter = iterCounter + 1;

    // Wait 5 seconds before starting controller
    if (iterCounter < 500) {
        // Reset interrupt
        Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);  // Clear the timer flag

        // Enable so that PWM interrupt continues occurring
        Interrupt_enableMaster();

        return;
    }

    //--------------- Controller ----------//
    prev_error = error; // Keep Track of the previous time step's error
    error = desired_Ang - (angle*M_PI/180); // Get the error for this time step
    derivative = (error - prev_error)/dt; // Determine the derivative of the error

    // Anti windup integral
    if (((magnitude > 1) || (magnitude < 0)) && (sign(error) == sign(integral))) {
        integral = integral;
    } else {
        integral = integral + error*dt; // determine integral of error
    }

    // Controller output
    pComponent = Kp*error;
    iComponent = Ki*integral;
    dComponent = Kd*derivative;
    output = pComponent + iComponent + dComponent;

    // Set direction of motor based on controller output
    if (output < 0.0) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //INA 1
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); //INA 2
    } else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); //INA 1
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); //INA 2
    }

    // Get duty cycle based on controller output
    magnitude = fabs(output);

    // Clamp output duty cycle between 0 and 1
    if(magnitude > 1){
        magnitude = 1;
    }


   // Save data
   if (dataCounter < dataSize) {
       angleArr[dataCounter] = angle;
       errorArr[dataCounter] = error;
       pArr[dataCounter] = pComponent;
       iArr[dataCounter] = iComponent;
       dArr[dataCounter] = dComponent;
       outputArr[dataCounter] = output;
       dataCounter++;
   } else {
       // After running for a certain amount of time, print out data
       TA0CCR1 = 0;
       Interrupt_disableMaster();
       int j;
       for (j = 0; j < dataSize; j++) {
           printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", angleArr[j], errorArr[j], pArr[j], iArr[j], dArr[j], outputArr[j]);
       }

       // Stop doing things after printing
       while(1){}
   }


    // Update PWM period.
    TA0CCR1 = (int)(magnitude * 6000);

    // Reset interrupt
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);  // Clear the timer flag

    // Enable so that PWM interrupt continues occurring
    Interrupt_enableMaster();
}


// Sign of input (1 for positive, 0 for negative)
int sign(float input) {
    if (input > 0) return 1;
    else return 0;
}


// UART setup with Arduino and computer
void initialize_UART() {
    Interrupt_disableMaster();

    //Configure additional pins for Arduino communication
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //Configure additional modules 2 for MSP communication
    UART_initModule(EUSCI_A2_BASE, &UART_init);
    UART_enableModule(EUSCI_A2_BASE);
    UART_initModule(EUSCI_A0_BASE, &UART_init);
    UART_enableModule(EUSCI_A0_BASE);

    //Enable interrupts for additional modules
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);

    Interrupt_setPriority(INT_EUSCIA2, 1);

    // Enable interrupts at the NVIC level
    Interrupt_enableMaster();  // No priority was set because this is the only interrupt in this program
}

// UART receive interrupt from Arduino
void EUSCIA2_IRQHandler(void){
    Interrupt_disableMaster();

    // Read from RX buffer
    temp = EUSCI_A_UART_receiveData(EUSCI_A2_BASE);

    //Stores the communicated value in a volatile buffer
    buffer[bufferLocation] = temp;

    // When a newline is received
    if (temp == '\n') {
        char angBuff[6]; //Non-volatile buffer for angle reading
        int i;
        for (i = 0; i < bufferLocation + 1; i++) {
            angBuff[i] = buffer[i]; //Copy over volatile data into non-volatile buffer
            buffer[i] = ""; //Reinitialize data to be null
        }

        // Zero order hold on angle data
        if (angleInt == 180) { // Initial angle value -> store result in angle
            bufferLocation = 0; //Reinititalize
            sscanf(angBuff, "%d", &angleInt); //Convert the character array into an integer
            angle = ((float)(angleInt/100.0)); //Convert integer into float with /100 to have the decimal values
            flag = 0;
        } else {
            bufferLocation = 0; //Reinititalize
            sscanf(angBuff, "%d", &angleInt); //Convert the character array into an integer
            newAngle = ((float)(angleInt/100.0)); //Convert integer into float with /100 to have the decimal values

            // If change in angle is wildly large, don't update the angle
            if (fabs(angle) / fabs(newAngle) > 9 ) {
                flag = 1;
            } else {
                angle = newAngle;
                flag = 0;
            }
        }
    } else {
        bufferLocation++; //Increment buffer counter
    }
    Interrupt_enableMaster();
}

// Initialize motor driver GPIO pins
void initialize_MOTOR() {
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);

    //INA1
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    //INA2
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
}
