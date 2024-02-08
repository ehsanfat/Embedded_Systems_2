/*
  |-----------------------------Embedded System --------------------------------|

  
   |-------------Buggy Car Embedded Systems Implementation Overview-------------|

        In the second assignment of our  embedded systems  course, our focus was on integrating
        modules from previous chapters to develop the software block  diagram for the Buggy Car
        robot. Throughout the project, we encountered  various  challenges  that  significantly  
        contributed to our  understanding  of hardware aspects  of programming, particularly in
        dealing with systems having limited resources.
 */

//------------------------------------------------------------------------------//
//---------------------------      C Libraries       ---------------------------//
//------------------------------------------------------------------------------//
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "parser.h"
#include <p33EP512MU810.h>

//------------------------------------------------------------------------------//
//---------------------------        Macros          ---------------------------//
//------------------------------------------------------------------------------//

// P33EP512MU810 board's Push Buttons & LEDs
#define LED_PIN LATAbits.LATA0
#define BUTTON_PIN1 PORTEbits.RE8
#define BUTTON_PIN2 PORTEbits.RE9

//Buggy Lights
#define LIGHT_LEFT          LATBbits.LATB8   
#define LIGHT_RIGHT         LATFbits.LATF1  
#define LIGHT_BREAK         LATFbits.LATF0  
#define LIGHT_LOW_INTENSITY LATGbits.LATG1 
#define LIGHT_BEAM          LATAbits.LATA7  

//Timers & Oscillators
#define TIMER1 1
#define TIMER2 2

//PWM's Constants 
#define PWM_LEFT     0
#define PWM_RIGHT    1
#define PWM_FREQ   10000
#define MAX_PWM     100

//Robot's State Machine (Lighting)
#define STATE_LIGHTING_WAIT_START                  1
#define STATE_LIGHTING_MOVING_FORWARD              2
#define STATE_LIGHTING_MOVING_SLOW                 3
#define STATE_LIGHTING_MOVING_TURN_CLOCKWISE       4
#define STATE_LIGHTING_MOVING_TURN_ANTI_CLOCKWISE  5
#define STATE_LIGHTING_MOVING_NO_TURNING           6

//Robot's State Machine (Waiting-Moving)
#define STATE_ROBOT_WATING                         7
#define STATE_ROBOT_MOVING                         8

#define STATE_ROBOT_MOVING_STOP                    9
#define STATE_ROBOT_MOVING_TURN                    10
#define STATE_ROBOT_MOVING_FORWARD                 11
#define STATE_ROBOT_MOVING_MIXED                   12


//------------------------------------------------------------------------------//
//------------------------    Variables and Structures  ------------------------//
//------------------------------------------------------------------------------//

//Robot's States
int robot_state = STATE_ROBOT_WATING;
int robot_moving_state = STATE_ROBOT_MOVING_STOP;
int robot_lighting_state = STATE_LIGHTING_WAIT_START;

//Signals
double surge, yaw_rate;

//PWMs
double left_pwm;
double right_pwm;
double pwmValues[4];

//Distance Sensor
double th_min = 20;
double th_max = 80;
double distance = 0;
double smoothedDistance = 0;
double sum = 0;

uint8_t index_sensor_buffer = 0;
double sensorBuffer[10];

//Scheduler
#define MAX_TASK  9

typedef struct{
int n;
int N;
} heartbeat;
heartbeat schedInfo[MAX_TASK];

//Circular Buffer
#define BUFF_SIZE 45

typedef struct {
    char buff[BUFF_SIZE];
    int readIdx;
    int writeIdx;
    int count;
} CircBuff;

CircBuff cirBufRx;
CircBuff cirBufTx;

//Parser
int ret;
parser_state pstate;

//------------------------------------------------------------------------------//
//---------------      Declarations - Low Level APIs     -----------------------//
//------------------------------------------------------------------------------//

//Initializing
void initDevices();

//Peripherals 
void peripheralConfig();

//Timers
void tmr_start(int timer, int ms);
void tmr_period(int timer);
void tmr_oneShot(int timer, int ms);
void tmr_stop(int timer);

//Interrupts
void interruptConfig();

//UARTs
void uartConfig();
void bufferInit(CircBuff* buff);
void bufferWrite(CircBuff *buff, char data);
char bufferRead(CircBuff *buff);
int checkAvailableBytes(CircBuff* buff);

//ADC
void adcConfig();

//PWM
void pwmConfig();

//------------------------------------------------------------------------------//
//---------------     Declarations - High Level APIs     -----------------------//
//------------------------------------------------------------------------------//
void nextRobotState();
void robotWaitingState();
void robotMoveingState();
void lightManager(int lightState);
void applyTorque(int pwm_side, double torque_motor_front , double torque_motor_back);
void scheduler();

void measureDistance();  //Task 1
void updaeSURGE_YAW();   //Task 2
void updatePWMs();       //Task 3
void updateLigths();     //Task 4
void reportBattery();    //Task 5
void reportDistance();   //Task 6
void reportPWMs();       //Task 7
void sentData();         //Task 8
void setParameters();    //Task 9

//------------------------------------------------------------------------------//
//---------------      Definitions - Low Level APIs     ------------------------//
//------------------------------------------------------------------------------//

//Initializing
void initDevices(){
    //All Analog Pins Disabled
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    peripheralConfig();      //Peripherals Configuration
    interruptConfig();       //Interrupt Configuration
    uartConfig();            //UART Configurations
    pwmConfig();             //PWM Configurations
    adcConfig();             //ADC Configurations
    bufferInit(&cirBufRx);   //Rx Buffer Configurations
    bufferInit(&cirBufTx);   //Tx Buffer Configurations
}

//Peripherals Configuration         
void peripheralConfig(){
   TRISAbits.TRISA0 = 0;  //Output for LED
   TRISEbits.TRISE8 = 1;  //Input for Button1
   TRISEbits.TRISE9 = 1;  //Input for Button2
   
    TRISBbits.TRISB8 = 0; //Left side lights
    TRISFbits.TRISF1 = 0; //Right-side lights 
    TRISFbits.TRISF0 = 0; //Brakes lights
    TRISGbits.TRISG1 = 0; //Low intensity lights
    TRISAbits.TRISA7 = 0; //Beam headlights
}

//Interrupt Configuration    
void interruptConfig(){
    RPINR0bits.INT1R = 0x58;  //RE8(RPI88):0x58, RE9(RPI89):0x59
    INTCON2bits.GIE = 1;      //Set global interrupt enable 
    INTCON2bits.INT1EP = 1;   //Interrupt on negative edge
    IFS1bits.INT1IF = 0;      //Clear interrupt flag
    IEC1bits.INT1IE = 1;      //Enable interrupt
}

//Interrupt Service Routine: External Interrupt Request(INT1)
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt(){
    IEC1bits.INT1IE = 0;     //Disable the interrupt 
    IFS1bits.INT1IF = 0;     //Reset the interrupt flag   
    
    IFS0bits.T2IF = 0;      //Clear Timer2 interrupt flag
    IEC0bits.T2IE = 1;      //Enable Timer2 interrupt
    tmr_start(TIMER2, 15);  //Start counting in timer 2
}

//Interrupt Service Routine: Timer 2 (T2)
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    tmr_stop(TIMER2);   //Disable the timer2 
    IFS0bits.T2IF = 0;  //Clear the timer2 flag
    IEC0bits.T2IE = 0; //Disable the timer2 interrupt
            
    //Check the RE8 button status
    if(PORTEbits.RE8 == 0){
        //Change the robot's state based on the button status (A simple task in the ISR-Based on Assignment_1_Discussion)
        if(robot_state == STATE_ROBOT_WATING){
            robot_state =  STATE_ROBOT_MOVING; //Set the state to "MOVING"
        }else if (robot_state == STATE_ROBOT_MOVING){
            robot_state =  STATE_ROBOT_WATING; //Set the state to "WATING"
        }  
    }
    
    IFS1bits.INT1IF = 0; //First reset the interrupt flag (To avoid potential issue related to the priority of interrupts-Based on Assignment_1_Discussion)
    IEC1bits.INT1IE = 1; //Then enabling the interrupt 
}

//Interrupt Service Routine: UART 2-Receiver (URRX)
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;  //Reset the interrupt flag 
    bufferWrite(&cirBufRx, U2RXREG);  //Write the data in the circular buffer
}

//Timer Configuration and Activation    
void  tmr_start(int timer, int ms)
{   
    int tcount =((72000000/256)/1000.0)*ms - 1; // fill the PR1 register with the proper number of clocks
    
    //Setup Timer 1
    if(timer == TIMER1)
    {
        TMR1 = 0;               // reset timer counter
        PR1 = tcount;
        
        T1CONbits.TCS = 0;      // select the internal clock source
        T1CONbits.TCKPS = 0b11; // set the prescaler to 256 
        T1CONbits.TON = 1;      // starts the timer! 
    //Setup Timer 2    
    }else if(timer == TIMER2)
    {
        TMR2 = 0;                // reset timer counter
        PR2 = tcount;
        
        T2CONbits.TCS = 0;      // select the internal clock source
        T2CONbits.TCKPS = 0b11; // set the prescaler to 256
        T2CONbits.TON = 1;      // starts the timer!       
    } 
}

//Waits Periodically
void tmr_period(int timer)
{
    //Check the status of Timer 1
    if(timer == TIMER1)
    {
         while(!IFS0bits.T1IF); 
         IFS0bits.T1IF = 0; 
    }
    
    //Check the status of Timer 2
    if(timer == TIMER2)
    {
         while(!IFS0bits.T2IF); 
         IFS0bits.T2IF = 0;
    }   
}

//Waits Only 1-Time (Not Periodic))
void tmr_oneShot(int timer, int ms)
{
    tmr_start(timer, ms); //Start the timer
    tmr_period(timer);    //Check the timer status
    tmr_stop(timer);      //Stop the timer
}

//Stop the timer
void tmr_stop(int timer){
    //Stop timer 1
    if(timer == TIMER1){
       T1CONbits.TON = 0;  // stop the timer!
    //Stop timer 2   
    }else if (timer == TIMER2){
       T2CONbits.TON = 0;  // stop the timer!
    }  
}

//UART Configuration       
void uartConfig() {
    
    //Remap UART2 pins
    RPOR0bits.RP64R = 0x03;
    RPINR19bits.U2RXR = 0x4B;
    
    //Configure the Baud Rate
    const int baund = 9600;
    U2BRG = (144000000 / 2) / (16L * baund) - 1; // = 11
    
    //Enable the UART
    U2MODEbits.UARTEN = 1; //Enable UART2
    U2STAbits.UTXEN = 1;   //Enable U2TX (must be after UARTEN)
    
    //Enable the UART's Interrupt Receiver 
    IEC1bits.U2RXIE = 1;
    U2STAbits.URXISEL = 1;  //Set the UART2 interrupt mode to interrupt for every char received
}

//Initialize the Circular Buffer
void bufferInit(CircBuff* buff) {
    buff->readIdx = 0;
    buff->writeIdx = 0;
    buff->count = 0;
}

//Write a Character in the Circular Buffer
void bufferWrite(CircBuff *buff, char data) {
    // Check write allowed or not
    if( buff->count<BUFF_SIZE){
        buff->buff[buff->writeIdx] = data;
        buff->writeIdx = (buff->writeIdx + 1) % BUFF_SIZE;
        buff->count++;
    }else{
        // Buffer is full and ignore the data
    }
}

//Read a Character From the Circular Buffer
char bufferRead(CircBuff *buff) {
    char data = buff->buff[buff->readIdx];
    buff->readIdx = (buff->readIdx + 1) % BUFF_SIZE;
    buff->count--;
    return data;
}

//ADC Configuration          
void adcConfig() {
    //Configure ADC for the IR Sensor on the the Analog Pin AN14
    TRISBbits.TRISB14 = 1;
    ANSELBbits.ANSB14 = 1;
    
    //Configure the Battery Sensing on the Analog PIn AN11
    TRISBbits.TRISB11 = 1;
    ANSELBbits.ANSB11 = 1;
    
    //Setup the ADC
    AD1CON3bits.ADCS = 14;  //14*T_CY
    AD1CON1bits.ASAM = 1;   //Automatic sampling start
    AD1CON1bits.SSRC = 7;   //Automatic conversion
    AD1CON3bits.SAMC = 16;  //Sampling lasts 16 Tad
    AD1CON2bits.CHPS = 0;   //Use CH0 2-channels sequential sampling mode
    AD1CON1bits.SIMSAM = 0; //Sequential sampling

	//Setup the Scan Mode 
	AD1CON2bits.CSCNA = 1;  //Scan mode enabled
    AD1CSSLbits.CSS11 = 1;  //Scan for AN11 battery
    AD1CSSLbits.CSS14 = 1;  //Scan for AN14 IR sensor
	AD1CON2bits.SMPI = 1;   //N-1 channels

    //Enable the ADC
    AD1CON1bits.ADON = 1; //Turn on ADC

    //Activate the IR Distance Sensor
    TRISBbits.TRISB9 = 0;
    LATBbits.LATB9 = 1; 
}

//PWM Configuration         
void pwmConfig(){
    //Configure the pins D1, D2, D3, D4 as PWM Pins
     TRISDbits.TRISD1 = 0;
     TRISDbits.TRISD2 = 0;
     TRISDbits.TRISD3 = 0;
     TRISDbits.TRISD4 = 0;
     
     //Remap the pins
     RPOR0bits.RP65R = 0b010000;   //OC1
     RPOR1bits.RP66R = 0b010001;   //OC2
     RPOR1bits.RP67R = 0b010010;   //OC3
     RPOR2bits.RP68R = 0b010011;   //OC4
     
    //Configure the Left Wheels
    //Clear all the contents of two control registers
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC1CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC1CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC1CON2bits.SYNCSEL = 0x1F; 
    
    //Clear all the contents of two control registers
    OC2CON1 = 0x0000;
    OC2CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC2CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC2CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC2CON2bits.SYNCSEL = 0x1F; 
    
    //Configure the Right Wheels
    //Clear all the contents of two control registers
    OC3CON1 = 0x0000;
    OC3CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC3CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC3CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC3CON2bits.SYNCSEL = 0x1F; 
    
    //Clear all the contents of two control registers
    OC4CON1 = 0x0000;
    OC4CON2 = 0x0000;
    //Set the peripheral clock as source for the OCx module
    OC4CON1bits.OCTSEL = 0b111;
    //Sets the OC modality to Edge-Aligned PWM mode
    OC4CON1bits.OCM = 0b110;
    //Sets the synchronization source for the OCx module to No Sync
    OC4CON2bits.SYNCSEL = 0x1F; 
}

//------------------------------------------------------------------------------//
//---------------     Definitions - High Level APIs     ------------------------//
//------------------------------------------------------------------------------//

//Task Scheduling
void scheduler(){
    int i;
    for(i=0; i<MAX_TASK;i++){
        schedInfo[i].n++;
        if(schedInfo[i].n >= schedInfo[i].N){
            switch(i){
                case 0:
                    //Task 1 (1K Hz): Measuring the Distance of Buggy from Obstacles by theIR Sensor
                    measureDistance();
                    break;
                case 1:
                    //Task 2 (1K Hz): Updating the Robot's State based on the Measured Distance and Computing the SURGE & Yaw Signals
                    nextRobotState();
                    break;
                case 2:
                    //Task 3 (1K Hz): Computing the Duty-Cycle of PWMs Signals for Driving the Wheels
                    updatePWMs();
                    break;
                case 3:
                    //Task 4 (1 Hz): Updating the Buggy's Lights based on the Robot's State
                    updateLigths();
                    break;
                case 4:
                    //Task 5 (1 Hz): Sampling the Battery and Writing the Data in the Circular Buffer
                    reportBattery();
                    break;
                case 5:
                    //Task 6 (10 Hz): Sampling the Distance and Writing the Data in the Circular Buffer
                    reportDistance();
                    break;
                case 6:
                    //Task 7 (10 Hz): Sampling the PWMs and Writing the Data in the Circular Buffer
                    reportPWMs();
                    break;
                case 7:
                    //Task 8 (500 Hz): Sending the Report (Data Logging)
                    sentData();
                    break;
                case 8:
                    //Task 9 (1K Hz): Adjusting the Threshold Values
                    setParameters(); 
                    break;
            }
            schedInfo[i].n=0;
        }
    }
}

//Task 1 (1K Hz): Measuring the Distance of Buggy from Obstacles by theIR Sensor
void measureDistance(){
    while(!AD1CON1bits.DONE);
    
    //Read from the IR sensor
    double v = ADC1BUF1;
    v = (v/1023)*3.3;
       
    //Convert the sensor's data into distance 
    distance = 100 *(2.34 - 4.74*v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v);
    
    //Subtract the oldest value from the sum
    sum -= sensorBuffer[index_sensor_buffer];

    //Add the new value to the sum
    sum += distance;

    //Store the new value in the circular buffer
    sensorBuffer[index_sensor_buffer] = distance;

    //Increment the index for the next iteration
    index_sensor_buffer = (index_sensor_buffer + 1) % 10;

    //Calculate and return the moving average
    smoothedDistance  =  sum / 10;
} 

//Task 2 (1K Hz): Updating the Robot's State based on the Measured Distance and Computing the SURGE & Yaw Signals
void nextRobotState(){
    switch(robot_state){
        case STATE_ROBOT_WATING:
            robotWaitingState(); //Configure the robot for 'WAITING' State
            break;
        case STATE_ROBOT_MOVING:
            robotMoveingState(); //Configure the robot for 'MOVING' State
            break;
    }
}

//'WAITING' State Operations
void robotWaitingState(){
    updaeSURGE_YAW();
}

//'MOVING' State Operations
void robotMoveingState(){
    //Select the Type of 'MOVING' State
    if(smoothedDistance < th_min){
        robot_moving_state = STATE_ROBOT_MOVING_TURN;
    }else if(smoothedDistance > th_max){
        robot_moving_state = STATE_ROBOT_MOVING_FORWARD;
    }else{
        robot_moving_state = STATE_ROBOT_MOVING_MIXED;
    } 
    
    //Compute the SURGE and YAW Signals
    updaeSURGE_YAW();
}

//Computing the SURGE & Yaw Signals
void updaeSURGE_YAW(){ 
    if(robot_state == STATE_ROBOT_WATING){
        surge = 0;
        yaw_rate = 0;
    }else if(robot_state == STATE_ROBOT_MOVING){
        switch(robot_moving_state){
            case STATE_ROBOT_MOVING_FORWARD:
                //Forward motion
                surge = MAX_PWM;
                yaw_rate = 0;
                break;
            case STATE_ROBOT_MOVING_TURN:
                //Clockwise rotation on the spot
                surge = 0;
                yaw_rate = MAX_PWM;
                break;
            case STATE_ROBOT_MOVING_MIXED:
                //Implement a proportional law to compute the surge and yaw signals
                surge = (smoothedDistance - th_min) / (th_max - th_min) * MAX_PWM;
                yaw_rate = MAX_PWM - surge;  // Yaw_rate decreases as surge increases
                break;
        }
    }
}  

//Task 3 (1K Hz): Computing the Duty-Cycle of PWMs Signals for Driving the Wheels
void updatePWMs(){
    
    //Motor control logic
    left_pwm = surge + yaw_rate;
    right_pwm = surge - yaw_rate;
    
    // Scale PWM values if they exceed MAX_PWM
    int max_val = (abs(left_pwm) > abs(right_pwm)) ? abs(left_pwm) : abs(right_pwm);
    if (max_val > MAX_PWM) {
        left_pwm = left_pwm * MAX_PWM / max_val;
        right_pwm = right_pwm * MAX_PWM / max_val;
    }
    
    if(left_pwm>0){
        //Left wheels forward (left_pwm > 0)
        applyTorque(PWM_LEFT, 0, left_pwm);       
    }else{
        //Left wheels backward (left_pwm < 0)
        applyTorque(PWM_LEFT, -left_pwm, 0);
    }
    
    if(right_pwm>0){
        //Right wheels forward (right_pwm > 0)
        applyTorque(PWM_RIGHT, 0, right_pwm);
    }else{
        //Right wheels backward (right_pwm < 0)
        applyTorque(PWM_RIGHT, -right_pwm, 0);
    }
}   

//Driving the Wheels by Applying the PWM Signals
void applyTorque(int pwm_side, double torque_motor_front , double torque_motor_back){
    if(pwm_side == PWM_LEFT){
        OC1RS = 72000000/PWM_FREQ; //Set the PWM frequency 
        OC1R = (int)((72000000/PWM_FREQ) * (torque_motor_front/100.0)); //Set the PWM Duty Cycle
        
        OC2RS = 72000000/PWM_FREQ; //Set the PWM frequency 
        OC2R = (int)((72000000/PWM_FREQ) * (torque_motor_back/100.0)); //Set the PWM Duty Cycle
        
        pwmValues[0] = torque_motor_front; //Set the PWM Duty Cycle
        pwmValues[1] = torque_motor_back; //Set the PWM Cycle
    }else if(pwm_side == PWM_RIGHT){
        OC3RS = 72000000/PWM_FREQ; //Set the PWM frequency 
        OC3R = (int)((72000000/PWM_FREQ) * (torque_motor_front/100.0)); //Set the PWM Duty Cycle
        
        OC4RS = 72000000/PWM_FREQ; //Set the PWM frequency 
        OC4R = (int)((72000000/PWM_FREQ) * (torque_motor_back/100.0)); //Set the PWM Duty Cycle
        pwmValues[2] = torque_motor_front; //Set the PWM Cycle
        pwmValues[3] = torque_motor_back; //Set the PWM Cycle
    }
}

//Task 4 (1 Hz): Updating the Buggy's Lights based on the Robot's State
void updateLigths(){
    
    LED_PIN = !LED_PIN;
    
    if(robot_state == STATE_ROBOT_WATING){
        //Lighting in the 'WATINH' State:
        lightManager(STATE_LIGHTING_WAIT_START);
    }else if(robot_state == STATE_ROBOT_MOVING){
        //Lighting in the 'MOVING' State:
        
        //Update lights based on the SURGE value
        if(surge>50){
            lightManager(STATE_LIGHTING_MOVING_FORWARD);
        }else{
            lightManager(STATE_LIGHTING_MOVING_SLOW);
        }
        
        //Update lights based on the YAW value
        if(yaw_rate>15){
           lightManager(STATE_LIGHTING_MOVING_TURN_CLOCKWISE);
        }else{
            lightManager(STATE_LIGHTING_MOVING_NO_TURNING);
        } 
    }
}     

//Managing Lights based on the Robot's State
void lightManager(int lightState){
    switch(lightState){
    
        case STATE_LIGHTING_WAIT_START:
            LIGHT_LEFT = !LIGHT_LEFT;
            LIGHT_RIGHT = !LIGHT_RIGHT;
            LIGHT_BREAK = 0;
            LIGHT_LOW_INTENSITY = 0;
            LIGHT_BEAM = 0;
            break;
        case STATE_LIGHTING_MOVING_FORWARD:
            LIGHT_BREAK = 0;
            LIGHT_LOW_INTENSITY = 0;
            LIGHT_BEAM = 1;
            break;
        case STATE_LIGHTING_MOVING_SLOW:
            LIGHT_BREAK = 1;
            LIGHT_LOW_INTENSITY = 1;
            LIGHT_BEAM = 0;
            break;
        case STATE_LIGHTING_MOVING_TURN_CLOCKWISE:
            LIGHT_LEFT = 0;
            LIGHT_RIGHT = !LIGHT_RIGHT;
            break;
        case STATE_LIGHTING_MOVING_TURN_ANTI_CLOCKWISE:
            LIGHT_LEFT = !LIGHT_LEFT;
            LIGHT_RIGHT = 0;
            break; 
        case STATE_LIGHTING_MOVING_NO_TURNING:
            LIGHT_LEFT = 0;
            LIGHT_RIGHT = 0;
    }
}

 //Task 5 (1 Hz): Sampling the Battery and Writing the Data in the Circular Buffer
void reportBattery(){
    while (!AD1CON1bits.DONE);
    double v = ADC1BUF0;    //Read data from ADC
    v = (v / 1023.0) * 3.3; //Convert the value into volt
    
    //The voltage divider circuit
    double R1 = 200;
    double R2 = 100;
    v = ((R1 + R2)/R2)*v; //The voltage of the battery node
    
    char buff[16];
    sprintf(buff, "$MBATT,%.2f*\n", v);
    
    //Write data in the circular buffer
    for (int i = 0; i < strlen(buff); i++){
        bufferWrite(&cirBufTx, buff[i]);
    }
}  

//Task 6 (10 Hz): Sampling the Distance and Writing the Data in the Circular Buffer
void reportDistance(){
//Write data in the circular buffer
    char buff[16];
    sprintf(buff, "$MDIST,%d*\n", (int)smoothedDistance);
    
    //Write data in the circular buffer
    for (int i = 0; i < strlen(buff); i++){
        bufferWrite(&cirBufTx, buff[i]);
    }
}  

 //Task 7 (10 Hz): Sampling the PWMs and Writing the Data in the Circular Buffer
void reportPWMs(){
//Write data in the circular buffer
    char buff[16];
    sprintf(buff, "$MPWM,%d,%d,%d,%d*\n", (int)pwmValues[0], (int)pwmValues[1], (int)pwmValues[2], (int)pwmValues[3]);
    
    //Write data in the circular buffer
    for (int i = 0; i < strlen(buff); i++){
        bufferWrite(&cirBufTx, buff[i]);
    }
}  

 //Task 8 (500 Hz): Sending the Report (Data Logging)
void sentData(){
    if(checkAvailableBytes(&cirBufTx)){
         char data = bufferRead(&cirBufTx);
         while (U2STAbits.UTXBF); //We used UTXBF bit instead of TMRT bit to avoid waiting for sending every character-Based 
         U2TXREG = data;
    }
}

 //Task 9 (1K Hz): Adjusting the Threshold Values
void setParameters(){
    while (checkAvailableBytes(&cirBufRx) > 0) {
        ret = parse_byte(&pstate, bufferRead(&cirBufRx));
        if (ret == NEW_MESSAGE) {
            if (strcmp(pstate.msg_type, "PCTH") == 0){
                parse_pcth(pstate.msg_payload, &th_min, &th_max);
            }
        }
    }
}

//Checking the Availability of Data in the Circular Buffer
int checkAvailableBytes(CircBuff* buff) {
    if (buff->readIdx <= buff->writeIdx)
        return buff->writeIdx - buff->readIdx;
    else
        return BUFF_SIZE - buff->readIdx + buff->writeIdx;
}

//Parsing the Received Data
int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}

//Update the Thresholds Values
void parse_pcth(const char* msg, double* minth, double* maxth) {
    int i = 0;

    *minth = extract_integer(msg);
    i = next_value(msg, i);
    *maxth = extract_integer(msg+i);
}

//Extract the Sign and Numerical Parts of the Payload 
int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;

    if (str[i] == '-') {
        sign = -1;  
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }

    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; //Multiply the current number by 10;
        number += str[i] - '0'; //Converting character to decimal number
        i++;
    }
    
    return sign*number;
}

//Find the Index of the Numerical Data in the Message
int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') i++;
    if (msg[i] == ',') i++;
    return i;
}

//------------------------------------------------------------------------------//
//---------------            Main Control Loop          ------------------------//
//------------------------------------------------------------------------------//

//User Application
int main(void) {
    
 //Hardware APIs Configurations
 initDevices();
 
 //Set the Timing for the Task Scheduler 
 schedInfo[0].N = 1;    //Task 1: Measuring the Distance
 schedInfo[1].N = 1;    //Task 2: Computing the SURGE & Yaw Signals
 schedInfo[2].N = 1;    //Task 3: Computing the Duty-Cycle of PWMs Signals
 schedInfo[3].N = 1000; //Task 4: Updating the Buggy's Lights
 schedInfo[4].N = 1000; //Task 5: Sampling the Battery
 schedInfo[5].N = 100;  //Task 6: Sampling the Distance
 schedInfo[6].N = 100;  //Task 7: Sampling the PWMs
 schedInfo[7].N = 1;    //Task 8: Sending the Report
 schedInfo[8].N = 2;    //Task 9: Adjusting the Threshold Values
 
 //Initializing the Parser
pstate.state = STATE_DOLLAR;
pstate.index_type = 0; 
pstate.index_payload = 0;

//Setup the Frequency of the Main Control Loop (1K Hz)
tmr_start(TIMER1, 1);

//Main Control Loop
 while(1){
     //Call the Scheduler for Checking the Next Task to Run
     scheduler();
     //Adjust the Frequency of the Main Control Loop (1K Hz)
     tmr_period(TIMER1);
 }
    return 0;
}