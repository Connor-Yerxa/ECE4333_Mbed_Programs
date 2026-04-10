#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <ratio>

#include <chrono>
using namespace std::chrono;

Mutex mPg;

void clampfloat(float * x, float max)
{
    if(*x > max) *x = max;
    if(*x < -1*max) *x = -1 * max;
}

void clampint(int * x, int max)
{
    if(*x > max) *x = max;
    if(*x < -1*max) *x = -1 * max;
}

// Function prototypes
void PiControlThread(void const *argument);
void PeriodicInterruptISR(void);
void ExtCollisionThread(void const *argument);
void ExtInterruptISR(void);
void WatchdogThread(void const *argument);
void WatchdogISR(void const *n);

void update_Speeds_to_line(
    float * leftVel, 
    float * rightVel, 
    uint16_t us_front, 
    uint16_t us_right, 
    uint16_t us_front_right, 
    uint16_t us_right_front,
    float ideal_distance
);
float us2mm(uint16_t us);
void applyPI(PwmOut * pwm, float * currentVel, float idealVel, int16_t dP, int16_t dT, float * integration, DigitalOut * MDIR);
void read_serial(UnbufferedSerial * serial);


// Processes and threads
osThreadId WatchdogId, ExtCollisionId, PiControlId;     // Thread ID's
osThreadDef(PiControlThread, osPriorityRealtime, 1024); // Declare PiControlThread as a thread
osThreadDef(ExtCollisionThread, osPriorityHigh, 1024);  // Declare ExtCollisionThread as a thread
osThreadDef(WatchdogThread, osPriorityRealtime, 1024);  // Declare WatchdogThread as a thread
osTimerDef(Wdtimer, WatchdogISR);   

// IO Port Configuration
DigitalOut MDIR0(PF_14);
DigitalOut MBRAKE0(PG_1);
PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0
DigitalOut MDIR1(PF_15);
DigitalOut MBRAKE1(PG_14);
PwmOut pwm1(PE_11); // PE_9 at CN10 configured as PWM out with alias pwm0

DigitalOut IoReset(PG_2);
DigitalOut SpiReset(PG_3);                    // Declare a watchdog timer

static UnbufferedSerial bt(PB_13,PB_12);

//Variables
bool dir0=0, brake0=0;
bool dir1=0, brake1=0;

uint8_t ID;

int period=500;
int maxDuty = 0.4*period;
int duty0=0, duty1=0;


Ticker PeriodicInt;

//PI controller vars
float kp = 20, ki = 5, distance_kp=0.004;
float currentVel0=0, currentVel1=0;
float offsetVel = 1.5, maxVel=2;
float idealVel0 = 0; // Left
float idealVel1 = 0; // Right
float ideal_distance = 300;
float sensor_rig_radius = 90; // mm
bool run_Wall_Following = false;
float filtered_error=0;
float new_error_weight = 1;
// float e0;
int stepsPerRotation = 1216;
int16_t dP0=0, dT0=1, dP1=0, dT1=1;

//COMs
UnbufferedSerial pc(USBTX, USBRX);
SPI FPGA(PB_5, PB_4, PB_3);

uint16_t USR0, USR1, USR2, USR3;

void ResetFPGA()
{
    IoReset = 0; 
    wait_us(5); 
    IoReset = 1;
    wait_us(5);
    IoReset = 0;
} 

void ResetFPGA_SPI()
{
    SpiReset = 0;
    SpiReset = 1;
    wait_us(5);
    SpiReset = 0;
}

void init()
{
    // Start execution of: PeriodicInterruptThread with ID, PeriodicInterruptId:
    PiControlId = osThreadCreate(osThread(PiControlThread), NULL);
    // Start periodic interrupt generaion, specifying the period, and address of the isr.
    PeriodicInt.attach(&PeriodicInterruptISR, 50ms);

    FPGA.format(16,1); // SPI format: 16-bit words, mode 1
    FPGA.frequency(500000);

    bt.baud(9600);
    pc.baud(115200);

    ResetFPGA();
    ResetFPGA_SPI();
    
    ID = FPGA.write(0x8008); // ID of SPI slave is returned as 0x0017

    pwm0.period_us(period); // This sets the PWM period to 500 us.

    pwm0.pulsewidth_us(duty0);
    MDIR0 = dir0;
    MBRAKE0 = brake0;
    
    pwm1.period_us(period); // This sets the PWM period to 500 us.

    pwm1.pulsewidth_us(duty1);
    MDIR1 = dir1;
    MBRAKE1 = brake1;

    int8_t Dummy;
    dP0 = FPGA.write(Dummy); // Read QEI-0 position register 
    dT0 = FPGA.write(Dummy); // Read QEI-0 time interval
    dP1 = FPGA.write(Dummy); // Read QEI-1 position register 
    dT1 = FPGA.write(Dummy); // Read QEI-1 time interval

    USR0 = FPGA.write(Dummy);
    USR1 = FPGA.write(Dummy);
    USR2 = FPGA.write(Dummy);
    USR3 = FPGA.write(Dummy);
}

// main() runs in its own thread in the OS
int main()
{
    init();
    const milliseconds printInterval(500);
    Kernel::Clock::time_point lastPrint = Kernel::Clock::now();

    while (true) {
        float cvel, idvel, err, cvel1, idvel1;
        uint16_t u0, u1, u2, u3;
        Kernel::Clock::time_point now = Kernel::Clock::now();
        if(now - lastPrint > printInterval)
        {
            // char c = '!';
            // bt.write(&c, 1);
            lastPrint = now;
            mPg.lock();
            // cvel = currentVel0;
            idvel = idealVel0;
            // cvel1 = currentVel1;
            idvel1 = idealVel1;
            u0 = USR0;
            u1 = USR1;
            u2 = USR2;
            u3 = USR3;
            mPg.unlock();
            // printf("\nC0: %d I0: %d\n", (int)(cvel * 100), (int)(idvel * 100));
            // printf("C1: %d I1: %d\n", (int)(cvel1 * 100), (int)(idvel1 * 100));
            // printf("%d %d\n", (int)(ki*10), (int)(kp*10));

            // printf("%d %d %d %d\n", u0, u1, u2, u3);
            printf("%d %d\n", (int)(idvel * 100), (int)(idvel1 * 100));
        }

        read_serial(&bt);
    }
}

// ******** Periodic Timer Interrupt Thread ********
void PiControlThread(void const *argument) {
    // int side=0, newSide=0;
    float integration0=0, integration1=0;

    // float posDeg=0;
    // int currentPosition=0;
    uint8_t Dummy=0;

    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalPi, is received.

        // May be executed in a loop or periodically
        dP0 = FPGA.write(Dummy); // Read QEI-0 position register 
        dT0 = FPGA.write(Dummy); // Read QEI-0 time interval
        dP1 = FPGA.write(Dummy); // Read QEI-1 position register 
        dT1 = FPGA.write(Dummy); // Read QEI-1 time interval

        USR0 = FPGA.write(Dummy);
        USR1 = FPGA.write(Dummy);
        USR2 = FPGA.write(Dummy);
        USR3 = FPGA.write(Dummy);

        // printf("%d %d %d %d\n", dP0, dT0, dP1, dT1);

        if(run_Wall_Following)
        {
            update_Speeds_to_line(&idealVel0, &idealVel1, USR0, USR3, USR1, USR2, ideal_distance);
        }
        
        applyPI(&pwm0, &currentVel0, idealVel0, dP0, dT0, &integration0, &MDIR0);
        applyPI(&pwm1, &currentVel1, idealVel1, dP1, dT1, &integration1, &MDIR1);
    } 
}

// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
    osSignalSet(PiControlId,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}

void applyPI(PwmOut * pwm, float * currentVel, float idealVel, int16_t dP, int16_t dT, float * integration, DigitalOut * MDIR)
{
    int duty;
    mPg.lock();
    *currentVel = 1000000 * (float)dP/((float)dT*(float)stepsPerRotation*10.24); // in rps
    mPg.unlock();
    if(idealVel != 0)
    {
        float e = idealVel - *currentVel;
        // printf("%d\n", (int)(e*100));

        *integration = (*integration + e);
        float propotion = kp * e;

        
        if(*integration > maxDuty) *integration = maxDuty;
        if(*integration < -1*maxDuty) *integration = -1 * maxDuty;


        duty = *integration*ki + propotion;

        clampint(&duty, maxDuty);

        bool dir = duty <= 0;
        *MDIR = dir;

        // printf("%d\n", duty);
        pwm->pulsewidth_us(abs(duty));
    }
    else 
    {
        *integration = 0;
        pwm->pulsewidth_us(0);
    }
}

void read_serial(UnbufferedSerial * serial)
{
    char key;
    float incremention=0.4;
    if(serial->readable())
    { 
        serial->read(&key, 1);
        // serial->write(&key, 1);

        mPg.lock();
        if(!run_Wall_Following)
        {
            if(key == 'w')
            {
                idealVel0 += incremention;
                idealVel1 += incremention;
            } else if(key == 's')
            {
                idealVel0 -= incremention;
                idealVel1 -= incremention;
            } else if(key == 'a')
            {
                idealVel0 -= incremention;
                idealVel1 += incremention;
            } else if(key == 'd')
            {
                idealVel0 += incremention;
                idealVel1 -= incremention;
            }
        }

        if(key == 'l')
        {
            run_Wall_Following = !run_Wall_Following;
        }
        
        if(key == ' ')
        {
            idealVel0 = 0;
            idealVel1 = 0;
            run_Wall_Following = false;
        }
        // if(key == 'i')
        // {
        //     ki += 0.1;
        // }
        // if(key == 'I')
        // {
        //     ki -= 0.1;
        // }
        // if(key == 'p')
        // {
        //     kp += 0.1;
        // }
        // if(key == 'P')
        // {
        //     kp -= 0.1;
        // }
        mPg.unlock();
        wait_us(100);
        key = '\n';
    }
}

void update_Speeds_to_line(
    float * leftVel, 
    float * rightVel, 
    uint16_t us_front, 
    uint16_t us_right, 
    uint16_t us_right_back, 
    uint16_t us_right_front,
    float ideal_distance
){
    mPg.lock();
    float mm_front = us2mm(us_front);
    float mm_right = us2mm(us_right);
    float mm_right_back = us2mm(us_right_back);
    float mm_right_front = us2mm(us_right_front);
    mPg.unlock();

    float error;

    printf("rf%d r%d\n", (int)mm_right_front, (int)mm_right);

    error = 0;
    int to_average=0;
    if(mm_right < 50000)
    {
        error += ideal_distance - mm_right;
        to_average++;
    }
    if(mm_right_front < 50000)
    {
        error += ideal_distance - (mm_right_front*cos(3.14159/6));
        to_average++;
    }
    // if(mm_right_back < 50000)
    // {
    //     error += ideal_distance - (mm_right_back*cos(3.14159/6));
    //     to_average++;
    // }

    // error = ideal_distance - mm_right;
    // error += ideal_distance - (mm_right_front - sensor_rig_radius*(1/cos(3.14159/6) - 1));
    // error += ideal_distance - (mm_right_front - sensor_rig_radius*(1/cos(3.14159/3) - 1));
    // error /= 2; // Average

    if(to_average != 0)
    {
        error /= to_average;
    }

    // printf("%d\n", (int)(error*100));
    // printf("%d %d\n", (int)(mm_right*100), (int)((mm_right_front - sensor_rig_radius*(1/cos(3.14159/6) - 1))*100));
    // printf("%d\n", (int)((mm_right_front - sensor_rig_radius*(1/cos(3.14159/6) - 1))*100));
    // printf("%d\n", (int)(error*100));

    clampfloat(&error, 10000.0);
    filtered_error = (1-new_error_weight)*filtered_error + new_error_weight*error;

    mPg.lock();
    *rightVel = distance_kp*filtered_error;
    *leftVel = -1 * *rightVel;

    if(mm_front > ideal_distance)
    {
        *leftVel += offsetVel;
        *rightVel += offsetVel;
    }
    clampfloat(leftVel, maxVel);    
    clampfloat(rightVel, maxVel);
    mPg.unlock();
}

float us2mm(uint16_t us)
{
    return (float)us / 5.83;
}
