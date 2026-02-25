
// C.P. Diduch EE4333 Robotics Lab-3 Template, January 6, 2026.
// Template for implementation of a PI Speed Control System

#include "mbed.h"

// Function prototypes
void PiControlThread(void const *argument);
void PeriodicInterruptISR(void);
void ExtCollisionThread(void const *argument);
void ExtInterruptISR(void);
void WatchdogThread(void const *argument);
void WatchdogISR(void const *n);

// Processes and threads
osThreadId WatchdogId, ExtCollisionId, PiControlId;     // Thread ID's
osThreadDef(PiControlThread, osPriorityRealtime, 1024); // Declare PiControlThread as a thread
osThreadDef(ExtCollisionThread, osPriorityHigh, 1024);  // Declare ExtCollisionThread as a thread
osThreadDef(WatchdogThread, osPriorityRealtime, 1024);  // Declare WatchdogThread as a thread
osTimerDef(Wdtimer, WatchdogISR);                       // Declare a watchdog timer

//  osPriorityIdle        = -3,  priority: idle (lowest)
//  osPriorityLow         = -2,  priority: low
//  osPriorityBelowNormal = -1,  priority: below normal
//  osPriorityNormal      =  0,  priority: normal (default)
//  osPriorityAboveNormal = +1,  priority: above normal
//  osPriorityHigh        = +2,  priority: high
//  osPriorityRealtime    = +3,  priority: realtime (highest)

// IO Port Configuration
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

UnbufferedSerial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
InterruptIn Bumper(BUTTON1);  // External interrupt pin declared as Bumper
Ticker PeriodicInt;      // Declare a timer interrupt: PeriodicInt

//Declare gloabal variables
int Position;
Mutex mPg;

int main()    // This thread executes first upon reset or power-on.
{
    char c,x;
    int Plocal;

    Bumper.rise(&ExtInterruptISR); // Attach address of ISR to the rising edge of Bumper
//  Start execution of instances of the threads: WatchdogThread() with ID, Watchdog,
//  ExtInterruptThread() with ID, ExtInterrupt, and PiControlThread() with ID PiControlId
    WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
    ExtCollisionId = osThreadCreate(osThread(ExtCollisionThread), NULL);
    PiControlId = osThreadCreate(osThread(PiControlThread), NULL);

//  Start the watch dog timer and enable the watch dog interrupt
    osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);

    printf("r\n\r\nPi Control Program Template\r\n");
//  Specify address of the PeriodicInt ISR as PeriodicInterruptISR and the interval
//  in seconds between interrupts, then start the timer and interrupt generation:
    PeriodicInt.attach(&PeriodicInterruptISR, 500ms);
    osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to  2000ms.

while(true) {

      if (pc.readable()){
         if(pc.read(&c,1)) { 
              pc.write(&c,1); //Echo keyboard entry
              x=c;
              if(x=='r') { 
                 led1=0;  // Turn LED off.
                 osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to  2000ms.
                 }
               }
            }

        x='X'; 

      mPg.lock();
      Plocal=Position; // Wrap mutex around access to a global resource / variable
      mPg.unlock();
      printf("\r\n %c %5d", x, Plocal);
  
      ThisThread::sleep_for(500ms); // Sleep on the waiting queue for 500 ms
    }
}

// ******** Control Thread ********
void PiControlThread(void const *argument)
{
    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalPi, is received.
        led2 = !led2; // Alive status - led2 toggles each time PiControlThread is signaled.
        mPg.lock();
        Position = Position + 1;
        mPg.unlock();
    }
}

// ******** Periodic Interrupt Handler ********
void PeriodicInterruptISR(void)
{
    osSignalSet(PiControlId,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}

// ******** Collision Thread ********
void ExtCollisionThread(void const *argument)
{
    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalExtCollision, is received
        led3 = !led3;
    }
}

// ******** Collision Interrupt Handler ********
void ExtInterruptISR(void)
{
    osSignalSet(ExtCollisionId,0x1); // Activate the signal, ExtCollision, with each external interrupt.
}

void WatchdogThread(void const *argument)
{
    while(true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until a signal is received
        led1 = 1; // led4 is activated when the watchdog timer times out
    }
}

// ******** Watchdog Interrupt Handler ********
void WatchdogISR(void const *n)
{
    osSignalSet(WatchdogId,0x1); // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread
}