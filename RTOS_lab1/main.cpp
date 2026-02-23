// C.P. Diduch - EE4333 Lab-2, January 1, 2026.
// Template-3 A Real Time System with 4 threads.

 #include "mbed.h"

//  osPriorityLow           = -2,   priority: low
//  osPriorityBelowNormal   = -1,   priority: below normal
//  osPriorityNormal        =  0,   priority: normal (default)
//  osPriorityAboveNormal   = +1,   priority: above normal
//  osPriorityHigh          = +2,   priority: high 
//  osPriorityRealtime      = +3,   priority: realtime (highest)

// IO Port Configuration
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

Ticker PeriodicInt; // Dclare a periodic timer for interrupt generation
static UnbufferedSerial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
InterruptIn Bumper(BUTTON1);

// Function prototypes
void ExtInterruptISR(void);
void ExtInterruptThread(void const *n);
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);
void PeriodicInterruptISR(void);
void PeriodicInterruptThread(void const *argument);

// Processes and threads
//int32_t SignalWatchdog; 
osThreadId WatchdogId, PeriodicInterruptId, ExtInterruptId;
osTimerDef(Wdtimer, WatchdogISR); // Declare a watch dog timer
osThreadDef(WatchdogThread, osPriorityRealtime, 1024); // Declare WatchdogThread as a thread/process
osThreadDef(PeriodicInterruptThread, osPriorityRealtime, 1024); // Declare PeriodicInterruptThread as a thread/process
osThreadDef(ExtInterruptThread, osPriorityHigh, 1024);  // Declare ExtInterruptThread as a thread/process

//Declare global variables
int Position;

// ******** Main Thread ********
   int main() {  // This thread executes first upon reset or power-on.
   char x,c;

   led1=0;
   led2=1;
   led3=1;

   pc.baud(9600);
   pc.format(/*bits*/ 8, /*parity*/ SerialBase::None, /*stop bit */1);
   printf("\r\n Hello World - RTOS Template Program - 4 Threads\r\n");

   // Start execution of: WatchdogThread with ID, WatchdogId:
   WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
   // Start execution of: PeriodicInterruptThread with ID, PeriodicInterruptId:
   PeriodicInterruptId = osThreadCreate(osThread(PeriodicInterruptThread), NULL);
   // Start execution of: ExtInterruptThreasd with ID, ExtnterruptId:
   ExtInterruptId = osThreadCreate(osThread(ExtInterruptThread), NULL);

   // Start the watch dog timer and enable the watch dog interrupt
   osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);

   // Start periodic interrupt generaion, specifying the period, and address of the isr.
   PeriodicInt.attach(&PeriodicInterruptISR, 250ms);

   Bumper.rise(&ExtInterruptISR); //Bumper is a rising edge triggered external interrupt

   do {
      if(pc.read(&c,1)) {
        pc.write(&c,1); //Echo keyboard entry
        x=c;
        if(x=='r') {
            led1=0;  // Turn LED off.
            osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to  2000ms.
        }
    }

         printf("\r\n %5d",Position);
         ThisThread::sleep_for(500ms); // Go to sleep for 500 ms
      }
      while(1);
   }

// ******** Watchdog Thread ********
void WatchdogThread(void const *argument) {
while (true) {
    osSignalWait(0x1, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
    led1 = 1;
    }
}

// ******** Watchdog Interrupt Handler ********
   void WatchdogISR(void const *n) {
      osSignalSet(WatchdogId,0x1); // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread.
      }

// ******** Periodic Timer Interrupt Thread ********
void PeriodicInterruptThread(void const *argument) {
    while (true) {
    osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalPi, is received.
    led2= !led2; // Alive status - led3 toggles each time PieriodicZInterruptsThread is signaled.
    Position = Position + 1;    
    } 
}

// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
    osSignalSet(PeriodicInterruptId,0x1); // Send signal to the thread with ID, PeriodicInterruptId, i.e., PeriodicInterruptThread.
}


// ******** External Interrupt Thread ********
void ExtInterruptThread(void const *argument) {
    while(true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalExtCollision, is received
        led3 = !led3;
        }
}
    
// ******** External Interrupt Handler ********
void ExtInterruptISR(void) {
    osSignalSet(ExtInterruptId,0x1); // Send signal to the thread with ID, ExtInterruptId, i.e., ExtInterruptThread.
    }
