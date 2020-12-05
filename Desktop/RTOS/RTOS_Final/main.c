// RTOS Framework - Fall 2020
// J Losh

// Student Name:
// TO DO: Matthew Grossweiler

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5
#define MEL_LEN 100

// SVC Case Defines
#define REBOOT 0
#define PIDOF 1
#define KILL 2
#define RUN 3
#define PS 4
#define IPCS 5
#define PRIO 6
#define PREEMPT 7
#define SCHED 8
#define THREAD_PRIO 10
#define YIELD 11
#define POST 22
#define WAIT 33
#define SLEEP 44
#define DESTROY 100

// MPU defines
#define NVIC_MPU_ATTR_AP_FULL 0x03000000
#define NVIC_MPU_ATTR_AP_PRIV_ONLY 0x01000000
#define NVIC_MPU_ATTR_AP_NONE 0x00000000



// REQUIRED: correct these bitbanding references for the off-board LEDs
// Bitband aliases for PE1 -> PE4 + PF2
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

// Bitband aliases for PA2 -> PA7
#define PUSH_BUTTON_0 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON_1 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON_2 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON_3 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON_4 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON_5 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

// PortE Masks
#define RED_LED_MASK 2
#define ORANGE_LED_MASK 4
#define YELLOW_LED_MASK 8
#define GREEN_LED_MASK 16

// PortF Masks
#define BLUE_LED_MASK 4

// PortA Masks
#define PB0 4
#define PB1 8
#define PB2 16
#define PB3 32
#define PB4 64
#define PB5 128

// Extern ASM functions

extern void setPSP(uint32_t * psp);
extern void setASP();
extern uint32_t * getPSP();
extern uint32_t * getMSP();
extern uint32_t * pushR4toR11toPSP();
extern uint32_t * popR4toR11fromPSP(uint32_t * sp);
extern void pushPSP(uint32_t val);
extern uint32_t getSVCnumAdd();
extern uint32_t getRfromPSP(uint32_t reg);


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

typedef struct _semaphore
{
    char name[16];
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    uint32_t last_task;
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

#define keyPressed 0
#define keyReleased 1
#define flashReq 2
#define resource 3

uint8_t semaphoreCount = 0;


// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_SUSPEND    5 // has run, but now killed

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t time;
} tcb[MAX_TASKS];

// Allocating space for the heap
#pragma DATA_SECTION(my_heap, ".heap")
uint32_t my_heap [7168];

bool prio_on = true;
bool preempt_on = false;
bool prio_inh = false;

//uint32_t time_tables[MAX_TASKS+1];

#define MAX_CHARS 80
#define MAX_FIELDS 5
#define MEL_LEN 100
char FS[MAX_CHARS+1];
char numtostring[MAX_CHARS+1];

typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    NVIC_ST_RELOAD_R = 0x3E7;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
        tcb[i].time = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    static uint8_t index = 0;
    uint8_t prio_index = 0;
    static int8_t index_reset = MAX_TASKS;

    ok = false;

    while (!ok)
    {
        if (prio_on == true)
        {
            while (prio_index < 16)
            {
                while (index != index_reset)
                {
                    if (tcb[index].currentPriority == prio_index)
                    {
                        if ((tcb[index].state == STATE_READY) || (tcb[index].state == STATE_UNRUN))
                        {
                            index_reset = index;
                            index = index + 1;
                            prio_index = 0;
                            return index - 1;
                        }

                    }

                    index = index + 1;
                    if (index == MAX_TASKS)
                    {
                        index = 0;
                        if (index_reset == 12)
                        {
                            prio_index = prio_index + 1;
                        }
                    }
                }

                prio_index = prio_index + 1;
                index = index + 1;

                if (index_reset == -1)
                {
                    index_reset = MAX_TASKS + 1;
                }

                if (prio_index == 16)
                {
                    prio_index = 0;
                }

            }

        }
        else
        {
            task++;
            if (task >= MAX_TASKS)
            {
                task = 0;
            }

            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    return task;
}

bool createThread(_fn fn, const char name1[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    uint8_t x = 0;
    bool found = false;
    uint32_t num_KiB = 0;
    uint8_t stack_temp = 0;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record

            stack_temp = stackBytes % 1024;
            if(stack_temp != 0)
            {
                num_KiB = (stackBytes / 1024) + 1;
            }
            else
            {
                num_KiB = stackBytes / 1024;
            }

            i = 0;

            while (tcb[i].state != STATE_INVALID)
            {
                i++;
            }
            while ((name1[x] != '\0') && (x < 16))
            {
                tcb[i].name[x] = name1[x];
                x = x + 1;
            }
            tcb[i].name[x+1] = '\0';
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            if(i == 0)
            {
                tcb[i].sp = (0x20001000) + (1024 * num_KiB);
            }
            else
            {
                tcb[i].sp = (tcb[i-1].sp) + (1024 * num_KiB);
            }
            tcb[i].spInit = tcb[i].sp;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }

    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    fn();
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    __asm("     SVC #100");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm("     SVC #10");
}

bool createSemaphore(uint8_t s, uint8_t count, char name[])
{
    uint8_t x = 0;
    bool ok = (s < MAX_SEMAPHORES);
    if (ok)
    {
        semaphores[s].count = count;
        while ((name[x] != '\0') && (x < 16))
        {
            semaphores[s].name[x] = name[x];
            x = x + 1;
        }
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    _fn fn;

    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;

    uint32_t temp1;

    if (tcb[taskCurrent].spInit < 0x20002001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20000000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x1;
        NVIC_MPU_BASE_R = 0x20000000;
        NVIC_MPU_ATTR_R = 0;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    }
    else if (tcb[taskCurrent].spInit < 0x20004001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20002000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x2;
        NVIC_MPU_BASE_R = 0x20002000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    }
    else if (tcb[taskCurrent].spInit < 0x20006001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20004000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x3;
        NVIC_MPU_BASE_R = 0x20004000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;


    }
    else if (tcb[taskCurrent].spInit < 0x20008001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20006000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x4;
        NVIC_MPU_BASE_R = 0x20006000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    }



    TIMER1_TAV_R = 0;

    // Turn ON MPU
    NVIC_MPU_CTRL_R |= 1 | 2;


    setPSP(tcb[taskCurrent].spInit);
    setASP();

    fn = (_fn)tcb[taskCurrent].pid;

    fn();

}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{

    __asm("     SVC #11");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm("     SVC #44");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm("     SVC #33");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm("     SVC #22");
}

void getPSdata(uint32_t *mydata)
{
    __asm("     SVC #4");
}

void getIPCSdata(uint32_t *mydata)
{
    __asm("     SVC #5");
}

void getPIDOFdata(char name[], uint32_t *mydata)
{
    __asm("     SVC #1");
}

void setKILLdata(uint32_t * mydata)
{
    __asm("     SVC #2");
}

void setRUNdata(char name[])
{
    __asm("     SVC #3");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{

    uint8_t x = 0;

    while (x < MAX_TASKS)
    {
        if (tcb[x].state == STATE_DELAYED)
        {
            tcb[x].ticks = tcb[x].ticks - 1;
        }
        if (tcb[x].ticks == 0)
        {
            if (tcb[x].state == STATE_DELAYED)
            {
                tcb[x].state = STATE_READY;
                if (preempt_on == true)
                {
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                }
            }

        }
        x = x + 1;
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //putsUart0("\nPendSV in process ");
    //putsUart0(tcb[taskCurrent].name);
    //putsUart0("\n\r");

    int timeval = TIMER1_TAV_R;

    //time_tables[taskCurrent] = time_talbes[taskCurrent] + timeval;

    //time_tables[MAX_TASKS + 1] = time_tables[MAX_TASKS + 1] + timeval;

    //tcb[taskCurrent].time = tcb[taskCurrent].time + timeval;

    tcb[taskCurrent].time = tcb[taskCurrent].time + 1;

    if (tcb[taskCurrent].time > 100000)
    {
        tcb[taskCurrent].time = 0;
    }

    tcb[taskCurrent].sp = pushR4toR11toPSP();


    taskCurrent = rtosScheduler();

    uint32_t temp1;


    if (tcb[taskCurrent].spInit < 0x20002001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20000000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x1;
        NVIC_MPU_BASE_R = 0x20000000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;



    }
    else if (tcb[taskCurrent].spInit < 0x20004001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20002000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x2;
        NVIC_MPU_BASE_R = 0x20002000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    }
    else if (tcb[taskCurrent].spInit < 0x20006001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20004000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x3;
        NVIC_MPU_BASE_R = 0x20004000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 - 1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;


    }
    else if (tcb[taskCurrent].spInit < 0x20008001)
    {
        temp1 = tcb[taskCurrent].spInit - 0x20006000;
        temp1 = (temp1*8) / 0x2000;

        NVIC_MPU_NUMBER_R = 0x4;
        NVIC_MPU_BASE_R = 0x20006000;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0x1 << (8 + temp1 -  1)) | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    }


    TIMER1_TAV_R = 0;

    if(tcb[taskCurrent].state == STATE_READY)
    {
        tcb[taskCurrent].sp = popR4toR11fromPSP(tcb[taskCurrent].sp);
        setPSP(tcb[taskCurrent].sp);
    }
    else
    {
        tcb[taskCurrent].state = STATE_READY;
        setPSP(tcb[taskCurrent].spInit);
        pushPSP(0x21000000);
        pushPSP(tcb[taskCurrent].pid);
        pushPSP(0xFFFFFFFD);
        pushPSP(0x12);
        pushPSP(0x3);
        pushPSP(0x2);
        pushPSP(0x1);

    }

}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr()
{
    //putsUart0("\n\rSVCall in process ");
    //putsUart0(tcb[taskCurrent].name);
    //putsUart0("\n\r");

    uint8_t svcNum;
    uint8_t x;
    uint8_t y = 0;
    uint8_t z = 0;
    uint8_t name_count;
    uint8_t queue_count;
    uint8_t temp;

    _fn fn;

    bool ok = true;

    uint32_t r0, r1, r2, r3;
    uint32_t * pointer32 = 0;
    uint8_t * pointer8 = 0;
    uint16_t * pointer16 = 0;

    r0 = getRfromPSP(0);
    r1 = getRfromPSP(4);
    r2 = getRfromPSP(8);
    r3 = getRfromPSP(12);


    svcNum = getSVCnum();


    switch(svcNum)
    {
        case YIELD :
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        case POST :
            semaphores[r0].count = semaphores[r0].count + 1;
            tcb[taskCurrent].semaphore = 0;
            semaphores[r0].last_task = taskCurrent;
            if (prio_inh == true && semaphores[r0].last_task == 1)
            {
                tcb[1].currentPriority = tcb[1].priority;
            }

            if (semaphores[r0].count > 0)
            {
                uint8_t temp_task;
                if (semaphores[r0].processQueue[0] != 0)
                {
                    temp_task = semaphores[r0].processQueue[0];
                    tcb[temp_task].state = STATE_READY;
                    semaphores[r0].count = semaphores[r0].count - 1;
                    for (x = 0; x < semaphores[r0].queueSize; x++)
                    {
                        semaphores[r0].processQueue[x] = semaphores[r0].processQueue[x+1];
                    }
                    if (semaphores[r0].queueSize > 0)
                    {
                        semaphores[r0].queueSize = semaphores[r0].queueSize - 1;
                    }
                    if (tcb[temp_task].currentPriority > tcb[taskCurrent].currentPriority)
                    {
                        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                    }
                }


            }
            break;
        case WAIT :
            if (semaphores[r0].count > 0)
            {
                semaphores[r0].count = semaphores[r0].count - 1;
            }
            else
            {
                tcb[taskCurrent].state = STATE_BLOCKED;
                if (prio_inh == true && semaphores[r0].last_task == 6)
                {
                    tcb[1].currentPriority = tcb[taskCurrent].currentPriority;
                }
                tcb[taskCurrent].semaphore = &semaphores[r0];
                semaphores[r0].processQueue[semaphores[r0].queueSize] = taskCurrent;
                semaphores[r0].queueSize = semaphores[r0].queueSize + 1;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }
            break;
        case SLEEP :
            tcb[taskCurrent].ticks = r0;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        case REBOOT :
            NVIC_APINT_R = 0x05FA0000 | NVIC_APINT_SYSRESETREQ;
            break;
        case PS :
            pointer32 = r0;
            while (y < MAX_TASKS)
            {
                *pointer32 = tcb[y].state;
                pointer32 = pointer32 + 1;

                *pointer32 = tcb[y].pid;
                pointer32 = pointer32 + 1;

                *pointer32 = tcb[y].spInit;
                pointer32 = pointer32 + 1;

                *pointer32 = tcb[y].sp;
                pointer32 = pointer32 + 1;

                pointer8 = pointer32;

                *pointer8 = tcb[y].priority;
                pointer8 = pointer8 + 1;

                *pointer8 = tcb[y].currentPriority;
                pointer8 = pointer8 + 3;

                pointer32 = pointer8;

                *pointer32 = tcb[y].ticks;
                pointer32 = pointer32 + 1;

                pointer8 = pointer32;

                name_count = 0;
                while(name_count < 16)
                {
                    *pointer8 = tcb[y].name[name_count];
                     pointer8 = pointer8 + 1;
                     name_count = name_count + 1;
                }

                pointer32 = pointer8;

                *pointer32 = tcb[y].semaphore;
                pointer32 = pointer32 + 1;

                *pointer32 = tcb[y].time;
                pointer32 = pointer32 + 1;

                y = y + 1;

            }
            break;
        case IPCS :
            pointer32 = r0;
            while (y < MAX_SEMAPHORES)
            {
                pointer8 = pointer32;

                name_count = 0;

                while (name_count < 16)
                {
                    *pointer8 = semaphores[y].name[name_count];
                    pointer8 = pointer8 + 1;
                    name_count = name_count + 1;
                }

                pointer16 = pointer8;

                *pointer16 = semaphores[y].count;
                pointer16 = pointer16 + 1;

                *pointer16 = semaphores[y].queueSize;
                pointer16 = pointer16 + 1;

                pointer32 = pointer16;

                queue_count = 0;
                while (queue_count < MAX_QUEUE_SIZE)
                {
                    *pointer32 = semaphores[y].processQueue[queue_count];
                    pointer32 = pointer32 + 1;
                    queue_count = queue_count + 1;
                }

                *pointer32 = semaphores[y].last_task;
                pointer32 = pointer32 + 1;

                y = y + 1;

            }
            break;
        case PIDOF :

            pointer8 = r0;
            pointer32 = r1;

            while (y < MAX_TASKS)
            {
                *pointer32 = mystrcmp(pointer8, tcb[y].name);
                if (*pointer32 == 0)
                {
                    *pointer32 = tcb[y].pid;
                    break;
                }
                y = y + 1;
            }

            break;
        case KILL :
            pointer32 = r0;
            while (y < MAX_TASKS)
            {
                if (pointer32 == tcb[0].pid)
                {
                    putsUart0("Cannot Kill Idle.\n\r");
                    break;
                }
                else if (pointer32 == tcb[y].pid)
                {
                    tcb[y].state = STATE_SUSPEND;
                    break;
                }
                y = y + 1;
            }
            break;
        case RUN :
            pointer8 = r0;
            uint8_t flag;

            while (y < MAX_TASKS)
            {
                flag = mystrcmp(pointer8, tcb[y].name);

                if (flag == 0)
                {
                    tcb[y].state = STATE_READY;
                    //call restartThread
                    break;
                }
                y = y + 1;
            }
            break;
        case PRIO :
            prio_inh = r0;
            break;
        case PREEMPT :
            preempt_on = r0;
            break;
        case SCHED :
            prio_on = r0;
            break;
        case THREAD_PRIO :
            while (y < MAX_TASKS)
            {
                if (tcb[y].pid == r0)
                {
                    tcb[y].currentPriority = r1;
                }
                y = y + 1;
            }
            break;
        case DESTROY :
            temp = 90;
            z = 0;
            pointer32 = r0;

            while (z < MAX_TASKS)
            {
                if (pointer32 == (_fn)tcb[z].pid)
                {
                    temp = z;
                }
                z = z + 1;
            }
            if (temp == 90)
            {
                break;
            }
            else
            {
                z = 0;
                tcb[temp].state = STATE_INVALID;
                uint32_t * point_temp;
                point_temp = tcb[temp].semaphore;

                point_temp = point_temp + 1;

                while (z < MAX_QUEUE_SIZE)
                {
                    if (*point_temp == temp)
                    {
                        *point_temp = point_temp + 1;
                        break;
                    }
                    point_temp = point_temp + 1;
                    z = z  + 1;
                }

            }
            break;
        default :
            break;

    }

}

// REQUIRED: code this function
void mpuFaultIsr()
{
    putsUart0("\nMPU fault in process ");
    IntToString(tcb[taskCurrent].pid);
    putsUart0("\n\n\r");

    uint32_t data_add;

    if ((1 << 1) && NVIC_FAULT_STAT_R)
    {
        data_add = NVIC_MM_ADDR_R;
    }

    uint32_t * msp_point = getMSP();

    putsUart0("MSP: ");
    IntToHexString(msp_point);
    putsUart0("\n\r");

    uint32_t * p = getPSP();

    putsUart0("PSP: ");
    IntToHexString(p);
    putsUart0("\n\r");

    putsUart0("Instruction at: ");
    IntToHexString(*(p + 6));
    putsUart0(" accessed by: ");
    IntToHexString(data_add);
    putsUart0("\n\r");

    putsUart0("  R0:   ");
    IntToHexString(*p);
    putsUart0("\n\r");

    putsUart0("  R1:   ");
    IntToHexString(*(p + 1));
    putsUart0("\n\r");

    putsUart0("  R2:   ");
    IntToHexString(*(p + 2));
    putsUart0("\n\r");

    putsUart0("  R3:   ");
    IntToHexString(*(p + 3));
    putsUart0("\n\r");

    putsUart0("  R12:  ");
    IntToHexString(*(p + 4));
    putsUart0("\n\r");

    putsUart0("  LR:   ");
    IntToHexString(*(p + 5));
    putsUart0("\n\r");

    putsUart0("  PC:   ");
    IntToHexString(*(p + 6));
    putsUart0("\n\r");

    putsUart0("  xPSR: ");
    IntToHexString(*(p + 7));
    putsUart0("\n\r");


    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;

}

// REQUIRED: code this function
void hardFaultIsr()
{
    putsUart0("\nHard fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0("\n\r");

    uint32_t *psp = getPSP();

    putsUart0("  PSP:  ");
    IntToHexString(psp);
    putsUart0("\n\r");

    uint32_t *msp = getMSP();

    putsUart0("  MSP:  ");
    IntToHexString(msp);
    putsUart0("\n\r");

    putsUart0("  xPSR: ");
    IntToHexString(*(msp + 7));
    putsUart0("\n\r");

}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("\nBus fault in process ");
    IntToString(tcb[taskCurrent].pid);
    putsUart0("\n\r");

    NVIC_FAULT_STAT_R |= (0xFF << 8);
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("\nUsage fault in process ");
    IntToString(tcb[taskCurrent].pid);
    putsUart0("\n\r");
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure PortE
    GPIO_PORTE_DIR_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK;   // bits 1, 2, 3, and 4 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK;   // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK;   // enable LEDs

    // Configure PortF
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;   // bit 2 as output, other pins are inputs
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;   // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;   // enable LED

    // Configure PortA
    GPIO_PORTA_DIR_R &= ~(PB0 | PB1 | PB2 | PB3 | PB4 | PB5);   // bits 2 - 7 are inputs
    GPIO_PORTA_DEN_R |= PB0 | PB1 | PB2 | PB3 | PB4 | PB5;   // enable Push Buttons

    GPIO_PORTA_PUR_R |= PB0 | PB1 | PB2 | PB3 | PB4 | PB5;   // enable internal pull-up for push button

    // Timer 1 configure
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;

}

void initMPU()
{
    // Configure ISRs
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM;

    // Configuring MPU
    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_PRIVDEFEN; // Set the Region -1

    // Flash
    NVIC_MPU_NUMBER_R = 0x7; // Region 7 and Highest Priority
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_AP_FULL | NVIC_MPU_ATTR_CACHEABLE | (0x11 << 1) | NVIC_MPU_ATTR_ENABLE;

    // Everything
    NVIC_MPU_NUMBER_R = 0x0; // Region 0 and Lowest Priority
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_FULL | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_BUFFRABLE | (0x1F << 1) | NVIC_MPU_ATTR_ENABLE;

    // SRAM - 1
    NVIC_MPU_NUMBER_R = 0x1; // Region 1 and Second Lowest Priority
    NVIC_MPU_BASE_R = 0x20000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;


    // SRAM - 2
    NVIC_MPU_NUMBER_R = 0x2; // Region 2 and Third Lowest Priority
    NVIC_MPU_BASE_R = 0x20002000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    // SRAM - 3
    NVIC_MPU_NUMBER_R = 0x3; // Region 3 and Fourth Lowest Priority
    NVIC_MPU_BASE_R = 0x20004000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    // SRAM - 4
    NVIC_MPU_NUMBER_R = 0x4; // Region 4 and Fifth Lowest Priority
    NVIC_MPU_BASE_R = 0x20006000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_PRIV_ONLY | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | (0xC << 1) | NVIC_MPU_ATTR_ENABLE;

    // SRAM - Bitband
    NVIC_MPU_NUMBER_R = 0x5; // Region 5 and Sixth Lowest Priority
    NVIC_MPU_BASE_R = 0x22000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_NONE | (0x13 << 1) | NVIC_MPU_ATTR_ENABLE;

    // System
    NVIC_MPU_NUMBER_R = 0x6; // Region 6 and Seventh Lowest Priority
    NVIC_MPU_BASE_R = 0xE0000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | NVIC_MPU_ATTR_AP_NONE | (0x12 << 1) | NVIC_MPU_ATTR_ENABLE;
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t presses = 0;

    if (!PUSH_BUTTON_0)
    {
        presses = presses + 1;
    }
    if (!PUSH_BUTTON_1)
    {
        presses = presses + 2;
    }
    if (!PUSH_BUTTON_2)
    {
        presses = presses + 4;
    }
    if (!PUSH_BUTTON_3)
    {
        presses = presses + 8;
    }
    if (!PUSH_BUTTON_4)
    {
        presses = presses + 16;
    }
    if (!PUSH_BUTTON_5)
    {
        presses = presses + 32;
    }
    return presses;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

uint8_t getSVCnum()
{
    uint32_t * p;

    p = getSVCnumAdd();

    return *p;
}


void getsUart0(USER_DATA* data)
{
    int count = 0;
    char c;
    while (true)
    {
        c = getcUart0();

        if (((c == 8) || (c == 127)) && (count > 0))
        {
             count = count - 1;
             continue;
        }

        if (c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }

        if (c >= 32)
        {
            data->buffer[count] = c;
            count = count + 1;
            if (count == MAX_CHARS+1)
            {
                data->buffer[count] = '\0';
                return;
            }
        }
    }

}

void IntToString(int num)
{
    int count;
    int temp = num;
    char numtostring[11];

    for(count = 0; count < 11; count ++)
    {
        numtostring[count] = '\0';
    }

    count = 0;

    while (temp > 0)
    {
        count = count + 1;
        temp = temp / 10;
    }

    numtostring[count + 1] = '\0';

    if (num == 0)
    {
        numtostring[0] = '0';
        numtostring[1] = '\0';
        putsUart0(numtostring);
        return;
    }

    while (num > 0)
    {
        numtostring[count - 1] = num % 10 + '0';
        num = num / 10;
        count = count - 1;
    }

    putsUart0(numtostring);
}

void IntToHexString(uint32_t num)
{
    int count;
    int temp = 0;
    int char_switch = 0;
    char temp_char;
    char hexstring[9];

    for (count = 0; count < 9; count++)
    {
        hexstring[count] = '\0';
    }

    count = 0;

    while (num != 0)
    {
        temp = num % 16;
        if (temp < 10)
        {
            hexstring[count] = temp + 48;
        }
        else
        {
            hexstring[count] = temp + 55;
        }

        count = count + 1;
        num = num / 16;
    }

    while (count != 8)
    {
        hexstring[count] = '0';
        count = count + 1;
    }

    count = count - 1;

    while (char_switch < count)
    {
        temp_char = hexstring[count];
        hexstring[count] = hexstring[char_switch];
        hexstring[char_switch] = temp_char;

        char_switch = char_switch + 1;
        count = count - 1;
    }

    putsUart0("0x");
    putsUart0(hexstring);

}

int mystrcmp(char* string1, char* string2)
{
    int x = 0;

    while(string1[x] != '\0' && string2[x] != '\0')
    {
        if (string1[x] != string2[x])
        {
            return 1;
        }

        x = x + 1;
    }

    if (string1[x] == '\0' && string2[x] == '\0')
    {
        return 0;
    }
    else
    {
        return 1;
    }

}

int mypow(int num1, int num2)
{
    if (num2 == 0)
    {
        return 1;
    }

    int final = 1;
    int x;

    for (x = 0; x < num2; x ++)
    {
        final = final * num1;
    }

    return final;

}

void parseField(USER_DATA* data)
{
   int ftc = 0;
   int fpc = 0;
   data->fieldCount = 0;
   while(true)
   {
       // Initial check for characters
       if (ftc == 0)
       {
           if (((data->buffer[ftc] >= 65) && (data->buffer[ftc] <= 90)) || ((data->buffer[ftc] >= 97) && (data->buffer[ftc] <= 122)))
           {
               data->fieldType[fpc] = 'a';
               data->fieldPosition[fpc] = ftc;
               fpc = fpc + 1;
               data->fieldCount = fpc;
           }
           if ((data->buffer[ftc] >= 48) && (data->buffer[ftc] <= 57))
           {
               data->fieldType[fpc] = 'n';
               data->fieldPosition[fpc] = ftc;
               fpc = fpc + 1;
               data->fieldCount = fpc;
           }
           ftc = ftc + 1;
           continue;
       }

       // Checking for new alphas
       if (((data->buffer[ftc] >= 65) && (data->buffer[ftc] <= 90)) || ((data->buffer[ftc] >= 97) && (data->buffer[ftc] <= 122)))
       {
           if (!(((data->buffer[ftc-1] >= 65) && (data->buffer[ftc-1] <= 90)) || ((data->buffer[ftc-1] >= 97) && (data->buffer[ftc-1] <= 122))) || (data->fieldType[fpc-1] == 'n'))
           {
               data->fieldType[fpc] = 'a';
               data->fieldPosition[fpc] = ftc;
               data->buffer[ftc-1] = '\0';
               fpc = fpc + 1;
               ftc = ftc + 1;
               data->fieldCount = fpc;
               continue;
           }

       }

       // Checking for new numerics
       if ((data->buffer[ftc] >= 48) && (data->buffer[ftc] <= 57))
       {
           if (!((data->buffer[ftc-1] >= 48) && (data->buffer[ftc-1] <= 57)) || (data->fieldType[fpc-1] == 'a'))
           {
               data->fieldType[fpc] = 'n';
               data->fieldPosition[fpc] = ftc;
               data->buffer[ftc-1] = '\0';
               fpc = fpc + 1;
               ftc = ftc + 1;
               data->fieldCount = fpc;
               continue;
           }

       }

       // Returning
       if ((data->fieldCount == MAX_FIELDS) || (data->buffer[ftc] == '\0'))
       {
           return;
       }
       ftc = ftc + 1;
   }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{

    char FS[MAX_CHARS+1];

    if ((fieldNumber <= MAX_FIELDS) && ((data->fieldType[fieldNumber] == 'a') || (data->fieldType[fieldNumber] == 'n')))
    {
       uint8_t str = 0;
       while(data->buffer[data->fieldPosition[fieldNumber] + str] != '\0')
       {
           FS[str] = data->buffer[data->fieldPosition[fieldNumber] + str];
           str = str + 1;
       }
       FS[str] = '\0';
       return FS;
    }
    else
    {
        FS[0] = '\0';
        return FS;
    }
}

uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    uint32_t FI = 0;
    int x = 0;
    int countup = 0;
    char start[20] = "PlaceHolderforstring";


    if (fieldNumber <= MAX_FIELDS)
    {
        start[0] = (data->buffer[data->fieldPosition[fieldNumber]]) - 48;

        while (start[x] != 208)
        {
            x = x + 1;
            start[x] = (data->buffer[x + data->fieldPosition[fieldNumber]]) - 48;
        }

        x = x - 1;

        while (x >= 0)
        {
            FI = FI + (start[x] * mypow(10,countup));
            x = x - 1;
            countup = countup + 1;
        }
    }

    return FI;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char* check = getFieldString(data->buffer, data->fieldPosition[0]);

    if ((mystrcmp(check, strCommand) == 0) && (minArguments >= data->fieldCount - 1))
    {
        return true;
    }
    return false;
}

void reboot()
{
    __asm("     SVC #0");
}

void ps()
{
    uint8_t x = 0;

    struct _tcb tcb_copy[MAX_TASKS];

    getPSdata(&tcb_copy);

    putsUart0("PID\tCPU TIME\tNAME\n\r");

    while ( x < MAX_TASKS)
    {
        if (tcb_copy[x].name[0] != '\0')
        {
            IntToString(tcb_copy[x].pid);
            putsUart0("\t");
            IntToString(tcb_copy[x].time);
            putsUart0("\t\t");
            putsUart0(tcb_copy[x].name);
            putsUart0("\n\r");
        }
        x = x + 1;
    }
}

void ipcs()
{
    uint8_t x = 0;
    uint8_t y = 0;

    struct _semaphore semaphore_copy[MAX_SEMAPHORES];

    getIPCSdata(&semaphore_copy);

    putsUart0("Sem Name\tSem Count\tWaiting Threads\n\r");

    while( x < MAX_SEMAPHORES)
    {
        if(semaphore_copy[x].name[0] != '\0')
        {
            putsUart0(semaphore_copy[x].name);
            putsUart0("\t");
            IntToString(semaphore_copy[x].count);
            putsUart0("\t\t");
            y = 0;
            while (y < MAX_QUEUE_SIZE)
            {
                IntToString(semaphore_copy[x].processQueue[y]);
                putsUart0(" ");
                y = y + 1;
            }
            putsUart0("\n\r");
        }
        x = x + 1;

    }
}

void kill(pid_num)
{
    setKILLdata(pid_num);
}

void pi(bool state)
{
    __asm("     SVC #6");
}

void preempt(bool state)
{
    __asm("     SVC #7");
}

void sched(bool prio_state)
{
    __asm("     SVC #8");
}

void pidof(char name[])
{

    uint32_t * pid_num;

    getPIDOFdata(name, &pid_num);

    if (pid_num == 0)
    {
        putsUart0("That task does not exist.");
    }
    else
    {
        putsUart0("The PID of ");
        putsUart0(name);
        putsUart0(" is ");
        IntToString(pid_num);
        putsUart0(".\n\r");
    }



}

void run(char name[])
{
    setRUNdata(name);
}


// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    USER_DATA data;

        while(true)
        {
            yield();
            if (kbhitUart0())
            {
                getsUart0(&data);

                putsUart0("\n\r");



                parseField(&data);

                bool valid = false;

                if(isCommand(&data,"reboot", 0))
                {
                    reboot();
                    valid = true;
                }
                if (isCommand(&data, "ps", 0))
                {
                    ps();
                    valid = true;
                }

                if (isCommand(&data, "ipcs", 0))
                {
                    ipcs();
                    valid = true;
                }

                if (isCommand(&data, "kill", 1))
                {
                    kill(getFieldInteger(&data,1));
                    valid = true;
                }

                if (isCommand(&data, "pi", 1))
                {
                    pi(getFieldInteger(&data,1));
                    valid = true;
                }

                if (isCommand(&data, "preempt" , 1))
                {
                    preempt(getFieldInteger(&data,1));
                    valid = true;
                }

                if (isCommand(&data, "sched" , 1))
                {
                    sched(getFieldInteger(&data,1));
                    valid = true;
                }

                if (isCommand(&data, "pidof" , 1))
                {
                    pidof(getFieldString(&data, 1));
                    valid = true;
                }

                if (isCommand(&data, "run" , 1))
                {
                    run(getFieldString(&data,1));
                    valid = true;
                }

                if (!valid)
                {
                    putsUart0("Invalid Command\n\r");
                }
            }
        }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;



    // Initialize hardware
    initHw();
    initUart0();
    initRtos();
    initMPU();


    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1, "keyPressed");
    createSemaphore(keyReleased, 0, "keyReleased");
    createSemaphore(flashReq, 5, "flashReq");
    createSemaphore(resource, 1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(errant, "Errant", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);

    putsUart0("\n\n\n\r");

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
