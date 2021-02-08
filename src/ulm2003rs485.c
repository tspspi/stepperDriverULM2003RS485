#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/twi.h>
#include <stdint.h>

#ifndef M_PI
        #define M_PI 3.14159265358979323846264338327950288
#endif

#ifndef __cplusplus
    typedef unsigned char bool;
    #define true 1
    #define false 0
#endif

/*
    Pin mapping used on arduino pro mini board:

    Stepper 1:
        Coil        Pin             Bank        Bit
        4           6               D           6
        3           7               D           7
        2           8               B           0
        1           9               B           1

    Stepper 2:
        Coil        Pin             Bank        Bit
        4           13              B           5
        3           12              B           4
        2           11              B           3
        1           10              B           2


    I/O bank configuration:
        PA:             0           IN      Nopull
                        1           IN      Nopull
                        2           IN      Nopull
                        3           IN      Nopull
                        4           IN      Nopull
                        5           IN      Nopull
                        6           IN      Nopull
                        7           IN      Nopull

        PB:             0           OUT     -           Stepper 1
                        1           OUT     -           Stepper 1
                        2           OUT     -           Stepper 2
                        3           OUT     -           Stepper 2
                        4           OUT     -           Stepper 2
                        5           OUT     -           Stepper 2
                        6           IN      Nopull
                        7           IN      Nopull

                        DDRB = 0x3F;
                        PORTB = 0x00;

        PC:             0           IN      Nopull
                        1           IN      Nopull
                        2           IN      Nopull
                        3           IN      Nopull
                        4           IN      Nopull
                        5           IN      Nopull
                        6           IN      Nopull      Reset
                        7           IN      Nopull

        PD:             0           -                   Serial TXD
                        1           -                   Serial RXD
                        2           IN      Nopull
                        3           IN      Nopull
                        4           IN      Nopull
                        5           IN      Nopull
                        6           OUT                 Stepper 1
                        7           OUT                 Stepper 1

                        DDRD = 0x;
                        PORTD = 0x;

*/

/*
        Master clock source will be TIMER2

        Running with one of the following settings:
                Prescaler               Frequency               OCR1A           Effective Pulse Frequency
                /64                     250 kHz                 2                       62.5 kHz
                /256                    62.5 kHz                1                       31.2 kHz
                /1024                   15.625 kHz              1                       7.812 kHz
*/

#define STEPPER_TIMERTICK_FRQ                   7812/2
#define STEPPER_TIMERTICK_PRESCALER             0x06
#define STEPPER_TIMERTICK_OVERFLOWVAL           0x01
#define STEPPER_WAKEUP_SLEEP_TICKS              10                      /* More than 1.2 ms! */


void delay(unsigned long millisecs);
static void handleStepperEvents();

static void setStepPin(
    unsigned char idxStepper,
    unsigned char idxPin,
    unsigned char bStatus
) {
    unsigned char tmp;

    if(idxStepper > 1) { return; }
    if(idxPin > 3) { return; }
    if(bStatus > 1) { bStatus = 1; }

    tmp = (idxStepper << 2) | idxPin;

    if(bStatus == 0) {
        switch(tmp) {
            case 0:     PORTB = PORTB & (~(0x02)); break;
            case 1:     PORTB = PORTB & (~(0x01)); break;
            case 2:     PORTD = PORTD & (~(0x80)); break;
            case 3:     PORTD = PORTD & (~(0x40)); break;

            case 4:     PORTB = PORTB & (~(0x04)); break;
            case 5:     PORTB = PORTB & (~(0x08)); break;
            case 6:     PORTB = PORTB & (~(0x10)); break;
            case 7:     PORTB = PORTB & (~(0x20)); break;

            default:    break;
        }
    } else {
        switch(tmp) {
            case 0:     PORTB = PORTB | 0x02; break;
            case 1:     PORTB = PORTB | 0x01; break;
            case 2:     PORTD = PORTD | 0x80; break;
            case 3:     PORTD = PORTD | 0x40; break;

            case 4:     PORTB = PORTB | 0x04; break;
            case 5:     PORTB = PORTB | 0x08; break;
            case 6:     PORTB = PORTB | 0x10; break;
            case 7:     PORTB = PORTB | 0x20; break;

            default:    break;
        }
    }
}

static unsigned char stepTable[8*4] = {
	1, 0, 0, 0,
	1, 1, 0, 0,
	0, 1, 0, 0,
	0, 1, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 1,
	0, 0, 0, 1,
	1, 0, 0, 1
};

static void setStepDisable(
    unsigned char idxStepper
) {
    unsigned long int pin;

    if(idxStepper > 1) { return; }

    for(pin = 0; pin < 4; pin=pin+1) {
        setStepPin(idxStepper, pin, 0);
    }
}

static void setStepState(
    unsigned char idxStepper,
    unsigned char stepIndex
) {
    unsigned int pin;

    if(idxStepper > 1) { return; }
    if(stepIndex > 7) { return; }

    for(pin = 0; pin < 4; pin=pin+1) {
        setStepPin(idxStepper, pin, stepTable[(stepIndex << 2) + pin]);
    }

    return;
}

/*
    Stepper current state
*/

#define DEFAULT_BOUNDS_X_MIN        1073741824
#define DEFAULT_BOUNDS_X_MAX        1073741824
#define DEFAULT_BOUNDS_Y_MIN        1073741824
#define DEFAULT_BOUNDS_Y_MAX        1073741824

#define DEFAULT_VELOCITY_X          80
#define DEFAULT_VELOCITY_Y          80

static unsigned char currentState[2];
static   signed long int currentPosition[2];
static   signed long int targetPosition[2];

static unsigned long int currentMin[2];
static unsigned long int currentMax[2];
static unsigned long int currentVelocity[2];

static unsigned long int currentVelocityCounter[2];

int main() {
	#ifndef FRAMAC_SKIP
		cli();
	#endif

	/* Setup system clock */
        TCCR0A = 0x00;
        TCCR0B = 0x03;          /* /64 prescaler */
        TIMSK0 = 0x01;          /* Enable overflow interrupt */

	/*
        Setup GPIO
    */
    DDRB = 0x3F;
    PORTB = 0x00;
    DDRD = DDRD | 0xC0;
    PORTD = 0x00;

    #ifndef FRAMAC_SKIP
		sei();
	#endif

    /*
        Reset stepper states
    */
    currentState[0]         = 0x00;
    currentState[1]         = 0x00;

    currentPosition[0]      = 0;
    currentPosition[1]      = 0;
    targetPosition[0]       = 0;
    targetPosition[1]       = 0;

    currentMin[0] = DEFAULT_BOUNDS_X_MIN;
    currentMin[1] = DEFAULT_BOUNDS_Y_MIN;

    currentMax[0] = DEFAULT_BOUNDS_X_MAX;
    currentMax[1] = DEFAULT_BOUNDS_Y_MAX;

    currentVelocity[0] = DEFAULT_VELOCITY_X;
    currentVelocity[1] = DEFAULT_VELOCITY_Y;

    /*
        Setup stepper timer
    */
    #ifndef FRAMAC_SKIP
        cli();
    #endif

    TCCR2B = 0;

    TCNT2 = 0;                              /* Current timer counter register 0 */
    TCCR2A = 0x02;                          /* CTC mode - count up to OCR2A, disable OCR output pins */
    OCR2A = 0x01;                           /* Counting up to one - triggers every pulse ... */
    TIMSK2 = 0x02;                          /* OCIE2A flag to enable interrupts on output compare */
    TCCR2B = STEPPER_TIMERTICK_PRESCALER;   /* Set prescaler, non FOCA, enable timer */

    #ifndef FRAMAC_SKIP
		sei();
	#endif

	for(;;) {
        handleStepperEvents();
	}
}

/*
    ================
    = System clock =
    ================
*/

#define SYSCLK_TIMER_OVERFLOW_MICROS                    (64L * 256L * (F_CPU / 1000000L))
#define SYSCLK_MILLI_INCREMENT                          (SYSCLK_TIMER_OVERFLOW_MICROS / 1000)
#define SYSCLK_MILLIFRACT_INCREMENT                     ((SYSCLK_TIMER_OVERFLOW_MICROS % 1000) >> 3)
#define SYSCLK_MILLIFRACT_MAXIMUM                       (1000 >> 3)

volatile unsigned long int systemMillis                 = 0;
volatile unsigned long int systemMilliFractional        = 0;
volatile unsigned long int systemMonotonicOverflowCnt   = 0;

/*@
        assigns systemMillis, systemMilliFractional, systemMonotonicOverflowCnt;
*/
ISR(TIMER0_OVF_vect) {
        unsigned long int m, f;

        m = systemMillis;
        f = systemMilliFractional;

        m = m + SYSCLK_MILLI_INCREMENT;
        f = f + SYSCLK_MILLIFRACT_INCREMENT;

        if(f >= SYSCLK_MILLIFRACT_MAXIMUM) {
                f = f - SYSCLK_MILLIFRACT_MAXIMUM;
                m = m + 1;
        }

        systemMonotonicOverflowCnt = systemMonotonicOverflowCnt + 1;

        systemMillis = m;
        systemMilliFractional = f;
}

/*@
        requires \valid(&SREG);
        assigns SREG;
*/
unsigned long int micros() {
        uint8_t srOld = SREG;
        unsigned long int overflowCounter;
        unsigned long int timerCounter;

        #ifndef FRAMAC_SKIP
                cli();
        #endif
        overflowCounter = systemMonotonicOverflowCnt;
        timerCounter = TCNT0;

        /*
                Check for pending overflow that has NOT been handeled up to now
        */
        if(((TIFR0 & 0x01) != 0) && (timerCounter < 255)) {
                overflowCounter = overflowCounter + 1;
        }

        SREG = srOld;

        return ((overflowCounter << 8) + timerCounter) * (64L / (F_CPU / 1000000L));
}

/*@
        requires millisecs >= 0;
        requires \valid(&SREG);
        assigns SREG;
*/
void delay(unsigned long millisecs) {
        //uint16_t lastMicro;
        unsigned int lastMicro;
        /*
                Busy waiting the desired amount of milliseconds ... by
                polling mircos
        */
        lastMicro = (unsigned int)micros();
        /*@
                loop assigns lastMicro;
                loop assigns millisecs;
        */
        while(millisecs > 0) {
                // uint16_t curMicro = (uint16_t)micros();
                unsigned int curMicro = micros();
                if(curMicro - lastMicro >= 1000)  {
                        /* Every ~ thousand microseconds tick ... */
                        lastMicro = lastMicro + 1000;
                        millisecs = millisecs - 1;
                }
        }
        return;
}

/*
==========================
= Stepper timer routines =
==========================+

Note that the actual work is not done inside an ISR. This is done that
way since it's better to miss a step than to stay inside an infinite loop
due to nesting of communication processing and timer interrupts
*/

volatile bool bIntTriggered = false;

ISR(TIMER2_COMPA_vect) {
    TCCR2B = TCCR2B; /* Dummy write */
    bIntTriggered = true;
}

static void handleStepperEvents() {
    int iStepper;

    /* Called from non interrupt context */
    if(bIntTriggered != true) { return; }
    bIntTriggered = false;

    /*
        - Count next step counters. If they reach the target value then
            - Check if the stepper is at it's target position. If not move
              one step into the direction.
    */
    for(iStepper = 0; iStepper < 2; iStepper = iStepper + 1){
        currentVelocityCounter[iStepper] = currentVelocityCounter[iStepper] + 1;
        if(currentVelocityCounter[iStepper] == currentVelocity[iStepper]) {
            currentVelocityCounter[iStepper] = 0;

            if(currentPosition[iStepper] > targetPosition[iStepper]) {
                if(currentState[iStepper] == 0) {
                    currentState[iStepper] = 7;
                } else {
                    currentState[iStepper] = currentState[iStepper] - 1;
                }
                currentPosition[iStepper] = currentPosition[iStepper] - 1;
                setStepState(iStepper, currentState[iStepper]);
            } else if(currentPosition[iStepper] < targetPosition[iStepper]){
                currentState[iStepper] = (currentState[iStepper] + 1) % 7;
                currentPosition[iStepper] = currentPosition[iStepper] + 1;
                setStepState(iStepper, currentState[iStepper]);
            } else {
                setStepDisable(iStepper);
            }
        }
    }
}
