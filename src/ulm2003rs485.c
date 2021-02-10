#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/twi.h>
#include <stdint.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846264338327950288
#endif
#ifndef RS485ADR
    #define RS485ADR 0x02
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

    I/O mode select (RX/TX):
        Pin         Bank            Bit
        3           D               3

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
                        2           OUT
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
static void serialInit();
static void serialHandleData();

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
    DDRD = DDRD | 0xC4;
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

    currentVelocityCounter[0] = 0;
    currentVelocityCounter[1] = 0;

    /*
        Setup serial port (RS485 half duplex mode)
    */
    serialInit();

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
        serialHandleData(); /* In case we received data, handle that data ... */
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

/*
    ============================================
    Serial protocol handler (RS485, half duplex)
    ============================================
*/

#ifndef SERIAL_RINGBUFFER_SIZE
    #define SERIAL_RINGBUFFER_SIZE 256
#endif

struct ringBuffer {
    volatile unsigned long int dwHead;
    volatile unsigned long int dwTail;

    volatile unsigned char buffer[SERIAL_RINGBUFFER_SIZE];
};

static inline void ringBuffer_Init(struct ringBuffer* lpBuf) {
    lpBuf->dwHead = 0;
    lpBuf->dwTail = 0;
}
static inline bool ringBuffer_Available(struct ringBuffer* lpBuf) {
    return (lpBuf->dwHead != lpBuf->dwTail) ? true : false;
}
static inline bool ringBuffer_Writable(struct ringBuffer* lpBuf) {
    return (((lpBuf->dwHead + 1) % SERIAL_RINGBUFFER_SIZE) != lpBuf->dwTail) ? true : false;
}
static inline unsigned long int ringBuffer_AvailableN(struct ringBuffer* lpBuf) {
    if(lpBuf->dwHead >= lpBuf->dwTail) {
        return lpBuf->dwHead - lpBuf->dwTail;
    } else {
        return (SERIAL_RINGBUFFER_SIZE - lpBuf->dwTail) + lpBuf->dwHead;
    }
}
static inline unsigned long int ringBuffer_WriteableN(struct ringBuffer* lpBuf) {
    return SERIAL_RINGBUFFER_SIZE - ringBuffer_AvailableN(lpBuf);
}

static unsigned char ringBuffer_ReadChar(struct ringBuffer* lpBuf) {
    char t;

    if(lpBuf->dwHead == lpBuf->dwTail) {
        return 0x00;
    }

    t = lpBuf->buffer[lpBuf->dwTail];
    lpBuf->dwTail = (lpBuf->dwTail + 1) % SERIAL_RINGBUFFER_SIZE;

    return t;
}
static unsigned long int ringBuffer_ReadChars(
    struct ringBuffer* lpBuf,
    unsigned char* lpOut,
    unsigned long int dwLen
) {
    char t;
    unsigned long int i;

    for(i = 0; i < dwLen; i=i+1) {
        if(lpBuf->dwHead == lpBuf->dwTail) {
            return 0x00;
        }

        t = lpBuf->buffer[lpBuf->dwTail];
        lpBuf->dwTail = (lpBuf->dwTail + 1) % SERIAL_RINGBUFFER_SIZE;
        lpOut[i] = t;
    }

    return i;
}
static uint16_t ringBuffer_ReadINT16(
    struct ringBuffer* lpBuf
) {
    unsigned char tmp[2];

    if(ringBuffer_AvailableN(lpBuf) < 2) { return 0; }

    tmp[0] = ringBuffer_ReadChar(lpBuf);
    tmp[1] = ringBuffer_ReadChar(lpBuf);

    return ((uint16_t)(tmp[0])) | (((uint16_t)(tmp[1])) << 8);
}
static uint32_t ringBuffer_ReadINT32(
    struct ringBuffer* lpBuf
) {
    unsigned char tmp[4];

    if(ringBuffer_AvailableN(lpBuf) < 4) { return 0; }

    tmp[0] = ringBuffer_ReadChar(lpBuf);
    tmp[1] = ringBuffer_ReadChar(lpBuf);
    tmp[2] = ringBuffer_ReadChar(lpBuf);
    tmp[3] = ringBuffer_ReadChar(lpBuf);

    return ((uint16_t)(tmp[0]))
           | (((uint16_t)(tmp[1])) << 8)
           | (((uint16_t)(tmp[2])) << 16)
           | (((uint16_t)(tmp[3])) << 24);
}
static signed long int ringBuffer_ReadSignedINT32(
    struct ringBuffer* lpBuf
) {
    unsigned char tmp[4];

    if(ringBuffer_AvailableN(lpBuf) < 4) { return 0; }

    tmp[0] = ringBuffer_ReadChar(lpBuf);
    tmp[1] = ringBuffer_ReadChar(lpBuf);
    tmp[2] = ringBuffer_ReadChar(lpBuf);
    tmp[3] = ringBuffer_ReadChar(lpBuf);

    uint32_t dwUnsigned = (((uint16_t)(tmp[0]))
           | (((uint16_t)(tmp[1])) << 8)
           | (((uint16_t)(tmp[2])) << 16)
           | (((uint16_t)(tmp[3])) << 24)) & 0x7FFFFFFF;

    signed long int res = ((tmp[3] & 0x80) == 0) ? dwUnsigned : -1 * dwUnsigned;

    return res;
}



static void ringBuffer_WriteChar(
    struct ringBuffer* lpBuf,
    unsigned char bData
) {
    if(((lpBuf->dwHead + 1) % SERIAL_RINGBUFFER_SIZE) == lpBuf->dwTail) {
        return; /* Simply discard data */
    }

    lpBuf->buffer[lpBuf->dwHead] = bData;
    lpBuf->dwHead = (lpBuf->dwHead + 1) % SERIAL_RINGBUFFER_SIZE;
}
static void ringBuffer_WriteChars(
    struct ringBuffer* lpBuf,
    unsigned char* bData,
    unsigned long int dwLen
) {
    unsigned long int i;

    for(i = 0; i < dwLen; i=i+1) {
        ringBuffer_WriteChar(lpBuf, bData[i]);
    }
}
static void ringBuffer_WriteINT16(
    struct ringBuffer* lpBuf,
    uint16_t bData
) {
    ringBuffer_WriteChar(lpBuf, (unsigned char)(bData & 0xFF));
    ringBuffer_WriteChar(lpBuf, (unsigned char)((bData >> 8) & 0xFF));
}
static void ringBuffer_WriteINT32(
    struct ringBuffer* lpBuf,
    uint16_t bData
) {
    ringBuffer_WriteChar(lpBuf, (unsigned char)(bData & 0xFF));
    ringBuffer_WriteChar(lpBuf, (unsigned char)((bData >> 8) & 0xFF));
    ringBuffer_WriteChar(lpBuf, (unsigned char)((bData >> 16) & 0xFF));
    ringBuffer_WriteChar(lpBuf, (unsigned char)((bData >> 24) & 0xFF));
}



static volatile struct ringBuffer rbRX;
static volatile struct ringBuffer rbTX;

static unsigned char bOwnAddressRS485;

static inline void serialModeRX() {
    /*
        Set to receive mode on RS485 driver
        Toggle receive enable bit on UART, disable transmit enable bit
    */
    PORTD = PORTD & (~(0x08)); /* Set RE and DE to low (RE: active, DE: inactive) */
    UCSR0B = (UCSR0B & (~0xE8)) | 0x10 | 0x80; /* Disable all transmit interrupts, enable receiver, enable receive complete interrupt */
    return;
}

static inline void serialModeTX() {
    /*
        Set to transmit mode on RS485 driver
        and toggle transmit enable bit in UART
    */
    PORTD = PORTD | 0x08; /* Set RE and DE to high (RE: inactive, DE: active) */
    UCSR0B = (UCSR0B & (~0x90)) | 0x08 | 0x20; /* Enable UDRE interrupt handler, enable transmitter and disable receive interrupt & receiver */
    return;
}

ISR(USART_RX_vect) {
    #ifndef FRAMAC_SKIP
        cli();
    #endif
    ringBuffer_WriteChar(&rbRX, UDR0);
    #ifndef FRAMAC_SKIP
        sei();
    #endif
}

ISR(USART_UDRE_vect) {
    /*
        Transmit as long as data is available to transmit. If there
        is no more data we simply stop to transmit and enter receive mode
        again
    */
    #ifndef FRAMAC_SKIP
        cli();
    #endif

    if(ringBuffer_Available(&rbTX) != true) {
        /* Disable transmit mode again ... */
        serialModeRX();
    } else {
        /* Shift next byte to the outside world ... */
        UDR0 = ringBuffer_ReadChar(&rbTX);
    }

    #ifndef FRAMAC_SKIP
        sei();
    #endif
}

static void serialInit() {
    #ifndef FRAMAC_SKIP
		cli();
	#endif

    /*
        Initialize structures
    */
    ringBuffer_Init(&rbRX);
    ringBuffer_Init(&rbTX);

    /*
        Initialize our own address to the default address
    */
    bOwnAddressRS485 = RS485ADR;

    /*
        Initialize UART
    */
    UBRR0   = 34;
    UCSR0A  = 0x02;
    UCSR0B  = 0x00;
    UCSR0C  = 0x06;

    /*
        Enable serial receiver on RS485 interface
    */
    serialModeRX();

    #ifndef FRAMAC_SKIP
		sei();
	#endif
}

static char serialHandleData__RESPONSE_IDENTIFY[20] = {
    0x00, /* Address */
    20, /* Length */
    0xb7, 0x9a, 0x72, 0xe1, 0x03, 0x6a, 0xeb, 0x11, 0x45, 0x80, 0xb4, 0x99, 0xba, 0xdf, 0x00, 0xa1, /* UUID */
    0x01, 0x00 /* Version */
};

static signed long int serialHandleData__DECODE_UINT32_TO_SINT(
    uint32_t dwData
) {
    if((dwData & 0x80000000) == 0) {
        return (signed long int)(dwData & 0x7FFFFFFF);
    } else {
        return ((signed long int)(dwData & 0x7FFFFFFF)) * (-1);
    }
}

static void serialHandleData() {
    unsigned char bLenByte;
    unsigned char bAdrByte;
    unsigned char bCommandByte;

    unsigned char bAvailable;

    /* In case not even a header is present ... ignore */
    bAvailable = ringBuffer_AvailableN(&rbRX);
    if(bAvailable < 2) {
        return;
    }

    /*
        In case there is a header check if there is enough data ...
        In case there is not simply check later (some kind of busy
        waiting loop)
    */
    bLenByte = rbRX.buffer[(rbRX.dwTail + 1) % SERIAL_RINGBUFFER_SIZE];
    if(bAvailable < bLenByte) {
        return;
    }

    /*
        We have really received a complete message, handle that message _if_
        the destination address matches ...
    */
    bAdrByte = ringBuffer_ReadChar(&rbRX);
    if(bAdrByte == bOwnAddressRS485) {
        ringBuffer_ReadChar(&rbRX); /* skip Length byte since we already know that value */
        bCommandByte = ringBuffer_ReadChar(&rbRX);
        switch(bCommandByte) {
            case 0x00: /* Identify */
                /*
                    This is a self contained command (already fully read) that basically
                    only requests an identification string
                */
                rbRX.dwTail = (rbRX.dwTail + (bLenByte-3)) % SERIAL_RINGBUFFER_SIZE; /* Compatibility with invalid protocol: Skip any remaining bytes */
                ringBuffer_WriteChars(&rbTX, serialHandleData__RESPONSE_IDENTIFY, sizeof(serialHandleData__RESPONSE_IDENTIFY));
                serialModeTX();
                break;

            case 0x04: /* Set position */
                /*
                    Set position requires 11 bytes, now only two signed integers are added; Note
                    that they are usually transmitted unsigned with a direction flag in the
                    uppermost position ...
                */
                {
                    uint32_t newPos[2];

                    newPos[0] = ringBuffer_ReadINT32(&rbRX);
                    newPos[1] = ringBuffer_ReadINT32(&rbRX);

                    if((newPos[0] & 0x80000000) == 0) {
                        if((newPos[0] & 0x7FFFFFFF) < currentMax[0]) {
                            targetPosition[0] = newPos[0] & 0x7FFFFFFF;
                        }
                    } else {
                        if((newPos[0] & 0x7FFFFFFF) < currentMin[0]) {
                            targetPosition[0] = -1 * (newPos[0] & 0x7FFFFFFF);
                        }
                    }

                    if((newPos[1] & 0x80000000) == 0) {
                        if((newPos[1] & 0x7FFFFFFF) < currentMax[1]) {
                            targetPosition[1] = newPos[1] & 0x7FFFFFFF;
                        }
                    } else {
                        if((newPos[1] & 0x7FFFFFFF) < currentMin[1]) {
                            targetPosition[1] = -1 * (newPos[1] & 0x7FFFFFFF);
                        }
                    }
                }
                rbRX.dwTail = (rbRX.dwTail + (bLenByte-3-8)) % SERIAL_RINGBUFFER_SIZE; /* Compatibility with invalid protocol: Skip any remaining bytes */
                break;
            case 0x02: /* Set boundaries */
                {
                    uint32_t newBounds[4];

                    newBounds[0] = ringBuffer_ReadINT32(&rbRX);
                    newBounds[1] = ringBuffer_ReadINT32(&rbRX);
                    newBounds[2] = ringBuffer_ReadINT32(&rbRX);
                    newBounds[3] = ringBuffer_ReadINT32(&rbRX);

                    currentMax[0] = serialHandleData__DECODE_UINT32_TO_SINT(newBounds[0]);
                    currentMin[0] = serialHandleData__DECODE_UINT32_TO_SINT(newBounds[1]);
                    currentMax[1] = serialHandleData__DECODE_UINT32_TO_SINT(newBounds[2]);
                    currentMin[1] = serialHandleData__DECODE_UINT32_TO_SINT(newBounds[3]);
                }
                rbRX.dwTail = (rbRX.dwTail + (bLenByte-3-16)) % SERIAL_RINGBUFFER_SIZE; /* Compatibility with invalid protocol: Skip any remaining bytes */
                break;
            case 0x06: /* Set speed */
                {
                    uint32_t newSpeedDelay[2];

                    currentVelocity[0] = ringBuffer_ReadINT32(&rbRX);
                    currentVelocity[1] = ringBuffer_ReadINT32(&rbRX);
                }
                rbRX.dwTail = (rbRX.dwTail + (bLenByte-3-8)) % SERIAL_RINGBUFFER_SIZE; /* Compatibility with invalid protocol: Skip any remaining bytes */
                break;
            default:
                /* Unknown command */
                rbRX.dwTail = (rbRX.dwTail + (bLenByte-3)) % SERIAL_RINGBUFFER_SIZE;
                break;
        }
    } else {
        /*
            Discard message ...
        */
        rbRX.dwTail = (rbRX.dwTail + (bLenByte-1)) % SERIAL_RINGBUFFER_SIZE;
    }
}
