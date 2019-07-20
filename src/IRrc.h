//******************************************************************************
// IRrc.h
// Version 0.0.1, 2019-07
//
//******************************************************************************

#ifndef IRrc_h
#define IRrc_h

#pragma once

// This will load the definition for common Particle variable types
#include "Particle.h"
#include "application.h"

//==============================================================================
// Supported IR protocols
// Each protocol you include costs memory and, during decode, costs time
// Disable (set to 0) all the protocols you do not need/want
//#define SEND_MAXE     1
//#define DECODE_MAXE   1

// MAXE
#define MAXE_BITS         52
#define MAXE_HDR_MARK    550
#define MAXE_HDR_SPACE  3550
#define MAXE_BIT_MARK    550
#define MAXE_ONE_SPACE  1600
#define MAXE_ZERO_SPACE  550


//==============================================================================
// Defines for Timer of Particle Photon
#define TIMER_PWM_PIN   A5

// microseconds per clock interrupt tick
#define USECPERTICK     50

// ISR State-Machine : Receiver States
#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

// Maximum length of raw duration buffer
#define RAWBUF  128  

// Minimum gap between IR transmissions
#define _GAP        5000
#define GAP_TICKS   (_GAP / USECPERTICK)

// IR detector output is active low
#define MARK    0
#define SPACE   1

// Defines for blinking the LED of Particle Photon
#define BLINKLED    D7

//------------------------------------------------------------------------------
// Pulse parms are ((X*50)-100) for the Mark and ((X*50)+100) for the Space.
// First MARK is the one after the long gap. Pulse parameters in uSec.
// Due to sensor lag, when received, Marks tend to be 100us too long and
// Spaces tend to be 100us too short.
#define MARK_EXCESS   100

// Upper and Lower percentage tolerances in measurements
#define TOLERANCE   25
#define LTOL        (1.0 - (TOLERANCE / 100.))
#define UTOL        (1.0 + (TOLERANCE / 100.))

#define TICKS_LOW(us)   (int)(((us) * LTOL / USECPERTICK))
#define TICKS_HIGH(us)  (int)(((us) * UTOL / USECPERTICK + 1))

//------------------------------------------------------------------------------
// Information for the Interrupt Service Routine

typedef struct {
    // The fields are ordered to reduce memory over caused by struct-padding
    uint8_t       rcvstate;       // State Machine state
    uint8_t       recvpin;        // Pin connected to IR data from detector
    uint8_t       blinkpin;       // Pin for blinking LED
    uint8_t       blinkflag;      // Enable blinking pin on IR processing
    uint8_t       rawlen;         // Counter of entries in rawbuf
    unsigned int  timer;          // State timer, counts 50uS ticks.
    unsigned int  rawbuf[RAWBUF]; // Raw data
    uint8_t       overflow;       // Raw buffer overflow occurred
}
irparams_t;

// Allow all parts of the code access to the ISR data
// The data can be changed by the ISR at any time, even mid-function
// Therefore we declare it as "volatile" to stop the compiler/CPU caching it
//extern volatile irparams_t irparams;

//------------------------------------------------------------------------------
// An enumerated list of all supported formats
// You do NOT need to remove entries from this list when disabling protocols

typedef enum {
    UNKNOWN = -1,
    UNUSED  =  0,
    MAXE,
}
decode_type_t;

//------------------------------------------------------------------------------
// Results returned from the decoder

class decode_results
{
    public:
        decode_type_t           decode_type;  // UNKNOWN, MAXE, ...
        unsigned long           address;      // Decoded address (max 32-bits)
        unsigned long           value;        // Decoded value (max 32-bits)
        int                     bits;         // Number of bits in decoded value
        volatile unsigned int   *rawbuf;      // Raw intervals in 50uS ticks
        int                     rawlen;       // Number of records in rawbuf
        int                     overflow;     // true if IR raw code too long
};

//------------------------------------------------------------------------------
// Main class for receiving IR

class IRrecv
{
    public:
        IRrecv (int recvpin);
        IRrecv (int recvpin, int blinkpin);

        void enableIRIn ();
        bool decode     (decode_results *results);
        void blink      (int blinkflag);
        bool isIdle     ();
        void resume     ();

    private:
        int  compare    (unsigned int oldval, unsigned int newval);
        bool decodeHash (decode_results *results);

        // Mark & Space matching functions
        //bool chk        (int measured, int desired);
        bool chkMark    (int measured_ticks, int desired_us);
        bool chkSpace   (int measured_ticks, int desired_us);

        bool  decodeMAXE    (decode_results *results) ;
};

//==============================================================================
// Main class for sending IR

class IRsend
{
    public:
        IRsend  () {}

        void enableIROut    (int khz);
        void mark           (unsigned int usec);
        void space          (unsigned int usec);
        void sendRaw        (const unsigned int buf[], unsigned int len, unsigned int hz);

        void sendMAXE       (unsigned long address, unsigned long data);

    private:
        int irout_khz;
};

//==============================================================================
#endif //IRrc_h
