//******************************************************************************
// IRrc.cpp
// Version 0.0.1, 2019-07
//
//******************************************************************************

#include "application.h"
#include <SparkIntervalTimer.h>
#include "IRrc.h"

IntervalTimer timer;

// The ISR data can be changed by the ISR at any time, even mid-function
// Therefore we declare it as "volatile" to stop the compiler/CPU caching it
volatile irparams_t irparams;

//==============================================================================
// Interrupt Service Routine
// Fires every 50uS TIMER interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50us
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a the first SPACE entry gets long:
//   Ready is set; State switches to IDLE; Timing of SPACE continues.
// As soon as first MARK arrives:
//   Gap width is recorded; Ready is cleared; New logging starts
//
void IRTimer() 
{
    // Read if IR Receiver -> SPACE (LED off) or a MARK (LED on)
    // digitalRead() is very slow. 
    // Optimisation is possible, but makes the code unportable
    uint8_t irdata = (uint8_t)digitalRead(irparams.recvpin);

    irparams.timer++;  // One more 50us tick
    if (irparams.rawlen >= RAWBUF)  // Buffer overflow
        irparams.rcvstate = STATE_OVERFLOW;

    switch (irparams.rcvstate) {
        case STATE_IDLE:  // In the middle of a gap
            if (irdata == MARK) {
                if (irparams.timer < GAP_TICKS) { // Not big enough to be a gap
                    irparams.timer = 0;
                } else {  // Gap just ended
                    // Record duration; Start recording transmission
                    irparams.overflow                  = false;
                    irparams.rawlen                    = 0;
                    irparams.rawbuf[irparams.rawlen++] = irparams.timer;
                    irparams.timer                     = 0;
                    irparams.rcvstate                  = STATE_MARK;
                }
            }
            break;

        case STATE_MARK:  // Timing Mark
            if (irdata == SPACE) {  // Mark ended; Record time
                irparams.rawbuf[irparams.rawlen++] = irparams.timer;
                irparams.timer                     = 0;
                irparams.rcvstate                  = STATE_SPACE;
            }
            break;

        case STATE_SPACE:  // Timing Space
            if (irdata == MARK) {  // Space just ended; Record time
                irparams.rawbuf[irparams.rawlen++] = irparams.timer;
                irparams.timer                     = 0;
                irparams.rcvstate                  = STATE_MARK;
            } else if (irparams.timer > GAP_TICKS) {  // Space
                // A long Space, indicates gap between codes
                // Flag the current code as ready for processing Switch to STOP
                // Don't reset timer; keep counting Space width
                irparams.rcvstate = STATE_STOP;
            }
            break;

        case STATE_STOP:  // Waiting; Measuring Gap
            if (irdata == MARK)  irparams.timer = 0;  // Reset gap timer
            break;

        case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
            irparams.overflow = true;
            irparams.rcvstate = STATE_STOP;
            break;
    }

    // If requested, flash LED while receiving IR data
    if (irparams.blinkflag) {
        if (irdata == MARK) {
            // Turn user defined pin LED on
            if (irparams.blinkpin) digitalWrite(irparams.blinkpin, HIGH);
            // if no user defined LED pin, turn default LED pin on
            else digitalWrite(BLINKLED, HIGH);
        } else {
            // Turn user defined pin LED off
            if (irparams.blinkpin) digitalWrite(irparams.blinkpin, LOW);
            // if no user defined LED pin, turn default LED pin off
            else digitalWrite(BLINKLED, LOW);
        }
    }
}

//==============================================================================
IRrecv::IRrecv (int recvpin)
{
    irparams.recvpin = recvpin;
    irparams.blinkflag = 0;
}

IRrecv::IRrecv (int recvpin, int blinkpin)
{
    irparams.recvpin = recvpin;
    irparams.blinkpin = blinkpin;
    pinMode(blinkpin, OUTPUT);
    irparams.blinkflag = 0;
}

//------------------------------------------------------------------------------
// Initialization

void IRrecv::enableIRIn ()
{
    // Interrupt Service Routine - Fires every 50uS
    timer.begin(IRTimer, 50, uSec);

    // Initialize state machine variables
    irparams.rcvstate = STATE_IDLE;
    irparams.rawlen = 0;

    // Set pin modes
    pinMode(irparams.recvpin, INPUT);
}

//------------------------------------------------------------------------------
// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results

bool IRrecv::decode (decode_results *results)
{
    results->rawbuf   = irparams.rawbuf;
    results->rawlen   = irparams.rawlen;
    results->overflow = irparams.overflow;

    if (irparams.rcvstate != STATE_STOP)  return false;

    Serial.println("Attempting MAXE decode");
    if (decodeMAXE(results))  return true;

    // decodeHash returns a hash on any input.
    // Thus, it needs to be last in the list.
    // If you add any decodes, add them before this.
    if (decodeHash(results))  return true;

    // Throw away and start over
    resume();
    return false;
}

//------------------------------------------------------------------------------
// Enable/disable blinking on IR processing

void IRrecv::blink (int blinkflag)
{
    irparams.blinkflag = blinkflag;
    if (blinkflag)  pinMode(BLINKLED, OUTPUT);
}

//------------------------------------------------------------------------------
// Return if receiving new IR signals

bool IRrecv::isIdle ()
{
    return (irparams.rcvstate == STATE_IDLE || irparams.rcvstate == STATE_STOP)
            ? true : false;
}

//------------------------------------------------------------------------------
// Restart the ISR state machine

void IRrecv::resume ()
{
    irparams.rcvstate = STATE_IDLE;
    irparams.rawlen = 0;
}

//------------------------------------------------------------------------------
// hashdecode - decode an arbitrary IR code.
// Instead of decoding using a standard encoding scheme (e.g. Sony, NEC, RC5), 
// the code is hashed to a 32-bit value.
// The algorithm: look at the sequence of MARK signals, and see if each one
// is shorter (0), the same length (1), or longer (2) than the previous.
// Do the same with the SPACE signals. Hash the resulting sequence of 
// 0's, 1's, and 2's to a 32-bit value. This will give a unique value for 
// each different code (probably), for most code systems.
// Compare two tick values, returning 0 if newval is shorter, 
// 1 if newval is equal, and 2 if newval is longer.
// Use a tolerance of 20%.

int IRrecv::compare (unsigned int oldval, unsigned int newval)
{
    if      (newval < oldval * .8)  return 0;
    else if (oldval < newval * .8)  return 2;
    else                            return 1;
}

//------------------------------------------------------------------------------
// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
// Converts the raw code values into a 32-bit hash code.
// Hopefully this code is unique for each button.
// This isn't a "real" decoding, just an arbitrary value.

#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

bool IRrecv::decodeHash (decode_results *results)
{
    long hash = FNV_BASIS_32;

    // Require at least 6 samples to prevent triggering on noise
    if (results->rawlen < 6)  return false;

    for (int i = 1; (i + 2) < results->rawlen; i++) {
        int value = compare(results->rawbuf[i], results->rawbuf[i+2]);
        // Add value into the hash
        hash = (hash * FNV_PRIME_32) ^ value;
    }

    results->value = hash;
    results->bits = 32;
    results->decode_type = UNKNOWN;
    return true;
}

//------------------------------------------------------------------------------
// Check the timing of Marks when decode.
// Due to sensor lag, when received, Marks tend to be 100us too long

bool IRrecv::chkMark (int measured_ticks, int desired_us)
{
    Serial.print("Checking mark (actual vs desired): ");
    Serial.print(measured_ticks * USECPERTICK, DEC);
    Serial.print("us vs ");
    Serial.print(desired_us, DEC);
    Serial.print("us : "); 
    Serial.print(TICKS_LOW(desired_us + MARK_EXCESS) * USECPERTICK, DEC);
    Serial.print(" <= ");
    Serial.print(measured_ticks * USECPERTICK, DEC);
    Serial.print(" <= ");
    Serial.print(TICKS_HIGH(desired_us + MARK_EXCESS) * USECPERTICK, DEC);

    bool passed = ((measured_ticks >= TICKS_LOW (desired_us + MARK_EXCESS))
                && (measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS)));
    if (passed) Serial.println("?; Passed");
    else        Serial.println("?; Failed"); 
    return passed;
}

//------------------------------------------------------------------------------
// Check the timing of Spaces when decode.
// Due to sensor lag, when received, Spaces tend to be 100us too short

bool IRrecv::chkSpace (int measured_ticks, int desired_us)
{
    Serial.print("Checking space (actual vs desired): ");
    Serial.print(measured_ticks * USECPERTICK, DEC);
    Serial.print("us vs ");
    Serial.print(desired_us, DEC);
    Serial.print("us : "); 
    Serial.print(TICKS_LOW(desired_us - MARK_EXCESS) * USECPERTICK, DEC);
    Serial.print(" <= ");
    Serial.print(measured_ticks * USECPERTICK, DEC);
    Serial.print(" <= ");
    Serial.print(TICKS_HIGH(desired_us - MARK_EXCESS) * USECPERTICK, DEC);

    bool passed = ((measured_ticks >= TICKS_LOW (desired_us - MARK_EXCESS))
                && (measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS)));
    if (passed) Serial.println("?; Passed");
    else        Serial.println("?; Failed"); 
    return passed;
}

//------------------------------------------------------------------------------
// Decode MAXE protocol

bool IRrecv::decodeMAXE (decode_results *results) 
{
    unsigned long addr = 0;
    unsigned long data = 0;
    int offset = 1;  // Skip the gap reading

    // Check the amount of data
    // 1 gap + 2 header + 2*52 bits + 3 footer
    if (irparams.rawlen != 1 + 2 + (2 * MAXE_BITS) + 3)  return false;

    // Check header
    if (!chkMark (results->rawbuf[offset++], MAXE_HDR_MARK ))  return false;
    if (!chkSpace(results->rawbuf[offset++], MAXE_HDR_SPACE))  return false;

    // Read address
    for (int i = 0; i < 32; i++) {
        if (!chkMark(results->rawbuf[offset++], MAXE_BIT_MARK)) return false;
        if (chkSpace(results->rawbuf[offset], MAXE_ONE_SPACE))
            addr = (addr << 1) | 1;
        else if (chkSpace(results->rawbuf[offset], MAXE_ZERO_SPACE))
            addr = (addr << 1) | 0;
        else  return false;
        offset++;
    }
    // Read data
    for (int i = 0; i < 20; i++) {
        if (!chkMark(results->rawbuf[offset++], MAXE_BIT_MARK)) return false;
        if (chkSpace(results->rawbuf[offset], MAXE_ONE_SPACE))
            data = (data << 1) | 1;
        else if (chkSpace(results->rawbuf[offset], MAXE_ZERO_SPACE))
            data = (data << 1) | 0;
        else  return false;
        offset++;
    }
    // Check footer
    if (!chkMark (results->rawbuf[offset++], MAXE_HDR_MARK ))  return false;
    if (!chkSpace(results->rawbuf[offset++], MAXE_HDR_SPACE))  return false;
    if (!chkMark (results->rawbuf[offset++], MAXE_BIT_MARK ))  return false;

    // Success
    results->value = data;
    results->address = addr;
    results->decode_type = MAXE;
    results->bits = MAXE_BITS;
    return true;
}

//==============================================================================
// Enables IR output. The khz value controls the modulation frequency in kHz.
// The IR output will be on TIMER_PWM_PIN.
// This routine is designed for 36-40KHz; if you use it for other values, it's 
// up to you to make sure it gives reasonable results.

void IRsend::enableIROut (int khz)
{
    // Disable the Timer Interrupt (which is used for receiving IR)
    timer.end();

    irout_khz = khz;
    pinMode(TIMER_PWM_PIN, OUTPUT);
    digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low
}

//------------------------------------------------------------------------------
// Sends an IR mark for the specified number of microseconds.
// The mark output is modulated at the PWM frequency.

void IRsend::mark (unsigned int time)
{
    analogWrite(TIMER_PWM_PIN, 128, irout_khz*1000);
    if (time > 0) delayMicroseconds(time);
}

//------------------------------------------------------------------------------
// Leave pin off for time (given in microseconds)
// Sends an IR space for the specified number of microseconds.
// A space is no output, so the PWM output is disabled.

void IRsend::space (unsigned int time)
{
    analogWrite(TIMER_PWM_PIN, 0, irout_khz*1000);
    if (time > 0) delayMicroseconds(time);
}

//------------------------------------------------------------------------------
// Send Raw IR code

void IRsend::sendRaw (const unsigned int buf[], unsigned int len, unsigned int hz)
{
    // Set IR carrier frequency
    enableIROut(hz);

    for (unsigned int i = 0; i < len; i++) {
        if (i & 1)  space(buf[i]);
        else        mark (buf[i]);
    }
    space(0);  // Always end with the LED off
}

//------------------------------------------------------------------------------
// Send MAXE IR code

void IRsend::sendMAXE (unsigned long addr, unsigned long data)
{
    // Set IR carrier frequency
    enableIROut(38);

    // Header
    mark(MAXE_HDR_MARK);
    space(MAXE_HDR_SPACE);

    // Address, 32bits
    for (unsigned long mask = 1UL << (32 - 1); mask; mask >>= 1) {
        mark(MAXE_BIT_MARK);
        if (addr & mask)    space(MAXE_ONE_SPACE);
        else                space(MAXE_ZERO_SPACE);
    }
    // Data, 20bits
    for (unsigned long mask = 1UL << (20 - 1); mask; mask >>= 1) {
        mark(MAXE_BIT_MARK);
        if (data & mask)    space(MAXE_ONE_SPACE);
        else                space(MAXE_ZERO_SPACE);
    }
    // Footer
    mark(MAXE_HDR_MARK);
    space(MAXE_HDR_SPACE);
    mark(MAXE_BIT_MARK);
    space(0);  // Always end with the LED off
}
