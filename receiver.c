#include "receiver.h"

/*** BEGIN PROTOTYPES ***/
ISR(PCINT2_vect, ISR_NAKED);
ISR(INT0_vect, ISR_NAKED);
ISR(INT1_vect, ISR_NAKED);
ISR(PCINT0_vect, ISR_NAKED);

static int16_t fastdiv8(int16_t x);
/*** END PROTOTYPES ***/

/*** BEGIN VARIABLES ***/
/*
 * Careful! Making these volatile makes it spew r24 crap in the _middle_ of
 * asm volatile statements. Always check assembler output of interrupt
 * routines.
 */
static uint16_t RxChannel1;
static uint16_t RxChannel2;
static uint16_t RxChannel3;
static uint16_t RxChannel4;

register uint16_t i_tmp asm("r2");               // ISR vars
register uint16_t RxChannel1Start asm("r4");
register uint16_t RxChannel2Start asm("r6");
register uint16_t RxChannel3Start asm("r8");
register uint16_t RxChannel4Start asm("r10");
register uint8_t i_sreg asm("r12");
/*** END VARIABLES ***/

/*** BEGIN RECEIVER INTERRUPTS ***/
/*
 * Rx interrupts with inline assembler that makes them much faster.
 * Please verify that GCC does not inject anything crazy in here,
 * such as completely unused movs that clobber other registers.
 */

ISR(PCINT2_vect, ISR_NAKED)
{
  if(RX_ROLL) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel1Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel1 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(INT0_vect, ISR_NAKED)
{
  if(RX_PITCH) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel2Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel2 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(INT1_vect, ISR_NAKED)
{
  if(RX_COLL) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel3Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel3 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(PCINT0_vect, ISR_NAKED)
{
  if(RX_YAW) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel4Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel4 = i_tmp;
  }
  asm volatile ("reti");
}
/*** END RECEIVER INTERRUPTS ***/


void receiverSetup()
{
  RX_ROLL_DIR   = INPUT;
  RX_PITCH_DIR  = INPUT;
  RX_COLL_DIR   = INPUT;
  RX_YAW_DIR    = INPUT;

  RX_ROLL   = 0;
  RX_PITCH  = 0;
  RX_COLL   = 0;
  RX_YAW    = 0;
  
  /*
   * timer1 (16bit) - run at 8MHz, used to measure Rx pulses
   * and to control ESC/servo pulse
   */
  TCCR1B = _BV(CS10);

  /*
   * Enable Rx pin interrupts
   */
  PCICR = _BV(PCIE0) | _BV(PCIE2);  // PCINT0..7, PCINT16..23 enable
  PCMSK0 = _BV(PCINT7);      // PB7
  PCMSK2 = _BV(PCINT17);      // PD1
  EICRA = _BV(ISC00) | _BV(ISC10);  // Any change INT0, INT1
  EIMSK = _BV(INT0) | _BV(INT1);    // External Interrupt Mask Register

}

/*
 * This adding 7 business is to emulate exactly a signed
 * divide at the zero point (-7 through 7 will become 0).
 */
int16_t fastdiv8(int16_t x) {
  if(x < 0)
    x+= 7;
  return x >> 3;
}

/*
 * Copy, scale, and offset the Rx inputs from the interrupt-modified
 * registers.
 *
 * If an intterupt occurs that updates an Rx variable here, SREG will be
 * copied to i_sreg. Bit 7 will not be set in that interrupt as it is
 * cleared by hardware, so i_sreg will not ever match 0xff. We use this
 * as a zero-cost flag to read again to avoid possibly-corrupted values.
 *
 * I could not find any way to avoid the optimizer killing the retry or
 * reordering it to be unsafe other than by doing the set in inline
 * assembler with a memory barrier.
 */
void receiverGetChannels(struct RX_STATE_S *state)
{
  uint8_t t = 0xff;
  do {
    asm volatile("mov %0, %1":"=r" (i_sreg),"=r" (t)::"memory");
    state->roll = fastdiv8(RxChannel1 - 1520 * 8);
    state->pitch = fastdiv8(RxChannel2 - 1520 * 8);
    state->collective = fastdiv8(RxChannel3 - 1120 * 8);
    state->yaw = fastdiv8(RxChannel4 - 1520 * 8);
  } while(i_sreg != t);
#ifdef TWIN_COPTER
  state->orgPitch = state->pitch;
#endif
}

void receiverStickCenter()
{
  struct RX_STATE_S rxState;
  uint8_t i;
  while(1) {
    receiverGetChannels(&rxState);
    i = abs(rxState.roll) + abs(rxState.pitch) + abs(rxState.yaw);
    LED = 1;
    while(i) {
      LED = 0;
      i--;
    }
  }
}
