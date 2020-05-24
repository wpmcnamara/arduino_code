#include <avr/power.h>

//These define the width of the on times.  Values are timer counts.  High, med, low are power settings.
#define WIDTH_LOW 6
#define WIDTH_MED 28
//Don't make this any higher as it makes the PWM generation unstable for phase b.
#define WIDTH_HIGH 45

//Counter reset value so we get the correct PWM frequency.
#define TOP 136

//This is the mid point of the clocking cycle.  Phase A is the first half
//phase B is the second
#define MID (TOP/2)

//Button presses
#define BUTTON_NONE 0
#define BUTTON_ONE 1
#define BUTTON_TWO 2
#define BUTTON_THREE 3

//The ADC runs at 125kHz.  Autotriggered conversion takes 13.5 clock cycles
//This is the number of consecutive ADC samples needed
//to register a button press.  Works out to ~75ms.
#define DEBOUNCE_TIMER 695

//State variables for each phase waveform generation
unsigned char state_a=0;
unsigned char state_b=0;

//These will hold the appropraite values for each state based on the output power.
//The state0 values are fixed and could be probably replaced by constants.
unsigned char ocr1a_state0=0;
unsigned char ocr1b_state0=MID;
unsigned char ocr1a_state1;
unsigned char ocr1b_state1;

//variables for handling output state machine for chirp generation
unsigned char cycle_count=0;  //count of PWM cycles for the output state machine.
unsigned char modulation_state=0;  //state of the chirp modulation generator
unsigned char modulation_enable=0;  //Should the PWMs be on or off based on the modulation state
unsigned char tx_enable=0;  //Should we be transmitting or not
unsigned char enable_state=1;  //current combination of transmit enable and pwm enable

//output power
unsigned char power=1;

//variables for input handling and button debouncing
unsigned char in_val;  //ADC results
unsigned int debounce_count;  //number of times we've gottent the same button reading
unsigned char in_proc;  //Flag to signify we have processesed the current button press
unsigned char in;  //Button pressed (ADC converted to a button)
unsigned char in_last; //The prior button read from the ADC

void set_power(unsigned char power);

void setup() {
  //Running on an Adafruit 5V Trinket.  enable 16MHz clock.
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);

  //Pins 0,1,3, and 4 are outputs.  Pin 2 is an Input.
  DDRB=B00011011;
  //Pin 2 input pullup disable as this will be the A/D input
  MCUCR |= _BV(PUD);
  //Pin 1 and 4 low;
  PORTB &= B11101101;

  //We start with the power level at the lowest setting.
  set_power(1);

  //Set the ADC for VCC reference and Left Adjust the conversion result.  We can ignore the bottom two bits
  //and treat it as an 8 bit value.
  ADMUX=_BV(ADLAR) | _BV(MUX0); 
  DIDR0 |= _BV(ADC1D); // Diable digital input PB2 for ADC use
  //Enable the ADC and set for auto trigger.  Select CLK/128 prescaler for ADC.
  ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  //Trigger the first ADC.  It will auto run from this point forward.
  ADCSRA |= _BV(ADSC);

  //Turn of interrupts while we configure them.
  cli();

  //Turn off Timer0.  This disables some Arduino functionality, specifically millis(), but will ensure
  //that we don't execute a Timer0 interrupt and mess with the timing of the PWM generation.
  TCCR0B=0;
 
  //Enable output compare match interrupts
  TIMSK = _BV(OCIE1A) | _BV(OCIE1B);

  //Set the reset count value for Timer1.  This is chosen to get as close to 15kHz PWM frequency as
  //possible.
  OCR1C = TOP;
  //Set timer 1 to clear on OCR1C match.  Set to toggle output compare A on match -- this is Phase A
  TCCR1 = _BV(CTC1) | _BV(COM1A0); //Toggle on match
  //Set time 1 to toggle output compare B on match -- this is Phase B
  GTCCR = _BV(COM1B0); //Toggle on match

  //initialize the phase A/phase B wave form state machines.
  OCR1A=ocr1a_state0;
  OCR1B=ocr1b_state0;
  
  //re-enable interrupts
  sei();
  
  //enable counter, Div by 8 prescale (2MHz clock).  For a 15kHz PWM frequency, this gives us ~133
  //steps total, or ~67 per phase.  In reality we have something less that 45, since we are 
  //partially generating the waveform via software.  A pulse width more than 45 results in instability
  //in phase 2 and a width less that 5 or 6 results in problems in phase 1.
  TCCR1 |= _BV(CS12); 
}

void loop() {
  //This is the state machine for the PWM enable, used to generate the "chirp" waveform used
  //by the transmitter.  During the high states, the PWM is outputing a two phase 15kHz
  //PWM signal.
  //    __________   _              ___
  //   |          | | |            |
  // __|          |_| |____________|
  //   0--------->1>2>3----------->0
  //All counts define the width of each state in PWM cycles.  The approximate timing of each
  //state:  0 - 12.7ms, 1 - 3.5ms, 2 - 3.3ms, 3 - 13ms
  switch(modulation_state) {
    case 0:
      if(cycle_count>=189) {
        cycle_count=0;
        modulation_state=1;
        modulation_enable=0;
      } 
      break;
    case 1:
       if(cycle_count>=52) {
        cycle_count=0;
        modulation_state=2;
        modulation_enable=1;
      } 
      break;   
    case 2:
      if(cycle_count>=49) {
        cycle_count=0;
        modulation_state=3;
        modulation_enable=0;
      }     
      break;
    case 3:
      if(cycle_count>=193) {
        cycle_count=0;
        modulation_state=0;
        modulation_enable=1;
      } 
      break;      
  }

  //Control the PWM output based on modulation enable and transmit enable.  The modulation
  //enable controls the CW modulation of the PWM output and the transmit enable is based on
  //the user setting of whether the transmitter should be on at all.  To keep code execution
  //as short as possible, we only change the PWM settings when the combined state changes.
  //Both modulation and transmit enables must be set for the PWM outputs to be active.
  if(enable_state!=(modulation_enable && tx_enable)) {
    //Save our new state for next time.
    enable_state=modulation_enable && tx_enable;
    if(enable_state) { 
      //PWM output turned on.
      //We set the whole register here to cut down on code execution.  As a result, we need
      //to make sure and preserve the CTC setting as the timer continues to run even when
      //the PWM output is off.
      TCCR1 = _BV(CS12) | _BV(CTC1) | _BV(COM1A0);  //Phase A, toggle on match
      GTCCR = _BV(COM1B0); //Phase B, toggle on match
    } else {
      //PWM output turned off.  
      //We don't disable the PWM match, we simply set it so that the output is forced high on
      //match.  The counter continues to run and the PWM match continues to occur.  This is
      //what generates the state clock for modulating the PWM output.  Make sure and preserve 
      // the CTC setting as the timer continues to run even when the PWM output is off.
      TCCR1 = _BV(CS12) | _BV(CTC1) | _BV(COM1A1)| _BV(COM1A0); //Phase A, force high
      GTCCR = _BV(COM1B1)| _BV(COM1B0); //Phase B, force high
    }
  }

  //Check to see if we have a new ADC value ready for processing.  This value will
  //represent the state of the three buttons.  Note that if multiple buttons are pressed
  //The highest priority button will register.  
  //Button priority, from low to high: down, up, tx on/off
  //The buttons are connected to the ADC input such that they form a voltage divider, setting
  //the ADC input to a postivity voltage based on the button pushed.
  //
  //                 ADC     ----\/\/\/\---*----------*-----+5V
  //                  ^      |      1K     |          |
  //                  |        /            /           /
  //                  |       / SW1        / SW2       / SW3
  //         10K      |      |            |           |
  // GND---\/\/\/\----*------*---\/\/\/\--*--\/\/\/\--
  //                               10K         10K
  //
  //The resulting (theoretical) voltages and approximate ADC values are below.
  //Each row gives a worst case +/- 10% range, as well as the exact value.  This
  //should cover an resistor variation.  VCC to the resistor divider must be the
  //regulated VCC for the ATTiny and not the unregulated Vusb or the voltages
  //and ADC values will vary with Vusb.
  //
  //SW1:  4.46V (227), 4.55V (232), 4.62V (235)
  //SW2:  2.25V (114), 2.50V (127), 2.75V (140)
  //SW3:  1.45V (73), 1.66V (84), 1.90V (96)

  if(ADCSRA & _BV(ADIF)) {
    //Remember that we left justify the ADC value so we only have to ready the high
    //order output register and then we can treat it as an 8 bit value.
    in_val=ADCH;
    //clear the ADC flag
    ADCSRA|=_BV(ADIF);

    //We debounce the switches in software.  We require many consecutive 75ms of the same
    //button in order to register it as a "push".  Once we hit 75ms, we will process
    //the button push.  The buttons don't "repeat".  In order to register a new push of any
    //button, all buttons must be released between pushes.
    if(in_val<32) {
      in=BUTTON_NONE;
    } else if(in_val>=73 && in_val<=96) {
      in=BUTTON_ONE;
    } else if(in_val>=114 && in_val<=140) {
      in=BUTTON_TWO;
    } else if(in_val>=227 && in_val<=235) {
      in=BUTTON_THREE;
    } else {
      in=BUTTON_NONE;
    }
    //If we get the same button reading multiple times in a row, then start the debounce
    //timer.
    if(in == in_last) {
      if(debounce_count<DEBOUNCE_TIMER) {
        debounce_count++;
      } 
    //Any time we get a different button, the debounce timer gets reset and we start 
    //over.  We also need to clear the "processed" flag, if it was set, since the
    //button value has changed.
    } else {
      in_last=in;
      debounce_count=0;
      in_proc=0;
    }

    //We've timed out the debounce and have a real button press to process
    if(debounce_count>=DEBOUNCE_TIMER && !in_proc) {
      //Flag that we've processes this button press.
      in_proc=1;
      switch(in) {
        //If above minimum power, then decrease and set new power level
        case BUTTON_ONE:
          if(power>1) {
            power--;
            set_power(power);
          }
          break;
        //If below maximum power, then increase and set new power level
        case BUTTON_TWO:
          if(power<3) {
            power++;
            set_power(power);
          }
          break;
        //Toggle the transmitter on or off.
        case BUTTON_THREE:
          //Off is simple, just disable transmit.
          if(tx_enable) {
            tx_enable=0;
          } else {
            //Enabling requires more work since we don't know where in the cycle 
            //the PWM timer is.  Since we rely on knowing the state of the PWM
            //counter for correctly toggling the output, we reset the PWM to the
            //start of the cycle.

            //Enable transmit
            tx_enable=1;
            //This disabled the timer and resets the compare match behavior
            TCCR1 = _BV(CTC1) | _BV(COM1A0);  //Phase A, toggle on match
            GTCCR = _BV(COM1B0); //Phase B, toggle on match
            //reset the output compare registers to their starting values
            OCR1A=ocr1a_state0;
            OCR1B=ocr1b_state0;
            //and the output compare state machine to its initial state
            state_b=0;
            state_b=0;
            //reset the timer to its initial state
            TCNT1=0;
            //and start the timer running again.
            TCCR1 |= _BV(CS12);
          }
          break;
        case BUTTON_NONE:
          break;
      }      
    }    
  }  
}

//This function both sets the power and LED power level indicators.
//The output power is controlled by the width of the PWM signal, with each
//phase being controlled by its OCR1x value.
//
//The three LEDs are multiplexed onto two output pins.  The output truth
//table is below.
//   pin#  | 1  4
// ----------------
// level 1 | L  L
// level 2 | H  L
// level 3 | H  H
//
//
//  pin1 ----*--|<|----/\/\/\/-----VCC
//           |
//           /
//           \
//           /
//           \
//           |
//           _
//           V
//          ---
//           |
//  pin4 ----*--/\/\/\/----|>|-----GND
//
void set_power(unsigned char power) {
  switch(power) {
    case 1:
      //Pin 1 and 4 low;
      PORTB = 0;
      ocr1a_state1=WIDTH_LOW;
      ocr1b_state1=MID+WIDTH_LOW;      
      break;
    case 2:
      //Pin 1 high, pin 4 low
      PORTB=_BV(0);
      ocr1a_state1=WIDTH_MED;
      ocr1b_state1=MID+WIDTH_MED; 
      break;
    case 3:
      //Pin 1 high, pin 4 high.
      PORTB=_BV(3) | _BV(0);
      ocr1a_state1=WIDTH_HIGH;
      ocr1b_state1=MID+WIDTH_HIGH;
      break;
    default:
      break;  
  }  
}

//Phase A PWM state machine.  Only has two states that control the next
//output compare match value.  Each call toggles to the other state and
//resets the OCR1A to the appropriate match value.
ISR(TIMER1_COMPA_vect){
  if(state_a==0) {
    state_a=1;
    OCR1A=ocr1a_state1;
  } else {
    state_a=0;
    OCR1A=ocr1a_state0;
    cycle_count++;
  }
}

//Phase B PWM state machine.  Only has two states that control the next
//output compare match value.  Each call toggles to the other state and
//resets the OCR1B to the appropriate match value.
ISR(TIMER1_COMPB_vect){
  if(state_b==0) {
    state_b=1;
    OCR1B=ocr1b_state1;
  } else {
    state_b=0;
    OCR1B=ocr1b_state0;
  }
}
