// .. "Copyright (c) 2008 Robert B. Reese, Bryan A. Jones, J. W. Bruce ("AUTHORS")"
//    All rights reserved.
//    (R. Reese, reese_AT_ece.msstate.edu, Mississippi State University)
//    (B. A. Jones, bjones_AT_ece.msstate.edu, Mississippi State University)
//    (J. W. Bruce, jwbruce_AT_ece.msstate.edu, Mississippi State University)
//
//    Permission to use, copy, modify, and distribute this software and its
//    documentation for any purpose, without fee, and without written agreement is
//    hereby granted, provided that the above copyright notice, the following
//    two paragraphs and the authors appear in all copies of this software.
//
//    IN NO EVENT SHALL THE "AUTHORS" BE LIABLE TO ANY PARTY FOR
//    DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
//    OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE "AUTHORS"
//    HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//    THE "AUTHORS" SPECIFICALLY DISCLAIMS ANY WARRANTIES,
//    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
//    AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
//    ON AN "AS IS" BASIS, AND THE "AUTHORS" HAS NO OBLIGATION TO
//    PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
//
//    Please maintain this header in its entirety when copying/modifying
//    these files.
//
// *********************************************************
// keypad.c - Implements a 4x3 key scanned keypad interface.
// *********************************************************
// Implements a 4x3 key scanned keypad interface. A periodic timer interrupt is used to poll the keypad. Thanks goes to David Weaver for suggestions on improvements to the scan function.

#include <stdio.h>
#include "pic24_all.h"
#include <stdlib.h>
#include <math.h>

#define VREF 3.3

#if (HARDWARE_PLATFORM == EMBEDDED_C1)
# define RS_HIGH()        (_LATC2 = 1)
# define RS_LOW()         (_LATC2 = 0)
# define CONFIG_RS()      CONFIG_RC2_AS_DIG_OUTPUT()

# define RW_HIGH()        (_LATC1 = 1)
# define RW_LOW()         (_LATC1 = 0)
# define CONFIG_RW()      CONFIG_RC1_AS_DIG_OUTPUT()

# define E_HIGH()         (_LATC0 = 1)
# define E_LOW()          (_LATC0 = 0)
# define CONFIG_E()       CONFIG_RC0_AS_DIG_OUTPUT()

# define LCD4O          (_LATC4)
# define LCD5O          (_LATC5)
# define LCD6O          (_LATC6)
# define LCD7O          (_LATC7)
# define LCD7I          (_RC7)

# define CONFIG_LCD4_AS_INPUT() CONFIG_RC4_AS_DIG_INPUT()
# define CONFIG_LCD5_AS_INPUT() CONFIG_RC5_AS_DIG_INPUT()
# define CONFIG_LCD6_AS_INPUT() CONFIG_RC6_AS_DIG_INPUT()
# define CONFIG_LCD7_AS_INPUT() CONFIG_RC7_AS_DIG_INPUT()

# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RC4_AS_DIG_OUTPUT()
# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RC5_AS_DIG_OUTPUT()
# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RC6_AS_DIG_OUTPUT()
# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RC7_AS_DIG_OUTPUT()

#else
# define RS_HIGH()        (_LATB5 = 1)
# define RS_LOW()         (_LATB5 = 0)
# define CONFIG_RS()      CONFIG_RB5_AS_DIG_OUTPUT()

# define RW_HIGH()        (_LATB3 = 1)
# define RW_LOW()         (_LATB3 = 0)
# define CONFIG_RW()      CONFIG_RB3_AS_DIG_OUTPUT()

# define E_HIGH()         (_LATB2 = 1)
# define E_LOW()          (_LATB2 = 0)
# define CONFIG_E()       CONFIG_RB2_AS_DIG_OUTPUT()

# define LCD4O          (_LATB6)
# define LCD5O          (_LATB7)
# define LCD6O          (_LATB8)
# define LCD7O          (_LATB9)
# define LCD7I          (_RB9)

# define CONFIG_LCD4_AS_INPUT() CONFIG_RB6_AS_DIG_INPUT()
# define CONFIG_LCD5_AS_INPUT() CONFIG_RB7_AS_DIG_INPUT()
# define CONFIG_LCD6_AS_INPUT() CONFIG_RB8_AS_DIG_INPUT()
# define CONFIG_LCD7_AS_INPUT() CONFIG_RB9_AS_DIG_INPUT()

# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RB6_AS_DIG_OUTPUT()
# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RB7_AS_DIG_OUTPUT()
# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RB8_AS_DIG_OUTPUT()
# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RB9_AS_DIG_OUTPUT()
#endif

#define GET_BUSY_FLAG()  (LCD7I)


volatile uint8_t u8_newKey = 0;
unsigned short kp = 0;
unsigned short f_save = 0;
uint16_t u16_adcVal;
volatile float f_adcVal;
volatile float f_temp;
float saver[10];
static uint16_t counters;
char string[225];
int num;


/**
 Functions above this line must be redefined for
 your particular PICmicro-to-LCD interface
*/
#define CONFIG_LED1() CONFIG_RA0_AS_DIG_OUTPUT()
#define LED1  (_LATA0)     //led1 state

//Configure 4-bit data bus for output
void configBusAsOutLCD(void) {
  RW_LOW();                  //RW=0 to stop LCD from driving pins
  CONFIG_LCD4_AS_OUTPUT();   //D4
  CONFIG_LCD5_AS_OUTPUT();   //D5
  CONFIG_LCD6_AS_OUTPUT();   //D6
  CONFIG_LCD7_AS_OUTPUT();   //D7
}

//Configure 4-bit data bus for input
void configBusAsInLCD(void) {
  CONFIG_LCD4_AS_INPUT();   //D4
  CONFIG_LCD5_AS_INPUT();   //D5
  CONFIG_LCD6_AS_INPUT();   //D6
  CONFIG_LCD7_AS_INPUT();   //D7
  RW_HIGH();                   // R/W = 1, for read
}

//Output lower 4-bits of u8_c to LCD data lines
void outputToBusLCD(uint8_t u8_c) {
  LCD4O = u8_c & 0x01;          //D4
  LCD5O = (u8_c >> 1)& 0x01;    //D5
  LCD6O = (u8_c >> 2)& 0x01;    //D6
  LCD7O = (u8_c >> 3)& 0x01;    //D7
}

//Configure the control lines for the LCD
void configControlLCD(void) {
  CONFIG_RS();     //RS
  CONFIG_RW();     //RW
  CONFIG_E();      //E
  RW_LOW();
  E_LOW();
  RS_LOW();
}

//Pulse the E clock, 1 us delay around edges for
//setup/hold times
void pulseE(void) {
  DELAY_US(1);
  E_HIGH();
  DELAY_US(1);
  E_LOW();
  DELAY_US(1);
}

/* Write a byte (u8_Cmd) to the LCD.
u8_DataFlag is '1' if data byte, '0' if command byte
u8_CheckBusy is '1' if must poll busy bit before write, else simply delay before write
u8_Send8Bits is '1' if must send all 8 bits, else send only upper 4-bits
*/
void writeLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag,
              uint8_t u8_CheckBusy, uint8_t u8_Send8Bits) {

  uint8_t u8_BusyFlag;
  uint8_t u8_wdtState;
  if (u8_CheckBusy) {
    RS_LOW();            //RS = 0 to check busy
    // check busy
    configBusAsInLCD();  //set data pins all inputs
    u8_wdtState = _SWDTEN;  //save WDT enable state
    CLRWDT();          //clear the WDT timer
    _SWDTEN = 1;            //enable WDT to escape infinite wait
    do {
      E_HIGH();
      DELAY_US(1);  // read upper 4 bits
      u8_BusyFlag = GET_BUSY_FLAG();
      E_LOW();
      DELAY_US(1);
      pulseE();              //pulse again for lower 4-bits
    } while (u8_BusyFlag);
    _SWDTEN = u8_wdtState;   //restore WDT enable state
  } else {
    DELAY_MS(10); // don't use busy, just delay
  }
  configBusAsOutLCD();
  if (u8_DataFlag) RS_HIGH();   // RS=1, data byte
  else    RS_LOW();             // RS=0, command byte
  outputToBusLCD(u8_Cmd >> 4);  // send upper 4 bits
  pulseE();
  if (u8_Send8Bits) {
    outputToBusLCD(u8_Cmd);     // send lower 4 bits
    pulseE();
  }
}

void outStringLCD(char *psz_s) {
  while (*psz_s) {
    writeLCD(*psz_s, 1, 0,1);
    psz_s++;
  }
}

// Initialize the LCD, modify to suit your application and LCD
void initLCD() {
  DELAY_MS(50);          //wait for device to settle
  writeLCD(0x20,0,0,0); // 4 bit interface
  writeLCD(0x28,0,0,1); // 2 line display, 5x7 font
  writeLCD(0x28,0,0,1); // repeat
  writeLCD(0x06,0,0,1); // enable display
  writeLCD(0x0C,0,0,1); // turn display on; cursor, blink is off
  writeLCD(0x01,0,0,1); // clear display, move cursor to home
  DELAY_MS(3);
  
}

void blink1(void) {
    counters = 0;
     while (counters < 1){
     LED1 = 0 ;
     DELAY_MS(500);
     LED1 = 1 ;
     DELAY_MS(500);
     counters++;}
     LED1 = 0;
}

void blink2(void) {
    counters = 0;
     while (counters < 2){
     LED1 = 0 ;
     DELAY_MS(500);
     LED1 = 1 ;
     DELAY_MS(500);
     counters++;}
     LED1 = 0;
}

void blink3(void) {
    counters = 0;
     while (counters < 3){
     LED1 = 0 ;
     DELAY_MS(500);
     LED1 = 1 ;
     DELAY_MS(500);
     counters++;}
     LED1 = 0;
}

//Output a string to the LCD
void initdevice() {

  blink3();

  outStringLCD("*INITIALIZATION*");
  writeLCD(0xC0,0,0,1);  // cursor to 2nd line
  outStringLCD("****COMPLETE****");
  DELAY_MS(1500)
  writeLCD(0x01,0,0,1);

}

#define C0 _RB14
#define C1 _RB13
#define C2 _RB12


static inline void CONFIG_COLUMN() {
  CONFIG_RB14_AS_DIG_INPUT();
  ENABLE_RB14_PULLUP();

  CONFIG_RB13_AS_DIG_INPUT();
  ENABLE_RB13_PULLUP();

  CONFIG_RB12_AS_DIG_INPUT();
  ENABLE_RB12_PULLUP();
}

#define R0 _LATA2
#define R1 _LATA3
#define R2 _LATB4
#define R3 _LATA4

#define CONFIG_R0_DIG_OUTPUT() CONFIG_RA2_AS_DIG_OUTPUT()
#define CONFIG_R1_DIG_OUTPUT() CONFIG_RA3_AS_DIG_OUTPUT()
#define CONFIG_R2_DIG_OUTPUT() CONFIG_RB4_AS_DIG_OUTPUT()
#define CONFIG_R3_DIG_OUTPUT() CONFIG_RA4_AS_DIG_OUTPUT()

void CONFIG_ROW() {
  CONFIG_R0_DIG_OUTPUT();
  CONFIG_R1_DIG_OUTPUT();
  CONFIG_R2_DIG_OUTPUT();
  CONFIG_R3_DIG_OUTPUT();
}

static inline void DRIVE_ROW_LOW() {
  R0 = 0;
  R1 = 0;
  R2 = 0;
  R3 = 0;
}

static inline void DRIVE_ROW_HIGH() {
  R0 = 1;
  R1 = 1;
  R2 = 1;
  R3 = 1;
}

void configKeypad(void) {
  CONFIG_ROW();
  DRIVE_ROW_LOW();
  CONFIG_COLUMN();
  DELAY_US(1);     //wait for pullups to stabilize inputs
}

//drive one row low
void setOneRowLow(uint8_t u8_x) {
  switch (u8_x) {
    case 0:
      R0 = 0;
      R1 = 1;
      R2 = 1;
      R3 = 1;
      break;

    case 1:
      R0 = 1;
      R1 = 0;
      R2 = 1;
      R3 = 1;
      break;

    case 2:
      R0 = 1;
      R1 = 1;
      R2 = 0;
      R3 = 1;
      break;

    default:
      R0 = 1;
      R1 = 1;
      R2 = 1;
      R3 = 0;
      break;
  }
}

#define NUM_ROWS 4
#define NUM_COLS 3

const uint8_t au8_keyTable[NUM_ROWS][NUM_COLS] = { 
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

#define KEY_PRESSED() (!C0 || !C1 || !C2)   //any low

#define KEY_RELEASED() (C0 && C1 && C2)  //all high

uint8_t doKeyScan(void) {
  uint8_t u8_row, u8_col;
  //determine column
  if (!C0) u8_col = 0;
  else if (!C1) u8_col = 1;
  else if (!C2) u8_col = 2;
  else return('E'); //error
  //determine row
  for (u8_row = 0; u8_row < NUM_ROWS; u8_row++) {
    setOneRowLow(u8_row); //enable one row low
    if (KEY_PRESSED()) {
      DRIVE_ROW_LOW(); //return rows to driving low
      return(au8_keyTable[u8_row][u8_col]);
    }
  }
  DRIVE_ROW_LOW(); //return rows to driving low
  return('E'); //error
}


typedef enum  {
  STATE_WAIT_FOR_PRESS = 0,
  STATE_WAIT_FOR_PRESS2,
  STATE_WAIT_FOR_RELEASE,
} ISRSTATE;


ISRSTATE e_isrState = STATE_WAIT_FOR_PRESS;

//Interrupt Service Routine for Timer3
void _ISR _T3Interrupt (void) {

    switch (e_isrState) {
    case STATE_WAIT_FOR_PRESS:
      if (KEY_PRESSED() && (u8_newKey == 0)) {
        //ensure that key is sampled low for two consecutive interrupt periods
        e_isrState = STATE_WAIT_FOR_PRESS2;
      }
      break;
    case STATE_WAIT_FOR_PRESS2:
      if (KEY_PRESSED()) {
        // a key is ready
        u8_newKey = doKeyScan();
        e_isrState = STATE_WAIT_FOR_RELEASE;
      } else e_isrState = STATE_WAIT_FOR_PRESS;
      break;

    case STATE_WAIT_FOR_RELEASE:
      //keypad released
      if (KEY_RELEASED()) {
        e_isrState = STATE_WAIT_FOR_PRESS;
      }
      break;
    default:
      e_isrState = STATE_WAIT_FOR_PRESS;
      break;
  }
  _T3IF = 0;                 //clear the timer interrupt bit
   //update_state ();
}


#define ISR_PERIOD     15      // in ms

void  configTimer3(void) {
  //ensure that Timer2,3 configured as separate timers.
  T2CONbits.T32 = 0;     // 32-bit mode off
  //T3CON set like this for documentation purposes.
  //could be replaced by T3CON = 0x0020
  T3CON = T3_OFF | T3_IDLE_CON | T3_GATE_OFF
          | T3_SOURCE_INT
          | T3_PS_1_64 ;  //results in T3CON= 0x0020
  PR3 = msToU16Ticks (ISR_PERIOD, getTimerPrescale(T3CONbits)) - 1;
  TMR3  = 0;                       //clear timer3 value
  _T3IF = 0;                       //clear interrupt flag
  _T3IP = 1;                       //choose a priority
  _T3IE = 1;                       //enable the interrupt
  T3CONbits.TON = 1;               //turn on the timer
}


void inithome() {
  writeLCD(0x01,0,0,1);
 outStringLCD("|TEMPERATURE| (1)READ(3)RECALL");
 writeLCD(0xC0,0,0,1);  // cursor to 2nd line
 outStringLCD("|**SENSOR***| (2)SAVE(4)CLEAR");
 do{

    writeLCD(0x18,0,0,1);  // shift left
    DELAY_MS(300);
 }while(KEY_RELEASED());
writeLCD(0x01,0,0,1);
}

void inittemp() {
    u16_adcVal = convertADC1();   //get ADC value
    f_adcVal = u16_adcVal;
    f_adcVal = (f_adcVal/1024.0) * VREF;
    //convert to float in range 0.0 to VREF
    f_temp = (f_adcVal - 0.424)/0.00625;
    sprintf(string,"%.3fdegC", (double)f_temp);
    //printf("ADC input: %4.2f V (0x%04x)\n", (double) f_adcVal, u16_adcVal);
   // outStringLCD("ADC input: %4.2f V (0x%04x)\n");
   // DELAY_MS(300);
}

void recording(void) {
 if(KEY_PRESSED() && doKeyScan() == '0'){
                    num = 0;
                    //saver[0]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '1'){
                    num = 1;
                    //saver[1]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '2'){
                    num = 2;
                   // saver[2]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '3'){
                    num = 3;
                    //saver[3]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '4'){
                    num = 4;
                    //saver[4]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '5'){
                    num = 5;
                    //saver[5]=f_temp;
                }

    else if(KEY_PRESSED() && doKeyScan() == '6'){
                    num = 6;
                    //saver[6]=f_temp;

                }

    else if(KEY_PRESSED() && doKeyScan() == '7'){
                    num = 7;
                    //saver[7]=f_temp;

                }

    else if(KEY_PRESSED() && doKeyScan() == '8'){
                    num = 8;
                    //saver[8]=f_temp;

                }

    else if(KEY_PRESSED() && doKeyScan() == '9'){
                    num = 9;
                    //saver[9]=f_temp;
                }

}

void initsave(void) {
    savers: {
            writeLCD(0x01,0,0,1);
            outStringLCD("Save Reading?");
            do {
            writeLCD(0xC0,0,0,1);  // cursor to 2nd line
            outStringLCD("Choose (0)-(9)");
            DELAY_MS(200);
            }while (KEY_RELEASED());
    }

    recording();

        if(KEY_PRESSED() && doKeyScan() == '*'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Slot...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);
 
            goto savers;

                }
    else if(KEY_PRESSED() && doKeyScan() == '#'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Slot...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);

            goto savers;
    }
               

            
    saver[num] = f_temp;
            DELAY_MS(1000);
            writeLCD(0x01,0,0,1);
            outStringLCD("Reading Saved");
            writeLCD(0xC0,0,0,1);  // cursor to 2nd line
            outStringLCD("Sucessfully");
            DELAY_MS(1000);

} 

int main (void) {
  configBasic(HELLO_MSG);
 // PIO config
  configKeypad();
  // Configure the Timer
  configTimer3();

  configControlLCD();      //configure the LCD control lines

  initLCD();               //initialize the LCD
  
  CONFIG_LED1();
  
  CONFIG_RA1_AS_ANALOG();

 configADC1_ManualCH0(RA1_AN, 31, 0);

 initdevice();

 inithome();

 // while (1) {
  //  doHeartbeat();

 // }
  while (1) {

    if (u8_newKey) {

    outChar(u8_newKey);
    kp = u8_newKey;
  
    switch (kp)
    {
        case '1':
            writeLCD(0x01,0,0,1);
            inittemp();
            outStringLCD("Temperature:");
            writeLCD(0xC0,0,0,1);
            outStringLCD(string);
            DELAY_MS(3000);
            blink1();
            inithome();
            break;

        case '2':
            initsave();
            blink2();
            inithome();
            break;

        case '3':
            recallers: {
            writeLCD(0x01,0,0,1);
            outStringLCD("Recall Reading?");
            do {
            writeLCD(0xC0,0,0,1);  // cursor to 2nd line
            outStringLCD("Choose (0)-(9)");
            DELAY_MS(200);
            }while (KEY_RELEASED());
            }
            recording();
            if(KEY_PRESSED() && doKeyScan() == '*'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);
            goto recallers;

                }
            else if(KEY_PRESSED() && doKeyScan() == '#'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);

            goto recallers;
    }
            writeLCD(0x01,0,0,1);
            outStringLCD("Temperature:");
            sprintf(string,"%.3fdegC", (double)saver[num]);
            writeLCD(0xC0,0,1,1);
            outStringLCD(string);
            DELAY_MS(2000);
            inithome();
            break;

        case '4':
            f_temp = 0;
            outStringLCD("Reading cleared");
            writeLCD(0xC0,0,0,1);  // cursor to 2nd line
            outStringLCD("without saving");
            DELAY_MS(2000);
            blink1();
            inithome();
             break;

        case '5':
            //writeLCD(0x01,0,0,1);
            //outStringLCD("Invalid Entry...");
            //writeLCD(0xC0,0,0,1);
            //outStringLCD("Back to Menu....");
            //DELAY_MS(2000);
            //inithome();
            clearers: {
            writeLCD(0x01,0,0,1);
            outStringLCD("Clear Reading?");
            do {
            writeLCD(0xC0,0,0,1);  // cursor to 2nd line
            outStringLCD("Choose (0)-(9)");
            DELAY_MS(200);
            }while (KEY_RELEASED());
            }
            recording();

            if(KEY_PRESSED() && doKeyScan() == '*'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);
            goto clearers;

                }
            else if(KEY_PRESSED() && doKeyScan() == '#'){

            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Choose Another");
            DELAY_MS(2000);

            goto clearers;
    }
            saver[num]=0;
            writeLCD(0x01,0,0,1);
            outStringLCD("Reading Cleared");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Successfully");
            DELAY_MS(2000);
            inithome();
            
             break;

         case '6':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;
             
        case '7':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;

         case '8':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;
             
        case '9':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;

         case '0':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;
         case '*':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;

         case '#':
            writeLCD(0x01,0,0,1);
            outStringLCD("Invalid Entry...");
            writeLCD(0xC0,0,0,1);
            outStringLCD("Back to Menu....");
            DELAY_MS(2000);
            inithome();
             break;

    }
    u8_newKey = 0;
    }

    doHeartbeat();     //ensure that we are alive
  } // end while (1)
}
