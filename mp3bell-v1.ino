/*************************************/
/* MP3 bell for Lilypad MP3 hardware */
/* By: Adrien                        */
/* Date: June 2017                   */
/*************************************/
/**
 * Sparkfun's page:
 * https://www.sparkfun.com/products/11013
 *
 *
 * Hardware spec for Lilypad MP3:
 * - ATmega 328p microprocessor with Arduino bootloader (Pro 3.3V/8MHz)
 * - VS1053B MP3 (and many other formats) decoder chip
 * - TPA2016D2 stereo amplifier
 * - MCP73831 3.7V Lipo charger (preset to 500mA, rate can be changed if desired)
 * - Headphone jack
 * - Five trigger inputs, also usable as analog, serial and I2C (“Wire”) connections
 * - Load new firmware (or write your own) using the free Arduino IDE
 * - 5V FTDI basic breakout for battery recharging and reprogramming
 * - Header for optional RGB rotary encoder (not included, requires soldering)
*/
/**
 * Required Libraries:
 * - SdFat library by William Greiman
 * - SFEMP3Shield library by Porter and Flaga
 * - PinChangeInt library by Lex Talionis
 */
/*************************************/

/* Include libraries */
#include <SPI.h>          // SPI library for SD and MP3 Chip
#include <SdFat.h>        // SD Fat Libraries
#include <SFEMP3Shield.h> // VS1053B MP3 chip
#include <SdFatUtil.h>    // FAT utilities, including FreeRam()
#include <PinChangeInt.h> // PCintPort

/* List of defines */
#define DEBUG_ENABLED 0

#define NUM_OF_INPUT_TRIGS 5
#define NUM_OF_INPUT_TRIGS_UART_EN 3
#define NUM_OF_SOUNDS_MAX 15
#define NUM_OF_NOTIF_MAX 15
#define NUM_OF_CHAR_FILENAME 13 // FAT format [8.3] plus null char at the end

/*************************************/

/* Globals and constants */

/* Triggers, thesee are all Arduino pin definitions */
const int IO_TRIG1 = A0;
const int IO_TRIG2_SDA = A4;  // This pin is shared with I2C SDA, connected to DIO12 (pin4) on XBEE
const int IO_TRIG3_SCL = A5;  // This pin is shared with I2C SCL, connected to SLEEP_RQ (pin9) on XBEE
const int IO_TRIG4_RXI = 1;   // This pin is shared with UART TX, connected to DOUT (pin2) on XBEE
const int IO_TRIG5_TXO = 0;   // This pin is shared with UART RX, connected to DIN (via 100 ohm resistor, pin3) on XBEE
int gTRIGS[NUM_OF_INPUT_TRIGS] = {
  IO_TRIG1,       // Actual bell trigger
  IO_TRIG2_SDA,   // XBEE DIO12, pin 4
  IO_TRIG3_SCL,   // XBEE SLEEP_RQ, pin 9
  IO_TRIG4_RXI,   // XBEE DOUT, pin 2
  IO_TRIG5_TXO    // XBEE DIN, pin 3
  };

/* Rotary encoder with RGB. This is optional hardware, but installed on BELL hardware */
const int IO_ROT_LED_R = 10;
const int IO_ROT_LED_G = A1;
const int IO_ROT_LED_B = 5;
const int IO_ROT_A = 3;
const int IO_ROT_B = A3;
const int IO_ROT_SW = 4;

/* MP3 chip */
const int IO_EN_GPIO1 = A2;
const int IO_RIGHT = A6;
const int IO_LEFT = A7;
const int IO_MP3_DREQ = 2;
const int IO_MP3_CS = 6;
const int IO_MP3_DCS = 7;
const int IO_MP3_RST = 8;

/* SD Card */
const int IO_SD_CS = 9;

/* SPI */
const int IO_SPI_MOSI = 11;
const int IO_SPI_MISO = 12;
const int IO_SPI_SCK = 13;

/* Default volume */
const uint16_t DEFAULT_VOLUME = 40; // 0: Loudest, 255: lowest
const uint16_t DEFAULT_MIN_VOLUME = 255; // 0: Loudest, 255: lowest
const uint16_t DEFAULT_MAX_VOLUME = 0; // 0: Loudest, 255: lowest
const uint16_t DEFAULT_STEP_VOLUME = 1; // 0: Loudest, 255: lowest
const uint32_t BUTTON_DEBOUNCE_MS = 10; // 10 millisecond debounce period

const uint32_t EXEC_TIMEOUT_MS = 1000; // 1000 millisecond execution timeout period

/* Enums */
typedef enum colors
{
  RED,
  GREEN,
  BLUE
};

typedef enum syserrors
{
  ERR_SD_INIT,
  ERR_MP3_INIT
};

typedef enum btnstate
{
  BTN_UP,
  BTN_DOWN
};

typedef enum volchange
{
  VOL_UP,
  VOL_DOWN
};

/*************************************/

/* Objects */
SdFat sd;  // Must be sd as it is referred by SFEMP3Shield.cpp
SFEMP3Shield gMp3Chip;

/* Globals */
char gBellSoundList[NUM_OF_SOUNDS_MAX][NUM_OF_CHAR_FILENAME] = {};  // Bell's sound list, up to 15 songs
int gBellSoundCount = 0;
char gBellNotifList[NUM_OF_SOUNDS_MAX][NUM_OF_CHAR_FILENAME] = {};  // Notification sound list, up to 15 songs
int gBellNotifCount = 0;

bool gRotEncEnable = false;
bool gXbeeEnable = false;

volatile bool gButtonPressed = 0;
volatile bool gButtonReleased = 0;
volatile uint32_t gButtonDowntime = 0;

volatile uint32_t gRotaryCounter = 0;
volatile bool gRotaryChanged = false;
volatile bool gRotaryDirection = false; // CW: true, CCW: false

volatile uint16_t gSoundVolume = 0;

/*************************************/

/* Arduino Setup */
void setup()
{
  /* Clean up filenames */
  clean_filename();

  /* Configure trigger input pins with weak-pullups */
  pinMode(gTRIGS[0], INPUT_PULLUP); // Bell trigger
  pinMode(gTRIGS[1], INPUT_PULLUP); // XBEE DIO12 (set as input on both side)
  pinMode(gTRIGS[2], OUTPUT); // XBEE SLEEP_RQ (set as input on XBEE side, active high)

  /* Set XBEE to power on */
  digitalWrite(gTRIGS[2], LOW);

  pinMode(IO_TRIG4_RXI, INPUT);
  pinMode(IO_TRIG5_TXO, OUTPUT);

  /* Initialize serial port */
  Serial.begin(9600);

  /* Print header */
  String hdr_msg = "FreeRam: " + String(FreeRam(), DEC);

  /* Print a header */    
  xbee_log_dbg(hdr_msg);

  /* RGB Rotary Encoder */
  pinMode(IO_ROT_A, INPUT_PULLUP);
  pinMode(IO_ROT_B, INPUT_PULLUP);
  pinMode(IO_ROT_SW, INPUT_PULLUP);
  
  pinMode(IO_ROT_LED_R, OUTPUT);
  pinMode(IO_ROT_LED_G, OUTPUT);
  pinMode(IO_ROT_LED_B, OUTPUT);
  update_rgb_led(0,0,0);  // Turn off LEDs

  /* MP3 Controls */
  pinMode(IO_EN_GPIO1, OUTPUT);
  pinMode(IO_RIGHT, INPUT);
  pinMode(IO_LEFT, INPUT);
  pinMode(IO_MP3_DREQ, INPUT);
  pinMode(IO_MP3_CS, OUTPUT);
  pinMode(IO_MP3_DCS, OUTPUT);
  pinMode(IO_MP3_RST, OUTPUT);

  /* SD Card */
  pinMode(IO_SD_CS, OUTPUT);

  /* Other peripherals */
  pinMode(IO_SPI_MOSI, OUTPUT);
  pinMode(IO_SPI_MISO, INPUT);
  pinMode(IO_SPI_SCK, OUTPUT);

  /* Shutdown amplifier, initialize MP3 system */
  digitalWrite(IO_EN_GPIO1, LOW);

  /* Initialize SD card */
  init_sdcard();

  /* Initialize MP3 chip */
  init_mp3chip();

  /* Initialize ISRs */
  init_isr();

  /* Initialize file playback */
  sd.chdir("/", true);  // Change to root dir

  /* Load file */


  /* Set volume */
  gSoundVolume = DEFAULT_VOLUME;  // TODO: Reload?
  gMp3Chip.setVolume(gSoundVolume); // Left and Right channel same level  

  /* Enable amplifier after MP3 system initialized */
  digitalWrite(IO_EN_GPIO1, HIGH);

  delay(2); // Magic delay, ensure Amplifier is fully enabled.
}

/*************************************/

/* Arduino Loop */
void loop()
{
  static byte r = 0;
  static byte g = 0;
  static byte b = 0;  
  
  xbee_log_info(F("Starting loop"));
  delay(50);
  update_rgb_led(r,g,b);  // Turn off LEDs

  r+=1;
  g+=2;
  b+=3;
}


/*************************************/
/* SD Card routines */
/*************************************/
void init_sdcard(void)
{
  byte result = 0;
  xbee_log_info(F("Initialize SD Card ..."));

  /* Begin */
  result = sd.begin(IO_SD_CS, SPI_HALF_SPEED);

  /* Check for SD Card error */
  if (1 != result)
  {
    xbee_log_error(F("Cannot init SD!"));
    error_blink(ERR_SD_INIT, RED);
  }
  else
  {
    xbee_log_info(F("SD init OK"));
  }
    
}

/*************************************/
/* MP3 routines */
/*************************************/
void init_mp3chip(void)
{
  byte result = 0;
  xbee_log_info(F("Initialize MP3 chip ..."));

  /* Begin */
  result = gMp3Chip.begin();

  /* Check, 0 and 6 are OK */
  if ( (0 != result) && (6 != result))
  {
    xbee_log_error(F("Cannot init MP3 chip!"));
    error_blink(ERR_MP3_INIT, RED);
  }
  else
  {
    xbee_log_info(F("MP3 init OK"));
  }
  
}

/*************************************/
/* Interrupt routines */
/*************************************/
void init_isr(void)
{
  /* Rotary Interrupt, only pin 3 is available for interrupt on 328p on lilypad mp3 */
  attachInterrupt(digitalPinToInterrupt(IO_ROT_A), rotary_isr, CHANGE);
  
  /* Button Interrupt */
  PCintPort::attachInterrupt(IO_ROT_SW, rotary_btn_isr, CHANGE);
}


void rotary_isr(void)
{
  static uint8_t rotary_state = 0x0;  // 2 LSB store current state from IO_ROT_A and IO_ROT_B

  /* Remember prev state */
  rotary_state = rotary_state << 2;
  /* Read current state */
  rotary_state |= (digitalRead(IO_ROT_A) | (digitalRead(IO_ROT_B) << 1));
  /* Only remember last state */
  rotary_state &= 0x0F;

  /* Check rotary state */
  if ((0x09 == rotary_state) || (0x06 == rotary_state))
  {
    /* Last and current state are: 1001 or 0110, this is CW motion */
    gRotaryCounter++;
    gRotaryDirection = true;
    gRotaryChanged = true;
  }
  else if ((0x03 == rotary_state) || (0x0C == rotary_state))
  {
    /* Last and current state are: 0011 or 1100, this is CCW motion */
    gRotaryCounter--;
    gRotaryDirection = false;
    gRotaryChanged = true;
  }
  
}

void rotary_btn_isr(void)
{
  static btnstate button_state = BTN_DOWN;
  static uint32_t button_start_time_ms = 0;
  static uint32_t button_end_time_ms = 0;

  /* Get button state */
  uint8_t curr_pin_state = PCintPort::pinState;

  /* Read button transition */
  if ((HIGH == curr_pin_state) && (BTN_DOWN == button_state))
  {
    /* Button is hold down (current pin is still high) */
    button_start_time_ms = millis();

    /* Debouncer */
    if(button_start_time_ms > (button_end_time_ms + BUTTON_DEBOUNCE_MS))
    {
      button_state = BTN_UP;
      gButtonPressed = true;
    }
  }
  else if ((LOW == curr_pin_state) && (BTN_UP == button_state))
  {
    /* Button is released (current pin is still low) */
    button_end_time_ms = millis();

    /* Debouncer */
    if( button_end_time_ms > (button_start_time_ms + BUTTON_DEBOUNCE_MS))
    {
      button_state = BTN_DOWN;
      gButtonReleased = true;

      /* Measure how long the button is pressed, to know the button mode */
      gButtonDowntime = button_end_time_ms - button_start_time_ms;
    }
  }
  
}

/*************************************/
/* File Utilities */
/*************************************/
void get_next_track(void)
{
  /* Get next playable file (must be music file) */
  uint32_t start_time = millis();
  uint32_t end_time = start_time;
  while(end_time < (start_time + EXEC_TIMEOUT_MS))
  {
    if (true == get_next_file())
    {
      /* File is found, break */
      break;
    }

    /* Update end time */
    end_time = millis();    
  }
}

void get_prev_track(void)
{
  /* Get previous playable file (must be music file) */
  uint32_t start_time = millis();
  uint32_t end_time = start_time;
  while(end_time < (start_time + EXEC_TIMEOUT_MS))
  {
    if (true == get_prev_file())
    {
      /* File is found, break */
      break;
    }

    /* Update end time */
    end_time = millis();    
  }
}

bool get_next_file(void)
{
  bool is_found = false;

  /* Look for next file */
  

  /* Is it playable? */

  return is_found;
}


bool get_prev_file(void)
{
  bool is_found = false;

  /* Look for prev file */


  /* Is it playable? */

  return is_found;
}

/*************************************/
/* Utilities */
/*************************************/
void clean_filename (void)
{  
  /* Clear sound list */
  for (int i = 0; i< NUM_OF_SOUNDS_MAX; i++)
  {
    memset(gBellSoundList[i], 0, NUM_OF_CHAR_FILENAME);
  }

  /* Clear notif list */
  for (int i = 0; i< NUM_OF_NOTIF_MAX; i++)
  {
    memset(gBellNotifList[i], 0, NUM_OF_CHAR_FILENAME);
  }
  /* Clear sound count */
  gBellSoundCount = 0;

  /* Clear notification count */
  gBellNotifCount = 0;
}

void update_rgb_led(byte r, byte g, byte b)
{
  /* Update color */
  analogWrite(IO_ROT_LED_R, (255-r)); // pin 10's PWM is 490 Hz
  digitalWrite(IO_ROT_LED_G, (0==g)?LOW:HIGH);  // Unfortunately this pin is not PWM, so... only on or off.
  analogWrite(IO_ROT_LED_B, (255-b)); // pin 5's PWM is 980 Hz
}

void change_volume(volchange volopt)
{
  switch(volopt)
  {
    case VOL_UP:
      /* Volume up and check max limits */
      if (gSoundVolume >= DEFAULT_MAX_VOLUME)
      {
        /* Loudest is 0 */
        gSoundVolume -= DEFAULT_STEP_VOLUME;
        if (gSoundVolume < DEFAULT_MAX_VOLUME) gSoundVolume = DEFAULT_MAX_VOLUME; // Volume cap
      }
      break;
      
    case VOL_DOWN:
      /* Volume down and check min limits */
      if (gSoundVolume <= DEFAULT_MIN_VOLUME)
      {
        /* Lowest is 255 */
        gSoundVolume += DEFAULT_STEP_VOLUME;
        if (gSoundVolume > DEFAULT_MIN_VOLUME) gSoundVolume = DEFAULT_MIN_VOLUME; // Volume cap
      }
      break;
      
    default:
      /* Not supported volume option */
      break;
  }
}

void error_blink(syserrors errorCode, colors color)
{
  int error_blinks = (int)errorCode;
  byte r = 0;
  byte g = 0;
  byte b = 0;

  switch(color)
  {
    case GREEN:
      g = 255;
      break;

    case BLUE:
      b = 255;
      break;
      
    case RED:
    default:
      r = 255;
      break;
  }

  /* Forever loop heree in case of fatal error */
  while(true)
  {
    for (int i = 0; i< error_blinks; i++)
    {
      update_rgb_led(0,0,0);
      delay(100);
      update_rgb_led(r,g,b);
      delay(200);
    }
    delay(1000);
  }
}


/*************************************/
/* XBEE / Debug Utilities */
/*************************************/
void xbee_log_error(String msg)
{
  if (0 == DEBUG_ENABLED) return;
  Serial.println("[ERR]" + msg);
}

void xbee_log_warn(String msg)
{
  if (0 == DEBUG_ENABLED) return;
  Serial.println("[WRN]" + msg);
}

void xbee_log_info(String msg)
{
  if (0 == DEBUG_ENABLED) return;
  Serial.println("[INF]" + msg);
}

void xbee_log_dbg(String msg)
{
  if (0 == DEBUG_ENABLED) return;
  Serial.println("[DBG]" + msg);
}

void xbee_write_string(String msg)
{
  
}

