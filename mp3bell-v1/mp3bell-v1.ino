/*************************************/
/* MP3 bell for Lilypad MP3 hardware */
/* By: Adrien                        */
/* Date: June 2017                   */
/*************************************/
/**
   Sparkfun's page:
   https://www.sparkfun.com/products/11013


   Hardware spec for Lilypad MP3:
   - ATmega 328p microprocessor with Arduino bootloader (Pro 3.3V/8MHz)
   - VS1053B MP3 (and many other formats) decoder chip
   - TPA2016D2 stereo amplifier
   - MCP73831 3.7V Lipo charger (preset to 500mA, rate can be changed if desired)
   - Headphone jack
   - Five trigger inputs, also usable as analog, serial and I2C (“Wire”) connections
   - Load new firmware (or write your own) using the free Arduino IDE
   - 5V FTDI basic breakout for battery recharging and reprogramming
   - Header for optional RGB rotary encoder (not included, requires soldering)
*/
/**
   Required Libraries:
   - SdFat library by William Greiman
   - SFEMP3Shield library by Porter and Flaga
   - PinChangeInt library by Lex Talionis
*/
/*************************************/

/* Include libraries */
#include <SPI.h>          // SPI library for SD and MP3 Chip
#include <SdFat.h>        // SD Fat Libraries
#include <SFEMP3Shield.h> // VS1053B MP3 chip
#include <SdFatUtil.h>    // FAT utilities, including FreeRam()
#include <PinChangeInt.h> // PCintPort
#include <XBee.h>         // XBee

#include <stdio.h>        // Std IO

/* Function Macros */
#define DEBUG_ENABLED 0 // Set to 255 for debug serial mode

#if (DEBUG_ENABLED==255)

#define LOG_FMT(f,...) do { \
    char msg[100] = {}; \
    Serial.print("[DBG] "); \
    sprintf(msg, f, __VA_ARGS__); \
    Serial.println(msg); \
  } while(0)

#define LOG_ERR(msg) do { Serial.print("[ERR] "); Serial.println(msg); } while(0)
#define LOG_DBG(msg) do { Serial.print("[DBG] "); Serial.println(msg); } while(0)
#define LOG_SHOW_FREERAM() do { LOG_FMT("Free RAM: %d", FreeRam()); } while (0)

#else

/* No debug or other values */
#define LOG_FMT(f,...) do {} while(0)
#define LOG_ERR(msg) do {} while(0)
#define LOG_DBG(msg) do {} while(0)
#define LOG_SHOW_FREERAM() do {} while(0)

#endif

/* List of defines */
#define NUM_OF_CHAR_FILENAME 13 // FAT format [8.3] plus null char at the end
#define XBEE_ENABLED 1  // 0: Disable XBEE support, 1: Enable XBEE

/*************************************/

/* Globals and constants */

/* Triggers, thesee are all Arduino pin definitions */
#define IO_TRIG1 A0
#define IO_TRIG2_SDA A4  // This pin is shared with I2C SDA, connected to DIO12 (pin4) on XBEE
#define IO_TRIG3_SCL A5  // This pin is shared with I2C SCL, connected to SLEEP_RQ (pin9) on XBEE
#define IO_TRIG4_TXO 1   // This pin is shared with UART TX, connected to DIN (via 100 ohm resistor, pin3) on XBEE
#define IO_TRIG5_RXI 0   // This pin is shared with UART RX, connected to DOUT (pin2) on XBEE

/** Note */
/*
   IO_TRIG1: Actual bell trigger
   IO_TRIG2_SDA: XBEE DIO12, pin 4
   IO_TRIG3_SCL: XBEE SLEEP_RQ, pin 9
   IO_TRIG4_RXI: XBEE DOUT, pin 2
   IO_TRIG5_TXO: XBEE DIN, pin 3
*/

/* Rotary encoder with RGB. This is optional hardware, but installed on BELL hardware */
#define IO_ROT_LED_R 10
#define IO_ROT_LED_G A1
#define IO_ROT_LED_B 5

#define IO_ROT_A 3
#define IO_ROT_B A3
#define IO_ROT_SW 4

/* MP3 chip */
#define IO_EN_GPIO1 A2
#define IO_RIGHT A6
#define IO_LEFT A7
#define IO_MP3_DREQ 2
#define IO_MP3_CS 6
#define IO_MP3_DCS 7
#define IO_MP3_RST 8

/* SD Card */
#define IO_SD_CS 9

/* SPI */
#define IO_SPI_MOSI 11
#define IO_SPI_MISO 12
#define IO_SPI_SCK 13

/* Default volume */
const uint8_t DEFAULT_VOLUME = 0; // 0: Loudest, 255: lowest
const uint8_t DEFAULT_MIN_VOLUME = 255; // 0: Loudest, 255: lowest
const uint8_t DEFAULT_MAX_VOLUME = 0; // 0: Loudest, 255: lowest
const uint16_t DEFAULT_STEP_VOLUME = 1; // 0: Loudest, 255: lowest
const uint32_t BUTTON_DEBOUNCE_MS = 10; // 10 millisecond debounce period

const uint32_t EXEC_TIMEOUT_MS = 10000; // 10000 millisecond execution timeout period

const uint16_t MAX_NUM_FILES = 1000; // 0 - 999 files supported at this moment.

const byte XBEE_MY_PROTOCOL_ID = 0; // Bell is ID 0

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
  ERR_MP3_INIT,
  ERR_XBEE_INV_RESPONSE,
  ERR_XBEE_NO_RESPONSE,
  ERR_XBEE_NO_LISTENER,
  ERR_XBEE_RX_NOT_ACKD,
  ERR_XBEE_DISASSOCIATED,
  ERR_XBEE_UNKNMSR,
  ERR_XBEE_UNKNRESP,
  ERR_XBEE_RX_NORESP
};

typedef enum sysokstat
{
  GOOD_XBEE_TX_SUCCESS,
  GOOD_XBEE_RX_SUCCESS,
  GOOD_XBEE_ASSOCIATED
};

typedef enum btnstate
{
  BTN_UP,
  BTN_DOWN
};

typedef enum volchange
{
  VOL_UP,
  VOL_DOWN,
  VOL_DEF,
  VOL_MAN
};

/*************************************/

/* Objects */
SdFat sd;  // Must be 'sd', referred by SFEMP3Shield.cpp
SdFile file;  // Must be 'file', referred by SFEMP3Shield.cpp

SFEMP3Shield gMp3Chip;
XBee gXbee = XBee(); // XBee (XBEE TH REG S2C)

/* Globals */
bool gRotEncEnable = false;
bool gXbeeEnable = false;

volatile bool gButtonPressed = 0;
volatile bool gButtonReleased = 0;
volatile uint32_t gButtonDowntime = 0;

volatile uint32_t gRotaryCounter = 0;
volatile bool gRotaryChanged = false;
volatile bool gRotaryDirection = false; // CW: true, CCW: false

volatile uint8_t gSoundVolume = 0;

char gCurrTrackName[13] = {};
uint16_t gMaxNumberOfFiles = 1;
byte gCurrToggleLed = 0;
byte gToggleLedArray[3] = {0x6, 0x5, 0x3};
bool gPlaybackActiveIndicator = false;

/*************************************/

/* Arduino Setup */
void setup()
{
  /* Configure trigger input pins with weak-pullups */
  pinMode(IO_TRIG1, INPUT_PULLUP); // Bell trigger
  pinMode(IO_TRIG2_SDA, INPUT_PULLUP); // XBEE DIO12 (set as input on both side)
  pinMode(IO_TRIG3_SCL, OUTPUT); // XBEE SLEEP_RQ (set as input on XBEE side, active high)

  /* Set XBEE to power on */
  digitalWrite(IO_TRIG3_SCL, LOW);

  pinMode(IO_TRIG4_TXO, OUTPUT);
  pinMode(IO_TRIG5_RXI, INPUT);

  /* Initialize serial port */
  Serial.begin(9600);

#if(XBEE_ENABLED==1)
  /* Initialize XBee */
  gXbee.setSerial(Serial);
#endif

  /* Print header */
  LOG_FMT("Free RAM: %d", FreeRam());

  /* RGB Rotary Encoder */
  pinMode(IO_ROT_A, INPUT_PULLUP);
  pinMode(IO_ROT_B, INPUT_PULLUP);
  pinMode(IO_ROT_SW, INPUT_PULLUP);

  pinMode(IO_ROT_LED_R, OUTPUT);
  pinMode(IO_ROT_LED_G, OUTPUT);
  pinMode(IO_ROT_LED_B, OUTPUT);

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

  /* List files */
  //sd.ls(LS_R | LS_DATE | LS_SIZE);

  /* Initialize file playback */
  sd.chdir("/", true); // Change to root dir

  /* Scan number of playable files */
  gMaxNumberOfFiles = scan_max_num_tracks();
  LOG_FMT(">> Max # of files: %d", gMaxNumberOfFiles);

  /* Set volume */
  gSoundVolume = DEFAULT_VOLUME; // TODO: Reload?
  gMp3Chip.setVolume(gSoundVolume, gSoundVolume); // Left and Right channel same level

  /* Clear LED */
  update_bit_rgb_led(0x7);

  /* Enable amplifier after MP3 system initialized */
  digitalWrite(IO_EN_GPIO1, HIGH);

  delay(2); // Magic delay, ensure Amplifier is fully enabled.
}

/*************************************/

/* Arduino Loop */
void loop()
{
  static bool s_button_down = false;
  static unsigned long int s_button_down_start = 0;
  static unsigned long int s_button_down_stop = 0;

  static bool s_bell_active = false;
  bool bell_confirmation = false; // true: confirm only, false: ring real bell on remote

  /* Check if rotary state changed */
  if (true == gRotaryChanged)
  {
    /* Change volume */
    if (true == gRotaryDirection)
    {
      /* Clockwise, increase volume */
      change_volume(VOL_UP);
    }
    else
    {
      /* Counter Clockwise, decrease volume */
      change_volume(VOL_DOWN);
    }

    /* Clear flag */
    gRotaryChanged = false;
  }

  /* Check for rotary button press */
  if (true == gButtonPressed)
  {
    LOG_DBG(F("Button Pressed!"));

    /* TODO: Do something ? */

    /* Clear flag */
    gButtonPressed = false;
  }

  /* Check for rotary button release */
  if (true == gButtonReleased)
  {
    LOG_DBG(F("Button Released!"));

    /* Set button active high (when released) */
    s_bell_active = true;

    /* Clear flag */
    gButtonReleased = false;
  }
  
#if(XBEE_ENABLED==1)
  byte rx_to_node = 254;  // 254 reserved for blank XBEE Command, 255: broadcast
  byte rx_cmd = 254;      // 254 reserved for blank XBEE Command
  byte rx_subcmd0 = 254;   // 254 reserved for blank XBEE Command
  byte rx_subcmd1 = 254;   // 254 reserved for blank XBEE Command
  
  /* Check XBEE Request */
  bool new_rx_ready = xbee_cmd_rx(rx_to_node, rx_cmd, rx_subcmd0, rx_subcmd1);
  //LOG_FMT(">> XBEE RX C: %d, S: %d", rx_cmd, rx_subcmd);

  /* Handle XBEE request */
  if ((true == new_rx_ready) 
      && ((XBEE_MY_PROTOCOL_ID == rx_to_node) || (255 == rx_to_node)))
  {
    switch(rx_cmd)
    {
      case 0:
        /* Ring command */
        s_bell_active = true;
        bell_confirmation = true;
        break;

      case 1:
        /* Ring Remote */
        break;
        
      case 2:
        /* Vol change */
        if (VOL_UP == rx_subcmd0)
        {
          /* Vol Up */
          change_volume(VOL_UP);
        }
        else if (VOL_DOWN == rx_subcmd0)
        {
          /* Vol Down */
          change_volume(VOL_DOWN);
        }
        else if(VOL_DEF == rx_subcmd0)
        {
          /* Vol Default */
          change_volume(VOL_DEF);
        }
        else if(VOL_MAN == rx_subcmd0)
        {
          /* Vol Manual */
          set_volume((uint8_t)rx_subcmd1);
        }
        else
        {
          /* Vol Default */
          change_volume(VOL_DEF);
        }
        break;
        
      default:
        /* Ignore this command */
        break;
    }
  }
  
#endif

  /* Stop here and do not proceed if bell is still playing */
  if (false == gMp3Chip.isPlaying())
  {
    if (true == gPlaybackActiveIndicator)
    {
      /* Turn off LED playback indicator */
      update_bit_rgb_led(0x7);
      
      /* Reset playback indicator */
      gPlaybackActiveIndicator = false;
    }
    
    /* Check if need to play next bell songs */
    if (true == s_bell_active)
    {
      /* Reset toggle LED */
      gCurrToggleLed = 0;

      /* Play Next Song */
      play_next_song();

#if(XBEE_ENABLED==1)
      /* Send broadcast command through XBEE if enabled */
      if (true == bell_confirmation)
      {
        xbee_cmd_bc_tx(255, 1, 0);
      }
      else
      {
        xbee_cmd_bc_tx(255, 0, 0);
      }
#endif

      /* Clear bell active */
      s_bell_active = false;

      /* Enable Playback Indicator */
      gPlaybackActiveIndicator = true;
    }
  }
  else
  {
    /* Toggle LED if playback is on */
    if ( true == gPlaybackActiveIndicator )
    {
      byte curr_toggle = gCurrToggleLed % 3;
      update_bit_rgb_led(gToggleLedArray[curr_toggle]);
      gCurrToggleLed++;
    }

    /* Automatically clear bell request */
    s_bell_active = false;
  }
}


/*************************************/
/* SD Card routines */
/*************************************/
void init_sdcard(void)
{
  byte result = 0;
  LOG_DBG(F("Initialize SD Card ..."));

  /* Begin */
  result = sd.begin(IO_SD_CS, SPI_HALF_SPEED);

  /* Check for SD Card error */
  if (1 != result)
  {
    LOG_DBG(F("Cannot init SD!"));
    error_blink(ERR_SD_INIT, RED, false);
  }
  else
  {
    LOG_DBG(F("SD init OK"));
  }

}

/*************************************/
/* MP3 routines */
/*************************************/
void init_mp3chip(void)
{
  byte result = 0;
  LOG_DBG(F("Initialize MP3 chip ..."));

  /* Begin */
  result = gMp3Chip.begin();

  /* Check, 0 and 6 are OK */
  if ( (0 != result) && (6 != result))
  {
    LOG_DBG(F("Cannot init MP3 chip!"));
    error_blink(ERR_MP3_INIT, RED, false);
  }
  else
  {
    LOG_DBG(F("MP3 init OK"));
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

  /* Bell Interrupt */
  PCintPort::attachInterrupt(IO_TRIG1, rotary_btn_isr, CHANGE);
}


void rotary_isr(void)
{
  static uint8_t rotary_state = 0x0; // 2 LSB store current state from IO_ROT_A and IO_ROT_B

  /* Remember prev state */
  rotary_state = rotary_state << 2;
  /* Read current state */
  rotary_state |= (digitalRead(IO_ROT_A) | (digitalRead(IO_ROT_B) << 1));
  /* Only remember last state */
  rotary_state &= 0x0F;

  /* Check rotary state */
  //if ((0x09 == rotary_state) || (0x06 == rotary_state))
  if (0x09 == rotary_state)
  {
    /* Last and current state are: 1001 or 0110, this is CW motion */
    gRotaryCounter++;
    gRotaryDirection = true;
    gRotaryChanged = true;
  }
  //else if ((0x03 == rotary_state) || (0x0C == rotary_state))
  else if (0x03 == rotary_state)
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
    if (button_start_time_ms > (button_end_time_ms + BUTTON_DEBOUNCE_MS))
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
    if ( button_end_time_ms > (button_start_time_ms + BUTTON_DEBOUNCE_MS))
    {
      button_state = BTN_DOWN;
      gButtonReleased = true;

      /* Measure how long the button is pressed, to know the button mode */
      gButtonDowntime = button_end_time_ms - button_start_time_ms;
    }
  }

}

/*************************************/
/* Utilities */
/*************************************/

void update_rgb_led(byte r, byte g, byte b)
{
  byte temp_r = 255 - r;
  byte temp_g = 255 - g;
  byte temp_b = 255 - b;

  /* Update color */
  analogWrite(IO_ROT_LED_R, (temp_r)); // pin 10's PWM is 490 Hz, 0 : 0% duty cycle, 255: 100% duty cycle
  digitalWrite(IO_ROT_LED_G, (0 == temp_g) ? LOW : HIGH); // Unfortunately this pin is not PWM, so... only on or off.
  analogWrite(IO_ROT_LED_B, (temp_b)); // pin 5's PWM is 980 Hz, 0 : 0% duty cycle, 255: 100% duty cycle
}

void update_bit_rgb_led(byte bitWiseRGB)
{
  byte temp_bitwise = bitWiseRGB & 0x7;

  /* Update color */
  digitalWrite(IO_ROT_LED_R, temp_bitwise & 0x1); // use as digital pin, only on or off.
  digitalWrite(IO_ROT_LED_G, temp_bitwise & 0x2); // use as digital pin, only on or off.
  digitalWrite(IO_ROT_LED_B, temp_bitwise & 0x4); // use as digital pin, only on or off.
}

void set_volume(uint8_t volumeLevel)
{
  /* Manually set volume level */
  gSoundVolume = volumeLevel;
  
  /* Update volume */
  gMp3Chip.setVolume(gSoundVolume, gSoundVolume);
}

void change_volume(volchange volopt)
{
  switch (volopt)
  {
    case VOL_UP:
      /* Volume up and check max limits */
      if (gSoundVolume > DEFAULT_MAX_VOLUME)
      {
        /* Loudest is 0 */
        gSoundVolume -= DEFAULT_STEP_VOLUME;
        if (gSoundVolume <= DEFAULT_MAX_VOLUME) gSoundVolume = DEFAULT_MAX_VOLUME; // Volume cap
      }
      break;

    case VOL_DOWN:
      /* Volume down and check min limits */
      if (gSoundVolume < DEFAULT_MIN_VOLUME)
      {
        /* Lowest is 255 */
        gSoundVolume += DEFAULT_STEP_VOLUME;
        if (gSoundVolume >= DEFAULT_MIN_VOLUME) gSoundVolume = DEFAULT_MIN_VOLUME; // Volume cap
      }
      break;

    case VOL_DEF:
    default:
      /* Not supported volume option, switch to default */
      gSoundVolume = DEFAULT_VOLUME;
      break;
  }

  /* Update volume */
  gMp3Chip.setVolume(gSoundVolume, gSoundVolume);

  /* Debug information */
  LOG_FMT(">> Vol: %d", gSoundVolume);
}

uint16_t scan_max_num_tracks(void)
{
  uint16_t track_counts = 0;

  /* Move to root folder */
  sd.chdir("/", true); // Change to root dir

  /* Go through each file and check for its mp3 extension */
  bool result = true;

  while (true == result)
  {
    result = file.openNext(sd.vwd(), O_READ);

    //LOG_FMT(">> Res: %d", result);

    if (true == result)
    {
      /* Is this file playable? */
      file.getFilename(gCurrTrackName);
      //LOG_FMT(">> Scanning: %s", gCurrTrackName);

      /* Check against simple rules */
      char *temp_ext;

      /* Seek to last . character */
      temp_ext = strrchr(gCurrTrackName, '.');
      temp_ext += 1; // Seek next character

      if ( (('T' == gCurrTrackName[0]) || ('t' == gCurrTrackName[0]))
           && (('R' == gCurrTrackName[1]) || ('r' == gCurrTrackName[1]))
           && (('A' == gCurrTrackName[2]) || ('a' == gCurrTrackName[2]))
           && ((0 == strcasecmp(temp_ext, "MP3")) || (0 == strcasecmp(temp_ext, "mp3")) )
         )
      {
        /* Supported file */
        track_counts++;
      }
    }
    else
    {
      /* This is all, no more file in this directory, ensure it is still under root folder */
      sd.chdir("/", true);
    }

    file.close();
  }

  /* Limit check: no file detected, however, SD card always have track000.mp3 file as default */
  if (track_counts <= 0) track_counts = 1;

  /* Limit check: Too many files detected, only play the first 999 files */
  if (track_counts >= MAX_NUM_FILES) track_counts = MAX_NUM_FILES - 1;

  return track_counts;
}

uint32_t play_next_song(void)
{
  uint32_t result = 0;

  /* Show free ram */
  LOG_SHOW_FREERAM();

  /* Play next track */
  static uint32_t sPlayIdx = 0;

  /* Generate track name, files must follow this rule! */
  char temp_trackname[13] = "TRACK000.MP3"; // Default name in 8.3 format (includes null)

  /* Play until at least a file is detected */
  do
  {
    sprintf(temp_trackname, "TRACK%03d.MP3", sPlayIdx);

    result = gMp3Chip.playMP3(temp_trackname);

    LOG_FMT(">> Playing: %s, status: %d ...", temp_trackname, result);

    /* Next track */
    sPlayIdx++;
    if (sPlayIdx > gMaxNumberOfFiles) sPlayIdx = 0;
  } while (0 != result);

  return result;
}

/*************************************/
/* System utilities */
/*************************************/

void error_blink(syserrors errorCode, colors color, bool singleTrigger)
{
  int error_blinks = (int)errorCode;
  byte temp_rgb = 0x7;  // Off

  switch (color)
  {
    case GREEN:
      temp_rgb = 0x5;
      break;

    case BLUE:
      temp_rgb = 0x3;
      break;

    case RED:
    default:
      temp_rgb = 0x6;
      break;
  }

  /* Forever loop here in case of fatal error */
  while (true)
  {
    for (int i = 0; i < error_blinks; i++)
    {
      update_bit_rgb_led(0x7);  // All off
      delay(100);
      update_bit_rgb_led(temp_rgb);
      delay(200);
    }
    delay(1000);

    /* If it is single trigger, break out of loop */
    if (true == singleTrigger) break;
  }

  update_bit_rgb_led(0x7);  // All off
}

void good_blink(sysokstat okCode, colors color)
{
  int ok_blinks = (int)okCode;
  byte temp_rgb = 0x7;  // Off

  switch (color)
  {
    case GREEN:
      temp_rgb = 0x5;
      break;

    case BLUE:
      temp_rgb = 0x3;
      break;

    case RED:
    default:
      temp_rgb = 0x6;
      break;
  }

  /* Blink only once */
  for (int i = 0; i < ok_blinks; i++)
  {
    update_bit_rgb_led(0x7);  // All off
    delay(100);
    update_bit_rgb_led(temp_rgb);
    delay(200);
  }
  delay(1000);

  update_bit_rgb_led(0x7);  // All off
}

/*************************************/
/* XBEE / Debug Utilities */
/*************************************/

/* Command Broadcast TX Request */
void xbee_cmd_bc_tx(byte toNode, byte commandId, byte subCommandId)
{
#if(XBEE_ENABLED==1)
  /* Command Payload */
  uint8_t temp_payload[8] = {};
  temp_payload[0] = toNode;
  temp_payload[1] = commandId;
  temp_payload[2] = subCommandId;
  temp_payload[3] = 0;

  /* Create address (to coordinator) */
  XBeeAddress64 addr64 = XBeeAddress64(0x0, 0xffff); // 0x000000000000ffff is broadcast
  
  ZBTxRequest zb_tx = ZBTxRequest(
    addr64,           // Address 64: (default: 0xffff or broadcast)
    0xfffe,           // Address 16: (default: to all router node)
    0x0,              // Broadcast radius
    0x0,              // Options
    temp_payload,      // Data payload
    sizeof(temp_payload), // Data payload length
    0x1               // FrameId (0: no response required, 1: response required)
    ); // To all routers, expecting response
    
  ZBTxStatusResponse tx_status = ZBTxStatusResponse();

  /* Begin transmit */
  gXbee.send(zb_tx);

  /* Wait for status response, 500ms */
  if (gXbee.readPacket(500))
  {
    /* Should be TX Status */
    if (ZB_TX_STATUS_RESPONSE == gXbee.getResponse().getApiId())
    {
      gXbee.getResponse().getZBTxStatusResponse(tx_status);

      /* Get delivery status, 5th byte */
      if (SUCCESS == tx_status.getDeliveryStatus())
      {
        /* GOOD! */
        good_blink(GOOD_XBEE_TX_SUCCESS, GREEN);
        LOG_DBG(F(">> XBEE TX OK!"));
      }
      else
      {
        /* No one received */
        error_blink(ERR_XBEE_NO_LISTENER, RED, true);
        LOG_ERR(F(">> XBEE NO RX!"));
      }
    }
  }
  else if (gXbee.getResponse().isError())
  {
    /* Received improper response */
    error_blink(ERR_XBEE_INV_RESPONSE, RED, true);
    LOG_ERR(F(">> XBEE Response Error!"));
  }
  else
  {
    /* No response at all ... */
    error_blink(ERR_XBEE_NO_RESPONSE, RED, true);
    LOG_ERR(F(">> XBEE No Response!"));
  }

#endif

}

/* Command RX */
bool xbee_cmd_rx(byte& toNode, byte& commandId, byte& subCommand0Id, byte& subCommand1Id)
{
  bool new_rx_avail = false;
  
#if(XBEE_ENABLED==1)
  ZBRxResponse rx = ZBRxResponse();
  ModemStatusResponse msr = ModemStatusResponse();

  /* Check XBEE packet */
  gXbee.readPacket();

  /* Check if any response */
  if (gXbee.getResponse().isAvailable())
  {
    if (ZB_RX_RESPONSE == gXbee.getResponse().getApiId())
    {
      /* API RX */
      gXbee.getResponse().getZBRxResponse(rx);

      if (ZB_PACKET_ACKNOWLEDGED == rx.getOption())
      {
        /* Sender Ack'd */
        good_blink(GOOD_XBEE_RX_SUCCESS, GREEN);
      }
      else
      {
        /* Sender didn't get Ack'd */
        error_blink(ERR_XBEE_RX_NOT_ACKD, RED, true);
      }

      /* Extract payload (b0: to, b1: cmd, b2: sub cmd) */
      byte pl_0 = rx.getData(0);
      byte pl_1 = rx.getData(1);
      byte pl_2 = rx.getData(2);
      byte pl_3 = rx.getData(3);

      /* Update value */
      toNode = pl_0;
      commandId = pl_1;
      subCommand0Id = pl_2;
      subCommand1Id = pl_3;

      /* RX Updated */
      new_rx_avail = true;
    }
    else if (MODEM_STATUS_RESPONSE == gXbee.getResponse().getApiId())
    {
      /* Modem status response */
      gXbee.getResponse().getModemStatusResponse(msr);
      if (ASSOCIATED == msr.getStatus())
      {
        /* Do something */
        good_blink(GOOD_XBEE_ASSOCIATED, GREEN);
        LOG_DBG(F(">> XBEE MSR: ASSOCIATED"));
      }
      else if (DISASSOCIATED == msr.getStatus())
      {
        /* Do something ? */
        error_blink(ERR_XBEE_DISASSOCIATED, RED, true);
        LOG_DBG(F(">> XBEE MSR: DISASSOCIATED"));
      }
      else
      {
        /* Do something ? */
        error_blink(ERR_XBEE_UNKNMSR, RED, true);
        LOG_DBG(F(">> XBEE MSR: ???"));
      }
    }
    else
    {
      /* Other response, ignore */
      error_blink(ERR_XBEE_UNKNRESP, RED, true);
      LOG_DBG(F(">> XBEE ?? RESP"));
    }
    
  }
  else if (gXbee.getResponse().isError())
  {
    error_blink(ERR_XBEE_RX_NORESP, RED, true);
    LOG_FMT("Error reading packet. %d", gXbee.getResponse().getErrorCode());
  }

#endif

  return new_rx_avail;
}

