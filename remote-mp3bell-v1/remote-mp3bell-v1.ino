/*************************************/
/* MP3 bell for Lilypad MP3 hardware */
/* By: Adrien                        */
/* Date: June 2017                   */
/*************************************/

/* Include libraries */
//#include <PinChangeInt.h> // PCintPort
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

#else

/* No debug or other values */
#define LOG_FMT(f,...) do {} while(0)
#define LOG_ERR(msg) do {} while(0)
#define LOG_DBG(msg) do {} while(0)

#endif

#define XBEE_ENABLED 1  // 0: Disable XBEE support, 1: Enable XBEE

/*************************************/

/* Globals and constants */

/* Triggers */
#define IO_TRIG_BTN_0   2 // D2 to user button
#define IO_OUT_PIEZO    6 // D6 for Piezo driver (w/ PWM support)
#define IO_RXI          0 // This pin is shared with UART RX, connected to DOUT (pin2) on XBEE
#define IO_TXO          1 // This pin is shared with UART TX, connected to DIN (via 100 ohm resistor, pin3) on XBEE

const uint32_t BUTTON_DEBOUNCE_MS = 10; // 10 millisecond debounce period

const byte XBEE_MY_PROTOCOL_ID = 1; // Remote Bell is ID 1

/* Melodies */
const int gRingMelodyLength = 42;
char gRingMelody[gRingMelodyLength] = "ccggaagffeeddcggffeedggffeedccggaagffeeddc";
int gRingBeats[gRingMelodyLength] = {1,1,1,1,1,1,2,1,1,1,1,1,1,2,1,1,1,1,1,1,2,1,1,1,1,1,1,2,1,1,1,1,1,1,2,1,1,1,1,1,1,2};
int gRingTempo = 260;

const int gRmtRingMelodyLength = 3;
char gRmtRingMelody[gRmtRingMelodyLength] = "ccg";
int gRmtRingBeats[gRmtRingMelodyLength] = {1,1,4};
int gRmtRingTempo = 200;

/* Enums */

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

/*************************************/
/* Objects */
XBee gXbee = XBee(); // XBee (XBEE TH REG S2C)

/* Globals */
volatile bool gButtonPressed = 0;
volatile bool gButtonReleased = 0;
volatile uint32_t gButtonDowntime = 0;

/*************************************/

/* Arduino Setup */
void setup()
{
  /* Configure trigger input pins with weak-pullups */
  pinMode(IO_TRIG_BTN_0, INPUT_PULLUP); // Bell trigger

  /* Configure output for PIEZO driver */
  pinMode(IO_OUT_PIEZO, OUTPUT);

  /* Configure Serial and XBEE interfaces */
  pinMode(IO_RXI, INPUT);
  pinMode(IO_TXO, OUTPUT);

  /* Initialize serial port */
  Serial.begin(9600);

#if(XBEE_ENABLED==1)
  /* Initialize XBee */
  gXbee.setSerial(Serial);
#endif

  /* Initialize ISRs */
  init_isr();

  LOG_DBG(F("Hello!"));
}

void loop()
{
  static bool s_bell_active = false;
  static bool s_ring_bell_active = false;
  static bool s_rmt_bell_active = false;
  
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
  //LOG_FMT(">> XBEE RX T: %d, C: %d, S: %d", rx_to_node, rx_cmd, rx_subcmd0);

  /* Handle XBEE request */
  if ((true == new_rx_ready) 
      && ((XBEE_MY_PROTOCOL_ID == rx_to_node) || (255 == rx_to_node)))
  {
    switch(rx_cmd)
    {
      case 0:
        /* Ring command */
        s_ring_bell_active = true;
        break;

      case 1:
        /* Ring Remote */
        s_rmt_bell_active = true;
        break;
        
      case 2:
      default:
        /* Ignore this command */
        break;
    }
  }
  
#endif

  /* Request bell */
  if (true == s_bell_active)
  {
#if(XBEE_ENABLED==1)
    /* Send broadcast command through XBEE if enabled */
    xbee_cmd_bc_tx(0, 0, 0);
#endif
    
    /* Clear bell flag */
    s_bell_active = false;
  }

  /* Ring Bell */
  if (true == s_ring_bell_active)
  {
    /* Play Melody */
    play_ring_melody();
    
    /* Clear bell flag */
    s_ring_bell_active = false;
  }
  
  /* Play Remote Bell (e.g. confirmation) */
  if (true == s_rmt_bell_active)
  {
    /* Play Melody on Remote */
    play_remote_ring_melody();
    
    /* Clear bell flag */
    s_rmt_bell_active = false;
  }
}


/*************************************/
/* Interrupt routines */
/*************************************/
void init_isr(void)
{
  /* D2 is capable of doing ISR */
  attachInterrupt(digitalPinToInterrupt(IO_TRIG_BTN_0), user_button_isr, CHANGE);
}

void user_button_isr(void)
{
  static btnstate button_state = BTN_DOWN;
  static uint32_t button_start_time_ms = 0;
  static uint32_t button_end_time_ms = 0;

  /* Get button state */
  uint8_t curr_pin_state = digitalRead(IO_TRIG_BTN_0);

  /* Read button transition */
  if ((LOW == curr_pin_state) && (BTN_DOWN == button_state))
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
  else if ((HIGH == curr_pin_state) && (BTN_UP == button_state))
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

void play_ring_melody(void)
{
  int i = 0;
  int duration = 0;

  for (i = 0; i < gRingMelodyLength; i++) // step through the song arrays
  {
    duration = gRingBeats[i] * gRingTempo;    // length of note/rest in ms

    if (gRingMelody[i] == ' ')   // is this a rest? 
    {
      delay(duration);      // then pause for a moment
    }
    else  // otherwise, play the note
    {
      tone(IO_OUT_PIEZO, lut_frequency(gRingMelody[i]), duration);
      delay(duration);  // wait for tone to finish
    }
    delay(gRingTempo/10);  // brief pause between notes
  }
}

void play_remote_ring_melody(void)
{  
  int i = 0;
  int duration = 0;
  int melody_length = sizeof(gRmtRingMelody);

  for (i = 0; i < gRmtRingMelodyLength; i++)
  {
    duration = gRmtRingBeats[i] * gRmtRingTempo;    // length of note/rest in ms

    if (gRmtRingMelody[i] == ' ')   // is this a rest? 
    {
      delay(duration);      // then pause for a moment
    }
    else  // otherwise, play the note
    {
      tone(IO_OUT_PIEZO, lut_frequency(gRmtRingMelody[i]), duration);
      delay(duration);  // wait for tone to finish
    }
    delay(gRmtRingTempo/2);  // brief pause between notes
  }
}

int lut_frequency(char note) 
{
  int i = 0;
  const int numNotes = 8;  // number of notes we're storing

  char names[] =      { 'c' , 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int frequencies[] = { 262 , 294, 330, 349, 392, 440, 494, 523 };

  for (i = 0; i < numNotes; i++)
  {
    if (names[i] == note)
    {
      return(frequencies[i]);
    }
  }
  return(0);
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
  temp_payload[0] = (uint8_t)toNode;
  temp_payload[1] = (uint8_t)commandId;
  temp_payload[2] = (uint8_t)subCommandId;
  temp_payload[3] = (uint8_t)0;

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
        LOG_DBG(F(">> XBEE TX OK!"));
      }
      else
      {
        /* No one received */
        LOG_ERR(F(">> XBEE NO RX!"));
      }
    }
  }
  else if (gXbee.getResponse().isError())
  {
    /* Received improper response */
    LOG_ERR(F(">> XBEE Response Error!"));
  }
  else
  {
    /* No response at all ... */
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
      }
      else
      {
        /* Sender didn't get Ack'd */
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
        LOG_DBG(F(">> XBEE MSR: ASSOCIATED"));
      }
      else if (DISASSOCIATED == msr.getStatus())
      {
        /* Do something ? */
        LOG_DBG(F(">> XBEE MSR: DISASSOCIATED"));
      }
      else
      {
        /* Do something ? */
        LOG_DBG(F(">> XBEE MSR: ???"));
      }
    }
    else
    {
      /* Other response, ignore */
      LOG_DBG(F(">> XBEE ?? RESP"));
    }
    
  }
  else if (gXbee.getResponse().isError())
  {
    LOG_FMT("Error reading packet. %d", gXbee.getResponse().getErrorCode());
  }

#endif

  return new_rx_avail;
}

