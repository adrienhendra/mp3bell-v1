/*************************************/
/* MP3 bell for Lilypad MP3 hardware */
/* By: Adrien                        */
/* Date: June 2017                   */
/*************************************/

/* Include libraries */
#include <PinChangeInt.h> // PCintPort
#include <XBee.h>         // XBee

#include <stdio.h>        // Std IO

/* Function Macros */
#define DEBUG_ENABLED 255//0 // Set to 255 for debug serial mode

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

void setup() {
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
