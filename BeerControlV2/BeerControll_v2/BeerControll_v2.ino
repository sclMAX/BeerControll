#include "pins_RAMPS.h"
#include "fastio.h"
#include <stdint.h>

#define BUTTON_PRESSED(BN) !READ(BTN_##BN)
#define LCD_UPDATE_INTERVAL 100
#define LCD_TIMEOUT_TO_STATUS 15000
#define BLEN_A 0
#define BLEN_B 1
// Encoder click is directly connected
#define BLEN_C 2
#define EN_C (_BV(BLEN_C))
#define EN_A (_BV(BLEN_A))
#define EN_B (_BV(BLEN_B))
#define EN_C (_BV(BLEN_C))
#define LCD_CLICKED (buttons & EN_C)
volatile uint8_t buttons;

/**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if defined(EN_A) && defined(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  

}

/**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
void lcd_buttons_update()
{
  uint8_t newbutton = 0;
  if (BUTTON_PRESSED(EN1))
    newbutton |= EN_A;
  if (BUTTON_PRESSED(EN2))
    newbutton |= EN_B;
  millis_t now = millis();
  if (ELAPSED(now, next_button_update_ms) && BUTTON_PRESSED(ENC))
    newbutton |= EN_C;
  buttons = newbutton;
  GET_BUTTON_STATES(buttons);

/*// Manage encoder rotation
#if ENABLED(REVERSE_MENU_DIRECTION) && ENABLED(REVERSE_ENCODER_DIRECTION)
  #define ENCODER_DIFF_CW (encoderDiff -= encoderDirection)
  #define ENCODER_DIFF_CCW (encoderDiff += encoderDirection)
#elif ENABLED(REVERSE_MENU_DIRECTION)
  #define ENCODER_DIFF_CW (encoderDiff += encoderDirection)
  #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
#elif ENABLED(REVERSE_ENCODER_DIRECTION)
  #define ENCODER_DIFF_CW (encoderDiff--)
  #define ENCODER_DIFF_CCW (encoderDiff++)
#else */
#define ENCODER_DIFF_CW (encoderDiff++)
#define ENCODER_DIFF_CCW (encoderDiff--)

#define ENCODER_SPIN(_E1, _E2) \
  switch (lastEncoderBits)     \
  {                            \
  case _E1:                    \
    ENCODER_DIFF_CW;           \
    break;                     \
  case _E2:                    \
    ENCODER_DIFF_CCW;          \
  }

  uint8_t enc = 0;
  if (buttons & EN_A)
    enc |= B01;
  if (buttons & EN_B)
    enc |= B10;
  if (enc != lastEncoderBits)
  {
    switch (enc)
    {
    case encrot0:
      ENCODER_SPIN(encrot3, encrot1);
      break;
    case encrot1:
      ENCODER_SPIN(encrot0, encrot2);
      break;
    case encrot2:
      ENCODER_SPIN(encrot1, encrot3);
      break;
    case encrot3:
      ENCODER_SPIN(encrot2, encrot0);
      break;
    }
  }
  lastEncoderBits = enc;
}