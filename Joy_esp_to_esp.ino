#ifdef CONFIG_IDF_TARGET_ESP32
#include "ABU_3.h"

ABU_Joy Joy;

// Joy PS2;
byte error_PS = 1;
byte type_PS = 0;

#define PS2_DAT 19  //MISO  19
#define PS2_CMD 23  //MOSI  23
#define PS2_SEL 5   //SS     5
#define PS2_CLK 18  //SLK   18

#define pin_G1 27
#define pin_G2 26
#define pin_G3 25

#define button_speedUP 4
#define button_speedDOWN 6

uint8_t broadAddress[6] = { 0x08, 0xD1, 0xF9, 0xE9, 0xB8, 0x24 };

void setup() {
  Serial.begin(115200);
  delay(150);
  // Joy.Setup_Joy_nRF24(8, 9, 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL);
  Joy.Setup_Joy_ESPNOW(broadAddress);

  Joy.Setup_PS2(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
  Joy.Setup_GEAR_joy(pin_G1, pin_G2, pin_G3);
}

void loop() {
  error_PS = Joy.PS2_error();
  type_PS = Joy.PS2_type();
  if (error_PS == 1) {
    Joy.Setup_PS2(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
    return;
  }
  if (type_PS != 2) {
    Joy.set_PS2_0();
    Joy.PS2_readValue();

    // Joy.Joy_Sendvalue_nRF24();
    Joy.Joy_Sendvalue_ESPNOW();

    Joy.print_PS2();

    Joy.Print_GEAR(button_speedUP, button_speedDOWN);
    if (Joy.seting[1] == 1) {
      Joy.Gear_joy = 0;
      return;
    }
  }
}
#endif