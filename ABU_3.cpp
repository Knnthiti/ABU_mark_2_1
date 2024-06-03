#include "ABU_3.h"

#include <SPI.h>
#include <RF24.h>

///////////////////////////////////////////Class Joy///////////////////////////////////////////////////////////////////////////
void Joy ::structToArray() {
  for (int i = 0; i < 4; ++i) {
    move[i] = Str_PS2.move[i];
  }
  for (int i = 0; i < 8; ++i) {
    attack[i] = Str_PS2.attack[i];
  }
  for (int i = 0; i < 2; ++i) {
    seting[i] = Str_PS2.seting[i];
  }
  for (int i = 0; i < 4; ++i) {
    stickValues[i] = Str_PS2.stickValues[i];
  }
}

void Joy ::Setup_PS2(uint8_t _clk, uint8_t _cmd, uint8_t _att, uint8_t _dat) {
  error = config_gamepad(_clk, _cmd, _att, _dat, true, true);
  if (error == 1) {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  } else if (error == 2) {
    Serial.println("Controller found but not accepting commands. See readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  } else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");
  }

  if (error == 0) {
    type = readType();
    switch (type) {
      case 0:
        Serial.println(" Unknown Controller type found ");
        break;
      case 1:
        Serial.println(" DualShock Controller found ");
        break;
      case 2:
        Serial.println(" GuitarHero Controller found ");
        break;
      case 3:
        Serial.println(" Wireless Sony DualShock Controller found ");
        break;
    }
  }
  delay(500);
}

byte Joy ::PS2_error() {
  return error;
}

byte Joy ::PS2_type() {
  return type;
}

void Joy ::PS2_readValue() {
  if (type == 1) {
    read_gamepad(false, vibrate);
    Str_PS2.move[0] = ButtonPressed(PSB_PAD_UP);
    Str_PS2.move[1] = Button(PSB_PAD_RIGHT);
    Str_PS2.move[2] = ButtonPressed(PSB_PAD_DOWN);
    Str_PS2.move[3] = Button(PSB_PAD_LEFT);

    Str_PS2.attack[0] = Button(PSB_GREEN);
    Str_PS2.attack[1] = Button(PSB_RED);
    Str_PS2.attack[2] = ButtonPressed(PSB_PINK);
    Str_PS2.attack[3] = Button(PSB_BLUE);
    Str_PS2.attack[4] = Button(PSB_L1);
    Str_PS2.attack[5] = ButtonPressed(PSB_R1);
    Str_PS2.attack[6] = Button(PSB_L2);
    Str_PS2.attack[7] = ButtonPressed(PSB_R2);

    Str_PS2.seting[0] = ButtonPressed(PSB_SELECT);
    Str_PS2.seting[1] = Button(PSB_START);

    vibrate = Analog(PSAB_BLUE);

    Str_PS2.stickValues[0] = map(Analog(PSS_LY), 255, 0, -128, 127);
    Str_PS2.stickValues[1] = map(Analog(PSS_LX), 255, 0, -128, 127);
    Str_PS2.stickValues[3] = map(Analog(PSS_RY), 255, 0, 127, -128);
    Str_PS2.stickValues[2] = map(Analog(PSS_RX), 255, 0, 127, -128);
  }

  structToArray();
}

void Joy ::print_PS2() {
  // structToArray();
  Serial.print("move ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(move[i]);
    Serial.print(" ");
  }

  Serial.print(" attack ");
  for (int i = 0; i < 8; ++i) {
    Serial.print(attack[i]);
    Serial.print(" ");
  }

  Serial.print(" seting ");
  for (int i = 0; i < 2; ++i) {
    Serial.print(seting[i]);
    Serial.print(" ");
  }

  Serial.print(" stickValues ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(stickValues[i]);
    Serial.print(" ");
  }

  Serial.println(" ");
}

void Joy ::set_PS2_0() {
  for (int i = 0; i < 4; ++i) {
    Str_PS2.move[i] = 0;
  }

  for (int i = 0; i < 8; ++i) {
    Str_PS2.attack[i] = 0;
  }

  for (int i = 0; i < 2; ++i) {
    Str_PS2.seting[i] = 0;
  }

  for (int i = 0; i < 4; ++i) {
    Str_PS2.stickValues[i] = 0;
  }
  structToArray();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////class ABU_joy///////////////////////////////////////////////////////////////
void ABU_Joy ::Setup_Joy_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1) {
  nRF24.ce_pin = CE;
  nRF24.csn_pin = CNS;
  pipes[0] = pipes_0;
  pipes[1] = pipes_1;

  SPI.setBitOrder(MSBFIRST);
  nRF24.begin();
  nRF24.openWritingPipe(pipes[0]);
  nRF24.openReadingPipe(true, pipes[1]);
  // setChannel(69);
}

void ABU_Joy ::Joy_Sendvalue_nRF24() {
  nRF24.stopListening();
  nRF24.write(&Str_PS2, sizeof(Str_PS2));
}

#ifdef CONFIG_IDF_TARGET_ESP32
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef ESPNOW
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success " : "Delivery Fail ");
#endif
}

void ABU_Joy ::Setup_Joy_ESPNOW(const uint8_t broadAddress[]) {
  // memcpy(&broadcastAddress, &broadAddress, sizeof(broadAddress));
  broadcastAddress[0] = broadAddress[0];
  broadcastAddress[1] = broadAddress[1];
  broadcastAddress[2] = broadAddress[2];
  broadcastAddress[3] = broadAddress[3];
  broadcastAddress[4] = broadAddress[4];
  broadcastAddress[5] = broadAddress[5];

  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Failed to add peer");
#endif
    return;
  }
}

void ABU_Joy ::Joy_Sendvalue_ESPNOW() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Str_PS2, sizeof(Str_PS2));

  if (result == ESP_OK) {
#ifdef ESPNOW
    Serial.println(" | Sending confirmed");
#endif
  } else {
#ifdef ESPNOW
    Serial.println(" | Sending error");
#endif
  }
  delay(10);  //COMMUNICATION
}
#endif

void ABU_Joy ::Print_GEAR(uint8_t button_speedUP, uint8_t button_speedDOWN) {
  if (attack[button_speedUP] == 1) {
    Gear_joy = 1;
  } else if (attack[button_speedDOWN] == 1) {
    Gear_joy = 2;
  }

  if (Gear_joy < 0) {
    Gear_joy = 0;
  } else if (Gear_joy == 0) {
    digitalWrite(pin_G1, 0);
    digitalWrite(pin_G2, 0);
    digitalWrite(pin_G3, 0);
    Serial.print("Gear-0 ");
  } else if (Gear_joy == 1) {
    digitalWrite(pin_G1, 1);
    digitalWrite(pin_G2, 0);
    digitalWrite(pin_G3, 0);
    Serial.print("Gear-1 ");
  } else if (Gear_joy == 2) {
    digitalWrite(pin_G1, 1);
    digitalWrite(pin_G2, 1);
    digitalWrite(pin_G3, 0);
    Serial.print("Gear-2 ");
  } else if (Gear_joy == 3) {
    digitalWrite(pin_G1, 1);
    digitalWrite(pin_G2, 1);
    digitalWrite(pin_G3, 1);
    Serial.print("Gear-3 ");
  } else {
    Gear_joy = 3;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////class wheel////////////////////////////////////////////////////////////////////////////////
void Wheel ::Setup_LF_pin(uint8_t _pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM) {
  ledcSetup_mega2560(_pin_PWM, freq, resolution_bits);
  pin_PWM[0] = _pin_PWM;
  pin_LPWM[0] = _pin_LPWM;
  pin_RPWM[0] = _pin_RPWM;
}

void Wheel ::Setup_LB_pin(uint8_t _pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM) {
  ledcSetup_mega2560(_pin_PWM, freq, resolution_bits);
  pin_PWM[1] = _pin_PWM;
  pin_LPWM[1] = _pin_LPWM;
  pin_RPWM[1] = _pin_RPWM;
}

void Wheel ::Setup_RF_pin(uint8_t _pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM) {
  ledcSetup_mega2560(_pin_PWM, freq, resolution_bits);
  pin_PWM[2] = _pin_PWM;
  pin_LPWM[2] = _pin_LPWM;
  pin_RPWM[2] = _pin_RPWM;
}

void Wheel ::Setup_RB_pin(uint8_t _pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM) {
  ledcSetup_mega2560(_pin_PWM, freq, resolution_bits);
  pin_PWM[3] = _pin_PWM;
  pin_LPWM[3] = _pin_LPWM;
  pin_RPWM[3] = _pin_RPWM;
}
void Wheel ::Wheel_equation(float scaled[]) {
  scaled[0] *= SCALE_FACTOR[0];
  scaled[1] *= SCALE_FACTOR[1];
  scaled[2] *= SCALE_FACTOR[2];

  float inverts[4] = { 0, 0, 0, 0 };
  inverts[0] = ((scaled[0] - scaled[1] - (ROBOT_TOTAL_LEN * scaled[2])) / Wheels_redius);  // LF
  inverts[1] = ((scaled[0] + scaled[1] - (ROBOT_TOTAL_LEN * scaled[2])) / Wheels_redius);  // LB
  inverts[2] = ((scaled[0] + scaled[1] + (ROBOT_TOTAL_LEN * scaled[2])) / Wheels_redius);  // RF
  inverts[3] = ((scaled[0] - scaled[1] + (ROBOT_TOTAL_LEN * scaled[2])) / Wheels_redius);  // RB

  for (uint8_t i = 0; i < 4; i++) {
    if (inverts[i] > (max_wheel_speed[Gear])) {
      inverts[i] = max_wheel_speed[Gear];
    } else if (inverts[i] < (-max_wheel_speed[Gear])) {
      inverts[i] = -max_wheel_speed[Gear];
    }
  }

  Wheels[0] = round(inverts[0]);
  Wheels[1] = round(inverts[1]);
  Wheels[2] = round(inverts[2]);
  Wheels[3] = round(inverts[3]);
}

void Wheel ::GEAR() {
  if (Gear < 0) {
    Gear = 0;
  } else if (Gear == 0) {
    SCALE_FACTOR[0] = 0.00f;  //SCALE_FACTOR_X
    SCALE_FACTOR[1] = 0.00f;  //SCALE_FACTOR_Y
    SCALE_FACTOR[2] = 0.00f;  //SCALE_FACTOR_Z
  } else if (Gear == 1) {
    SCALE_FACTOR[0] = SCALE_FACTOR_Gear_1;  //SCALE_FACTOR_X
    SCALE_FACTOR[1] = SCALE_FACTOR_Gear_1;  //SCALE_FACTOR_Y
    SCALE_FACTOR[2] = SCALE_FACTOR_Gear_1;  //SCALE_FACTOR_Z
  } else if (Gear == 2) {
    SCALE_FACTOR[0] = SCALE_FACTOR_Gear_2;  //SCALE_FACTOR_X
    SCALE_FACTOR[1] = SCALE_FACTOR_Gear_2;  //SCALE_FACTOR_Y
    SCALE_FACTOR[2] = SCALE_FACTOR_Gear_2;  //SCALE_FACTOR_Z
  } else if (Gear == 3) {
    SCALE_FACTOR[0] = SCALE_FACTOR_Gear_3;  //SCALE_FACTOR_X
    SCALE_FACTOR[1] = SCALE_FACTOR_Gear_3;  //SCALE_FACTOR_Y
    SCALE_FACTOR[2] = SCALE_FACTOR_Gear_3;  //SCALE_FACTOR_Z
  } else {
    Gear = 3;
  }
}

void Wheel ::Ramp(float speed[]) {
  for (uint8_t n = 0; n < 4; n++) {
    eror_speed[n] = speed[n] - Past_speed[n];
    Proportional[n] = eror_speed[n] * Kp;  //P
    integnator[n] += eror_speed[n] * Ki;   //I

    Ramp_speed[n] = Proportional[n] + integnator[n];  //OUTPUT PID

    Past_speed[n] = Ramp_speed[n];

    Ramp_speed[n] = round(Ramp_speed[n]);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////class motor//////////////////////////////////////////////////////////////////////////
#ifdef __AVR_ATmega2560__
void motor ::ledcSetup_mega2560(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits) {
  pinMode(pin_PWM, OUTPUT);
  if (resolution_bits == 8) { resolution_bits = 255; }
  if (resolution_bits == 10) { resolution_bits = 1023; }

  //Timer 1
  if ((pin_PWM == 11) || (pin_PWM == 12) || (pin_PWM == 13)) {
    TCCR1A = 0;
    TCCR1B = 0;
    //setup mode Fast PWM
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    // Set resolution_bits
    ICR1L = 0;
    ICR1H = 0;
    ICR1H = (uint8_t)(resolution_bits >> 8);
    ICR1L = (uint8_t)resolution_bits;

    // Set freq
    if (resolution_bits == 1023) {
      if (freq == 15000) { TCCR1B |= (1 << CS10); }
      if (freq == 2000) { TCCR1B |= (1 << CS11); }
    }
    if (resolution_bits == 255) {
      if (freq == 8000) { TCCR1B |= (1 << CS11); }
    }
  }

  //Timer 3
  if ((pin_PWM == 2) || (pin_PWM == 3) || (pin_PWM == 5)) {
    TCCR3A = 0;
    TCCR3B = 0;
    //setup mode Fast PWM
    TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
    TCCR3B |= (1 << WGM33) | (1 << WGM32);

    // Set resolution_bits
    ICR3L = 0;
    ICR3H = 0;
    ICR3H = (uint8_t)(resolution_bits >> 8);
    ICR3L = (uint8_t)resolution_bits;

    // Set freq
    if (resolution_bits == 1023) {
      if (freq == 15000) { TCCR3B |= (1 << CS30); }
      if (freq == 2000) { TCCR3B |= (1 << CS31); }
    }
    if (resolution_bits == 255) {
      if (freq == 8000) { TCCR3B |= (1 << CS31); }
    }
  }

  //Timer 4
  if ((pin_PWM == 6) || (pin_PWM == 7) || (pin_PWM == 8)) {
    TCCR4A = 0;
    TCCR4B = 0;
    //setup mode Fast PWM
    TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM41);
    TCCR4B |= (1 << WGM43) | (1 << WGM42);

    // Set resolution_bits
    ICR4L = 0;
    ICR4H = 0;
    ICR4H = (uint8_t)(resolution_bits >> 8);
    ICR4L = (uint8_t)resolution_bits;

    // Set freq
    if (resolution_bits == 1023) {
      if (freq == 15000) { TCCR4B |= (1 << CS40); }
      if (freq == 2000) { TCCR4B |= (1 << CS41); }
    }
    if (resolution_bits == 255) {
      if (freq == 8000) { TCCR4B |= (1 << CS41); }
    }
  }

  //Timer 2
  if ((pin_PWM == 9) || (pin_PWM == 10)) {
    TCCR2A = 0;
    TCCR2B = 0;
    //setup mode Fast PWM
    TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);

    // Set freq
    if (resolution_bits == 255) {
      if (freq == 8000) { TCCR2B |= (1 << CS21); }
    }
  }
}

void motor ::motorDrive_BTS7960(uint8_t pin_PWM, int32_t duty, uint8_t pin_LPWM, uint8_t pin_RPWM) {
  uint16_t duty_16b = ((duty < 0) ? -duty : duty);

  switch (pin_PWM) {
      //Timer 1
    case 11:
      OCR1AH = (uint8_t)(duty_16b >> 8);
      OCR1AL = (uint8_t)duty_16b;
      break;

    case 12:
      OCR1BH = (uint8_t)(duty_16b >> 8);
      OCR1BL = (uint8_t)duty_16b;
      break;

    case 13:
      OCR1CH = (uint8_t)(duty_16b >> 8);
      OCR1CL = (uint8_t)duty_16b;
      break;


    case 2:
      OCR3BH = (uint8_t)(duty_16b >> 8);
      OCR3BL = (uint8_t)duty_16b;
      break;

    case 3:
      OCR3CH = (uint8_t)(duty_16b >> 8);
      OCR3CL = (uint8_t)duty_16b;
      break;

    case 5:
      OCR3AH = (uint8_t)(duty_16b >> 8);
      OCR3AL = (uint8_t)duty_16b;
      break;

    //Timer 4
    case 6:
      OCR4AH = (uint8_t)(duty_16b >> 8);
      OCR4AL = (uint8_t)duty_16b;
      break;

    case 7:
      OCR4BH = (uint8_t)(duty_16b >> 8);
      OCR4BL = (uint8_t)duty_16b;
      break;

    case 8:
      OCR4CH = (uint8_t)(duty_16b >> 8);
      OCR4CL = (uint8_t)duty_16b;
      break;

    //Timer 2
    case 9:
      OCR2B = (uint8_t)duty_16b;
      break;

    case 10:
      OCR2A = (uint8_t)duty_16b;
      break;

    default:
      return;
  }

  if (digital_Setup[pin_PWM] == 0) {
    pinMode(pin_LPWM, OUTPUT);
    pinMode(pin_RPWM, OUTPUT);
    digital_Setup[pin_PWM]++;
  }

  if (duty == 0) {
    digitalWrite(pin_LPWM, 0);
    digitalWrite(pin_RPWM, 0);
  } else {
    digitalWrite(pin_LPWM, (duty <= 0) ? 0 : 1);
    digitalWrite(pin_RPWM, (duty <= 0) ? 1 : 0);
  }
}

void motor ::motorDrive_BTS7960_DC(uint8_t pin_LPWM, uint8_t pin_RPWM, int32_t duty) {
  ////////////////////////////////////ท่ายาก by AUSTIN รุ่น RB 27 /////////////////////////
  // int i;
  // uint16_t PWM_BASE[3] = {(uint16_t)OCR1AL ,(uint16_t)OCR1BL ,(uint16_t)OCR1CL};

  // *(uint16_t *)PWM_BASE[i] = (uint8_t)duty;
  // *(uint16_t *)(PWM_BASE[i]+1) = (uint8_t)(duty >> 8);
  /////////////////////////////////////////////////////////////////////////////////////
  analogWrite(pin_LPWM, (duty < 0) ? 0 : duty);
  analogWrite(pin_RPWM, (duty < 0) ? -duty : 0);
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////class ABU_ROBOT/////////////////////////////////////////////////////////////////
void ABU_ROBOT ::Setup_ROBOT_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1) {
  ce_pin = CE;
  csn_pin = CNS;
  pipes[0] = pipes_0;
  pipes[1] = pipes_1;

  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setBitOrder(MSBFIRST);
  begin();
  openWritingPipe(pipes[1]);
  openReadingPipe(1, pipes[0]);
  // setChannel(69);
  startListening();
}

void ABU_ROBOT ::Readvalue_nrf24() {
  startListening();
  if (available()) {
    read(&Str_PS2, sizeof(Str_PS2));
    structToArray();
#ifdef DEBUG_PS2
    print_PS2();
#endif
    Timeactivate = millis();
    status_nRF24 = 1;
    Ready = 1;
  } else if (millis() - Timeactivate > 500) {
    Serial.println("T_T nrf24 Not connected");
    set_PS2_0();
    structToArray();
    status_nRF24 = 0;
    Ready = 0;
  }
}

void ABU_ROBOT ::Readvalue_slave(int SLAVE_ADDR, int ANSWERSIZE) {
  delay(10);  //COMMUNICATION
  // Write a charatre to the Slave
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write("send dataPS_2");
  Wire.endTransmission();

  // Read response from Slave
  // Read back 18 characters
  Wire.requestFrom(SLAVE_ADDR, ANSWERSIZE);

  // Add characters to string
  while (Wire.available()) {
    Wire.readBytes((byte *)&Str_PS2, sizeof(Str_PS2));
    status_slave = 1;
  }

  if (status_slave == 1) {
    structToArray();
#ifdef I2C
    Serial.print("Slave_Receive ");
    print_PS2();
#endif
    Timeactivate = millis();
    status_slave = 0;
    Ready = 1;
  } else if (millis() - Timeactivate > 1000) {
#ifdef I2C
    Serial.println("NotReceive ");
#endif
    set_PS2_0();
    structToArray();
    status_slave = 0;
    Ready = 0;
  }
}

void ABU_ROBOT ::Move() {
  float _stickValues[4];
  for (int i = 0; i < 4; ++i) {
    _stickValues[i] = stickValues[i];
  }

  Wheel_equation(_stickValues);
  Ramp(Wheels);

#ifdef DEBUG_Wheel
  Serial.print("LF : ");
  Serial.print(Ramp_speed[0]);
  Serial.print(" LB : ");
  Serial.print(Ramp_speed[1]);
  Serial.print(" RF : ");
  Serial.print(Ramp_speed[2]);
  Serial.print(" RB : ");
  Serial.println(Ramp_speed[3]);
#endif
  motorDrive_BTS7960(pin_PWM[0], Ramp_speed[0], pin_LPWM[0], pin_RPWM[0]);
  motorDrive_BTS7960(pin_PWM[1], Ramp_speed[1], pin_LPWM[1], pin_RPWM[1]);
  motorDrive_BTS7960(pin_PWM[2], Ramp_speed[2], pin_LPWM[2], pin_RPWM[2]);
  motorDrive_BTS7960(pin_PWM[3], Ramp_speed[3], pin_LPWM[3], pin_RPWM[3]);

  // motorDrive_BTS7960(pin_PWM[0], Wheels[0], pin_LPWM[0], pin_RPWM[0]);
  // motorDrive_BTS7960(pin_PWM[1], Wheels[1], pin_LPWM[1], pin_RPWM[1]);
  // motorDrive_BTS7960(pin_PWM[2], Wheels[2], pin_LPWM[2], pin_RPWM[2]);
  // motorDrive_BTS7960(pin_PWM[3], Wheels[3], pin_LPWM[3], pin_RPWM[3]);
}


void ABU_ROBOT ::CLAMP(uint8_t button_Clamp_1, uint8_t button_Clamp_2, uint8_t button_Clamp_Up_Down, uint8_t button_Clamp_Grab_Poll) {
  if (attack[button_Clamp_1] == 1) {
    if (status_Clamp_1 == 0) {
      status_Clamp_1 = 1;
      digitalWrite(Clamp_1, 1);
#ifdef DEBUG_Clamp
      Serial.print("Clamp_1 : 1 ");
#endif
    } else {
      status_Clamp_1 = 0;
      digitalWrite(Clamp_1, 0);
#ifdef DEBUG_Clamp
      Serial.print("Clamp_1 : 0 ");
#endif
    }
  } else {
    if (status_Clamp_1 == 0) {
#ifdef DEBUG_Clamp
      Serial.print("Clamp_1 : 1 ");
#endif
    } else {
#ifdef DEBUG_Clamp
      Serial.print("Clamp_1 : 0 ");
#endif
    }
  }


  if (attack[button_Clamp_2] == 1) {
    if (status_Clamp_2 == 0) {
      status_Clamp_2 = 1;
      digitalWrite(Clamp_2, 1);
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_2 : 1 ");
#endif
    } else {
      status_Clamp_2 = 0;
      digitalWrite(Clamp_2, 0);
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_2 : 0 ");
#endif
    }
  } else {
    if (status_Clamp_2 == 0) {
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_2 : 1 ");
#endif
    } else {
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_2 : 0 ");
#endif
    }
  }


  if (move[button_Clamp_Up_Down] == 1) {
    if (status_Clamp_Up_Down == 0) {
      status_Clamp_Up_Down = 1;
      digitalWrite(Clamp_Up_Down, 1);
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_Up_Down : 1 ");
#endif
    } else {
      status_Clamp_Up_Down = 0;
      digitalWrite(Clamp_Up_Down, 0);
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_Up_Down : 0 ");
#endif
    }
  } else {
    if (status_Clamp_Up_Down == 0) {
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_Up_Down : 1 ");
#endif
    } else {
#ifdef DEBUG_Clamp
      Serial.print("| Clamp_Up_Down : 0 ");
#endif
    }
  }


  if (move[button_Clamp_Grab_Poll] == 1) {
    if (status_Clamp_Grab_Poll == 0) {
      status_Clamp_Grab_Poll = 1;
      digitalWrite(Clamp_Grab_Poll, 1);
#ifdef DEBUG_Clamp
      Serial.println("| Clamp_Grab_Poll : 1 ");
#endif
    } else {
      status_Clamp_Grab_Poll = 0;
      digitalWrite(Clamp_Grab_Poll, 0);
#ifdef DEBUG_Clamp
      Serial.println("| Clamp_Grab_Poll : 0 ");
#endif
    }
  } else {
    if (status_Clamp_Grab_Poll == 0) {
#ifdef DEBUG_Clamp
      Serial.println("| Clamp_Grab_Poll : 1 ");
#endif
    } else {
#ifdef DEBUG_Clamp
      Serial.println("| Clamp_Grab_Poll : 0 ");
#endif
    }
  }
}

void ABU_ROBOT ::KEEP_Ball(uint8_t button_keep) {
  if (attack[button_keep] == 1) {
    if (status_Keep_ball == 0) {
      status_Keep_ball = 1;
      digitalWrite(pin_keep_Ball, 1);
#ifdef DEBUG_Shoot
      Serial.print("Catapult : Keep_KEEP ");
#endif
    } else {
      status_Keep_ball = 0;
      digitalWrite(pin_keep_Ball, 0);
#ifdef DEBUG_Shoot
      Serial.print("Catapult : UnKeep_Ball ");
#endif
    }
  } else {
    if (status_Keep_ball == 0) {
#ifdef DEBUG_Shoot
      Serial.print("Catapult : Keep_KEEP ");
#endif
    } else {
#ifdef DEBUG_Shoot
      Serial.print("Catapult : UnKeep_Ball ");
#endif
    }
  }
}
void ABU_ROBOT ::Shoot_Ball(uint8_t button_Shoot, uint8_t button_UP, uint8_t button_Reload, int32_t force_Shoot, int32_t force_UP, int32_t force_Reload) {
  if (attack[button_Shoot] == 1) {
    if (status_Clamp_Grab_Poll == 1) {
      if (digitalRead(Set_Attack_Ball) == 1) {
        motorDrive_BTS7960(pin_Shoot, force_Shoot, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
        Serial.println("| Shoot_SHOOT ");
#endif
      }
    } else {
      motorDrive_BTS7960(pin_Shoot, 0, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
      Serial.println("| Shoot_STOP ");
#endif
    }

  } else if (attack[button_UP] == 1) {
    if (digitalRead(Set_Attack_Ball) == 1) {
      motorDrive_BTS7960(pin_Shoot, force_UP, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
      Serial.println("| Shoot_UP ");
#endif
    } else {
      motorDrive_BTS7960(pin_Shoot, 0, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
      Serial.println("| Shoot_STOP ");
#endif
    }

  } else if (attack[button_Reload] == 1) {
    if (digitalRead(Set_Reload_Ball) == 1) {
      motorDrive_BTS7960(pin_Shoot, -force_Reload, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
      Serial.println("| Shoot_Reload ");
#endif
    } else {
      motorDrive_BTS7960(pin_Shoot, 0, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
      Serial.println("| Shoot_STOP ");
#endif
    }

  } else {
    motorDrive_BTS7960(pin_Shoot, 0, pin_Attack, pin_Reload);
#ifdef DEBUG_Shoot
    Serial.println("| Shoot_STOP ");
#endif
  }
}

void ABU_ROBOT ::UP_speed(uint8_t button_speedUP, uint8_t button_speedDOWN) {
  if (attack[button_speedUP] == 1) {
    Gear = 1;
  } else if (attack[button_speedDOWN] == 1) {
    Gear = 2;
  }

  GEAR();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
