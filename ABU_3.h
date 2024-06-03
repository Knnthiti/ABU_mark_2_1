#ifndef ABU_3
#define ABU_3

#include <Arduino.h>

//PS2
#include <PS2X_lib.h>

#ifdef CONFIG_IDF_TARGET_ESP32
//ESPNOW
#include <esp_now.h>
#include <WiFi.h>

#endif

//I2C mater&slave
#include <Wire.h>

//nrf24
#include <RF24.h>

// #define ESPNOW
// #define I2C
// #define DEBUG_PS2
// #define DEBUG_Wheel
// #define DEBUG_Clamp
#define DEBUG_Shoot


class Joy : public PS2X {
public:
  boolean move[4];
  boolean attack[8];
  boolean seting[2];
  int8_t stickValues[4];

  uint8_t error = 1;
  uint8_t type = 0;
  uint8_t vibrate = 0;

  struct ControllerData {
    boolean move[4];
    boolean attack[8];
    boolean seting[2];
    int8_t stickValues[4];
  };
  ControllerData Str_PS2;

  Joy() {  // Default constructor
  }

  void structToArray();
  void Setup_PS2(uint8_t _clk, uint8_t _cmd, uint8_t _att, uint8_t _dat);
  uint8_t PS2_error();
  uint8_t PS2_type();
  void set_PS2_0();
  void PS2_readValue();
  void print_PS2();
};

class ABU_Joy : public Joy {
public:
  //nRf24
  RF24 nRF24;

#ifdef CONFIG_IDF_TARGET_ESP32
  //ESPNOW
  esp_now_peer_info_t peerInfo;
  uint8_t broadcastAddress[6];
#endif

  uint64_t pipes[2];

  int8_t Gear_joy = 0;
  uint8_t pin_G1;
  uint8_t pin_G2;
  uint8_t pin_G3;

  ///////////////////////////////////////////////////////////////////nRF24/////////////////////////////////////////////////////////////////
  void Setup_Joy_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1);
  void Joy_Sendvalue_nRF24();
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////ESPNOW/////////////////////////////////////////////////////////////////
#ifdef CONFIG_IDF_TARGET_ESP32
  void Setup_Joy_ESPNOW(const uint8_t broadAddress[]);
  void Joy_Sendvalue_ESPNOW();
#endif
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void Setup_GEAR_joy(uint8_t _pin_G1, uint8_t _pin_G2, uint8_t _pin_G3) {
    pin_G1 = _pin_G1;
    pin_G2 = _pin_G2;
    pin_G3 = _pin_G3;

    pinMode(pin_G1, OUTPUT);
    pinMode(pin_G2, OUTPUT);
    pinMode(pin_G3, OUTPUT);
  }
  void Print_GEAR(uint8_t button_speedUP, uint8_t button_speedDOWN);
};


class motor {
public:
  boolean digital_Setup[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  void ledcSetup_mega2560(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits);
  void motorDrive_BTS7960(uint8_t pin_PWM, int32_t duty, uint8_t pin_LPWM, uint8_t pin_RPWM);
  void motorDrive_BTS7960_DC(uint8_t pin_LPWM, uint8_t pin_RPWM, int32_t duty);
};

class Wheel : public motor {
public:
  float ROBOT_TOTAL_LEN;
  float Wheels_redius;

  float Wheels[4] = { 0, 0, 0, 0 };

  uint8_t pin_PWM[4];   //{LF_pin ,LB_pin ,RF_pin ,RB_pin}
  uint8_t pin_LPWM[4];  //{LF_pin ,LB_pin ,RF_pin ,RB_pin}
  uint8_t pin_RPWM[4];  //{LF_pin ,LB_pin ,RF_pin ,RB_pin}

  float SCALE_FACTOR[3];  //{SCALE_FACTOR_X ,SCALE_FACTOR_Y ,SCALE_FACTOR_Z}

  int8_t Gear = 0;
  float SCALE_FACTOR_Gear_1;
  float SCALE_FACTOR_Gear_2;
  float SCALE_FACTOR_Gear_3;

  float max_wheel_speed[4] = { 0.0, 150.0, 170.0, 190.0 };

  float Ramp_speed[4] = { 0.0, 0.0, 0.0, 0.0 };  //{LF_speed ,LB_speed ,RF_speed ,RB_speed}
  float Kp;
  float Ki;
  float eror_speed[4] = { 0.0, 0.0, 0.0, 0.0 };
  float Proportional[4] = { 0.0, 0.0, 0.0, 0.0 };
  float integnator[4] = { 0.0, 0.0, 0.0, 0.0 };
  float Past_speed[4] = { 0.0, 0.0, 0.0, 0.0 };

  void Setup_LF_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_LB_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_RF_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_RB_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);

  void Setup_Scale_factor(float SCALE_FACTOR_X, float SCALE_FACTOR_Y, float SCALE_FACTOR_Z) {
    SCALE_FACTOR[0] = SCALE_FACTOR_X;
    SCALE_FACTOR[1] = SCALE_FACTOR_Y;
    SCALE_FACTOR[2] = SCALE_FACTOR_Z;
  }

  void Setup_Wheel(float _ROBOT_LEN, float _ROBOT_WID, float _Wheels_redius) {
    ROBOT_TOTAL_LEN = _ROBOT_LEN + _ROBOT_WID;
    Wheels_redius = _Wheels_redius;
  }
  void Wheel_equation(float _scaled[]);

  void Setup_max_speed(float Gear_0, float Gear_1, float Gear_2, float Gear_3) {
    max_wheel_speed[0] = Gear_0;
    max_wheel_speed[1] = Gear_1;
    max_wheel_speed[2] = Gear_2;
    max_wheel_speed[3] = Gear_3;
  }

  void GEAR_Setup(float _SCALE_FACTOR_Gear_1, float _SCALE_FACTOR_Gear_2, float _SCALE_FACTOR_Gear_3) {
    SCALE_FACTOR_Gear_1 = _SCALE_FACTOR_Gear_1;
    SCALE_FACTOR_Gear_2 = _SCALE_FACTOR_Gear_2;
    SCALE_FACTOR_Gear_3 = _SCALE_FACTOR_Gear_3;
  }
  void GEAR();

  void SetupRamp(float _Kp, float _Ki) {
    Kp = _Kp;
    Ki = _Ki;
  }

  void Ramp(float speed[]);
};

class ABU_ROBOT : public Joy, public RF24, public Wheel {
public:
  uint64_t pipes[2];

  long Timeactivate = 0;
  boolean Ready = 0;

  boolean status_nRF24 = 0;  //(1=Pass ,0=False)
  boolean status_slave = 0;  //(1=Pass ,0=False)

  //pneumatic Clamp
  uint8_t Clamp_1;
  uint8_t Clamp_2;
  uint8_t Clamp_Up_Down;
  uint8_t Clamp_Grab_Poll;

  //pneumatic status
  boolean status_Clamp_1 = 0;
  boolean status_Clamp_2 = 0;
  boolean status_Clamp_Up_Down = 0;
  boolean status_Clamp_Grab_Poll = 0;

  //driver motor keep ball
  uint8_t pin_keep_Ball;
  boolean status_Keep_ball = 0;
  // uint8_t pin_Keep;
  // uint8_t pin_UnKeep;

  //driver motor shoot ball
  uint8_t pin_Shoot;
  uint8_t pin_Attack;
  uint8_t pin_Reload;
  uint8_t Set_Attack_Ball;  //limit switch
  uint8_t Set_Reload_Ball;  //limit switch

  ABU_ROBOT() {  // Default constructor
  }
  /////////////////////////////////////////////////////////////nRF24///////////////////////////////////////////////////////////
  void Setup_ROBOT_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1);
  void Readvalue_nrf24();
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////ESPNOW/////////////////////////////////////////////////////////
  void Setup_ROBOT_ESPNOW();
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////I2C//////////////////////////////////////////////////////////
  void Setup_ROBOT_master() {
    Wire.begin();
  }
  void Readvalue_slave(int SLAVE_ADDR, int ANSWERSIZE);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void Move();

  void Setup_Clamp(uint8_t pin_Clamp_1, uint8_t pin_Clamp_2, uint8_t pin_Clamp_Up_Down, uint8_t pin_Clamp_Grab_Poll) {
    Clamp_1 = pin_Clamp_1;
    Clamp_2 = pin_Clamp_2;
    Clamp_Up_Down = pin_Clamp_Up_Down;
    Clamp_Grab_Poll = pin_Clamp_Grab_Poll;

    pinMode(Clamp_1, OUTPUT);
    pinMode(Clamp_2, OUTPUT);
    pinMode(Clamp_Up_Down, OUTPUT);
    pinMode(Clamp_Grab_Poll, OUTPUT);
  }

  void CLAMP(uint8_t button_Clamp_1, uint8_t button_Clamp_2, uint8_t button_Clamp_Up_Down, uint8_t button_Clamp_Grab_Poll);

  void KEEP_Ball_Setup(uint8_t _pin_keep_Ball) {
    pin_keep_Ball = _pin_keep_Ball;

    pinMode(pin_keep_Ball, OUTPUT);
  }
  void KEEP_Ball(uint8_t button_keep);


  void Shoot_Ball_Setup(uint8_t _pin_Shoot, uint8_t _pin_Attack, uint8_t _pin_Reload, uint8_t _Set_Attack_Ball, uint8_t _Set_Reload_Ball) {
    pin_Shoot = _pin_Shoot;
    pin_Attack = _pin_Attack;
    pin_Reload = _pin_Reload;
    Set_Attack_Ball = _Set_Attack_Ball;
    Set_Reload_Ball = _Set_Reload_Ball;

    ledcSetup_mega2560(pin_Shoot, 8000, 8);
    pinMode(Set_Attack_Ball, INPUT_PULLUP);
    pinMode(Set_Reload_Ball, INPUT_PULLUP);
  }
  void Shoot_Ball(uint8_t button_Shoot, uint8_t button_UP, uint8_t button_Reload, int32_t force_Shoot, int32_t force_UP, int32_t force_Reload);

  // void UP_speed(uint8_t button_speedUP, uint8_t button_speedDOWN);
  void UP_speed(uint8_t button_speedUP, uint8_t button_speedDOWN);
};
#endif