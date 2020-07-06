// setup and initialize things

////////////////////////////  ODrives
// Serial to the ODrive
//RX to ODrive TX(gpio1), TX to ODrive RX(gpio2)
// ODrive object
ODriveArduino odrv_FrontRight(Serial1);     // RX(pin0) to ODrive TX(gpio1), TX(pin1) to ODrive TX(gpio2)
ODriveArduino odrv_FrontAdductor(Serial2);  // RX(pin9) to ODrive TX(gpio1), TX(pin10) to ODrive TX(gpio2)
ODriveArduino odrv_FrontLeft(Serial3);      // RX(pin9) to ODrive TX(gpio1), TX(pin10) to ODrive TX(gpio2)

#define ODRIVE_BAUD_RATE 500000



////////////////////////////  Timer Finite State Machine
TimerFiniteStateMachine FSM(10); // setup state machine with 10s loop interval


// Leg Kinematic Model
LegKinematics modelFRL(4.1339, 5.0, 2.556693);
LegKinematics modelFLL(4.1339, 5.0, 2.556693);


// odrive motor objects
// FRONT RIGHT LEG
ODrive_Motor FRL_Hip(odrv_FrontRight, 1, 1); // is not reversed
ODrive_Motor FRL_Knee(odrv_FrontRight, 0, -1); // is reversed
ODrive_Motor FRL_Adductor(odrv_FrontAdductor, 0, 1); // is not reversed

// FRONT LEFT LEG
ODrive_Motor FLL_Hip(odrv_FrontLeft, 0, -1); // is  reversed
ODrive_Motor FLL_Knee(odrv_FrontLeft, 1, 1); // is not reversed
ODrive_Motor FLL_Adductor(odrv_FrontAdductor, 1, 1); // is not reversed


// Setup Leg Controller Walkcycle
LegController FRL_Controller(modelFRL);
LegController FLL_Controller(modelFLL);

// void setup
void setup() {
  // Begin ODrive Serials
  Serial1.begin(ODRIVE_BAUD_RATE);
  Serial2.begin(ODRIVE_BAUD_RATE);
  Serial3.begin(ODRIVE_BAUD_RATE);

  // Begin Serial to PC
  Serial.begin(115200);

  // set initial position
  modelFRL.setPosXY(0, 8.5);
  modelFLL.setPosXY(0, 8.5);


  // define properties of legs:
  // FRONT RIGHT LEG
  FRL_Knee.setAngleConv(25 * 2048 / 360.0); // GR * cpr / 360
  FRL_Knee.setRangeOfMotion(145.0); // +/- degrees
  FRL_Knee.setVelocityLimit(100000);

  FRL_Hip.setAngleConv(25 * 2130 / 360.0);
  FRL_Hip.setRangeOfMotion(180.0);
  FRL_Hip.setVelocityLimit(100000);


  FRL_Adductor.setAngleConv(25 * 2048 / 360.0);
  FRL_Adductor.setRangeOfMotion(90.0);
  FRL_Adductor.setVelocityLimit(100000);


  // FRONT LEFT LEG
  FLL_Knee.setAngleConv(25 * 2048 / 360.0); // GR * cpr / 360
  FLL_Knee.setRangeOfMotion(145.0); // +/- degrees
  FLL_Knee.setVelocityLimit(100000);

  FLL_Hip.setAngleConv(25 * 2130 / 360.0);
  FLL_Hip.setRangeOfMotion(180.0);
  FLL_Hip.setVelocityLimit(100000);
  
  FLL_Adductor.setAngleConv(25 * 2048 / 360.0);
  FLL_Adductor.setRangeOfMotion(90.0);
  FLL_Adductor.setVelocityLimit(100000);

  // Arduino Calibration does not work for some reason
  //FRL_Knee.fullCalibrate();
  //FRL_Hip.fullCalibrate();

  //setupIMU();

  FLL_Controller.setPhaseShift(0);
}
