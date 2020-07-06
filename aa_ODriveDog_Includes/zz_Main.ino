// Main Loop
void loop() {
  // if imu interrupt has recently triggered, read the IMU data
  //if (mpuInterrupt) {
  //  readIMU();
  //}

  // serial input
  if (Serial.available() > 0) {
    // read the incoming byte:
    String incomingData = Serial.readString();

    if (incomingData.substring(0, 1) == "X") {
      // change x target
      modelFRL.setPosXY(incomingData.substring(4).toFloat(), modelFRL.getYTarg());
      Serial.print("x changed to: ");
      Serial.println((incomingData.substring(4).toFloat()));
    }
    else if (incomingData.substring(0, 1) == "Y") {
      // change y target
      modelFRL.setPosXY(modelFRL.getXTarg(), incomingData.substring(4).toFloat());
      Serial.print("y changed to: ");
      Serial.println((incomingData.substring(4).toFloat()));
    }
  }


  // Finite State Machine
  FSM.updateMillis();

  // if the timer is ready to do next cycle
  if (FSM.isReady()) {

    // balance adductor
    //float roll_rads = (ypr[2]);
    
    
    modelFRL.setPosZ(2.55);
    modelFLL.setPosZ(2.55);

    // update the controller for the walk cycle
    FRL_Controller.update();
    FLL_Controller.update();

    // update leg positions for walk cycle
    FRL_Knee.setPositionDeg(modelFRL.angle2E_DOWN());
    FRL_Hip.setPositionDeg(modelFRL.angle1E_DOWN() - 90);
    //FRL_Adductor.setPositionDeg(- (ypr[2] * 180 / PI) );

    FLL_Knee.setPositionDeg(modelFRL.angle2E_DOWN());
    FLL_Hip.setPositionDeg(modelFRL.angle1E_DOWN() - 90);
    //FLL_Adductor.setPositionDeg((ypr[2] * 180 / PI) );


    // Print
    Serial.print(FSM.getDeltaTime());
    Serial.print(",\t");
/*
    float a1 = 2.55669;
    float rL = sqrt(FRL_Controller.x * FRL_Controller.x + FRL_Controller.y * FRL_Controller.y);
    float Rz = sqrt(a1*a1 + rL * rL);

    float theta0 = FRL_Hip.posEstimate() * PI / 180.0;
    float theta1 = FRL_Adductor.posEstimate() * PI / 180.0;

    Serial.print(Rz * cos(theta0));
    Serial.print(",\t");
    Serial.print(Rz * sin(theta0) * sin(theta1));
    Serial.print(",\t");
    Serial.print(Rz * cos(theta1));
    Serial.print(",\t");
*/

    Serial.print(FRL_Hip.target());
    Serial.print(",\t");
    Serial.print(FRL_Knee.target());
    Serial.print(",\t");
    Serial.print(FRL_Adductor.target());
    Serial.print(",\t");
    
    //Serial.print((float) (- (ypr[2] * 180 / M_PI)));
    Serial.print(",\t");

    /*
      float w_theta2 = FRL_Hip.velEstimate() * PI / 180.0;
      Serial.print(w_theta2);
      Serial.print(", ");
      float w_theta3 = FRL_Knee.velEstimate() * PI / 180.0;
      Serial.print(w_theta3);
      Serial.print(", ");

      float theta1 = 0.0;
      float theta2 = FRL_Hip.posEstimate() * PI / 180.0;
      float theta3 = FRL_Knee.posEstimate() * PI / 180.0;

      float a1 = 2.0;
      float a2 = 4.1339;
      float a3 = 5.0;


      float v_x  =  -( -sin(theta1) * (a3 * sin(-(PI / 2.0) + theta1) + a2 * sin(-(PI / 2.0) + theta1) + a1 * cos( -(PI / 2.0) + theta1)) ) * w_theta2   +
                  -(-sin(theta1) * a3 * sin(-(PI / 2.0) + theta1)) * w_theta3;
      float v_y  =  -( sin(theta1) * (a3 * cos(-(PI / 2.0) + theta1) + a2 * cos(-(PI / 2.0) + theta1) + a1 * -sin( -(PI / 2.0) + theta1))  ) * w_theta2   +
                  -(sin(theta1) * a3 * cos(-(PI / 2.0) + theta1) ) * w_theta3;
      float v_z  =  (cos(theta1) * (a3 * sin(-(PI / 2.0) + theta1) + a2 * sin(-(PI / 2.0) + theta1) + a1 * cos( -(PI / 2.0) + theta1)) ) * w_theta2   +
                  (cos(theta1) * a3 * sin(-(PI / 2.0) + theta1) ) * w_theta3;

      Serial.print(v_x);
      Serial.print(", ");
      Serial.print(v_y);
      Serial.print(", ");
      Serial.print(v_z);
    */
    Serial.println();
  }
}
