
// Custom "ODrive_Motor" class for controlling an individual motor on the odrive
class ODrive_Motor {
  private:
    ODriveArduino *odrive;
    int axis;
    float kp;
    float ki;
    float kd;

    float angleOffset = 0.0; // default to no offset
    float angleTarget = 0; // default to 0

    float outputAngleConv;

    float rangeOfMotion; // in +/- degrees

    int reversed = 1;

  public:
    ODrive_Motor() {}
    ODrive_Motor(ODriveArduino &odrive, int axis): odrive(&odrive) {
      this->odrive = &odrive;
      this->axis = axis;
    }

    ODrive_Motor(ODriveArduino &odrive, int axis, int reversed): odrive(&odrive) {
      this->odrive = &odrive;
      this->axis = axis;
      this->reversed = reversed;
    }

    void setAngleOffset(float angle){
      this->angleOffset = angle;
    }
    
    void setAngleConv(float factor) {
      outputAngleConv = factor;
    }
    void setRangeOfMotion(float range) {
      rangeOfMotion = range;
    }

    void setVelocityLimit(float vel){
      (*odrive).serial_ << "w axis" << axis << ".controller.config.vel_limit " << vel << "\n";
    }

    void setPositionDeg(float deg) {
      if (fabs(deg) > rangeOfMotion) {
        Serial << "Motor " << axis << " Out Of range!!" << '\n';

        return; // do not set new target
      }
      else {
        angleTarget = deg * outputAngleConv * reversed + angleOffset * outputAngleConv * reversed;
        (*odrive).SetPosition(axis, angleTarget);
      }
    }

    float target(){
      return angleTarget;
    }

    ODriveArduino getODrive() {
      return *odrive;
    }

    void fullCalibrate() {
      int requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
      Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
      (*odrive).run_state(axis, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
      (*odrive).run_state(axis, requested_state, false); // don't wait
    }

    float posEstimate(){
      // read encoder
      (*odrive).serial_ << "r axis" << axis << ".encoder.pos_estimate\n";
      float counts = (*odrive).readInt();
      return counts / outputAngleConv * reversed;
    }

    float velEstimate(){
      // read vel
      (*odrive).serial_ << "r axis" << axis << ".encoder.vel_estimate\n";
      float cps = (*odrive).readInt();
      return cps / outputAngleConv * reversed;
    }

    void setPosition(float pos) {
      (*odrive).SetPosition(axis, pos * reversed);
    }
};


