// WALK CYCLE
// stance variables
float RStance = 8.0;
float LStance = 7.0;
float ThetaStance = acos(LStance / (2.0 * RStance));
float TStance = 1.0; // time for stance period
float VDesired = LStance / TStance;
float distIMF = 0.2;

// non essential variables
float YStance = -RStance*sin(ThetaStance);

// swing variables
float TSwing = TStance; // make stance and swing phases same amount of time
float HStep = 2.0; // make max height step available
// HStep = - 3.0 - (YStance); // == distance between YStance and -3" which is the minimum leg extension

class LegController{
  private:

  // Timer variables:
  float t = 0;
  
  unsigned long endTime = TStance*1000;
  unsigned long startTime;

  float phaseShift = 0;

  // Current Walk Cycle:
  int currentWalkCycle = 1;

  LegKinematics *kModel;


  unsigned long currTime(){
    return millis() + phaseShift;
  }

  public:

  // kinematic model targets:
  float x = 0;
  float y = 0;
  
  LegController(LegKinematics &kModel){
    this->kModel = &kModel;
  }


  void setPhaseShift(float seconds){
    phaseShift = seconds * 1000.0;
  }
  bool timeUp(){
    return (currTime() >= endTime);
  }

  void updateCycleTimes(){
    // set new delta time;
    if (currentWalkCycle == 1) {
      // set delta time to TStance seconds after current time
      endTime = currTime() + (TStance * 1000);
    }
    else if (currentWalkCycle == -1) {
      // set delta time to TSwing seconds after current time
      endTime = currTime() + (TSwing * 1000);
    }
  }

  void nextCycle(){
    // set new startTime
    startTime = currTime();
    
    // iterate to next walk cycle
    currentWalkCycle *= -1;

    // set new time limits:
    updateCycleTimes();
  }

  void update(){
    t = (currTime() - startTime)/1000.0; // time in seconds

    // if ready for next cycle
    if (timeUp()) {
      // start the next cycle
      nextCycle();
    }
    // update x and y
    // if in stance cycle:
    if (currentWalkCycle == 1) {
      x = (VDesired * (TStance-t) - LStance / 2.0);
      y = - distIMF * cos(PI * ( (VDesired * t) / LStance ) - PI / 2.0) - RStance * sin(ThetaStance);
    }
    else if (currentWalkCycle == -1) {
      x = (LStance / 2.0) * sin(PI * ( t / TSwing ) - PI / 2.0);
      y = HStep * cos(PI * (t/TSwing) - PI/2.0) - RStance * sin(ThetaStance);
    }

    
    (*kModel).setPosXY(x, y);
  }
};

