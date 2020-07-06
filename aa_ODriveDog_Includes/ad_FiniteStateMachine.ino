/* Finite State Machine with states
1. delay
2. active
*/
class TimerFiniteStateMachine {
  private:
    unsigned long interval;
    unsigned long lastMillis;
    unsigned long currMillis;
    unsigned long deltaTime;

  public:
    TimerFiniteStateMachine(unsigned long interval) {
      this->interval = interval;
    }

    bool isReady() {
      deltaTime = currMillis - lastMillis;

      if (deltaTime >= interval) {
        lastMillis = currMillis;
        return true;
      }
      else return false;
    }

    void updateMillis() {
      currMillis = millis();
      deltaTime = currMillis - lastMillis;
    }
    unsigned long getDeltaTime() {
      return deltaTime;
    }
};
