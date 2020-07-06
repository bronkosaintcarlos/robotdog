/* Leg Kinematic Model Class
 * 
 * 
 */
class LegKinematics {
  private:
    float len1;
    float len2;
    float len0;

    float xTarg;
    float yTarg;
    float zTarg;

    float xPos;
    float yPos;

    float angle0_deg;
    float angle1E_UP_deg;
    float angle2E_UP_deg;
    float angle1E_DOWN_deg;
    float angle2E_DOWN_deg;


  public:
    LegKinematics() {}
    
    LegKinematics(float len1, float len2, float len0) {
      this->len1 = len1;
      this->len2 = len2;
      this->len0 = len0;
    }

    void updateAngleTargets() {

      float rZ = (sqrt(xTarg * xTarg + yTarg * yTarg + zTarg * zTarg));

      angle0_deg = (atan((zTarg - len0)/yTarg)) * 180 / PI;
      float phi_rad = (acos(xTarg/rZ));
      float rL = (sqrt(rZ * rZ - len0 * len0));
            
      angle2E_DOWN_deg = ( acos(((rL * rL) - ( len1 * len1 + len2 * len2)) / (2 * len1 * len2))                                     ) * 180 / PI;
      angle1E_DOWN_deg = ( phi_rad - atan((len2 * sin(angle2E_DOWN_deg * PI / 180.0) / (len1 + len2 * cos(angle2E_DOWN_deg * PI / 180.0))))   ) * 180 / PI;

      angle2E_UP_deg = ( -acos(((rL * rL) - ( len1 * len1 + len2 * len2)) / (2 * len1 * len2))                         ) * 180 / PI;
      angle1E_UP_deg = ( phi_rad - atan((len2 * sin(angle2E_UP_deg * PI / 180.0) / (len1 + len2 * cos(angle2E_UP_deg * PI / 180.0))))   ) * 180 / PI;

      

      /*if(xTarg < 0){
        // subtract 180 degrees from angle 1 because acos range
        angle1E_DOWN_deg += -180;

        angle1E_UP_deg += -180;
      }*/
    }

    bool setPosZ(float zReq) {
      zTarg = zReq;

      updateAngleTargets();
    }
    
    bool setPosXY(float xReq, float yReq) {
      float rCheck = sqrt(xReq * xReq + yReq * yReq);
      if (rCheck > len1 + len2 || rCheck < 3.0) { // 3.0 inches is minimum R attainable
        Serial.print("bad r");
        return false;
      }
      else {
        xTarg = xReq;
        yTarg = yReq;

        updateAngleTargets();
        return true;
      }
    }

    bool setPosXYZ(float xReq, float yReq, float zReq) {
      float rCheck = sqrt(xReq * xReq + yReq * yReq + zReq * zReq);
      if (rCheck > sqrt((len1 + len2) * (len1 + len2) + len0 * len0) || rCheck < 3.0) {
        Serial.print("bad r");
        return false;
      }
      else {
        xTarg = xReq;
        yTarg = yReq;
        zTarg = zReq;

        updateAngleTargets();
        return true;
      }
    }

    float angle1E_UP() {
      return angle1E_UP_deg;
    }
    float angle2E_UP() {
      return angle2E_UP_deg;
    }

    float angle1E_DOWN() {
      return angle1E_DOWN_deg;
    }
    float angle2E_DOWN() {
      return angle2E_DOWN_deg;
    }

    float angle0() {
      return angle0_deg;
    }

    float getXTarg() {
      return xTarg;
    }
    float getYTarg() {
      return yTarg;
    }
    float getXPos() {
      return xPos;
    }
    float getYPos() {
      return yPos;
    }
};
