/*********************************************

  mtr_funcs.ino
  Motor functions for 
  "Build More Robot" Series - session 2

*********************************************/
// motor functions
void set_motors(int16_t Lmtr_spd, int16_t Rmtr_spd)
{
   set_Lmtr_speed(Lmtr_spd);
   set_Rmtr_speed(Rmtr_spd); 
}

void set_Lmtr_speed(int16_t speed)
{
  uint8_t direct = 0;    // forward
  if (speed < 0){
    direct = 1;          // reverse
  }
  switch (direct) {
    case 0:
      digitalWrite(PIN_IN1_L_MTR, HIGH);
      digitalWrite(PIN_IN2_L_MTR, LOW);   
      analogWrite(PIN_ENA_L_PWM, speed);
      break;
    case 1:
      digitalWrite(PIN_IN1_L_MTR, LOW);
      digitalWrite(PIN_IN2_L_MTR, HIGH);   
      analogWrite(PIN_ENA_L_PWM, -speed);
      break;
    default :
      digitalWrite(PIN_IN1_L_MTR, LOW);
      digitalWrite(PIN_IN2_L_MTR, LOW);
  }
}

void set_Rmtr_speed(int16_t speed)
{
  uint8_t direct = 0;    // forward
  if (speed < 0){
    direct = 1;          // reverse
  }
  switch (direct) {
    case 0:
      digitalWrite(PIN_IN3_R_MTR, HIGH);
      digitalWrite(PIN_IN4_R_MTR, LOW);   
      analogWrite(PIN_ENB_R_PWM, speed);
      break;
    case 1:
      digitalWrite(PIN_IN3_R_MTR, LOW);
      digitalWrite(PIN_IN4_R_MTR, HIGH);   
      analogWrite(PIN_ENB_R_PWM, -speed);
      break;
    default :
      digitalWrite(PIN_IN3_R_MTR,LOW);
      digitalWrite(PIN_IN4_R_MTR,LOW);
  }
}
