// IR Obstacle Collision Detection Module

#define PIN_LED  13           // onboard Uno LED
#define PIN_IR_SIGNAL 7       // IR module's signal pin
#define PIN_LSR_L A0          // Left CdS LSR sensor
#define PIN_LSR_R A5          // Right CdS LSR sensor

void setup() {
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_IR_SIGNAL, INPUT);
    digitalWrite(PIN_LSR_L, INPUT_PULLUP);  // set pullup on analog pin 0 
    digitalWrite(PIN_LSR_R, INPUT_PULLUP);  // set pullup on analog pin 0 
    Serial.begin(9600);
}

void loop() {
    int is_obstacle = HIGH;  // HIGH means no obstacle found
    int R_light_val = 0;       // Right LSR reading
    int L_light_val = 0;       // Left LSR reading
    int LSR_offset =  30;      // single pt offset
    is_obstacle = digitalRead(PIN_IR_SIGNAL);
    if ( is_obstacle == LOW) {
        Serial.println("OBSTACLE!!");
        digitalWrite(PIN_LED, HIGH);
    }
    else {
        Serial.println("clear");
        digitalWrite(PIN_LED, LOW);
    }
    L_light_val = analogRead(PIN_LSR_L) + LSR_offset;
    Serial.print("LSR_values: L ");
    Serial.print(L_light_val);
    R_light_val = analogRead(PIN_LSR_R);
    Serial.print(", R ");
    Serial.println(R_light_val);

    delay(200);
}
