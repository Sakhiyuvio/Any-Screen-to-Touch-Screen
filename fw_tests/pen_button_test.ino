#define PEN_BUTTON 21

void setup() {
    Serial.begin(115200);
    delay(1000);

    // init
    pinMode(PEN_BUTTON, INPUT_PULLUP);
    // digitalWrite(PEN_BUTTON, HIGH); 
}

void loop() {
    if (digitalRead(PEN_BUTTON) == LOW){
        Serial.println("SUCCESS: button is pressed");
    }
    else {
    Serial.println("WAIT: button is not pressed");
    }
}