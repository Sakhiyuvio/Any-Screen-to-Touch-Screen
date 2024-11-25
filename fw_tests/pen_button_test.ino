#define PEN_BUTTON 21

void setup() {
    Serial.begin(115200);
    delay(1000);

    // init
    pinMode(PEN_BUTTON, INPUT);
    digitalWrite(PEN_BUTTON, HIGH); 
}

void loop() {
    if (digitalRead(PEN_BUTTON) == LOW){
        Serial.println("SUCCESS: button is pressed");
    }

    Serial.println("WAIT: button is not pressed");
}