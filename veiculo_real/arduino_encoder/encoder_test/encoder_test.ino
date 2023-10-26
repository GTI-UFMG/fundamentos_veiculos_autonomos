//Test code for LPD3806 Incrimental Rotary Encoder Written By Nnamdi .M 2/29/2020.
//Will increment or decrement based on shaft position.
//Lose position once power is shut or reset button is pressed.
// Red Wire - 5V, Black Wire - GND, Sheild Wire - GND, Green Wire - D2 - Channel A, White Wire - D3 - Channel B.

const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
long encoder = 0;

void setup() {
  Serial.begin(9600);
  pinMode(encoder_a, INPUT_PULLUP);
  pinMode(encoder_b, INPUT_PULLUP);

  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);
}

void loop() {
  Serial.println(encoder);
}

void encoderPinChangeA() {
  encoder += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  encoder += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}
