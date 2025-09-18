//Test code for LPD3806 Incrimental Rotary Encoder Written By Nnamdi .M 2/29/2020.
//Will increment or decrement based on shaft position.
//Lose position once power is shut or reset button is pressed.
// Red Wire - 5V, Black Wire - GND, Sheild Wire - GND, Green Wire - D2 - Channel A, White Wire - D3 - Channel B.

#define BAUDRATE 		(115200)
#define SAMPLE_DELAY 	(30)
#define RESOLUTION 		(360)

const uint8_t PIN_A = 2;   // Green - pin 2 - Digital
const uint8_t PIN_B = 3;   // White - pin 3 - Digital

unsigned int tempo_anterior = (unsigned int)millis();
long encoder = 0;
unsigned int pulseCount = 0;

//----------------------------------------------------------
// Conta os pulsos do Encoder
//----------------------------------------------------------
void conta_pulsos()
{
	// conta um pulso
	pulseCount += 1;
	
	// Sentido positivo (para frente +1), sentido negativo (para tras -1)
	// Incrementa se PIN_B está HIGH, senão decrementa
	encoder += (digitalRead(PIN_B) == HIGH) ? 1 : -1;
 }

//----------------------------------------------------------
// configuracao inicial
//----------------------------------------------------------
void setup() {
	// Abre a serial
	Serial.begin(BAUDRATE);
	
	// Associa pinos
	pinMode(PIN_A, INPUT_PULLUP); //INPUT
	pinMode(PIN_B, INPUT_PULLUP); //INPUT

	// encoder pin on interrupt 0 (pin 2)
	attachInterrupt(digitalPinToInterrupt(PIN_A), conta_pulsos, RISING);
}

//----------------------------------------------------------
// main loop
//----------------------------------------------------------
void loop() {	
	// calculo velocidade em rpm
	unsigned int dt = (unsigned int)millis() - tempo_anterior;
	float rpm = ( pulseCount * (60000.f/dt) ) / RESOLUTION;

	// Se encoder for negativo, inverte rpm
	if (encoder < 0) rpm = -rpm;

	// comeca nova contagem
	pulseCount = 0;

	// atualiza tempo anterior
	tempo_anterior = (unsigned int)millis();

	// envia informacao com duas casas decimais
	Serial.println(rpm, 2);

	// espera proximo ciclo
	delay(SAMPLE_DELAY);
}
