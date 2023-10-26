//Test code for LPD3806 Incrimental Rotary Encoder Written By Nnamdi .M 2/29/2020.
//Will increment or decrement based on shaft position.
//Lose position once power is shut or reset button is pressed.
// Red Wire - 5V, Black Wire - GND, Sheild Wire - GND, Green Wire - D2 - Channel A, White Wire - D3 - Channel B.

#define BAUDRATE (57600)
#define SAMPLE_DELAY (100)
#define RESOLUTION (360)

const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
long encoder = 0;
unsigned int dt = 0.0;
unsigned int tempo_anterior = 0;
unsigned int pulseCount = 0;
float rpm = 0;


//----------------------------------------------------------
// configuracao inicial
//----------------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE);
  pinMode(encoder_a, INPUT_PULLUP); //INPUT
  pinMode(encoder_b, INPUT_PULLUP); //INPUT

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, pulsos, RISING);
}

//----------------------------------------------------------
// main loop
//----------------------------------------------------------
void loop() {
  
  // calculo velocidade em rpm
  dt = (unsigned int)millis() - tempo_anterior;
  rpm = (pulseCount*(60000.f/dt))/RESOLUTION;
  
  // analisa o sentido de rotacao  a partir do sinal da variavel EncoderPos e se for negativo atribui velocidade negativa e deslocamento negativo.
  if (encoder < 0) 
  {
    rpm = -1*rpm;
  }
  
  // comeca nova contagem
  pulseCount = 0;
   
  // atualiza tempo anterior
  tempo_anterior = (unsigned int)millis();
  
  // envia informacao
  Serial.println(rpm);
    
  // espera proximo ciclo
  delay(SAMPLE_DELAY);
}

//----------------------------------------------------------
// Conta os pulsos do Encoder
//----------------------------------------------------------
void pulsos()
{
    pulseCount = pulseCount + 1;
    
    // Checa o sentido de rotacao do encoder
    if(abs(rpm) < 300)
    {
      // Sentido positivo (para frente)
      if (digitalRead(encoder_b) == HIGH)
      {
        encoder = encoder + 1;
      }
      // Sentido negativo (para tras)
      else
      {
        encoder = encoder - 1;
      }
    }
 }
