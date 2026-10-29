// ============================================================
// FVA - Arduino Nano
//
// Funcoes:
//   1. Odometria por encoder incremental
//   2. Leitura do receptor RC
//   3. Leitura das chaves RC/AUTO
//
// ------------------------------------------------------------
// PINAGEM
//
// Encoder:
//   D2  -> Canal A
//   D3  -> Canal B
//
// Chaves:
//   D8  -> Selecao DIRECAO
//   D9  -> Selecao TRACAO
//
// Receptor RC:
//   D11 -> CH1 DIRECAO
//   D12 -> CH2 ACELERADOR
//
// ------------------------------------------------------------
// CHAVES
//
// LOW  (0) -> RC
// HIGH (1) -> AUTO
//
// ------------------------------------------------------------
// SAIDA SERIAL
//
// RPM,RC_DIRECAO,RC_ACELERADOR,SEL_DIRECAO,SEL_TRACAO
//
// Exemplo:
// 0.00,1498,1503,0,1
//
// ============================================================


// ============================================================
// PINOS
// ============================================================

#define PIN_ENCODER_A     2
#define PIN_ENCODER_B     3

#define PIN_SEL_DIRECAO   8
#define PIN_SEL_TRACAO    9

#define PIN_RC_DIRECAO   11
#define PIN_RC_ACEL      12


// ============================================================
// CONFIGURACAO
// ============================================================

// Pulsos do canal A por volta
// considerando somente RISING
#define RESOLUTION 360

// Periodo de envio para Raspberry [ms]
#define SAMPLE_TIME 30

#define BAUDRATE 115200

// Faixa aceitavel para pulsos RC [us]
// Serve para rejeitar leituras espurias
#define RC_MIN_VALID 800
#define RC_MAX_VALID 2200


// ============================================================
// ODOMETRIA
// ============================================================

volatile long pulseDelta = 0;

unsigned long previousTime = 0;


// ============================================================
// RADIO RC
// ============================================================

// instante da borda de subida
volatile unsigned long rcDirecaoRise = 0;
volatile unsigned long rcAcelRise = 0;

// largura do ultimo pulso valido [us]
volatile uint16_t rcDirecaoPulse = 0;
volatile uint16_t rcAcelPulse = 0;

// estado anterior dos pinos
volatile uint8_t rcDirecaoLastState = 0;
volatile uint8_t rcAcelLastState = 0;


// ============================================================
// INTERRUPCAO DO ENCODER
// ============================================================

void countPulse()
{
    // Canal B determina o sentido
    if (digitalRead(PIN_ENCODER_B) == HIGH)
    {
        pulseDelta++;
    }
    else
    {
        pulseDelta--;
    }
}


// ============================================================
// PIN CHANGE INTERRUPT
//
// D8..D13 pertencem ao grupo PCINT0.
//
// D11 = PCINT3
// D12 = PCINT4
//
// A interrupcao ocorre em qualquer mudanca de estado.
// ============================================================

ISR(PCINT0_vect)
{
    unsigned long now = micros();

    // Le diretamente PORTB para reduzir o tempo dentro da ISR
    uint8_t port = PINB;

    // --------------------------------------------------------
    // D11 = PB3 = CH1 DIRECAO
    // --------------------------------------------------------

    uint8_t estadoDirecao = (port & _BV(PB3)) ? HIGH : LOW;

    if (estadoDirecao != rcDirecaoLastState)
    {
        rcDirecaoLastState = estadoDirecao;

        if (estadoDirecao == HIGH)
        {
            // borda de subida
            rcDirecaoRise = now;
        }
        else
        {
            // borda de descida
            unsigned long largura = now - rcDirecaoRise;

            if (
                largura >= RC_MIN_VALID &&
                largura <= RC_MAX_VALID
            )
            {
                rcDirecaoPulse = (uint16_t)largura;
            }
        }
    }


    // --------------------------------------------------------
    // D12 = PB4 = CH2 ACELERADOR
    // --------------------------------------------------------

    uint8_t estadoAcel = (port & _BV(PB4)) ? HIGH : LOW;

    if (estadoAcel != rcAcelLastState)
    {
        rcAcelLastState = estadoAcel;

        if (estadoAcel == HIGH)
        {
            // borda de subida
            rcAcelRise = now;
        }
        else
        {
            // borda de descida
            unsigned long largura = now - rcAcelRise;

            if (
                largura >= RC_MIN_VALID &&
                largura <= RC_MAX_VALID
            )
            {
                rcAcelPulse = (uint16_t)largura;
            }
        }
    }
}


// ============================================================
// CONFIGURACAO INICIAL
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(BAUDRATE);


    // --------------------------------------------------------
    // Encoder
    // --------------------------------------------------------

    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(PIN_ENCODER_A),
        countPulse,
        RISING
    );


    // --------------------------------------------------------
    // Chaves RC/AUTO
    //
    // Terminal central -> Arduino
    // Lateral 1        -> GND
    // Lateral 2        -> 5 V
    // --------------------------------------------------------

    pinMode(PIN_SEL_DIRECAO, INPUT);
    pinMode(PIN_SEL_TRACAO, INPUT);


    // --------------------------------------------------------
    // Receptor RC
    // --------------------------------------------------------

    pinMode(PIN_RC_DIRECAO, INPUT);
    pinMode(PIN_RC_ACEL, INPUT);


    // Estado inicial dos canais RC
    rcDirecaoLastState = digitalRead(PIN_RC_DIRECAO);
    rcAcelLastState = digitalRead(PIN_RC_ACEL);


    // --------------------------------------------------------
    // Habilita Pin Change Interrupt para D11 e D12
    //
    // D11 = PCINT3
    // D12 = PCINT4
    // --------------------------------------------------------

    PCICR |= _BV(PCIE0);

    PCMSK0 |= _BV(PCINT3);
    PCMSK0 |= _BV(PCINT4);


    previousTime = millis();
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= SAMPLE_TIME)
    {
        unsigned long dt =
            currentTime - previousTime;

        previousTime = currentTime;


        // ====================================================
        // Copia dados compartilhados pelas interrupcoes
        // ====================================================

        noInterrupts();

        long pulses = pulseDelta;
        pulseDelta = 0;

        uint16_t rcDirecao = rcDirecaoPulse;
        uint16_t rcAcel = rcAcelPulse;

        interrupts();


        // ====================================================
        // ODOMETRIA
        // ====================================================

        float rpm =
            ((float)pulses * 60000.0f) /
            ((float)RESOLUTION * (float)dt);


        // ====================================================
        // CHAVES
        // ====================================================

        uint8_t selDirecao =
            digitalRead(PIN_SEL_DIRECAO);

        uint8_t selTracao =
            digitalRead(PIN_SEL_TRACAO);


        // ====================================================
        // ENVIA PARA RASPBERRY
        //
        // RPM,
        // RC_DIRECAO,
        // RC_ACELERADOR,
        // SEL_DIRECAO,
        // SEL_TRACAO
        // ====================================================

        Serial.print(rpm, 2);

        Serial.print(",");
        Serial.print(rcDirecao);

        Serial.print(",");
        Serial.print(rcAcel);

        Serial.print(",");
        Serial.print(selDirecao);

        Serial.print(",");
        Serial.println(selTracao);
    }
}
