// ============================================================
// Odometria por encoder incremental em quadratura
// Arduino -> Raspberry Pi via Serial
//
// Canal A: gera interrupção
// Canal B: determina o sentido de rotação
//
// Saída serial:
// RPM com sinal
// ============================================================

#define PIN_A 2
#define PIN_B 3

// Número de pulsos do canal A por volta,
// considerando apenas borda de subida (RISING).
#define RESOLUTION 360

// Período de amostragem em milissegundos
#define SAMPLE_TIME 30
#define BAUDRATE  115200

// Pulsos ocorridos desde a última amostragem.
// Possui sinal: positivo em um sentido e negativo no outro.
volatile long pulseDelta = 0;

unsigned long previousTime = 0;

// ============================================================
// Rotina de interrupção
// ============================================================
void countPulse()
{
    // O estado do canal B determina a direção.
    // Se o sentido estiver invertido no seu carrinho,
    // basta trocar +1 por -1.
    if (digitalRead(PIN_B) == HIGH)
    {
        pulseDelta++;
    }
    else
    {
        pulseDelta--;
    }
}

// ============================================================
// configuracao inicial
// ============================================================
void setup()
{
	// Abre a serial
    Serial.begin(BAUDRATE);

	// Associa pinos
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);

	// encoder pin on interrupt 0 (pin 2)
    attachInterrupt(
        digitalPinToInterrupt(PIN_A),
        countPulse,
        RISING
    );

    previousTime = millis();
}

// ============================================================
// Loop principal
// ============================================================
void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= SAMPLE_TIME)
    {
        unsigned long dt = currentTime - previousTime;
        previousTime = currentTime;

        // ----------------------------------------------------
        // Copia os dados da ISR de maneira atômica
        // ----------------------------------------------------
        noInterrupts();

        long pulses = pulseDelta;
        pulseDelta = 0;

        interrupts();

        // ----------------------------------------------------
        // Calcula RPM
        //
        // pulses / RESOLUTION = número de voltas no intervalo
        //
        // 60000 / dt = número de intervalos equivalentes
        //               por minuto
        // ----------------------------------------------------
        float rpm =
            ((float)pulses * 60000.0f) /
            ((float)RESOLUTION * (float)dt);

        // ----------------------------------------------------
        // Envia para Raspberry
        // ----------------------------------------------------
        Serial.println(rpm, 2);
    }
}
