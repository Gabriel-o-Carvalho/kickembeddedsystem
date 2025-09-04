#include "esp32-hal.h"
#include <Arduino.h>
#include <cstdint>

// User Interface
#define READ_BIT(REG, BIT) (REG & (1 << BIT)) >> BIT

const float valorLimite1 = 5.0;          // 5V
const float valorLimite2 = 10.0;          // 10V
const float valorLimite3 = 15.0;          // 15V
const float valorLimite4 = 22.0;  // 22V
const float valorLimite5 = 26.0;  // 26V

const uint8_t led_count = 5;
const uint8_t led_pins[led_count] = {17, 2, 4, 16, 15};

const int pinoPot = 32;                     // Potenciômetro no GPIO 32
const int pinoSaida = 22;                   // PINO DE CHUTE
const int pinoPWM = 23;
const int pinoBotao = 5;                    // PINO BOTAO
bool chuteHabilitado = false;
bool chutar = false;

volatile bool ativarSaida = false; // Flag ativada na interrupção
unsigned long tempoAtivacao = 0;

void set_leds(const uint8_t *pins, uint8_t l_count, uint8_t flags);
uint8_t measure2flags(float measure);
// End User Interface

void IRAM_ATTR botaoPressionado() {
  if(chuteHabilitado && !(ativarSaida)){
    ativarSaida = true;
    tempoAtivacao = millis();  // Armazena o tempo do acionamento 
  }
}

float mapSensorToVoltage(uint16_t adc_value) {
    // Converte ADC -> tensão real medida
    float voltage = (adc_value / 4095.0) * 3.3;

    // Mapeia 1.0–2.4V -> 0–30V
    float result = (voltage - 1) * 30 / (2.3 - 1);

    // Limita a faixa
    if (result < 0) result = 0;
    if (result > 30) result = 30;

    return result;
}


void setup() {
  Serial.begin(115200);

  pinMode(pinoPWM, OUTPUT);

  for(int i = 0; i < led_count; i++)
    pinMode(led_pins[i], OUTPUT);

  pinMode(pinoBotao, INPUT_PULLDOWN); // Botão entre GND e pino (usa pull-up interno)
  //pinMode(pinoPot, INPUT);
  pinMode(pinoSaida, OUTPUT);
  digitalWrite(pinoSaida, LOW);
  analogWriteFrequency(5000);

  attachInterrupt(digitalPinToInterrupt(pinoBotao), botaoPressionado, RISING); // Interrupção na borda de subida

}



void loop() {
  float leitura = mapSensorToVoltage(analogRead(32)); // Lê valor do potenciômetro (0-4095)
  Serial.println(leitura); // Imprime valor para debug

  uint8_t flags = measure2flags(leitura);
  set_leds(led_pins, led_count, flags);

  if(leitura > valorLimite4 && leitura < valorLimite5){
    chuteHabilitado = true;
  } else {
    chuteHabilitado = false;
  }
  
  analogWrite(pinoPWM, 150);


  if(ativarSaida) {
    digitalWrite(pinoSaida, HIGH); // Liga o pino
    // Espera os 100ms sem travar
    if (leitura < 10.0) {
      digitalWrite(pinoSaida, LOW); // Desliga o pino
      ativarSaida = false;          // Reinicia flag
    }
  }
  
  delay(10); // Pequeno atraso para estabilidade
}



void set_leds(const uint8_t *pins, uint8_t l_count, uint8_t flags){
  for(uint8_t i; i < l_count; i++)
    digitalWrite(*(pins+i), READ_BIT(flags, i));
}

uint8_t measure2flags(float measure){
  uint8_t flags = 0;

  if(measure > valorLimite1) flags |= (1 << 0);
  if(measure > valorLimite2) flags |= (1 << 1);
  if(measure > valorLimite3) flags |= (1 << 2);
  if(measure > valorLimite4) flags |= (1 << 3);
  if(measure > valorLimite5) flags |= (1 << 4);

  return flags;
}

