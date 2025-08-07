#include <Arduino.h>
#include <cstdint>

// User Interface
#define READ_BIT(REG, BIT) (REG & (1 << BIT)) >> BIT

const int valorLimite1 = 4095 / 6;          // 5V
const int valorLimite2 = 4095 / 3;          // 10V
const int valorLimite3 = 4095 / 2;          // 15V
const int valorLimite4 = (4095 / 30) * 22;  // 22V
const int valorLimite5 = (4095 / 30) * 26;  // 26V

const uint8_t led_count = 5;
const uint8_t led_pins[led_count] = {17, 2, 4, 16, 15};

const int pinoPot = 32;                     // Potenciômetro no GPIO 32
const int pinoSaida = 33;                   // PINO DE CHUTE
const int pinoBotao = 5;                    // PINO BOTAO
bool chuteHabilitado = false;
bool chutar = false;

volatile bool ativarSaida = false; // Flag ativada na interrupção
unsigned long tempoAtivacao = 0;

void set_leds(const uint8_t *pins, uint8_t l_count, uint8_t flags);
uint8_t measure2flags(uint16_t measure);
// End User Interface

void setup() {
  Serial.begin(115200);
  
  for(int i = 0; i < led_count; i++)
    pinMode(led_pins[i], OUTPUT);

  pinMode(pinoBotao, INPUT_PULLDOWN); // Botão entre GND e pino (usa pull-up interno)
  //pinMode(pinoPot, INPUT);
  pinMode(pinoSaida, OUTPUT);
  digitalWrite(pinoSaida, LOW);

  attachInterrupt(digitalPinToInterrupt(pinoBotao), botaoPressionado, RISING); // Interrupção na borda de subida

}

void loop() {
  uint16_t leitura = analogRead(32); // Lê valor do potenciômetro (0-4095)
  Serial.println((float)leitura/4095 * 30); // Imprime valor para debug

  uint8_t flags = measure2flags(leitura);
  set_leds(led_pins, led_count, flags);

  if(leitura > valorLimite4 && leitura < valorLimite5){
    chuteHabilitado = true;
  } else {
    chuteHabilitado = false;
  }

if (ativarSaida) {
    digitalWrite(pinoSaida, HIGH); // Liga o pino
    // Espera os 100ms sem travar
    if (millis() - tempoAtivacao >= 10000) {
      digitalWrite(pinoSaida, LOW); // Desliga o pino
      ativarSaida = false;          // Reinicia flag
    }
  }
  
  delay(10); // Pequeno atraso para estabilidade
}


void IRAM_ATTR botaoPressionado() {
  if(chuteHabilitado && !(ativarSaida)){
    ativarSaida = true;
    tempoAtivacao = millis();  // Armazena o tempo do acionamento 
  }
}

void set_leds(const uint8_t *pins, uint8_t l_count, uint8_t flags){
  for(uint8_t i; i < l_count; i++)
    digitalWrite(*(pins+i), READ_BIT(flags, i));
}

uint8_t measure2flags(uint16_t measure){
  uint8_t flags = 0;

  if(measure > valorLimite1) flags |= (1 << 0);
  if(measure > valorLimite2) flags |= (1 << 1);
  if(measure > valorLimite3) flags |= (1 << 2);
  if(measure > valorLimite4) flags |= (1 << 3);
  if(measure > valorLimite5) flags |= (1 << 4);

  return flags;
}
https://www.youtube.com/

