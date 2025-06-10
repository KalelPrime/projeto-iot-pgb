#include <TimerOne.h>               // Biblioteca para controle de temporizador e PWM via Timer1
#include <Adafruit_INA219.h>        // Biblioteca do sensor INA219 (medição de tensão e corrente)
#include <Wire.h>                   // Biblioteca I2C (necessária para comunicação com o INA219)
#include "ChargerClasses.hpp"       // Arquivo de classes personalizado (presumivelmente inclui o SimplePID)

Adafruit_INA219 ina219;             // Cria uma instância do sensor de corrente INA219

// Controle de carregamento
#define NPN_PIN 9                   // Define o pino 9 como saída para controle PWM (transistor NPN)
SimplePID voltagePID;              // Instância do controlador PID
bool charging = true;              // Variável booleana para indicar se está carregando
const int led = 8;  // Pino onde o LED está conectado

void setup() {

  // Configuração básica
  pinMode(led, OUTPUT);  // Define o pino como saída
  digitalWrite(led, HIGH);  // Acende o LED
  
  Serial.begin(115200);            // Inicializa a comunicação serial com velocidade de 115200 bps
  pinMode(A0,INPUT);               // Define o pino A0 como entrada (usado para medir a tensão de entrada)

  // Inicialização do sensor de corrente
  ina219.begin();                  // Inicializa a comunicação com o INA219

  // Configuração do Timer1 para PWM
  Timer1.initialize(100);         // Inicializa o Timer1 com período de 100 microssegundos => frequência de 10 kHz
  Timer1.pwm(NPN_PIN, 0);         // Inicia o PWM no pino NPN_PIN (pino 9) com ciclo de trabalho 0%

  // Configuração do PID de controle de tensão
  voltagePID.setParams(0, 0.2e-3, 0, 0, 1); // Define os parâmetros do PID (Kp=0, Ki=0.0002, Kd=0, bias=0, Ts=1s)
  

}

void loop() {

  // Leitura de corrente da bateria
  float iBatt = ina219.getCurrent_mA();  // Corrente medida em mA

  // Medição de tensão
  float vBatt = ina219.getBusVoltage_V() - ina219.getShuntVoltage_mV()/1000; 
  // Tensão no barramento menos a queda no resistor de derivação (shunt)

  float vA0 = analogRead(A0);  // Leitura do pino A0 (analógico)
  float vSrc = vA0 * 5.0 / 1023.0 * (5 + 1) / 1;
  // Conversão da leitura A0 em tensão da fonte de entrada (assume divisor resistivo 5:1)

  // Controle de carregamento
  // Controle PID de corrente
  float targetCurrent = 500;                   // Corrente alvo em mA
  float u = voltagePID.evalu(iBatt, targetCurrent);  // Avalia o erro do PID com base na corrente medida

  float duty = u;                              // Ciclo de trabalho PWM igual ao sinal de controle
  duty = 0.5;                                   // **IMPORTANTE**: sobrescreve o duty com valor fixo de 50%

  // Define o ciclo de trabalho do PWM
  Timer1.pwm(NPN_PIN, duty * 1023);           // Converte duty (0~1) para escala 0~1023 do Timer1

  // Imprime os resultados na porta serial
  Serial.print(targetCurrent); Serial.print(" ");
  Serial.print(iBatt); Serial.print(" ");
  Serial.print(vSrc); Serial.print(" ");
  Serial.print(vBatt); Serial.print(" ");
  Serial.print(duty * 100); Serial.println();  // Duty em porcentagem
}
