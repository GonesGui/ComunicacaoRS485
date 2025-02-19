#include <Arduino.h>
#include <HardwareSerial.h>
#include <HX711.h>
HX711 scale;

// Defina os pinos RX, TX, DE e RE
#define RS485_RX_PIN 16  // RX1
#define RS485_TX_PIN 17  // TX1
#define RS485_DE_PIN 4  // Pino DE (Driver Enable)
#define RS485_RE_PIN 15  // Pino RE (Receiver Enable)

#define PINO_ENTRADA 13

double medidabruta = -1;
double medidaFiltrada = -1;

bool zerar = false;

unsigned long tempoInicio = 0; // Armazena o tempo no início do loop
unsigned long tempoFim = 0;    // Armazena o tempo no fim do loop
unsigned long duracaoLoop = 0; // Duração em ms

HardwareSerial SerialPort(2); // UART1 para RS485


void setup() {
  Serial.begin(115200); // Monitor Serial (debug)
  Serial.println("Inicializando transmissor RS485...");

  // Configura UART1
  SerialPort.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  scale.begin(23, 22); // CONFIGURANDO OS PINOS DA BALANÇA
  scale.set_scale(); // LIMPANDO O VALOR DE CALIBRACAO
  scale.tare(); // ZERANDO A BALANÇA PARA DESCONSIDERAR A MASSA DA ESTRUTURA

  // Configura os pinos DE e RE
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_RE_PIN, OUTPUT);

  // Configura como modo transmissão inicial
  digitalWrite(RS485_DE_PIN, HIGH); // Habilita transmissão
  digitalWrite(RS485_RE_PIN, HIGH); // Desabilita recepção

  pinMode(PINO_ENTRADA, INPUT);

  Serial.println("Setup concluído. Pronto para transmitir.");
}

void loop() {
    tempoInicio = millis(); // Marca o início do loop

  scale.power_up(); // LIGANDO O SENSOR
  medidabruta = scale.read_average(40); // SALVANDO NA VARIAVEL O VALOR BRUTO DA MÉDIA DE 40 MEDIDAS
  scale.power_down(); // DESLIGANDO O SENSOR  

  
  // Envia mensagem via RS485
  Serial.println("Medida enviada");
  Serial.println(medidabruta);
  SerialPort.print(medidabruta);

  tempoFim = millis(); // Marca o fim do loop
  duracaoLoop = tempoFim - tempoInicio;
  Serial.print("Duracao loop:");
  Serial.println(duracaoLoop);

  delay(10); // Aguarde antes de repetir
}
