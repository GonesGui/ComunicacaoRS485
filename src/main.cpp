#include <Arduino.h>
#include <HardwareSerial.h>

// Defina os pinos RX, TX, DE e RE
#define RS485_RX_PIN 16  // RX1
#define RS485_TX_PIN 17  // TX1
#define RS485_DE_PIN 4  // Pino DE (Driver Enable)
#define RS485_RE_PIN 15  // Pino RE (Receiver Enable)

HardwareSerial SerialPort(2); // UART1 para RS485

void setup() {
  Serial.begin(115200); // Monitor Serial (debug)
  Serial.println("Inicializando receptor RS485...");

  // Configura UART1
  SerialPort.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  SerialPort.setTimeout(50);

  // Configura os pinos DE e RE
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_RE_PIN, OUTPUT);

  // Configura como modo recepção inicial
  digitalWrite(RS485_DE_PIN, LOW);  // Desabilita transmissão
  digitalWrite(RS485_RE_PIN, LOW); // Habilita recepção

  Serial.print("Setup concluído. Pronto para receber dados.");
}

void loop() {
  // Verifica se há dados disponíveis no RS485
  if (SerialPort.available()) {
    String received = SerialPort.readStringUntil(';'); // Lê até o delimitador ;
    Serial.print("Mensagem recebida: ");
    Serial.println(received);
  }

  delay(100); // Evite sobrecarregar o loop
}
