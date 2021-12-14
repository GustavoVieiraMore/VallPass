/*
ESP32 30-PIN
MOSI (VSPID) --> 23
MISO (VSPIQ) --> 19
SCK (VSPICLK)--> 18
NSS -----------> 17
RES -----------> 16
DI0 -----------> 22
*/

#include <SPI.h>
#include <LoRa.h>

#define LED 15   //LED para indicar envio de pacotes
int cont = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Transmissor LoRa");

  LoRa.setPins(17,16,22);   //NSS, RES e DIO0
  if(!LoRa.begin(868E6)){
    Serial.println("LoRa falhou ao iniciar");
    while(1);
  }
  pinMode(LED, OUTPUT);
}

void loop() {
  Serial.print("Enviando pacote n° ");
  Serial.println(cont);

  LoRa.beginPacket();
  LoRa.print("Pacote n°");
  LoRa.print(cont);      //imprime o número do pacote enviado
  LoRa.endPacket();

  digitalWrite(LED,HIGH);
  delay(100);
  digitalWrite(LED,LOW);

  cont++;
  delay(500);
}
