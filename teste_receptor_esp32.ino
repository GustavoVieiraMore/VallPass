/*
ESP32 38-PIN
MOSI (VSPID) --> 23
MISO (VSPIQ) --> 19
SCK (VSPICLK)--> 18
NSS -----------> 17
RES -----------> 16
DI0 -----------> 22
*/

#include <SPI.h>
#include <LoRa.h>

#define LED 15   //LED para indicar recebimento de pacote

void setup() {
  Serial.begin(115200);
  Serial.println("Receptor LoRa:");

  LoRa.setPins(17,16,22);  //NSS, RES e DIO0
  if(!LoRa.begin(868E6)){
    Serial.println("LoRa falhou ao iniciar");
    while(1);
  }
  pinMode(LED,OUTPUT);
}

void loop() {
  int packetTam = LoRa.parsePacket();  //checa se o pacote foi recebido e retorna o tamanho em bytes

  if(packetTam){
    Serial.print("Pacote recebido: '");
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);

    while(LoRa.available()){
      Serial.print((char)LoRa.read());  //printa cada caractere lido do pacote
    }
    Serial.print("' com RSSI médio de ");
    Serial.print(LoRa.packetRssi());   //retorna a média do RSSI em dBm do último pacote recebido
    Serial.println(" dBm");
  }
}
