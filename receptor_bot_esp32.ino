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

#define NSS 17
#define RES 16
#define DIO0 22
#define LED_sinal 32    //LED de sinalização
#define LED_tx 15      //LED para indicar envio de pacotes
#define BOT 33
#define botDelay 500  //delay para debounce

void iniciLED(){
  pinMode(LED_tx,OUTPUT);
  digitalWrite(LED_tx,LOW);
  pinMode(LED_sinal,OUTPUT);
  digitalWrite(LED_sinal,LOW);
}

/*
void onReceive(int packetTam){
  msg = LoRa.read();
}
*/

int cont = 1;
byte botaoPress = 0;
long ultTempo = 0;    //tempo da última interrupção

void ISR(){
  if (!digitalRead(BOT)){    //negado por causa do pull-up
    botaoPress = 1;
  }
  else{
    botaoPress = 0;
  }
}

void iniciBot(){
  pinMode(BOT,INPUT);
  attachInterrupt(BOT,ISR,FALLING);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Conectando via LoRa (38-Pin)");
  delay(500);
  iniciBot();
  iniciLED();
  
  LoRa.setPins(NSS, RES, DIO0);
  if(!LoRa.begin(868E6)) {
    Serial.println("LoRa falhou ao iniciar");
    while (1);
  }
   LoRa.setTxPower(14);
  
  //LoRa.onReceive(onReceive);
  //LoRa.receive();
}

void loop() {
  int packetTam = LoRa.parsePacket();   //checa se o pacote foi recebido e retorna o tamanho em bytes
  if(packetTam){
    while(LoRa.available()){
     Serial.print((char)LoRa.read());  //printa cada caractere lido do pacote
    }
    Serial.print("' com RSSI médio de ");
    Serial.print(LoRa.packetRssi());   //retorna a média do RSSI em dBm do último pacote recebido
    Serial.println(" dBm");

    digitalWrite(LED_tx,HIGH);
    delay(100);
    digitalWrite(LED_tx,LOW);
    
    digitalWrite(LED_sinal,HIGH);
    delay(2000);
    digitalWrite(LED_sinal,LOW);
  }
    if (botaoPress){
    if(millis() - ultTempo > botDelay){      //debounce
      Serial.print("Enviando pacote n° ");
      Serial.println(cont);
      
      LoRa.beginPacket();
      LoRa.print(botaoPress);
      LoRa.print("Pacote n°");
      LoRa.print(cont);      //imprime o número do pacote enviado
      LoRa.endPacket();

      ultTempo = millis();
      digitalWrite(LED_tx,HIGH);
      delay(100);
      digitalWrite(LED_tx,LOW);
      cont++;
    }
    botaoPress = 0;
  }
}
