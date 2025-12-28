/* 
 *  Código Receptor - Prueba STM32(TX)/Arduino(RX)
 *  David Evangelista
 *  29/08/25
 */

#include <SoftwareSerial.h>
#define PIN_RX 3
#define PIN_TX 2

SoftwareSerial receptor(PIN_TX, PIN_RX);

void setup() {
  Serial.begin(9600);
  receptor.begin(9600);

  delay(500);
  Serial.println("Conectado");
}

void loop() {

  if (receptor.available()){
    Serial.write(receptor.read());  
  }

  delay(500);

}
