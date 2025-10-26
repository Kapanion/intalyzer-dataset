#include "SPI.h"

char str[50];
volatile byte i;
volatile bool pin;

void setup()
{
  Serial.begin(115200);    // velocidad en baudios en 115200 para UART
  Serial.println("ESCLAVO");
  pinMode(MISO, OUTPUT);   // configurar MISO como salida esclavo 
  SPCR |= _BV(SPE);        // Activa el bus SPI en modo esclavo
  i = 0;                   // buffer vacio
  pin = false;
  SPI.attachInterrupt();   // activa las interrupciones en el bus SPI 
}

void loop() {
  Serial.println(str); // imprimir el array en el monitor serie
  delay(1000);
}

// función de interrupción
ISR(SPI_STC_vect) {
  char c = SPDR;           // leer byte del registro de datos SPI
  if (i < sizeof(str)) {
    str[i++] = c;          // guardar datos en el siguiente índice en la matriz buff
    if ((c == '\r') || (c == '\n') || (c == '\0')) // comprobar el final de la palabra
      pin = true;
  }
}
