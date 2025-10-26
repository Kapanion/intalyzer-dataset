#include <LiquidCrystal_I2C.h>
#include "DHT.h"

#define DHTPIN1 8   
#define DHTPIN2 3   
#define DHTPIN3 4   
#define DHTPIN4 5

#define DHTTYPE1 DHT22 
#define DHTTYPE2 DHT22 
#define DHTTYPE3 DHT22 
#define DHTTYPE4 DHT22   

DHT dht1(DHTPIN1, DHTTYPE1);
DHT dht2(DHTPIN2, DHTTYPE2);
DHT dht3(DHTPIN3, DHTTYPE3);
DHT dht4(DHTPIN4, DHTTYPE4);

LiquidCrystal_I2C lcd(0x27,16,2);

// Contadores y variables de ayuda
int a=1;
//int modificar;
volatile bool modificar = 0;
bool plata = 0;
bool estado = 0;

void setup()
{
  lcd.init();
  lcd.clear();         
  lcd.backlight();      

  // Botón
  pinMode(2,INPUT_PULLUP);
  pinMode(13,INPUT);

  // LED set_point
  pinMode(7,OUTPUT);

  // Potenciómetro
  pinMode(A2,INPUT);

  // Termistores
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  
  // Relevador
  pinMode(6,OUTPUT);

  Serial.begin(9600);
  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();
  
  attachInterrupt(digitalPinToInterrupt(2),cambio_setpoint,RISING);

}

void loop()
{
  // DEFINIR SET POINT //

  // bool boton = digitalRead(13);
  // Serial.println("-Empieza-");
//   if (modificar==0){
//     modificar=1;
//   }
//   else {
//     modificar=0;
//   }
// Serial.println("-Empieza-");
  while (plata==0){
  float pot = analogRead(A2);
  int set_point = map(pot,0,1023,14,34);
  // Serial.print("Set point = ");
  // Serial.println(set_point);
  delay(500);  
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Temp. Ideal:");
  lcd.setCursor(6,1);
  
  lcd.print(set_point);
  lcd.print(" C");
  }

  // MEDIR TEMPERATURA //
  float temperaturaPromedio = tempAmb();
  float temperaturaEntrada = tempTermistor(A0);
  float temperaturaSalida = tempTermistor(A1);

  // EVALUAR RELEVADOR //
  float pot = analogRead(A2);
  int set_point = map(pot,0,1023,14,34);
  relevador(temperaturaPromedio,set_point);
  
  // MANDAR RESULTADOS A ESP-32

  // VISUALIZAR RESULTADOS //

  // Monitor serial
  // Serial.print("Promedio: ");
  //Serial.println("--");
  Serial.println(temperaturaPromedio);
  // Serial.print("Termistor 1: ");
  Serial.println(temperaturaEntrada);
  // Serial.print("Termistor 2: ");
  Serial.println(temperaturaSalida);
  
  // Display
  mostrarDisplay(8000,2,temperaturaPromedio,temperaturaEntrada,temperaturaSalida);
}


// FUNCIONES

void cambio_setpoint() {
  if (modificar==0){
    plata=!plata;
    estado=!estado;
    digitalWrite(7,estado);
  }
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void relevador(float temperatura,int set_point){
  if (temperatura>=set_point+1){
    delay(1000);
    digitalWrite(6,LOW);

    }
  else if(temperatura<=set_point-1){
    digitalWrite(6,HIGH);

}
}

float tempAmb(){
  delay(2000);
  float t1 = dht1.readTemperature();
  float t2 = dht2.readTemperature();
  float t3 = dht3.readTemperature();
  float t4 = dht4.readTemperature();

  if (isnan(t1) || isnan(t2) || isnan(t3) || isnan(t4)) {
    Serial.println(F("Error en leer temperatura de DHT sensor!"));
    return;
  }
  

  float Tp = (t1+t2+t3+t4)/4;
  return Tp;
}


float tempTermistor(int pin){
	float sensorValue = analogRead(pin);
	float Vt = mapFloat(sensorValue,0.0,1023,0.0,5.0);
	float Rt = (10000 * Vt) / (5 - Vt);  
  float Beta = .00004*Rt*Rt-1.4048*Rt+14542;
  float T_log = 1 / ((log(Rt/10000.0)/Beta) + 1/(25+273.15));
  float temperatura = T_log-273.15;
  // Serial.print("Resistencia: ");
  // Serial.println(Rt);
  // Serial.print("Beta: ");
  // Serial.println(Beta);

  return temperatura;
  
}

void mostrarDisplay(int tiempo, int vueltas,float temperaturaPromedio, float temperaturaEntrada, float temperaturaSalida){
  int espera = (tiempo)/3;
  
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Temperatura");
  lcd.setCursor(0,1);
  lcd.print("Entrada: ");
  lcd.print(temperaturaEntrada);
  lcd.print(" ");
  lcd.print("C");
  delay(espera);
  
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Temperatura");
  lcd.setCursor(0,1);
  lcd.print("Salida:  ");
  lcd.print(temperaturaSalida);
  lcd.print(" ");
  lcd.print("C");
  delay(espera);  

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Temperatura");
  lcd.setCursor(0,1);
  lcd.print("Ambiente:");
  lcd.print(temperaturaPromedio);
  lcd.print(" ");
  lcd.print("C");
  delay(espera);

}
