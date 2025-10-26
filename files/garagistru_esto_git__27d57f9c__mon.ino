#include <Keypad.h>

#include <LiquidCrystal_I2C.h>

#include <Wire.h>

 #include <EEPROM.h>

/*
  Keyboard.begin();

RISING (рост) - срабатывает при изменении сигнала с LOW на HIGH
FALLING (падение) - срабатывает при изменении сигнала с HIGH на LOW
CHANGE (изменение) - срабатывает при изменении сигнала (с LOW на HIGH и наоборот)
LOW (низкий) - срабатывает постоянно при сигнале LOW
(не поддерживается на ESP8266)
*/

struct Data
{
    int DiametroMini = 2.5;
    int DiametroMaxi = 4.5;
    int largo = 15;
    float fvalue = 4.5;
};
// глобальный экземпляр для личного использования
Data data;
  long timpoPas = 0;// временная
int ps = 0;           // временная
long largos = 0;       // временная
long x = 0;            // нажатая цифра
long n = 0;            // число на вводе
long tiempoNumber = 0; // наборная из клавиатуры


int testDiod=4;//////////////////////
// клавиатура
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte rowPin[ROWS] = {12, 11, 10, 9};
byte colPin[COLS] = {8, 7, 6};

Keypad kpd = Keypad(makeKeymap(keys), rowPin, colPin, ROWS, COLS);

LiquidCrystal_I2C lcd(0x27, 16, 2); // экземпляр жк экрана

volatile int counter = 0;      // переменная-счётчик
volatile bool intFlag = false; // флаг
int houser = 0;                // сделать память
bool status;
const int PinLamp = 5;      // заменить на цикл укладчика
const int recalculo = A0;   // кнопка пересчета длинны
bool recalculoFlag = false; // флаг
int moto=13;//motor rele
void setup()
{

    /* EEPROM.get(0, data); // прочитать из адреса 0
                          //  меняем

     data.largo = 12;
     data.fvalue = 3.14;
     EEPROM.put(0, data); // поместить в EEPROM по адресу 0
 */
    Serial.begin(9600); // открыли порт для связи
    lcd.init();
  Keyboard.begin();// работа клавиатуры
    lcd.backlight();

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print(String("BOBINADORA"));
    delay(1000);
    lcd.setCursor(0, 1); // ввод
    lcd.print("www.katalina.ru");
    delay(2000);
  
  
 pinMode(moto, OUTPUT);
   digitalWrite(moto, 1);
    pinMode(recalculo, INPUT_PULLUP); // пин кнопки входа в память
    // подключили кнопку на D2 и GND
    pinMode(PinLamp, OUTPUT);
  digitalWrite(moto, 1);
pinMode(testDiod, OUTPUT);/////////////////////////////
      pinMode(2, INPUT_PULLUP);
 
  //  pinMode(2, INPUT);
    // FALLING - при нажатии на кнопку будет сигнал 0, его и ловим
    // attachInterrupt(0, btnIsr, FALLING);
    digitalWrite(PinLamp, 1); //  сигнал 0 на реле
    // attachInterrupt(0, buttonTick, RISING); //+
    // attachInterrupt(0, buttonTick, FALLING);//+
    attachInterrupt(0, buttonTick, CHANGE); //++
                                            // attachInterrupt(0, buttonTick, LOW);
    lcd.clear();
}
//*****************************************************************
///--------------------------
void MiEeprom(long a)
{
  
     EEPROM.get(0, data); // прочитать из адреса 0
                          //  меняем

     data.largo = a;
     data.fvalue = 3.14;  // запасные данныые
     EEPROM.put(0, data); // поместить в EEPROM по адресу 0
 
}

//---------------------------------
int DetectButtons();
//------------------------
void Hablo(String a);
void Hablo_2(String a);
/////////----------------------------------------------

/////////////-----------------------------------------------

//-----------------------------------------------//
void Recalculo()
{
    bool btnState = !digitalRead(recalculo);
    if (btnState && !recalculoFlag)
    { // обработчик нажатия
        recalculoFlag = true;
        Serial.println("press");

 lcd.clear();
   lcd.setCursor(0, 0);
     lcd.print(String("Trabajo ")+data.largo );
  
    delay(2000);     
      
        
            Serial.println(String("Trabajo "));
      
       digitalWrite(moto, 0);
      lcd.clear();
     
    }
    if (!btnState && recalculoFlag)
    { // обработчик отпускания
        recalculoFlag = false;
      
   
      
      
      
      
      ///////выйти в перезагрузку
    }

    // вызываем клавиатуру
    // Serial.println(String("enciende el teclado"));  // выводим
}

void buttonTick()
{
    intFlag = true; // подняли флаг прерывания
  
       if (intFlag && counter < data.largo)
       {

           intFlag = false; // сбрасываем
           // совершаем какие-то действия
           counter++; // + нажатие
           Serial.println(counter);

           //lcd.clear();
        //   lcd.setCursor(0, 0);
         //  lcd.print(String("hizo ") + counter);
         //  lcd.setCursor(0, 1);
         //  lcd.print(String("dado ") + houser);

           digitalWrite(PinLamp, 0); //  сигнал 0 на реле
         
           digitalWrite(testDiod, 1); //  сигнал diod
       }

       else if (counter >= data.largo)
       {
           digitalWrite(PinLamp, 1); //  сигнал 0 на реле
          digitalWrite(testDiod, 0); //  сигнал diod
           
          digitalWrite(moto, 1);
       Serial.println("Fin");
           counter = 0;
       }

}
//***********
void btnIsr()
{
    counter++; // + нажатие
    status = 1;
  
    Serial.println(counter); // выводим

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print(String("BOBINADORA"));
    delay(1000);
    lcd.setCursor(0, 1); // ввод
    lcd.print("www.katalina.ru");
    delay(2000);
}
//**************
void Fu()
{
    Serial.println("FU"); // выводим
    Fu2();
}

void Fu2()
{
    Serial.println("FU_2"); // выводим
}



//*************************************************************
void loop()
{
   Recalculo();
  //if(bool btnState = !digitalRead(recalculo))
  {
  
  
 // lcd.clear();
   lcd.setCursor(3, 0);
     lcd.print(String(counter));
  }
  
  
  
  
  if(counter>0)
  {
  lcd.clear();
   lcd.setCursor(3, 0);
     lcd.print(String(counter));
  delay(100);
  }
 // DetectButtons();
  
  
 /* digitalWrite(testDiod, 1);
        delay(500);///общая скорость деленная на два
        digitalWrite(testDiod, 0);
        delay(500);
  Serial.println(String("Tick-Tack"));  // выводим
  */
 
 
  
  //DetectButtons();

    
// Recalculo(); // считываем кнопку памяти
   
       // вызываем клавиатуру
       // Serial.println(String("enciende el teclado"));  // выводим




      
       // Serial.println(String("largo ")+data.largo);  // выводим
       // delay(600);
        houser= 10;// сделать память
       houser = data.largo; // сделать память
       // Serial.println(houser);  // выводим
       // delay(600);
/*
       if (intFlag && counter < data.largo)
       {

           intFlag = false; // сбрасываем
           // совершаем какие-то действия
           counter++; // + нажатие
           Serial.println(counter);

           lcd.clear();
           lcd.setCursor(0, 0);
           lcd.print(String("hizo ") + counter);
           lcd.setCursor(0, 1);
           lcd.print(String("dado ") + houser);

           digitalWrite(PinLamp, 0); //  сигнал 0 на реле
       }

       else if (counter >= data.largo)
       {
           digitalWrite(PinLamp, 1); //  сигнал 0 на реле
           Serial.println("Fin");

           counter = 0;
       }
*/
     
}

///--------------------------------------//

//************************************************/
void Hablo(String a)
{
    lcd.setCursor(1, 0);
    lcd.print(a);
}
//////////////---------------------------------------------
void Hablo_2(String a)
{
    lcd.setCursor(2, 1);
    lcd.print(a);
}
int DetectButtons() // ввод с клавиатуры
{
  lcd.setCursor(1, 0);
    lcd.print((String("L= ") ));
    x = kpd.getKey();
    if (x > 47)
    {
        x = x - 48;
        n = n * 10 + x;
        Hablo_2(String("numero: ") + n);
    }
    if (x == '*') // откат последней цыфры
    {
        n = n / 10;
        Hablo("L=            ");
      //  Hablo(("L= ") + n);
       	Hablo_2(String("             "));
        Hablo_2(String("numero: ") + n);
      
      
    }
    if (x == '#') // запоминание значения
    {
        tiempoNumber = n;
        n = 0;
       Hablo(("L= ") + String(tiempoNumber));
        Hablo_2(String("memorizaste!!! "));
      Serial.println("memorizaste!!! ");///закладка на запомнить
    }
 
  
    return tiempoNumber;
}