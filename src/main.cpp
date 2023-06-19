//Наливатор Т-017 (release 1.0.2)

//Плата Arduino Mega имеет три дополнительных последовательных порта:
//Serial1 на портах 19 (RX) и 18 (TX),
//Serial2 на портах 17 (RX) и 16 (TX), 
//Serial3 на портах 15 (RX) и 14 (TX).

//создаем объект myNextion на аппаратном Serial2 arduino mega и передаем номера управляющих пинов RX и TX
//RX - цифровой вывод 17, необходимо соединить с выводом TX дисплея Nextion
//TX - цифровой вывод 16, необходимо соединить с выводом RX дисплея Nextion

// Включен откладочный PIN
#define DEBUG_TEST
// Включаем отладку на Serial port
// #define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
    #define DEBUGLN(x) Serial.println(x)
    #define DEBUG(x)   Serial.print(x)
#else
    #define DEBUGLN(x)
    #define DEBUG(x)
#endif

// Подключаем библиотеки
#include <Arduino.h>
#include "Nextion.h"
#include "PinChangeInterrupt.h"
// The following pins are usable for PinChangeInterrupt:
//   Arduino Mega: 10, 11, 12, 13, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64),
//                A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)
// Вариант распиновки для MEGA (для клапанов, см аннотацию выше)
#ifdef __AVR_ATmega2560__
    #define portHMI Serial2
    // Определяем пины для управления клапанами и насосом
    #define valve1Pin 2
    #define valve2Pin 3
    #define valve3Pin 4
    #define valve4Pin 5
    #define pumpPin 6
    #define flowSensor1Pin 7
    #define flowSensor2Pin 8
    #define flowSensor3Pin 9
    #define flowSensor4Pin 10
    // Определяем пины для кнопок "Пуск" и "Стоп"
    #define buttonPinStart 11
    #define buttonPinStop 12

// Вариант распиновки для UNO
#elif __AVR_ATmega328P__
    #define portHMI Serial
    // Определяем пины для управления клапанами и насосом
    #define valve1Pin 2
    #define valve2Pin 3
    #define valve3Pin 4
    #define valve4Pin 5
    #define pumpPin 6
    #define flowSensor1Pin 7
    #define flowSensor2Pin 8
    #define flowSensor3Pin 9
    #define flowSensor4Pin 10
    // Определяем пины для кнопок "Пуск" и "Стоп"
    #define buttonPinStart 11
    #define buttonPinStop 12
#endif

#ifdef DEBUG_TEST
    #define debugLed 13
#endif

#define cFlowRatePule 1.2 // Мл/П

// Определяем переменные обрабатывания кнопок "Пуск" и "Стоп"
volatile bool buttonStateStart = false;
volatile bool buttonStateStop = false;
// Переменные для подсчета пульсаций расходомера
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
volatile int pulseCount3 = 0;
volatile int pulseCount4 = 0;

// Определяем начальное время
bool bDelay  = false; // Старт
bool bDelay1 = false; // Останов
bool bDelay2 = false; // Расходомеры
bool bDelay3 = false; // Резерв
unsigned int startOpenTime = 0;
unsigned int stopCloseTime = 0;
unsigned int startCloseTime = 0;
unsigned int startPumpTime = 0;
unsigned int calcTime = 0; // Время начала расчета расхода
unsigned int flowRate1 = 0; // переменная для хранения текущей скорости потока
unsigned int totalVolume1 = 0; // переменная для хранения общего объема жидкости
unsigned int flowRate2 = 0; // переменная для хранения текущей скорости потока
unsigned int totalVolume2 = 0; // переменная для хранения общего объема жидкости
unsigned int flowRate3 = 0; // переменная для хранения текущей скорости потока
unsigned int totalVolume3 = 0; // переменная для хранения общего объема жидкости
unsigned int flowRate4 = 0; // переменная для хранения текущей скорости потока
unsigned int totalVolume4 = 0; // переменная для хранения общего объема жидкости

// Определяем задержки в миллисекундах
#define valveOpenDelay 500 // задержка при открытии клапана
#define valveCloseDelay 2000 // задержка при закрытии клапана
#define pumpStartDelay 200 // задержка при запуске насоса
#define pumpStopDelay 1000 // задержка при остановке насоса

// Определяем переменные для считывания данных с дисплея
int num1ValMax = 0;
int num2ValMax = 0;
int num3ValMax = 0;
int num4ValMax = 0;
bool btn1State = false;
bool btn2State = false;
bool btn3State = false;
bool btn4State = false;
bool btnFlushState = false;

//Определяем флаг состояния работы программы
bool isRunning = false; // Флаг запуска системы

// Определяем объект для работы с дисплеем
Nextion myNextion(portHMI, 9600);

// Объявляем функции
void setValve(int valve, int state);
void startPump();
void stopPump();
void flushValves();
void updateValues();
void controlSystem();
void startButtonPressed();
void stopButtonPressed();
void pulseCounter1();
void pulseCounter2();
void pulseCounter3();
void pulseCounter4();
void flushSystem();
void getHMICommands();

bool isAllValvesOpened();
bool isAllValvesClosed();

void getFlowSensorValue(unsigned int timeDelay = 1000);

void setup() {
    // Настроим пины на выход
    pinMode(valve1Pin, OUTPUT);
    pinMode(valve2Pin, OUTPUT);
    pinMode(valve3Pin, OUTPUT);
    pinMode(valve4Pin, OUTPUT);
    pinMode(pumpPin, OUTPUT);

    // Инициализируем дисплей
    myNextion.beginCom();
    myNextion.init();

    // Настраиваем пины на входы с включенными прерываниями и подтяжкой
    pinMode(buttonPinStart, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(buttonPinStart), startButtonPressed, RISING);
    pinMode(buttonPinStop, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(buttonPinStop), stopButtonPressed, RISING);
    pinMode(flowSensor1Pin, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(flowSensor1Pin), pulseCounter1, RISING);
    pinMode(flowSensor2Pin, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(flowSensor2Pin), pulseCounter2, RISING);
    pinMode(flowSensor3Pin, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(flowSensor3Pin), pulseCounter3, RISING);
    pinMode(flowSensor4Pin, INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(flowSensor4Pin), pulseCounter4, RISING);

    #ifdef __AVR_ATmega2560__
        // Инициализируем Serial для отладки
        Serial.begin(9600); 
    #endif

    // Передаем значения с дисплея в переменные
    updateValues();

    #ifdef DEBUG_TEST
        pinMode(debugLed,OUTPUT);
    #endif
}

void loop() {
    // Команды с HMI
    getHMICommands();
    // Управляем клапанами, расходомерами и насосом
    controlSystem();
    // Расчет расходомеров
    getFlowSensorValue();
    // Функция промывки системы
    flushSystem();

    digitalWrite(debugLed, isRunning);
}
// Команды с HMI
void getHMICommands(){
    // Входящие сообщения с экрана
    String message = myNextion.listen();
    if(message != ""){
        if(message == "UPDVOL" && !isRunning){ // Проверка "Система не в работе"
            // Обновляем значения с дисплея
            updateValues();
        }
        if(message == "UPDVOL" && isRunning){ // Проверка "Система в работе"
            // Обновляем значения с дисплея
            updateValues();
            myNextion.setComponentValue("num1", num1ValMax);
        }
    }
}

// Управление клапанами
void setValve(int valve, int state) {
  switch (valve) {
    case valve1Pin:
        digitalWrite(valve1Pin, state);
        break;
    case valve2Pin:
        digitalWrite(valve2Pin, state);
        break;
    case valve3Pin:
        digitalWrite(valve3Pin, state);
        break;
    case valve4Pin:
        digitalWrite(valve4Pin, state);
        break;
    default:
        break;
  }
}

void startPump() {
  digitalWrite(pumpPin, HIGH);
  DEBUGLN("start");
}

void stopPump() {
  digitalWrite(pumpPin, LOW);
  DEBUGLN("stop");
}

void flushValves() {
  // Открываем все клапаны
  setValve(valve1Pin, HIGH);
  setValve(valve2Pin, HIGH);
  setValve(valve3Pin, HIGH);
  setValve(valve4Pin, HIGH);
}

void updateValues() {
    // Считываем значения элементов
    num1ValMax = myNextion.getComponentValue("num1");
    DEBUGLN(num1ValMax);
    num2ValMax = myNextion.getComponentValue("num2");
    DEBUGLN(num2ValMax);
    num3ValMax = myNextion.getComponentValue("num3");
    DEBUGLN(num3ValMax);
    num4ValMax = myNextion.getComponentValue("num4");
    DEBUGLN(num4ValMax);
}

void flushSystem() {
  // Запуск промывки
  if(isRunning && !buttonStateStart){ //Если флаг запуска false И кнопка старт не нажата
    btnFlushState = myNextion.getComponentValue("btnFlush");
    flushValves();
    startPump();
    isRunning = true; // Флаг запуска системы
    buttonStateStart = false; //Флаг состояния нажатия кнопки
    DEBUGLN("startFlush");
  }
  
  // ОСТАНОВ
  if(!isRunning && buttonStateStop){ //Если флаг запуска true И кнопка стоп не нажата
    btnFlushState = myNextion.getComponentValue("btnFlush");
    stopPump();
    setValve(valve1Pin, LOW);
    setValve(valve2Pin, LOW);
    setValve(valve3Pin, LOW);
    setValve(valve4Pin, LOW);
    isRunning = false; // Флаг запуска системы
    buttonStateStop = false; //Флаг состояния нажатия кнопки
    DEBUGLN("stopFlush");
  }
}

void controlSystem() {
    // Запуск системы
    if(!isRunning && buttonStateStart){
        if(bDelay == false){
            bDelay = true;
            startOpenTime = millis();
            // Проверяем состояние кнопок 1 РАЗ!!!
            // 1 - разрешить; 0 - запретить
            btn1State = myNextion.getComponentValue("btn1");
            DEBUGLN(btn1State);
            btn2State = myNextion.getComponentValue("btn2");
            DEBUGLN(btn2State);
            btn3State = myNextion.getComponentValue("btn3");
            DEBUGLN(btn3State);
            btn4State = myNextion.getComponentValue("btn4");
            DEBUGLN(btn4State);
            // Считываем значения налива
            updateValues();
        }

        // Управляем клапанами
        if (!btn1State) {
            setValve(valve1Pin, LOW);
        } else {
            if(!digitalRead(valve1Pin)){
                if (num1ValMax > 0 && (millis() - startOpenTime > valveOpenDelay)) {
                    setValve(valve1Pin, HIGH);
                }
            }
        }
        if (!btn2State) {
            setValve(valve2Pin, LOW);
        } else {
            if(!digitalRead(valve2Pin)){
                if (num2ValMax > 0 && (millis() - startOpenTime > valveOpenDelay)) {
                    setValve(valve2Pin, HIGH);
                }
            }
        }
        if (!btn3State) {
            setValve(valve3Pin, LOW);
        } else {
            if(!digitalRead(valve3Pin)){
                if (num3ValMax > 0 && (millis() - startOpenTime > valveOpenDelay)) {
                    setValve(valve3Pin, HIGH);
                }
            }
        }
        if (!btn4State) {
            setValve(valve4Pin, LOW);
        } else {
            if(!digitalRead(valve4Pin)){
                if (num4ValMax > 0 && (millis() - startOpenTime > valveOpenDelay)) {
                    setValve(valve4Pin, HIGH);
                }
            }
        }
        if(isAllValvesOpened()){ // проработать вариант открытия при не всех клапанах
            if (millis() - startOpenTime > pumpStartDelay) { // ждем, пока пройдет задержка pumpStartDelay
                startPump();
                bDelay = false;
                isRunning = true; // Флаг запуска системы
                buttonStateStart = false;
                myNextion.sendCommand("page 1");
            }
        }
    }
    // ОСТАНОВ
    if(isRunning && buttonStateStop){
        if(bDelay1 == false){
            bDelay1 = true;
            stopCloseTime = millis();
        }
        if (millis() - stopCloseTime > valveCloseDelay) {
            setValve(valve1Pin, LOW);
        }
        if (millis() - stopCloseTime > valveCloseDelay) {
            setValve(valve2Pin, LOW);
        }
        if (millis() - stopCloseTime > valveCloseDelay) {
            setValve(valve3Pin, LOW);
        }
        if (millis() - stopCloseTime > valveCloseDelay) {
            setValve(valve4Pin, LOW);
        }
        if (millis() - stopCloseTime > pumpStopDelay) { // ждем, пока пройдет задержка pumpStopDelay
            stopPump();
            isRunning = false; // Флаг запуска системы
            buttonStateStop = false;
            myNextion.sendCommand("page 0");
            bDelay1 = false;
        }
    }  // if(isRunning && buttonStateStop)
} 
 // Расчет раходомеров
void getFlowSensorValue(unsigned int timeDelay = 1000){
    if(bDelay2 == false){
        bDelay2 = true;
        calcTime = millis();
    }
    if (millis() - calcTime > timeDelay) {
        // Умножаем на 10, чтобы не было дробной части в расчетах (экономим память и быстродействие)
        flowRate1 = pulseCount1 * uint16_t(cFlowRatePule * 10); // Расход на единицу времени timeDelay
        totalVolume1 += flowRate1 / 10; 
        flowRate2 = pulseCount2 * uint16_t(cFlowRatePule * 10); // Расход на единицу времени timeDelay
        totalVolume2 += flowRate2 / 10; 
        flowRate3 = pulseCount3 * uint16_t(cFlowRatePule * 10); // Расход на единицу времени timeDelay
        totalVolume3 += flowRate3 / 10; 
        flowRate4 = pulseCount4 * uint16_t(cFlowRatePule * 10); // Расход на единицу времени timeDelay
        totalVolume4 += flowRate4 / 10; 

        pulseCount1 = 0;
        pulseCount2 = 0;
        pulseCount3 = 0;
        pulseCount4 = 0;

        bDelay2 = false;
    }
}

bool isAllValvesClosed() {
  if (digitalRead(valve1Pin) == LOW &&
      digitalRead(valve2Pin) == LOW &&
      digitalRead(valve3Pin) == LOW &&
      digitalRead(valve4Pin) == LOW) {
    return true;
  } else {
    return false;
  }
}

bool isAllValvesOpened() {
  if (digitalRead(valve1Pin) == HIGH &&
      digitalRead(valve2Pin) == HIGH &&
      digitalRead(valve3Pin) == HIGH &&
      digitalRead(valve4Pin) == HIGH) {
    return true;
  } else {
    return false;
  }
}
// ПРЕРЫВАНИЯ
void startButtonPressed() {
    buttonStateStart = true;
}
void stopButtonPressed() {
    buttonStateStop = true;
}
void pulseCounter1(){
    pulseCount1++;
}
void pulseCounter2(){
    pulseCount2++;
}
void pulseCounter3(){
    pulseCount3++;
}
void pulseCounter4(){
    pulseCount4++;
}