/*
  Created 2017  by AlexGyver  AlexGyver Home Labs Inc.
  Edited 2024 by ZavoraTeam ZavoraDev Libs Inc.
  Refactored 2024 by AI Assistant

  ВНИМАНИЕ! ПУТЬ К ПАПКЕ СО СКЕТЧЕМ НЕ ДОЛЖЕН СОДЕРЖАТЬ РУССКИХ СИМВОЛОВ
  ВО ИЗБЕЖАНИЕ ПРОБЛЕМ ПОЛОЖИТЕ ПАПКУ В КОРЕНЬ ДИСКА С

  При подключении и открытии монитора порта будет запущен процесс калибровки.
  Вам нужно при помощи вольтметра измерить напряжение на пинах 5V и GND,
  затем отправить его в монитор В МИЛЛИВОЛЬТАХ, т.е. если на вольтметре 4.56
  то отправить примерно 4560. После этого изменить initial_calibration на 0
  и перепрошить Arduino.
  Если хотите пропустить процесс калибровки, то введите то же самое напряжение,
  что было показано вам при калибровке (real VCC). И снова прошейте код.
*/

//---------------------------- НАСТРОЙКИ ------------------------------------
// Базовые параметры
#define DEBUG 0                  // 1 - включить отладку по Serial, 0 - выключить
#define INITIAL_CALIBRATION 0    // калибровка вольтметра 1 - включить, 0 - выключить
#define WELCOME 1                // приветствие (слова ZAVR VAPE при включении)
#define BATTERY_INFO 0           // отображение напряжения аккумулятора при запуске
#define SLEEP_TIMER 10           // таймер сна в секундах
#define VAPE_THRESHOLD 6         // отсечка затяжки, в секундах
#define TURBO_MODE 0             // турбо режим 1 - включить, 0 - выключить
#define BATTERY_PERCENT 1        // отображать заряд в процентах
#define BATTERY_LOW 2.5          // нижний порог защиты аккумулятора, в Вольтах

// Пины
#define BUTT_UP 5                // кнопка вверх
#define BUTT_DOWN 4              // кнопка вниз
#define BUTT_SET 3               // кнопка выбора
#define BUTT_VAPE 2              // кнопка "парить"
#define MOSFET 10                // пин мосфета (нагрев спирали)
#define BATTERY_PIN 5            // пин измерения напряжения акума
#define DISP_VCC 13              // питание дисплея

// Дисплей
#define SCLK 6
#define RCLK 7
#define DIO 8

// Адреса хранения в EEPROM
#define EEPROM_VOLTS 0
#define EEPROM_WATTS 2
#define EEPROM_OHMS 4
#define EEPROM_VCC_CONST 8

// Константы режимов
#define MODE_VOLT 0
#define MODE_WATT 1
#define MODE_COIL 2

// Константы фильтрации
#define BATTERY_FILTER_K 0.04
#define PWM_FILTER_K 0.1

// Необходимые библиотеки
#include <EEPROMex.h>
#include <LowPower.h>
#include <TimerOne.h>
#include <TM74HC595Display.h>

// Класс для обработки кнопок с защитой от дребезга
class Button {
  private:
    byte pin;
    bool lastState;
    bool flagState;
    unsigned long debounceTime;
    unsigned long pressTime;
    const unsigned long DEBOUNCE_DELAY = 50;

  public:
    Button(byte buttonPin) {
      pin = buttonPin;
      lastState = false;
      flagState = false;
      debounceTime = 0;
      pressTime = 0;
      pinMode(pin, INPUT_PULLUP);
    }

    // Обновить состояние кнопки
    bool update() {
      bool newState = !digitalRead(pin);
      
      if (newState != lastState) {
        debounceTime = millis();
      }
      
      if ((millis() - debounceTime) > DEBOUNCE_DELAY) {
        if (newState && !flagState) {
          pressTime = millis();
          flagState = true;
          return true;
        }
      }
      
      lastState = newState;
      
      if (!newState && flagState) {
        flagState = false;
      }
      
      return false;
    }

    // Возвращает true если кнопка отпущена
    bool released() {
      bool newState = !digitalRead(pin);
      
      if (!newState && flagState) {
        flagState = false;
        return true;
      }
      
      return false;
    }

    // Проверить, нажата ли кнопка сейчас
    bool isPressed() {
      return !digitalRead(pin);
    }

    // Проверить, как долго нажата кнопка (в мс)
    unsigned long getPressTime() {
      return flagState ? millis() - pressTime : 0;
    }
};

// Класс для управления настройками устройства
class VapeSettings {
  private:
    int _volts;
    int _watts;
    float _ohms;
    float _vccConst;
    
  public:
    VapeSettings() {
      _volts = 0;
      _watts = 0;
      _ohms = 0.0;
      _vccConst = 1.1;
    }
    
    void loadFromEEPROM() {
      _volts = EEPROM.readInt(EEPROM_VOLTS);
      _watts = EEPROM.readInt(EEPROM_WATTS);
      _ohms = EEPROM.readFloat(EEPROM_OHMS);
      _vccConst = EEPROM.readFloat(EEPROM_VCC_CONST);
    }
    
    void saveVolts() {
      EEPROM.writeInt(EEPROM_VOLTS, _volts);
    }
    
    void saveWatts() {
      EEPROM.writeInt(EEPROM_WATTS, _watts);
    }
    
    void saveOhms() {
      EEPROM.writeFloat(EEPROM_OHMS, _ohms);
    }
    
    void saveVccConst(float vccConst) {
      _vccConst = vccConst;
      EEPROM.writeFloat(EEPROM_VCC_CONST, _vccConst);
    }
    
    int getVolts() { return _volts; }
    int getWatts() { return _watts; }
    float getOhms() { return _ohms; }
    float getVccConst() { return _vccConst; }
    
    void setVolts(int volts) { 
      _volts = constrain(volts, 0, 9000); // Безопасное ограничение
    }
    
    void setWatts(int watts) { 
      _watts = constrain(watts, 0, 100); // Безопасное ограничение 
    }
    
    void setOhms(float ohms) { 
      _ohms = constrain(ohms, 0.05, 3.0); // Безопасное ограничение
    }
};

// Создаем основные объекты
TM74HC595Display disp(SCLK, RCLK, DIO);
VapeSettings settings;
Button buttUp(BUTT_UP);
Button buttDown(BUTT_DOWN);
Button buttSet(BUTT_SET);
Button buttVape(BUTT_VAPE);

// Символы для дисплея
unsigned char SYM[47];

// Состояние устройства
enum StateType {
  STATE_NORMAL,
  STATE_VAPING,
  STATE_SLEEP,
  STATE_WAKE_PUZZLE,
  STATE_SET_HOLD
};

StateType currentState = STATE_NORMAL;
byte currentMode = MODE_VOLT;
bool modeChanged = true;
volatile bool wakeUpFlag = false;
volatile byte vapeReleaseCount = 0;
byte vapeMode = 0;

// Таймеры
unsigned long lastUpdateTime = 0;
unsigned long wakeTimer = 0;
unsigned long vapeStartTime = 0;
unsigned long setHoldStartTime = 0;

// Измерения батареи
int batteryVoltage = 0;
int batteryVoltageFiltered = 0;
bool isBatteryLow = false;

// PWM контроль
int pwmValue = 0;
int pwmOldValue = 800;
int pwmFilteredValue = 0;

// Надписи на дисплей
byte VVOL[4] = { 41, 36, 32, 30 };
byte VAVA[4] = { 41, 36, 20, 42 };
byte COIL[4] = { 22, 32, 28, 30 };
byte GYVE[4] = { 2, 20, 36, 48 };    // GYVE -> ZAVR
byte YVEA[4] = { 20, 36, 48, 36 };   // YVEA -> AVRV
byte VAPE[4] = { 36, 20, 33, 24 };
byte BVOL[4] = { 44, 36, 32, 30 };
byte vape1[4] = { 46, 45, 46, 45 };
byte vape2[4] = { 45, 46, 45, 46 };
byte LOWB[4] = { 8, 45, 45, 0 };     // low battery - lbat
byte BYE[4] = { 49, 20, 28, 47 };    // sleep - wait
byte BLANK[4] = { 42, 42, 42, 42 };
byte V[4] = { 36, 42, 42, 42 };      // V
byte A[4] = { 36, 20, 42, 42 };      // VA
byte P[4] = { 36, 20, 33, 42 };      // VAP
byte E[4] = { 36, 20, 33, 24 };      // VAPE

// Инициализация символов для дисплея
void initSymbols() {
  //--------цифры--------
  SYM[0] = 0xC0;          //0
  SYM[1] = 0xF9;          //1
  SYM[2] = 0xA4;          //2
  SYM[3] = 0xB0;          //3
  SYM[4] = 0x99;          //4
  SYM[5] = 0x92;          //5
  SYM[6] = 0x82;          //6
  SYM[7] = 0xF8;          //7
  SYM[8] = 0x80;          //8
  SYM[9] = 0x90;          //9
  //---цифры с точкой----
  SYM[10] = 0b01000000;   //.0
  SYM[11] = 0b01111001;   //.1
  SYM[12] = 0b00100100;   //.2
  SYM[13] = 0b00110000;   //.3
  SYM[14] = 0b00011001;   //.4
  SYM[15] = 0b00010010;   //.5
  SYM[16] = 0b00000010;   //.6
  SYM[17] = 0b01111000;   //.7
  SYM[18] = 0b00000000;   //.8
  SYM[19] = 0b00010000;   //.9
  //---заглавные буквы---
  SYM[20] = 0x88;         //A
  SYM[22] = 0xC6;         //C
  SYM[24] = 0x86;         //E
  SYM[25] = 0x8E;         //F
  SYM[26] = 0xC2;         //G
  SYM[27] = 0x89;         //H
  SYM[28] = 0xF9;         //I
  SYM[29] = 0xF1;         //J
  SYM[30] = 0xC3;         //L
  SYM[32] = 0xC0;         //O
  SYM[33] = 0x8C;         //P
  SYM[48] = 0x0c;         //P. ~ R
  SYM[35] = 0x92;         //S
  SYM[36] = 0xC1;         //U
  SYM[40] = 0b01000001;   //U.
  SYM[37] = 0x91;         //Y
  //---маленькие буквы---
  SYM[47] = 0x07;         //t
  SYM[21] = 0x83;         //b
  SYM[44] = 0b00000011;   //b.
  SYM[23] = 0xA1;         //d
  SYM[31] = 0xA9;         //n
  SYM[34] = 0x98;         //q
  SYM[49] = 0xaa;         //W. ~ w
  SYM[41] = 0b01100011;   //u.
  SYM[43] = 0b11100011;   //u
  //-------символы-------
  SYM[38] = 0xFE;         //hight -
  SYM[39] = 0b11110111;   //low -
  SYM[42] = 0b11111111;   //(пробел)
  SYM[45] = 0b10111111;   //mid -
  SYM[46] = 0b11110110;   //mid&high -
}

// Функция для вывода слов на дисплей
void displaySend(byte sym[]) {
  for (int i = 0; i < 4; i++) {
    disp.set(SYM[sym[i]], 3 - i);
  }
}

// Измерить напряжение VCC
long readVcc() {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);               // Ждем установки опорного напряжения
  ADCSRA |= _BV(ADSC);    // Начинаем преобразование
  while (bit_is_set(ADCSRA, ADSC)); // Ждем окончания преобразования
  uint8_t low = ADCL;     // Сначала читаем ADCL 
  uint8_t high = ADCH;    // Затем ADCH
  long result = (high << 8) | low;

  result = settings.getVccConst() * 1023 * 1000 / result;  // Расчет реального VCC
  return result;
}

// Проверить заряд батареи
void checkBattery() {
  batteryVoltage = readVcc();
  batteryVoltageFiltered = BATTERY_FILTER_K * batteryVoltage + (1 - BATTERY_FILTER_K) * batteryVoltageFiltered;
  
  if (batteryVoltageFiltered < BATTERY_LOW * 1000) {
    isBatteryLow = true;
    disableCoil();
    disp.clear();
    displaySend(LOWB);
  } else {
    isBatteryLow = false;
  }
  
#if DEBUG
  Serial.print("Battery voltage: ");
  Serial.println((float)batteryVoltageFiltered / 1000);
#endif
}

// Отключить койл
void disableCoil() {
  Timer1.disablePwm(MOSFET);  
  digitalWrite(MOSFET, LOW);   
}

// Перейти в режим сна
void goToSleep() {
  // Прощаемся
  displaySend(BYE);  
  delay(500);
  disp.clear();
  
  // Отключаем все что можно
  disableCoil();
  delay(50);
  
  // Настраиваем прерывание для пробуждения
  attachInterrupt(0, wakeUp, FALLING);
  delay(50);
  
  // Отключаем дисплей перед сном
  digitalWrite(DISP_VCC, LOW);
  
  // Уходим в сон
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

// Функция пробуждения (вызывается прерыванием)
void wakeUp() {
  digitalWrite(DISP_VCC, HIGH);
  disableCoil();
  wakeTimer = millis();
  wakeUpFlag = true;
  vapeReleaseCount = 0;
  vapeMode = 0;
  modeChanged = true;
}

// Функция пазла для пробуждения (нужно сделать 5 нажатий)
void wakePuzzle() {
  detachInterrupt(0);
  
  boolean wakeStatus = false;
  byte clickCount = 0;
  bool vapeButtonFlag = false;
  
  while (1) {
    if (buttVape.update()) {
      clickCount++;
      switch (clickCount) {
        case 1: displaySend(V); break;
        case 2: displaySend(A); break;
        case 3: displaySend(P); break;
        case 4: displaySend(E); break;
      }
      
      if (clickCount > 4) {
        wakeStatus = true;
#if DEBUG
        Serial.println("Принято. Просыпаюсь");
#endif
        break;
      }
    }
    
    if (buttVape.released()) {
      delay(70); // Небольшая задержка между нажатиями
    }
    
    if (millis() - wakeTimer > 3000) {
#if DEBUG
      Serial.println("Отказ. Меньше пяти нажатий. Продолжаю сон");
#endif
      break;
    }
  }
  
  if (wakeStatus) {
    wakeUpFlag = false;
    disp.clear();
    delay(100);
  } else {
    disp.clear();
    goToSleep();
  }
}

// Калибровка вольтметра
void calibration() {
  // Чистим EEPROM перед калибровкой
  for (byte i = 0; i < 7; i++) EEPROM.writeInt(i, 0);
  
  float tempVccConst = 1.1;
  settings.saveVccConst(tempVccConst);
  
#if DEBUG
  Serial.print("VCC измеренное: ");
  Serial.println(readVcc());
  Serial.println("Напиши ваше VCC (в милливольтах)");
#endif

  while (Serial.available() == 0);
  
  int userVcc = Serial.parseInt();
  float realConst = (float)1.1 * userVcc / readVcc();
  
#if DEBUG
  Serial.print("Новая константа VCC: ");
  Serial.println(realConst, 3);
#endif

  settings.saveVccConst(realConst);
  
  // После записи в EEPROM калибровку в настройках нужно выключить
  while (1); // Уходим в бесконечный цикл
}

// Вариваттный режим
void handleWattMode() {
  if (modeChanged) {
    modeChanged = false;
    displaySend(VAVA);
    delay(400);
    disp.clear();
  }
  
  // Изменение ватт кнопкой вверх
  if (buttUp.update()) {
    int watts = settings.getWatts() + 1;
    float batteryVoltage = (float)batteryVoltageFiltered / 1000;
    byte maxW = (sq(batteryVoltage)) / settings.getOhms();
    watts = min(watts, maxW);
    settings.setWatts(watts);
    disp.clear();
  }
  
  if (buttUp.released()) {
    settings.saveWatts();
  }
  
  // Изменение ватт кнопкой вниз
  if (buttDown.update()) {
    int watts = settings.getWatts() - 1;
    settings.setWatts(watts);
    disp.clear();
  }
  
  if (buttDown.released()) {
    settings.saveWatts();
  }
  
  // Отображаем текущее значение
  disp.digit4(settings.getWatts());
}

// Варивольтный режим
void handleVoltMode() {
  if (modeChanged) {
    modeChanged = false;
    displaySend(VVOL);
    delay(400);
    disp.clear();
  }
  
  // Изменение вольт кнопкой вверх
  if (buttUp.update()) {
    int volts = settings.getVolts() + 100;
    volts = min(volts, batteryVoltageFiltered);
    settings.setVolts(volts);
    disp.clear();
  }
  
  if (buttUp.released()) {
    settings.saveVolts();
  }
  
  // Изменение вольт кнопкой вниз
  if (buttDown.update()) {
    int volts = settings.getVolts() - 100;
    settings.setVolts(volts);
    disp.clear();
  }
  
  if (buttDown.released()) {
    settings.saveVolts();
  }
  
  // Отображаем текущее значение
  disp.float_dot((float)settings.getVolts() / 1000, 2);
}

// Режим настройки сопротивления
void handleCoilMode() {
  if (modeChanged) {
    modeChanged = false;
    displaySend(COIL);
    delay(400);
    disp.clear();
  }
  
  // Изменение омов кнопкой вверх
  if (buttUp.update()) {
    float ohms = settings.getOhms() + 0.05;
    settings.setOhms(ohms);
    disp.clear();
  }
  
  if (buttUp.released()) {
    settings.saveOhms();
  }
  
  // Изменение омов кнопкой вниз
  if (buttDown.update()) {
    float ohms = settings.getOhms() - 0.05;
    settings.setOhms(ohms);
    disp.clear();
  }
  
  if (buttDown.released()) {
    settings.saveOhms();
  }
  
  // Отображаем текущее значение
  disp.float_dot(settings.getOhms(), 2);
}

// Обработка режима парения
void handleVaping() {
  // Анимация парения
  if (round(millis() / 150) % 2 == 0) {
    displaySend(vape1);
  } else {
    displaySend(vape2);
  }
  
  // Рассчитываем мощность для варивольта
  if (currentMode == MODE_VOLT) {
    int volts = settings.getVolts();
    pwmValue = (float)volts / batteryVoltageFiltered * 1024;
    pwmValue = constrain(pwmValue, 0, 1023);
    pwmFilteredValue = PWM_FILTER_K * pwmValue + (1 - PWM_FILTER_K) * pwmOldValue;
    pwmOldValue = pwmFilteredValue;
    Timer1.pwm(MOSFET, pwmFilteredValue);
  }
  
  // Проверяем время парения
  if (millis() - vapeStartTime > VAPE_THRESHOLD * 1000) {
    disableCoil();
    currentState = STATE_NORMAL;
  }
}

// Обработка удержания кнопки SET
void handleSetHold() {
  if (round(millis() / 150) % 2 == 0) {
    if (!BATTERY_PERCENT) {
      disp.float_dot((float)batteryVoltageFiltered / 1000, 2);
    } else {
      disp.digit4(map(batteryVoltageFiltered, BATTERY_LOW * 1000, 4200, 0, 99));
    }
  }
  
  if (!buttSet.isPressed()) {
    currentState = STATE_NORMAL;
    modeChanged = true;
  }
}

// Обновление таймера сна
void updateWakeTimer() {
  if (buttUp.isPressed() || buttDown.isPressed() || 
      buttSet.isPressed() || buttVape.isPressed()) {
    wakeTimer = millis();
  }
}

// Обработчик прерывания для дисплея
void timerIsr() {
  disp.timerIsr();
}

void setup() {
#if DEBUG
  Serial.begin(9600);
  Serial.println("GyverMOD starting...");
#endif

  // Калибровка вольтметра если включена
  if (INITIAL_CALIBRATION) calibration();

  // Загрузка настроек из EEPROM
  settings.loadFromEEPROM();

  // Инициализация дисплея и символов
  initSymbols();
  Timer1.initialize(1500);
  Timer1.attachInterrupt(timerIsr);

  // Настройка пинов
  pinMode(MOSFET, OUTPUT);
  pinMode(DISP_VCC, OUTPUT);
  digitalWrite(DISP_VCC, HIGH);
  disableCoil();

  // Приветствие
  if (WELCOME) {
    displaySend(GYVE);
    delay(400);
    displaySend(YVEA);
    delay(400);
    displaySend(VAPE);
    delay(400);
  }

  // Измерение напряжения аккумулятора
  batteryVoltage = readVcc();
  batteryVoltageFiltered = batteryVoltage;
  
  // Проверка заряда батареи
  if (batteryVoltage < BATTERY_LOW * 1000) {
    isBatteryLow = true;
    disp.clear();
    displaySend(LOWB);
    disableCoil();
  }

  // Отображение информации о батарее при запуске
  if (BATTERY_INFO) {
    disp.clear();
    displaySend(BVOL);
    delay(500);
    disp.float_dot((float)batteryVoltage / 1000, 2);
    delay(1000);
    disp.clear();
  }
  
  // Инициализация таймера сна
  wakeTimer = millis();
}

void loop() {
  // Периодическое измерение напряжения батареи
  if (millis() - lastUpdateTime > 50) {
    lastUpdateTime = millis();
    checkBattery();
  }
  
  // Обработка кнопок и состояний
  updateWakeTimer();
  
  // Состояние низкой батареи имеет приоритет
  if (isBatteryLow) {
    disableCoil();
    displaySend(LOWB);
    return;
  }
  
  // Основная машина состояний
  switch (currentState) {
    case STATE_NORMAL:
      // Обработка SET
      if (buttSet.update()) {
        setHoldStartTime = millis();
      }
      
      // Долгое нажатие SET (> 300ms)
      if (buttSet.isPressed() && millis() - setHoldStartTime > 300) {
        currentState = STATE_SET_HOLD;
        disp.clear();
        return;
      }
      
      // Короткое нажатие SET (смена режима)
      if (buttSet.released() && millis() - setHoldStartTime <= 300) {
        currentMode = (currentMode + 1) % 3;
        modeChanged = true;
      }
      
      // Кнопка парения
      if (buttVape.update() && !isBatteryLow) {
        currentState = STATE_VAPING;
        vapeStartTime = millis();
        vapeMode = 1;
        return;
      }
      
      // Обработка текущего режима
      switch (currentMode) {
        case MODE_VOLT:
          handleVoltMode();
          break;
        case MODE_WATT:
          handleWattMode();
          break;
        case MODE_COIL:
          handleCoilMode();
          break;
      }
      break;
      
    case STATE_VAPING:
      handleVaping();
      
      // Обработка двойного нажатия
      if (buttVape.released()) {
        if (vapeReleaseCount == 0) {
          vapeReleaseCount = 1;
        } else if (vapeReleaseCount == 1) {
          // Турбо режим
          if (TURBO_MODE) {
            vapeMode = 2;
            digitalWrite(MOSFET, HIGH);
          }
          vapeReleaseCount = 2;
        } else if (vapeReleaseCount == 2) {
          // Вызов функции сна при тройном нажатии
          currentState = STATE_NORMAL;
          goToSleep();
        }
      }
      
      // Выход из режима парения
      if (!buttVape.isPressed()) {
        disableCoil();
        currentState = STATE_NORMAL;
        disp.clear();
      }
      break;
      
    case STATE_SET_HOLD:
      handleSetHold();
      break;
      
    case STATE_WAKE_PUZZLE:
      wakePuzzle();
      break;
      
    case STATE_SLEEP:
      // Не должны оказаться здесь, т.к. мы в режиме сна
      goToSleep();
      break;
  }
  
  // Проверка таймера сна
  if (millis() - wakeTimer > SLEEP_TIMER * 1000) {
    goToSleep();
  }
  
  // Обработка флага пробуждения
  if (wakeUpFlag) {
    currentState = STATE_WAKE_PUZZLE;
  }
} 