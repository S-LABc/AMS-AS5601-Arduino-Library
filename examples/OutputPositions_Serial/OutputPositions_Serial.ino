/*
 * OutputPositions_Serial
 * 
 * Демонстрация изменения количества шагов на оборот
 * от магнитного энкодера AS5601 в "Монитор порта"
 * 
 * Примечания:
 * 1. Для плат с AVR используются внешние прерывания INT_0(D3) и INT_1(D2)
 * 2. Для плат с STM32 используются внешние прерывания EXTI_10(PB10) и EXTI_11(PB11)
 * 
 * Подключение энкодера:
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * A     -> PB10 (D3)
 * B     -> PB11 (D2)
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключить энкодер согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Выбрать "нет конца строки" и "115200 бод"
 * 5. Вводить значения от 0 до 9 меняя положение вала энкодера
 * 6. Наблюдать за изменеием чувствительности экодера по значениям счетчика
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5601_DS000395_3-00.pdf
 * 
 * Контакты:
 ** YouTube - https://www.youtube.com/channel/UCbkE52YKRphgkvQtdwzQbZQ
 ** Telegram - https://www.t.me/slabyt
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5601.h>

// Контакты подключения энкодера STM32
#define PHASE_A PB10 // Фаза A
#define PHASE_B PB11 // Фаза B
/*
// Контакты подключения энкодера AVR
#define PHASE_A 3 // Фаза A
#define PHASE_B 2 // Фаза B
*/

// Счетчик тиков (шагов) энкодера
int enc_step_count = 0;
// Для обработчика прерываний и направления вращения
volatile bool encoder_reset = false, encoder_flag = false, turnLeft = false, turnRight = false;
volatile byte encoder_state = 0x00, previous_encoder_state = 0x00;

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  Serial.begin(115200);

  // Настройка контактов МК на вход
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);

  // Привязка прерываний на контакты от энкодера
  attachInterrupt(digitalPinToInterrupt(PHASE_A), isWheel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PHASE_B), isWheel, CHANGE);

  // Запускаем соединение
  Encoder.begin();
  // Настраиваем шину I2C на 400кГц
  Encoder.setClock();

  // Вывод справки
  printInfo();
}

void loop() {
  // Опрос UART для смены шагов на оборот
  if (Serial.available() == 1) {
    changeOutput();
  } else { // Опрос энкодера для изменения значения счетчика при повороте вала
    wheelEvent();
  }
}
// Информация о доступных действиях
void printInfo() {
  Serial.println("Enter code to select encoder resolution");
  Serial.println("0 - Show Info");
  Serial.println("1 - 8 Steps per Turn");
  Serial.println("2 - 16 Steps per Turn");
  Serial.println("3 - 32 Steps per Turn");
  Serial.println("4 - 64 Steps per Turn");
  Serial.println("5 - 128 Steps per Turn");
  Serial.println("6 - 256 Steps per Turn");
  Serial.println("7 - 512 Steps per Turn");
  Serial.println("8 - 1024 Steps per Turn");
  Serial.println("9 - 2048 Steps per Turn\n");
}
// Изменение количества шагов на один оборот
void changeOutput() {
  byte code = Serial.read() - '0'; // Для преобразования символов ASCII в число. ASCII цифра 0 это DEC 48
  switch (code) {
    case 0:
      printInfo();
    break;
    case 1:
      Encoder.enableEncoderOutputPositions8();
      Serial.println("Selected 8 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 2:
      Encoder.enableEncoderOutputPositions16();
      Serial.println("Selected 16 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 3:
      Encoder.enableEncoderOutputPositions32();
      Serial.println("Selected 32 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 4:
      Encoder.enableEncoderOutputPositions64();
      Serial.println("Selected 64 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 5:
      Encoder.enableEncoderOutputPositions128();
      Serial.println("Selected 128 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 6:
      Encoder.enableEncoderOutputPositions256();
      Serial.println("Selected 256 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 7:
      Encoder.enableEncoderOutputPositions512();
      Serial.println("Selected 512 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 8:
      Encoder.enableEncoderOutputPositions1024();
      Serial.println("Selected 1024 steps per revolution. Change the position of the encoder shaft\n");
    break;
    case 9:
      Encoder.enableEncoderOutputPositions2048();
      Serial.println("Selected 2048 steps per revolution. Change the position of the encoder shaft\n");
    break;
    default:
      Serial.println("ERROR CODE\n");
      printInfo();
    break;
  }
}
// Обработчик прерываний от энкодера
void isWheel() {
  encoder_state = digitalRead(PHASE_B) | digitalRead(PHASE_A) << 1;
  if (encoder_state == 0x00) { // Сброс
    encoder_reset = true;
  }
  if (encoder_reset && encoder_state == 0x03) {
    if (previous_encoder_state == 0x01) { // Лево
      turnLeft = true;
    }
    if (previous_encoder_state == 0x02) { // Право
      turnRight = true;
    }
    encoder_reset = false;
    encoder_flag = true;
  }
  previous_encoder_state = encoder_state;
}
// Действия при повороте энкодера
void wheelEvent() {
  if (encoder_flag) {
    if (turnLeft) {
      enc_step_count--; // Уменьшение значения счетчика
      turnLeft = false;
    }
    else if (turnRight) {
      enc_step_count++; // Увеличение значения счетчика
      turnRight = false;
    }
    Serial.println(enc_step_count); // Вывод текущего значения счетчика в "Монитор порта"
    encoder_flag = false;
  }
}
