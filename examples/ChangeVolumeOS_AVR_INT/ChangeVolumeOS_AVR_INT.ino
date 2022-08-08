/*
 * ChangeVolumeOS_AVR_INT
 *  
 * Демонстрация управления уровнем громкости в ОС
 * при помощи энкодера AS5601 и микроконтроллера AVR
 * с поддержкой USB (Leonardo, Pro Micro, и т.д.)
 * с использованием внешних прерываний INT
 * 
 * Примечания:
 * 1. Скетч использует стандартное ядро и библиотеку HID-Project
 * 2. Скетч настраивает микрокотроллер как мутимедийную клавиатуру
 *    используя класс USB HID Consumer
 * 3. Используются внешние прерывания INT_0(D3) и INT_1(D2)
 * 
 * Подключение:
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * A     -> D3
 * B     -> D2
 * P     -> D7
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключить энкодер согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Менять положение магнита
 * 4. Наблюдать изменение громкости в ОС
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5601_DS000395_3-00.pdf
 * 
 * Зависимости:
 * https://github.com/NicoHood/HID
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеки
#include <AMS_AS5601.h>
#include <HID-Project.h>

// Контакты подключения энкодера
#define PHASE_A 3 // Фаза A
#define PHASE_B 2 // Фаза B
#define BTN_ENC 7 // Кнопка P

// Состояние кнопки энкодера
bool btn_enc_flag = false;
// Для обработчика прерываний и направления вращения
volatile bool encoder_reset = false, encoder_flag = false, turnLeft = false, turnRight = false;
volatile byte encoder_state = 0x00, previous_encoder_state = 0x00;

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  // Настройка контактов МК на вход
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);
  pinMode(BTN_ENC, INPUT);

  // Привязка прерываний на контакты от энкодера
  attachInterrupt(digitalPinToInterrupt(PHASE_A), isWheel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PHASE_B), isWheel, CHANGE);
  
  // Запускаем соединение
  Encoder.begin();
  // Настраиваем шину I2C на 400кГц
  Encoder.setClock();
  // Настройка порога срабатывания кнопки энкодера
  Encoder.setPushbuttonThreshold(42); // см пример PushbuttonThreshold_Serial
  // Настройка шагов на один оборот
  Encoder.enableOutputPositions128(); // см пример OutputPositions_Serial

  // Запукаем клавиатуру
  Consumer.begin();
}

void loop() {
  // Обработка событий от кнопки и энкодера
  wheelEvent();
  btnEvent();
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

// Действия клавиатуры при повороте энкодера
void wheelEvent() {
  if (encoder_flag) {
    if (turnLeft) {
      Consumer.write(MEDIA_VOLUME_DOWN);
      turnLeft = false;
    }
    else if (turnRight) {
      Consumer.write(MEDIA_VOLUME_UP);
      turnRight = false;
    }
    encoder_flag = false;
  }
}

// Действия клавиатуры при управлении кнопкой
void btnEvent() {
  if (!btn_enc_flag && digitalRead(BTN_ENC) == HIGH) {
    Consumer.write(MEDIA_VOLUME_MUTE);
    btn_enc_flag = true;
  }
  if (btn_enc_flag && digitalRead(BTN_ENC) == LOW) {
    btn_enc_flag = false;
  }
}
