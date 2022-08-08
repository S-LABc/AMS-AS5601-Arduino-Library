/*
 * ChangeVolumeOS_STM32F1_EXTI
 *  
 * Демонстрация управления уровнем громкости в ОС
 * при помощи датчика AS5601 и микроконтроллера STM32F1
 * с использованием внешних прерываний EXTI
 * 
 * Примечания:
 * 1. Скетч использует ядро Arduino_STM32 и библиотеку USBComposite_stm32f1
 * 2. Скетч настраивает микрокотроллер как мутимедийную клавиатуру
 *    используя класс USB HID Consumer
 * 3. Используются внешние прерывания EXTI_10(PB10) и EXTI_11(PB11)
 * 
 * Подключение:
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * A     -> PB10
 * B     -> PB11
 * P     -> PB0
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
 * https://github.com/rogerclarkmelbourne/Arduino_STM32
 * https://github.com/arpruss/USBComposite_stm32f1
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеки
#include <AMS_AS5601.h>
#include <USBComposite.h>

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Коды кнопок изменения громкости
#define VOLUME_UP        0xE9
#define VOLUME_DOWN      0xEA
#define VOLUME_MUTE      0xE2
/* не используются в этом скетче, но можете задействовать самостоятельно */
#define MEDIA_PLAY_PAUSE 0xCD
#define MEDIA_NEXT       0xB5
#define MEDIA_PREV       0xB6

// Контакты подключения энкодера
#define PHASE_A PB10 // Фаза A
#define PHASE_B PB11 // Фаза B
#define BTN_ENC PB0  // Кнопка P

// Состояние кнопки энкодера
bool btn_enc_flag = false;
// Для обработчика прерываний и направления вращения
volatile bool encoder_reset = false, encoder_flag = false, turnLeft = false, turnRight = false;
volatile byte encoder_state = 0x00, previous_encoder_state = 0x00;

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

// Настройка USB как мультимедийной клавиатуры
USBHID HID;
const uint8_t reportDescription[] = {
   HID_CONSUMER_REPORT_DESCRIPTOR()
};
HIDConsumer MediaKeyboard(HID);

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
  HID.begin(reportDescription, sizeof(reportDescription));
  // Ждем успешной инициализации USB
  while(!USBComposite);
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
      MediaKeyboard.press(VOLUME_DOWN);
      turnLeft = false;
    }
    else if (turnRight) {
      MediaKeyboard.press(VOLUME_UP);
      turnRight = false;
    }
    MediaKeyboard.release();
    encoder_flag = false;
  }
}

// Действия клавиатуры при управлении кнопкой
void btnEvent() {
  if (!btn_enc_flag && digitalRead(BTN_ENC) == HIGH) {
    MediaKeyboard.press(VOLUME_MUTE);
    MediaKeyboard.release();
    btn_enc_flag = true;
  }
  if (btn_enc_flag && digitalRead(BTN_ENC) == LOW) {
    btn_enc_flag = false;
  }
}
