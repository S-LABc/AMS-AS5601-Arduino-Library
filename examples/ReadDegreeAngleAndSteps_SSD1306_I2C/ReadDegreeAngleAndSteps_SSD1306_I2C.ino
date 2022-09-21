/*
 * ReadDegreeAngleAndSteps_SSD1306_I2C
 * 
 * Демонстрация вывода значений угла в градусах, счетчика шагов
 * от энкодера AS5601 на экран OLED_SSD1306_I2C
 * 
 * Используются внешние прерывания INT_0(D3) и INT_1(D2) для AVR,
 * и внешние прерывания EXTI_10(PB10) и EXTI_11(PB11) для STM32
 * 
 * Подключение энкодера:
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * A     -> PB10 (D3)
 * B     -> PB11 (D2)
 * P     -> PB0  (D7)
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Подключение дисплея:
 * SSD1306   Board
 * VCC    -> +3V3
 * GND    -> GND
 * SDA    -> SDA
 * SCL    -> SCL
 * 
 * Проверка:
 * 1. Подключить энкодер согласно распиновке
 * 2. Подключить дисплей согласно распиновке
 * 3. Загрузить скетч в плату
 * 4. Менять положение магнита
 * 5. Нажимать кнопку энкодера
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5601_DS000395_3-00.pdf
 * 
 * Зависимости:
 * https://github.com/adafruit/Adafruit_SSD1306
 * https://github.com/adafruit/Adafruit-GFX-Library
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеки
#include <AMS_AS5601.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Контакты подключения энкодера STM32
#define PHASE_A         PB10 // Фаза A
#define PHASE_B         PB11 // Фаза B
#define BTN_ENC         PB0  // Кнопка P
/*
// Контакты подключения энкодера AVR
#define PHASE_A         3 // Фаза A
#define PHASE_B         2 // Фаза B
#define BTN_ENC         7  // Кнопка P
*/
// Состояние кнопки энкодера
bool btn_enc_flag = false;
// Счетчик тиков (шагов) энкодера
int enc_step_count = 0;
// Для обработчика прерываний и направления вращения
volatile bool encoder_reset = false, encoder_flag = false, turnLeft = false, turnRight = false;
volatile byte encoder_state = 0x00, previous_encoder_state = 0x00;

// Ширина и высота дисплея
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
// Адрес дисплея: 128х32 - 0x3C, для 128х64 может быть 0x3D
#define SCREEN_ADDRESS 0x3C
// Контакт сброса (если используется)
#define OLED_RESET 4
// Создаем объект display по стандартному методу
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  // Запускаем дисплей
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // Если с датчиком порядок, настраивам дисплей для показаний угла
  displayInit();
  
  // Настройка контактов МК на вход
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);
  pinMode(BTN_ENC, INPUT);

  // Привязка прерываний на контакты от энкодера
  attachInterrupt(digitalPinToInterrupt(PHASE_A), isWheel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PHASE_B), isWheel, CHANGE);

  // Настройка порога срабатывания кнопки энкодера
  Encoder.setPushbuttonThreshold(42); // см пример PushbuttonThreshold_Serial
  // Настройка шагов на один оборот
  Encoder.enableOutputPositions256(); // см пример OutputPositions_Serial
  
  /*
   * Encoder.begin() и Encoder.setClock() не используются
   * Adafruit_SSD1306 делает все это сама
   */
}

void loop() {
  // Выводим новые данные на экран
  createText(Encoder.getRawAngle());
  // Опрос кнопки и энкодера, для совершения нужных действий
  wheelEvent();
  btnEvent();
}

// Подготовка картики с новыми значениями
void createText(uint16_t angle_raw) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ANG:");
  display.println(angle_raw * 0.08789); // Угол в градусах. 360/4096=0,087890625, 5 знаков после точки для АЦП 12 бит достаточно
  display.print("NUM:");
  display.print(enc_step_count); // Количество шагов
  display.display();
}
// Очистка старого ИЗО, установка шрифта, цвета, курсора
void displayInit() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
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
    encoder_flag = false;
  }
}
// Действия при управлении кнопкой
void btnEvent() {
  if (!btn_enc_flag && digitalRead(BTN_ENC) == HIGH) { // Нажата
    enc_step_count = 0; // Сброс значения счетчика
    btn_enc_flag = true;
  }
  if (btn_enc_flag && digitalRead(BTN_ENC) == LOW) { // Отпущена
    btn_enc_flag = false;
  }
}
