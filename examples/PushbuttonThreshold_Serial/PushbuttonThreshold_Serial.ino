/*
 * PushbuttonThreshold_Serial
 * 
 * Демонстрация вычисления и настройки порога срабатывания кнопки
 * магнитного энкодера AS5601 с использованием "Монитор порта"
 * 
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * P     -> PB0 (D7)
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключите энкодер согласно распиновке
 * 2. Загрузите скетч в исходном виде
 * 3. Откройте "Монитор порта"
 * 4. Нажмите кнопку энкодера и запишите куда-нибудь полученное значение
 * 5. Отпустите кнопку энкодера и запишите куда-нибудь полученное значение
 * 6. Посчитайте(см. ниже) ЗНАЧЕНИЕ и подставьте его в Encoder.setPushbuttonThreshold(ЗНАЧЕНИЕ) в блоке setup()
 * 7. Закомментируйте строку Serial.println(Encoder.getAutomaticGainControl())
 * 8. Раскомментируйте строку btnEvents();
 * 9. Загрузите измененный скетч
 * 10. Откройте "Монитор порта" и нажимайте/отпускайте кнопку энкодера
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5601_DS000395_3-00.pdf
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5601.h>

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Контакт подключения кнопки энкодера STM32
#define BTN_ENC PB0
// Контакт подключения кнопки энкодера AVR
//#define BTN_ENC 7

// Для хранения состояния кнопки
boolean btn_enc_flag = false;

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  Serial.begin(115200);

  // Настройка вывода кнопки на вход БЕЗ ПОДТЯГИВАЮЩЕГО РЕЗИСТОРА
  pinMode(BTN_ENC, INPUT);

  // Запускаем соединение
  Encoder.begin();
  // Настраиваем шину I2C на 400кГц
  Encoder.setClock();
  /* 
   *  В моем ЧАСТНОМ СЛУЧАИ(у вас могут быть другие значения) получились такие расчеты:
   *    При нажатой кнопке Encoder.getAutomaticGainControl() вернул значение 53
   *    При отпущеной кнопке Encoder.getAutomaticGainControl() вернул значение 69
   *    Вычитаю из значения 69 значение 53, получаю 16
   *    Делю 16 на 2, плучаю 8
   *      (69 - 53) / 2 = 8
   *    Передаю результат 8 как аргумент методу Encoder.setPushbuttonThreshold(8)
   */
  Encoder.setPushbuttonThreshold(8);
}

void loop() {
  // Опрос кнопки
  //btnEvents();
  // Опредиление значений при нажатой и при отпущеной кнопке
  Serial.println(Encoder.getAutomaticGainControl());
}

// Обработчик кнопки
void btnEvents() {
  if (!btn_enc_flag && digitalRead(BTN_ENC) == HIGH) { // Нажата
    Serial.println("btn: DOWN");
    btn_enc_flag = true;
  }
  if (btn_enc_flag && digitalRead(BTN_ENC) == LOW) { // Отпущена
    Serial.println("btn: UP");
    btn_enc_flag = false;
  }
}
