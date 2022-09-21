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
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5601.h>

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
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Encoder.setClock(AS5601_I2C_CLOCK_100KHZ); // 100кГц
  //Encoder.setClock(AS5601_I2C_CLOCK_1MHZ); // 1МГц
  //Encoder.setClock(725000); // Пользовательское значение 725кГц
  /* 
   *  В моем ЧАСТНОМ СЛУЧАЕ(у вас могут быть другие значения) получились такие расчеты:
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
    Serial.println("Кнопка НАЖАТА");
    btn_enc_flag = true;
  }
  if (btn_enc_flag && digitalRead(BTN_ENC) == LOW) { // Отпущена
    Serial.println("Кнопка ОТПУЩЕНА");
    btn_enc_flag = false;
  }
}
