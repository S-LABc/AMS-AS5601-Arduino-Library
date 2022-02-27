/*
 * ReadMagnitudeAndAGC_Serial
 * 
 * Демонстрация вывода значений магнитуды и автоусиления
 * от магнитного энкодера AS5601 в "Монитор порта"
 * 
 * Подключение энкодера:
 * AS5601   Board
 * 3V3   -> +3V3
 * 5V*   -> +3V3
 * GND   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключить энкодер согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Приближать и отдалять магнит
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

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  Serial.begin(115200);

  // Запускаем соединение
  Encoder.begin();
  // Настраиваем шину I2C на 400кГц
  Encoder.setClock();
  
  // Пока не подключен энкодер
  while (!Encoder.isConnected()) {
    // Выводим сообщение об отсутствии энкодера
    Serial.println("AS5601 not detected!");
    delay(1000);
  }
  // Выводим сообщение о наличии энкодера
  Serial.println("AS5601 detected!");
}

void loop() {
  Serial.print("Magnitude: ");
  Serial.println(Encoder.getMagnitude()); // Значение магнитуды
  Serial.print("Automatic Gain Control: ");
  Serial.println(Encoder.getAutomaticGainControl()); // Значение автоусиления AGC
  
  Serial.println();
  delay(50);
}
