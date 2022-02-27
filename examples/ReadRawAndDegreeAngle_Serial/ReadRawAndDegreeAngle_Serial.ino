/*
 * ReadRawAndDegreeAngle_Serial
 * 
 * Демонстрация вывода значений угла в градусах от энкодера AS5601 в "Монитор порта"
 * 
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
 * 4. Менять положение магнита
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

  // Пока не подключен датчик
  while (!Encoder.isConnected()) {
    // Выводим сообщение об отсутствии датчика
    Serial.println("AS5601 not detected!");
    delay(1000);
  }
  // Выводим сообщение о наличии датчика
  Serial.println("AS5601 detected!");

  // Пока датчик не обнаружил магнит
  while (!Encoder.isMagnetDetected()) {
    // Выводим сообщение об отсутствии магнита
    Serial.println("Magnet not detected!");
    delay(1000);
  }
  // Выводим сообщение о наличии магнита
  Serial.println("Magnet detected!");
}

void loop() {
  // Получаем значения АЦП для полного круга
  uint16_t raw = Encoder.getRawAngle();
  
  // Выводим "сырые" значения (от 0 до 4095)
  Serial.print("Raw Angle: ");
  Serial.println(raw);

  // Выводим значения в градусах (от 0 до 360)
  Serial.print("Degree Angle: ");
  Serial.println(raw * 0.08789); // 360/4096=0,087890625, 5 знаков после точки для АЦП 12 бит достаточно

  // Разделение и задержка
  Serial.println();
  delay(100);
}
