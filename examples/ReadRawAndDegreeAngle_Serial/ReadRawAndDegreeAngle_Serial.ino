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
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5601.h>

// Создаем объект Encoder с указанием ссылки на объект Wire
AS5601 Encoder(&Wire);

void setup() {
  Serial.begin(115200);

  // Запускаем соединение
  Encoder.begin();
  // Настраиваем шину I2C на 400кГц
  Encoder.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Encoder.setClock(AS5601_I2C_CLOCK_100KHZ); // 100кГц
  //Encoder.setClock(AS5601_I2C_CLOCK_1MHZ); // 1МГц
  //Encoder.setClock(725000); // Пользовательское значение 725кГц

  // Пока не подключен датчик
  while (!Encoder.isConnected()) {
    // Выводим сообщение об отсутствии датчика
    Serial.println("AS5601 не обнаружен!");
    Serial.println("Проверьте I2C шину");
    delay(1000);
  }
  // Выводим сообщение о наличии датчика
  Serial.println("AS5601 обнаружен!");

  // Пока датчик не обнаружил магнит
  while (!Encoder.isMagnetDetected()) {
    // Выводим сообщение об отсутствии магнита
    Serial.println("Магнит не обнаружен!");
    Serial.println("Возможно он слабый");
    delay(1000);
  }
  // Выводим сообщение о наличии магнита
  Serial.println("Магнит обнаружен!");
}

void loop() {
  // Получаем значения АЦП для полного круга
  uint16_t raw = Encoder.getRawAngle();
  
  // Выводим "сырые" значения (от 0 до 4095)
  Serial.print("Угол в АЦП: ");
  Serial.println(raw);

  // Выводим значения в градусах (от 0 до 360)
  Serial.print("Угол в градусах: ");
  Serial.println(raw * 0.08789); // 360/4096=0,087890625, 5 знаков после точки для АЦП 12 бит достаточно

  // Разделение и задержка
  Serial.println();
  delay(100);
}
