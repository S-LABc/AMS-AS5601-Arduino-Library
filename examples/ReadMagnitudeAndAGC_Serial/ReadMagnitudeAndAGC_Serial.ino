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
 * https://look.ams-osram.com/m/6dd0193ab2116bc6/original/AS5601-DS000395.pdf
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v1.2 / Скляр Роман S-LAB
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
  
  // Пока не подключен энкодер
  while (!Encoder.isConnected()) {
    // Выводим сообщение об отсутствии энкодера
    Serial.println("AS5601 не обнаружен!");
    Serial.println("Проверьте I2C шину");
    delay(1000);
  }
  // Выводим сообщение о наличии энкодера
  Serial.println("AS5601 обнаружен!");
}

void loop() {
  Serial.print("Магнитуда: ");
  Serial.println(Encoder.getMagnitude()); // Значение магнитуды
  Serial.print("Автоусиление: ");
  Serial.println(Encoder.getAutomaticGainControl()); // Значение автоусиления AGC

  /*
  // Или через ссылку
  word magn = 0;
  word agc = 0;
  Encoder.getMagnitude(magn);
  Encoder.getAutomaticGainControl(agc);
  Serial.print("Магнитуда: ");
  Serial.println(magn);
  Serial.print("Автоусиление: ");
  Serial.println(agc);
  */
  
  Serial.println();
  delay(50);
}
