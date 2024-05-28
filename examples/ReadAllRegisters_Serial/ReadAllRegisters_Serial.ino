/*
 * ReadAllRegisters_Serial
 * 
 * Демонстрация вывода значений всей карты памяти от энкодера AS5601 в "Монитор порта"
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
 * Соответствие массива и регистров:
 * reg_map[0] - ZMCO (0x00)
 * reg_map[1] - ZPOS_H (0x01)
 * reg_map[2] - ZPOS_L (0x02)
 * reg_map[3] - CONF_H (0x07)
 * reg_map[4] - CONF_L (0x08)
 * reg_map[5] - ABN (0x09)
 * reg_map[6] - PUSHTHR (0x0A)
 * reg_map[7] - STATUS (0x0B)
 * reg_map[8] - RAW_ANGLE_H (0x0C)
 * reg_map[9] - RAW_ANGLE_L (0x0D)
 * reg_map[10] - ANGLE_H (0x0E)
 * reg_map[11] - ANGLE_L (0x0F)
 * reg_map[12] - AGC (0x1A)
 * reg_map[13] - MAGNITUDE_H (0x1B)
 * reg_map[14] - MAGNITUDE_L (0x1C)
 * reg_map[15] - BURN (0xFF)
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/6dd0193ab2116bc6/original/AS5601-DS000395.pdf
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5601.h>

const String desc_map[AS5601_REGISTER_MAP_SIZE] = {
  "ZMCO (0x00)",
  "ZPOS_H (0x01)",
  "ZPOS_L (0x02)",
  "CONF_H (0x07)",
  "CONF_L (0x08)",
  "ABN (0x09)",
  "PUSHTHR (0x0A)",
  "STATUS (0x0B)",
  "RAW_ANGLE_H (0x0C)",
  "RAW_ANGLE_L (0x0D)",
  "ANGLE_H (0x0E)",
  "ANGLE_L (0x0F)",
  "AGC (0x1A)",
  "MAGNITUDE_H (0x1B)",
  "MAGNITUDE_L (0x1C)",
  "BURN (0xFF)"
};

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
  byte reg_map[AS5601_REGISTER_MAP_SIZE]; // Не должен быть меньше 16, AS5601_REGISTER_MAP_SIZE = 16
  Encoder.getAllRegisters(reg_map, sizeof(reg_map)); // Получаем данные в массив
  for (int i = 0; i < sizeof(reg_map); i++) {
    Serial.print(desc_map[i] + " "); // Вывод названий регистров в "Монитор порта"
    Serial.println(reg_map[i], HEX); // Вывод значений регистров в "Монитор порта"
  }

  // Разделение и задержка
  Serial.println();
  delay(500);
}
