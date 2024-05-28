/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с бесконтактным магнитным энкодером
 * AS5601 от компании AMS https://ams.com/ams-start
 * 
 * Документация к датчику:
 ** https://look.ams-osram.com/m/6dd0193ab2116bc6/original/AS5601-DS000395.pdf
 ** https://ams-osram.com/products/sensors/position-sensors/ams-as5601-position-sensor
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v1.5 / License MIT / Скляр Роман S-LAB
 */

#include "AMS_AS5601.h"

// ########## CONSTRUCTOR ##########
/*
 * @brief: использовать только интерфейс I2C
 * @param _twi: доступ к методам объекта Wire
 */
AS5601::AS5601(TwoWire *_twi) : _wire_(_twi ? _twi : &Wire) {
  // Ничего
}
 
// ########## PROTECTED ##########
/* 
 * @brief: передать один байт адреса регистра с которого будет идти чтение
 */
void AS5601::AS_SendFirstRegister(uint8_t _reg_addr) {
  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  // Отправить байт регистра
  _wire_->write(_reg_addr);
  // Завершить соединение
  _wire_->endTransmission();
}
/* 
 * @brief: запросить один байт данных из буфера
 * @return: значение байта из регистра, который был запрошен ранее
 * @note: использовать для одиночного регистра, например 0x1A
 */
uint8_t AS5601::AS_RequestSingleRegister(void) {
  uint8_t single_byte = 0;
  
  // Запросить байт данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)1);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    single_byte = _wire_->read();
  }
  // Завершить соединение
  _wire_->endTransmission();

  return single_byte;
}
/* 
 * @brief: запросить два байта данных из буфера
 * @return: значения двух байтов из регистров, которые были запрошены ранее в виде uint16_t
 * @note: использовать для парных регистров, например 0x0C 0x0D
 */
uint16_t AS5601::AS_RequestPairRegisters(void) {
  uint8_t low_byte = 0;
  uint8_t high_byte = 0;
  
  // Запросить два байта данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)2);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    high_byte = _wire_->read();
    low_byte = _wire_->read();
  }
  // Завершить соединение
  _wire_->endTransmission();
  
  return ((high_byte << 8) | low_byte);
}
/*
 * @brief: записать значение размером один байт в произвольный регистр размером один байт
 * @param _reg: один байт адреса регистра
 * @param _payload: один байт полезных данных
 */
void AS5601::AS_WriteOneByte(uint8_t _reg, uint8_t _payload) {
  // Начать передачу по адресу для прередачи байта данных в регистр
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(_reg);
  _wire_->write(_payload);
  // Завершить соединение
  _wire_->endTransmission();
}
/*
 * @brief: записать значение размером два байта в произвольный регистр размером два байта
 * @param _low_register: младший байт регистра
 * @param _high_register: старший байт регистра
 * @param _payload: два байта полезных данных
 */
void AS5601::AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload) {
  // Начать передачу по адресу для прередачи старшего байта данных в старший регистр
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(_high_register);
  _wire_->write(_payload >> 8);
  // Завершить соединение
  _wire_->endTransmission();
  // Начать передачу по адресу для передачи младшего байта данных в младший регистр
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(_low_register);
  _wire_->write(_payload & 0xFF);
  // Завершить соединение
  _wire_->endTransmission();
}

// ########## PUBLIC ##########
/* 
 * @brief: вызов метода Wire.begin()
 * @note: использовать, если действие не было выполнено ранее
 */
void AS5601::begin(void) {
  _wire_->begin();
}
/* 
 * @brief: вызов метода Wire.begin(SDA, SCL) с указанием выводов
 * @param _sda_pin: пользовательский контакт SDA
 * @param _scl_pin: пользовательский контакт SCL
 * @note: использовать, если действие не было выполнено ранее.
 *   Применимо для платформ на базе ESP8266 и ESP32
 */
#if defined(ESP8266) || defined(ESP32)
void AS5601::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->begin(_sda_pin, _scl_pin);
}
#endif
/* 
 * @brief: вызов методов Wire.setSDA(SDA) Wire.setSCL(SCL) Wire.begin()
 * @param _sda_pin: пользовательский контакт SDA
 * @param _scl_pin: пользовательский контакт SCL
 * @note: использовать, если действие не было выполнено ранее.
 *   Применимо для платформы STM32
 */
#if defined(ARDUINO_ARCH_STM32)
void AS5601::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->setSDA(_sda_pin);
  _wire_->setSCL(_scl_pin);
  _wire_->begin();
}
#endif
/* 
 * @brief: настройка частоты шины I2C
 * @note: использовать, если частота шины меняется из-за разных устройств. по умолчанию 400кГц
 */
void AS5601::setClock(uint32_t _freq_hz) {
  _wire_->setClock(_freq_hz);
}
/*
 * @brief: загружает данные из энергонезависимой памяти датчика в регистры ZPOS(11:0), CONF(13:0)
 *   если были установлены какие-либо значения в эти регистры то, они будут заменены значениями из энергонезависимой памяти
 * @note: назначение каждой команды не описано в документации, порядок команд описан в -
 *   Zero Position and Resolution Programming Procedure (Step 7)
 */
void AS5601::loadSavedValues(void) {
  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(AS5601_BURN_REG);
  // Отправить 0x01
  _wire_->write(AS5601_CMD_BURN_LOAD_OTP_CONTENT_0);
  // Завершить соединение
  _wire_->endTransmission();
  
  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(AS5601_BURN_REG);
  // Отправить 0x11
  _wire_->write(AS5601_CMD_BURN_LOAD_OTP_CONTENT_1);
  // Завершить соединение
  _wire_->endTransmission();
  
  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  _wire_->write(AS5601_BURN_REG);
  // Отправить 0x10
  _wire_->write(AS5601_CMD_BURN_LOAD_OTP_CONTENT_2);
  // Завершить соединение
  _wire_->endTransmission();
}
/*
 * @brief: узнать подключен ли датчик к линии I2C
 * @note: алгоритм стандортного поиска устройств на шина I2C
 * @return:
 *  0 AS5601_DEFAULT_REPORT_ERROR - не подключен
 *  1 AS5601_DEFAULT_REPORT_OK - подключен
 */
bool AS5601::isConnected(void) {
  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  return (!_wire_->endTransmission(AS5601_I2C_ADDRESS)) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: получить значения всех регистров датчика
 * @note: 
 */
bool AS5601::getAllRegisters(byte *_registers, byte _array_size) {
  // Проверка на размер массива
  if (_array_size < AS5601_REGISTER_MAP_SIZE) {
    return AS5601_DEFAULT_REPORT_ERROR;
  }

  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  // Отправить адрес регистра 0x00
  _wire_->write(AS5601_CONFIG_REG_ZMCO);
  // Завершить соединение
  _wire_->endTransmission();
  // Запросить 3 байта данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)3);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    _registers[0] = _wire_->read(); // AS5601_CONFIG_REG_ZMCO (0x00)
    _registers[1] = _wire_->read(); // AS5601_CONFIG_REG_ZPOS_H (0x01)
    _registers[2] = _wire_->read(); // AS5601_CONFIG_REG_ZPOS_L (0x02)
  }
  // Завершить соединение
  _wire_->endTransmission();

  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  // Отправить адрес регистра 0x07
  _wire_->write(AS5601_CONFIG_REG_CONF_H);
  // Завершить соединение
  _wire_->endTransmission();
  // Запросить 9 байт данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)9);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    _registers[3] = _wire_->read(); // AS5601_CONFIG_REG_CONF_H (0x07)
    _registers[4] = _wire_->read(); // AS5601_CONFIG_REG_CONF_L (0x08)
    _registers[5] = _wire_->read(); // AS5601_CONFIG_REG_ABN (0x09)
    _registers[6] = _wire_->read(); // AS5601_CONFIG_REG_PUSHTHR (0x0A)
    _registers[7] = _wire_->read(); // AS5601_STATUS_REG (0x0B)
    _registers[8] = _wire_->read(); // AS5601_OUT_REG_RAW_ANGLE_H (0x0C)
    _registers[9] = _wire_->read(); // AS5601_OUT_REG_RAW_ANGLE_L (0x0D)
    _registers[10] = _wire_->read(); // AS5601_OUT_REG_ANGLE_H (0x0E)
    _registers[11] = _wire_->read(); // AS5601_OUT_REG_ANGLE_L (0x0F)
  }
  // Завершить соединение
  _wire_->endTransmission();

  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  // Отправить адрес регистра 0x1A
  _wire_->write(AS5601_STATUS_REG_AGC);
  // Завершить соединение
  _wire_->endTransmission();
  // Запросить 3 байта данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)3);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    _registers[12] = _wire_->read(); // AS5601_STATUS_REG_AGC (0x1A)
    _registers[13] = _wire_->read(); // AS5601_STATUS_REG_MAGNITUDE_H (0x1B)
    _registers[14] = _wire_->read(); // AS5601_STATUS_REG_MAGNITUDE_L (0x1C)
  }
  // Завершить соединение
  _wire_->endTransmission();

  // Начать передачу по адресу
  _wire_->beginTransmission(AS5601_I2C_ADDRESS);
  // Отправить адрес регистра 0xFF
  _wire_->write(AS5601_BURN_REG);
  // Завершить соединение
  _wire_->endTransmission();
  // Запросить 1 байт данных по адресу
  _wire_->requestFrom(AS5601_I2C_ADDRESS, (uint8_t)1);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    _registers[15] = _wire_->read(); // AS5601_BURN_REG (0xFF)
  }
  // Завершить соединение
  _wire_->endTransmission();

  return AS5601_DEFAULT_REPORT_OK;
}

/*********************************/
/**** CONFIGURATION REGISTERS ****/
/*********************************/
/*
 * @brief: получить количество записей значения в ZPOS из ZMCO(1:0)
 * @return:
 *  0 - заводское значение
 *  1 - в ZPOS записано один раз
 *  2 - в ZPOS записано два раза
 *  3 - в ZPOS записано три раза
 */
byte AS5601::getBurnPositionsCount(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ZMCO);
  return AS_RequestSingleRegister();
}
/*
 * @brief: получить количество записей значения в ZPOS из ZMCO(1:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - заводское значение
 *  1 - в ZPOS записано один раз
 *  2 - в ZPOS записано два раза
 *  3 - в ZPOS записано три раза
 */
void AS5601::getBurnPositionsCount(byte &_burn_positions_count) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ZMCO);
  _burn_positions_count = AS_RequestSingleRegister();
}
/* 
 * @brief: получить значение начального положения ZPOS(11:0) (начальный угол)
 * @return:
 *  0 - 4095
 */
word AS5601::getZeroPosition(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ZPOS_H);
  return AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение начального положения ZPOS(11:0) (начальный угол)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 4095
 */
void AS5601::getZeroPosition(word &_zero_position) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ZPOS_H);
  _zero_position = AS_RequestPairRegisters();
}
/* 
 * brief: установить новое начальное положение ZPOS(11:0)
 * @param _start_angle:
 *  0 - 4095
 */
void AS5601::setZeroPosition(word _zero_position) {
  AS_WriteTwoBytes(AS5601_CONFIG_REG_ZPOS_L, AS5601_CONFIG_REG_ZPOS_H, _zero_position);
}
/* 
 * @brief: установить новое начальное положение ZPOS(11:0) с подтверждением
 * @param _zero_position:
 *  0 - 4095
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5601_DEFAULT_REPORT_OK - новое значение успешно установлено
 */
bool AS5601::setZeroPositionVerify(word _zero_position) {
  setZeroPosition(_zero_position);
  return (getZeroPosition() == _zero_position) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита
 * @note: получает и отправляет значение полученное от метода getRawAngle
 */
void AS5601::setZeroPositionViaRawAngle(void) {
  word raw_angle = getRawAngle();
  AS_WriteTwoBytes(AS5601_CONFIG_REG_ZPOS_L, AS5601_CONFIG_REG_ZPOS_H, raw_angle);
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита с подтверждением
 * @note: получает и отправляет значение полученное от метода getRawAngle
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5601_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5601::setZeroPositionViaRawAngleVerify(void) {
  word raw_angle = getRawAngle();
  AS_WriteTwoBytes(AS5601_CONFIG_REG_ZPOS_L, AS5601_CONFIG_REG_ZPOS_H, raw_angle);
  return (getZeroPosition() == raw_angle) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита
 * @note: получает и отправляет значение полученное от метода getAngle (10LBS)
 */
void AS5601::setZeroPositionViaAngle10LSB(void) {
  word angle = getAngle();
  AS_WriteTwoBytes(AS5601_CONFIG_REG_ZPOS_L, AS5601_CONFIG_REG_ZPOS_H, angle);
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита с подтверждением
 * @note: получает и отправляет значение полученное от метода getAngle (10LBS)
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5601_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5601::setZeroPositionViaAngle10LSBVerify(void) {
  word angle = getAngle();
  AS_WriteTwoBytes(AS5601_CONFIG_REG_ZPOS_L, AS5601_CONFIG_REG_ZPOS_H, angle);
  return (getZeroPosition() == angle) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: получить значение конфигураций CONF(13:0)
 * @return: целое шестнадцатиричное число
 */
word AS5601::getRawConfigurationValue(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение конфигураций CONF(13:0)
 * @note: метод работает через ссылку
 * @return: целое шестнадцатиричное число
 */
void AS5601::getRawConfigurationValue(word &_configuration_value) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  _configuration_value = AS_RequestPairRegisters();
}
/* 
 * @brief: установить новое значение конфигураций CONF(13:0)
 * @param _configuration_value: новое значение конфигураций
 */
void AS5601::setRawConfigurationValue(word _configuration_value) {
  AS_WriteTwoBytes(AS5601_CONFIG_REG_CONF_L, AS5601_CONFIG_REG_CONF_H, _configuration_value);
}
/* 
 * @brief: установить новое значение конфигураций CONF(13:0) с подтверждением
 * @param _configuration_value: новое значение конфигураций
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5601_DEFAULT_REPORT_OK - новое значение успешно установлено
 */
bool AS5601::setRawConfigurationValueVerify(word _configuration_value) {
  setRawConfigurationValue(_configuration_value);
  return (getRawConfigurationValue() == _configuration_value) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: получить значение текущего режима питания PM(1:0)
 * @return: 
 *  AS5601_NOM_POWER_MODE
 *  AS5601_LOW_POWER_MODE_1
 *  AS5601_LOW_POWER_MODE_2
 *  AS5601_LOW_POWER_MODE_3
 */
AS5601PowerModes AS5601::getPowerMode(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  return (AS5601PowerModes)(AS_RequestSingleRegister() & 0x03); // 0x03=0b00000011
}
/*
 * @brief: получить значение текущего режима питания PM(1:0)
 * @note: метод работает через ссылку
 * @return: 
 *  AS5601_NOM_POWER_MODE
 *  AS5601_LOW_POWER_MODE_1
 *  AS5601_LOW_POWER_MODE_2
 *  AS5601_LOW_POWER_MODE_3
 */
void AS5601::getPowerMode(AS5601PowerModes &_power_mode) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  _power_mode = (AS5601PowerModes)(AS_RequestSingleRegister() & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новое значение режима питания PM(1:0)
 * @param _power_mode:
 *  AS5601_NOM_POWER_MODE
 *  AS5601_LOW_POWER_MODE_1
 *  AS5601_LOW_POWER_MODE_2
 *  AS5601_LOW_POWER_MODE_3
 */
void AS5601::setPowerMode(AS5601PowerModes _power_mode) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_L, conf_l_raw |= _power_mode);
}
/*
 * @brief: установить новое значение режима питания PM(1:0) с подтверждением
 * @param _power_mode:
 *  AS5601_NOM_POWER_MODE
 *  AS5601_LOW_POWER_MODE_1
 *  AS5601_LOW_POWER_MODE_2
 *  AS5601_LOW_POWER_MODE_3
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить новый режим 
 *  AS5601_DEFAULT_REPORT_OK - новый режим установлен
 */
bool AS5601::setPowerModeVerify(AS5601PowerModes _power_mode) {
  setPowerMode(_power_mode);
  return (getPowerMode() == _power_mode) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить нормальный режим питания
 */
void AS5601::enableNomPowerMode(void) {
  setPowerMode(AS5601_NOM_POWER_MODE);
}
/*
 * @brief: включить нормальный режим питания с подтверждением
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5601_DEFAULT_REPORT_OK - режим включиен
 */
bool AS5601::enableNomPowerModeVerify(void) {
  return setPowerModeVerify(AS5601_NOM_POWER_MODE);
}
/*
 * @brief: включить режим питания 1
 */
void AS5601::enableLowPowerMode1(void) {
  setPowerMode(AS5601_LOW_POWER_MODE_1);
}
/*
 * @brief: включить режим питания 1 с подтверждением
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5601_DEFAULT_REPORT_OK - режим включен
 */
bool AS5601::enableLowPowerMode1Verify(void) {
  return setPowerModeVerify(AS5601_LOW_POWER_MODE_1);
}
/*
 * @brief: включить режим питания 2
 */
void AS5601::enableLowPowerMode2(void) {
  setPowerMode(AS5601_LOW_POWER_MODE_2);
}
/*
 * @brief: включить режим питания 2 с подтверждением
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5601_DEFAULT_REPORT_OK - режим включен
 */
bool AS5601::enableLowPowerMode2Verify(void) {
  return setPowerModeVerify(AS5601_LOW_POWER_MODE_2);
}
/*
 * @brief: включить режим питания 3
 */
void AS5601::enableLowPowerMode3(void) {
  setPowerMode(AS5601_LOW_POWER_MODE_3);
}
/*
 * @brief: включить режим питания 3 с подтверждением
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5601_DEFAULT_REPORT_OK - режим включен
 */
bool AS5601::enableLowPowerMode3Verify(void) {
  return setPowerModeVerify(AS5601_LOW_POWER_MODE_3);
}
/*
 * @brief: получить установленное значение гистерезиса HYST(1:0)
 * @return:
 *  AS5601_HYSTERESIS_OFF
 *  AS5601_HYSTERESIS_1_LSB
 *  AS5601_HYSTERESIS_2_LSB
 *  AS5601_HYSTERESIS_3_LSB
 */
AS5601Hysteresis AS5601::getHysteresis(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  return (AS5601Hysteresis)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_HYST_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: получить установленное значение гистерезиса HYST(1:0)
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_HYSTERESIS_OFF
 *  AS5601_HYSTERESIS_1_LSB
 *  AS5601_HYSTERESIS_2_LSB
 *  AS5601_HYSTERESIS_3_LSB
 */
void AS5601::getHysteresis(AS5601Hysteresis &_hysteresis) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  _hysteresis = (AS5601Hysteresis)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_HYST_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новые значения гистерезиса HYST(1:0)
 * @param _hysteresis:
 *  AS5601_HYSTERESIS_OFF
 *  AS5601_HYSTERESIS_1_LSB
 *  AS5601_HYSTERESIS_2_LSB
 *  AS5601_HYSTERESIS_3_LSB
 */
void AS5601::setHysteresis(AS5601Hysteresis _hysteresis) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_L, conf_l_raw |= (_hysteresis << AS5601_CONF_BIT_HYST_0));
}
/*
 * @brief: установить новые значения гистерезиса HYST(1:0) с подтверждением
 * @param _hysteresis:
 *  AS5601_HYSTERESIS_OFF
 *  AS5601_HYSTERESIS_1_LSB
 *  AS5601_HYSTERESIS_2_LSB
 *  AS5601_HYSTERESIS_3_LSB
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить новое значение
 *  AS5601_DEFAULT_REPORT_OK - новое значение установлено
 */
bool AS5601::setHysteresisVerify(AS5601Hysteresis _hysteresis) {
  setHysteresis(_hysteresis);
  return (getHysteresis() == _hysteresis) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: выключить гистерезис
 */
void AS5601::disableHysteresis(void) {
  setHysteresis(AS5601_HYSTERESIS_OFF);
}
/*
 * @brief: выключить гистерезис с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5601_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5601::disableHysteresisVerify(void) {
  return setHysteresisVerify(AS5601_HYSTERESIS_OFF);
}
/*
 * @brief: включить гистерезис на 1 LS
 */
void AS5601::enableHysteresis1LSB(void) {
  setHysteresis(AS5601_HYSTERESIS_1_LSB);
}
/*
 * @brief: включить гистерезис на 1 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5601_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5601::enableHysteresis1LSBVerify(void) {
  return setHysteresisVerify(AS5601_HYSTERESIS_1_LSB);
}
/*
 * @brief: включить гистерезис на 2 LSB
 */
void AS5601::enableHysteresis2LSB(void) {
  setHysteresis(AS5601_HYSTERESIS_2_LSB);
}
/*
 * @brief: включить гистерезис на 2 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5601_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5601::enableHysteresis2LSBVerify(void) {
  return setHysteresisVerify(AS5601_HYSTERESIS_2_LSB);
}
/*
 * @brief: включить гистерезис на 3 LSB
 */
void AS5601::enableHysteresis3LSB(void) {
  setHysteresis(AS5601_HYSTERESIS_3_LSB);
}
/*
 * @brief: включить гистерезис на 3 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5601_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5601::enableHysteresis3LSBVerify(void) {
  return setHysteresisVerify(AS5601_HYSTERESIS_3_LSB);
}
/*
 * @brief: получить значение коэффициента медленной фильтрации SF(1:0)
 * @return:
 *  AS5601_SLOW_FILTER_16X
 *  AS5601_SLOW_FILTER_8X
 *  AS5601_SLOW_FILTER_4X
 *  AS5601_SLOW_FILTER_2X
 */
AS5601SlowFilter AS5601::getSlowFilter(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return (AS5601SlowFilter)(AS_RequestSingleRegister() & 0x03); // 0x03=0b00000011
}
/*
 * @brief: получить значение коэффициента медленной фильтрации SF(1:0)
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_SLOW_FILTER_16X
 *  AS5601_SLOW_FILTER_8X
 *  AS5601_SLOW_FILTER_4X
 *  AS5601_SLOW_FILTER_2X
 */
void AS5601::getSlowFilter(AS5601SlowFilter &_slow_filter) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  _slow_filter = (AS5601SlowFilter)(AS_RequestSingleRegister() & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новое значение коэффициента медленной фильтрации SF(1:0)
 * @param _slow_filter:
 *  AS5601_SLOW_FILTER_16X
 *  AS5601_SLOW_FILTER_8X
 *  AS5601_SLOW_FILTER_4X
 *  AS5601_SLOW_FILTER_2X
 */
void AS5601::setSlowFilter(AS5601SlowFilter _slow_filter) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_H, conf_h_raw |= _slow_filter);
}
/*
 * @brief: установить новое значение коэффициента медленной фильтрации SF(1:0) с подтверждением
 * @param _slow_filter:
 *  AS5601_SLOW_FILTER_16X
 *  AS5601_SLOW_FILTER_8X
 *  AS5601_SLOW_FILTER_4X
 *  AS5601_SLOW_FILTER_2X
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::setSlowFilterVerify(AS5601SlowFilter _slow_filter) {
  setSlowFilter(_slow_filter);
  return (getSlowFilter() == _slow_filter) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить коэффициент медленной фильтрации 16х
 */
void AS5601::enableSlowFilter16x(void) {
  setSlowFilter(AS5601_SLOW_FILTER_16X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 16х с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableSlowFilter16xVerify(void) {
  return setSlowFilterVerify(AS5601_SLOW_FILTER_16X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 8х
 */
void AS5601::enableSlowFilter8x(void) {
  setSlowFilter(AS5601_SLOW_FILTER_8X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 8х с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableSlowFilter8xVerify(void) {
  return setSlowFilterVerify(AS5601_SLOW_FILTER_8X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 4х
 */
void AS5601::enableSlowFilter4x(void) {
  setSlowFilter(AS5601_SLOW_FILTER_4X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 4х с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableSlowFilter4xVerify(void) {
  return setSlowFilterVerify(AS5601_SLOW_FILTER_4X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 2х
 */
void AS5601::enableSlowFilter2x(void) {
  setSlowFilter(AS5601_SLOW_FILTER_2X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 2х с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableSlowFilter2xVerify(void) {
  return setSlowFilterVerify(AS5601_SLOW_FILTER_2X);
}
/*
 * @brief: получить значение порога быстрой фильтрации FTH(2:0)
 * @return:
 *  AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5601_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_10_LSB
 */
AS5601FastFilterThreshold AS5601::getFastFilterThreshold(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return (AS5601FastFilterThreshold)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_FTH_0) & 0x07); // 0x07=0b00000111
}
/*
 * @brief: получить значение порога быстрой фильтрации FTH(2:0)
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5601_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_10_LSB
 */
void AS5601::getFastFilterThreshold(AS5601FastFilterThreshold &_fast_filter_threshold) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  _fast_filter_threshold = (AS5601FastFilterThreshold)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_FTH_0) & 0x07); // 0x07=0b00000111
}
/*
 * @brief: установить новое значение порога быстрой фильтрации FTH(2:0)
 * @param _fast_filter_thredhold:
 *  AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5601_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_10_LSB
 */
void AS5601::setFastFilterThreshold(AS5601FastFilterThreshold _fast_filter_thredhold) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_H, conf_h_raw |= (_fast_filter_thredhold << AS5601_CONF_BIT_FTH_0));
}
/*
 * @brief: установить новое значение порога быстрой фильтрации FTH(2:0) с подтверждением
 * @param _fast_filter_thredhold:
 *  AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5601_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5601_FAST_FILTER_THRESHOLD_10_LSB
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::setFastFilterThresholdVerify(AS5601FastFilterThreshold _fast_filter_thredhold) {
  setFastFilterThreshold(_fast_filter_thredhold);
  return (getFastFilterThreshold() == _fast_filter_thredhold) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить только медленную фильтрацию
 */
void AS5601::enableSlowFilterOnly(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY);
}
/*
 * @brief: включить только медленную фильтрацию с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableSlowFilterOnlyVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY);
}
/*
 * @brief: включить быструю фильтрацию с порогом 6 LSB
 */
void AS5601::enableFastFilterThreshold6LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_6_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 6 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold6LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_6_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 7 LSB
 */
void AS5601::enableFastFilterThreshold7LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_7_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 7 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold7LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_7_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 9 LSB
 */
void AS5601::enableFastFilterThreshold9LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_9_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 9 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold9LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_9_LSB);
}
/*
 * brief: включить быструю фильтрацию с порогом 18 LSB
 */
void AS5601::enableFastFilterThreshold18LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_18_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 18 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold18LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_18_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 21 LSB
 */
void AS5601::enableFastFilterThreshold21LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_21_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 21 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold21LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_21_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 24 LSB
 */
void AS5601::enableFastFilterThreshold24LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_24_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 24 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold24LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_24_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 10 LSB
 */
void AS5601::enableFastFilterThreshold10LSB(void) {
  setFastFilterThreshold(AS5601_FAST_FILTER_THRESHOLD_10_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 10 LSB с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableFastFilterThreshold10LSBVerify(void) {
  return setFastFilterThresholdVerify(AS5601_FAST_FILTER_THRESHOLD_10_LSB);
}
/*
 * @brief: проверить состояние бита сторожевого таймера WD5
 * @return:
 *  AS5601_WATCHDOG_OFF - сторожевой таймер выключен
 *  AS5601_WATCHDOG_ON - сторожевой таймер включен
 */
bool AS5601::isWatchdog(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return (bool)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_WD) & 0x01);
}
/*
 * @brief: включить сторожевой таймер WD5
 */
void AS5601::enableWatchdog(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_H, conf_h_raw |= (1 << AS5601_CONF_BIT_WD));
}
/*
 * @brief: включить сторожевой таймер WD5 с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - включение не удалось
 *  AS5601_DEFAULT_REPORT_OK - включение удалось
 */
bool AS5601::enableWatchdogVerify(void) {
  enableWatchdog();
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return (bool)((AS_RequestSingleRegister() >> AS5601_CONF_BIT_WD) & 0x01);
}
/*
 * @brief: выключить сторожевой таймер WD5
 */
void AS5601::disableWatchdog(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS_RequestSingleRegister();
  AS_WriteOneByte(AS5601_CONFIG_REG_CONF_H, conf_h_raw &= ~(1 << AS5601_CONF_BIT_WD));
}
/*
 * @brief: выключить сторожевой таймер WD5 с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - выключение не удалось
 *  AS5601_DEFAULT_REPORT_OK - выключение удалось
 */
bool AS5601::disableWatchdogVerify(void) {
  disableWatchdog();
  AS_SendFirstRegister(AS5601_CONFIG_REG_CONF_H);
  return (bool)(!((AS_RequestSingleRegister() >> AS5601_CONF_BIT_WD) & 0x01));
}
/*
 * @brief: получить значение количества позиций энкодера ABN(3:0)
 * @return:
 *  AS5601_OUTPUT_POSITIONS_8
 *  AS5601_OUTPUT_POSITIONS_16
 *  AS5601_OUTPUT_POSITIONS_32
 *  AS5601_OUTPUT_POSITIONS_64
 *  AS5601_OUTPUT_POSITIONS_128
 *  AS5601_OUTPUT_POSITIONS_256
 *  AS5601_OUTPUT_POSITIONS_512
 *  AS5601_OUTPUT_POSITIONS_1024
 *  AS5601_OUTPUT_POSITIONS_2048
 */
AS5601OutputPositions AS5601::getQuadratureOutputPositions(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ABN);
  return (AS5601OutputPositions)AS_RequestSingleRegister();
}
/*
 * @brief: получить значение количества позиций энкодера ABN(3:0)
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_OUTPUT_POSITIONS_8
 *  AS5601_OUTPUT_POSITIONS_16
 *  AS5601_OUTPUT_POSITIONS_32
 *  AS5601_OUTPUT_POSITIONS_64
 *  AS5601_OUTPUT_POSITIONS_128
 *  AS5601_OUTPUT_POSITIONS_256
 *  AS5601_OUTPUT_POSITIONS_512
 *  AS5601_OUTPUT_POSITIONS_1024
 *  AS5601_OUTPUT_POSITIONS_2048
 */
void AS5601::getQuadratureOutputPositions(AS5601OutputPositions &_output_positions) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_ABN);
  _output_positions = (AS5601OutputPositions)AS_RequestSingleRegister();
}
/*
 * @brief: установить новое значение количества позиций энкодера ABN(3:0)
 * @param _output_positions:
 *  AS5601_OUTPUT_POSITIONS_8
 *  AS5601_OUTPUT_POSITIONS_16
 *  AS5601_OUTPUT_POSITIONS_32
 *  AS5601_OUTPUT_POSITIONS_64
 *  AS5601_OUTPUT_POSITIONS_128
 *  AS5601_OUTPUT_POSITIONS_256
 *  AS5601_OUTPUT_POSITIONS_512
 *  AS5601_OUTPUT_POSITIONS_1024
 *  AS5601_OUTPUT_POSITIONS_2048
 */
void AS5601::setQuadratureOutputPositions(AS5601OutputPositions _output_positions) {
  AS_WriteOneByte(AS5601_CONFIG_REG_ABN, _output_positions);
}
/*
 * @brief: установить новое значение количества позиций энкодера ABN(3:0) с подтверждением
 * @param _output_positions:
 *  AS5601_OUTPUT_POSITIONS_8
 *  AS5601_OUTPUT_POSITIONS_16
 *  AS5601_OUTPUT_POSITIONS_32
 *  AS5601_OUTPUT_POSITIONS_64
 *  AS5601_OUTPUT_POSITIONS_128
 *  AS5601_OUTPUT_POSITIONS_256
 *  AS5601_OUTPUT_POSITIONS_512
 *  AS5601_OUTPUT_POSITIONS_1024
 *  AS5601_OUTPUT_POSITIONS_2048
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::setQuadratureOutputPositionsVerify(AS5601OutputPositions _output_positions) {
  setQuadratureOutputPositions(_output_positions);
  return (getQuadratureOutputPositions() == _output_positions) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить 8 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions8(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_8);
}
/*
 * @brief: включить 8 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions8Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_8);
}
/*
 * @brief: включить 16 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions16(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_16);
}
/*
 * @brief: включить 16 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions16Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_16);
}
/*
 * @brief: включить 32 выходные позиции (шага на оборот)
 */
void AS5601::enableOutputPositions32(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_32);
}
/*
 * @brief: включить 32 выходные позиции (шага на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions32Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_32);
}
/*
 * @brief: включить 64 выходные позиции (шага на оборот)
 */
void AS5601::enableOutputPositions64(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_64);
}
/*
 * @brief: включить 64 выходные позиции (шага на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions64Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_64);
}
/*
 * @brief: включить 128 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions128(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_128);
}
/*
 * @brief: включить 128 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions128Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_128);
}
/*
 * @brief: включить 256 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions256(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_256);
}
/*
 * @brief: включить 256 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions256Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_256);
}
/*
 * @brief: включить 512 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions512(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_512);
}
/*
 * @brief: включить 512 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions512Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_512);
}
/*
 * @brief: включить 1024 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions1024(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_1024);
}
/*
 * @brief: включить 1024 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions1024Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_1024);
}
/*
 * @brief: включить 2048 выходных позиций (шагов на оборот)
 */
void AS5601::enableOutputPositions2048(void) {
  setQuadratureOutputPositions(AS5601_OUTPUT_POSITIONS_2048);
}
/*
 * @brief: включить 2048 выходных позиций (шагов на оборот) с подтверждением
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::enableOutputPositions2048Verify(void) {
  return setQuadratureOutputPositionsVerify(AS5601_OUTPUT_POSITIONS_2048);
}
/*
 * @brief: получить значение порога срабатывания кнопки энкодера PUSHTHR(7:0)
 * @return:
 *  0 - 255
 */
byte AS5601::getPushbuttonThreshold(void) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_PUSHTHR);
  return AS_RequestSingleRegister();
}
/*
 * @brief: получить значение порога срабатывания кнопки энкодера PUSHTHR(7:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 255
 */
void AS5601::getPushbuttonThreshold(byte &_pushbutton_threshold) {
  AS_SendFirstRegister(AS5601_CONFIG_REG_PUSHTHR);
  _pushbutton_threshold = AS_RequestSingleRegister();
}
/*
 * @brief: установить новое значение порога срабатывания кнопки энкодера PUSHTHR(7:0)
 * @param _push_thr_value:
 *  0 - 255
 */
void AS5601::setPushbuttonThreshold(byte _push_thr_value) {
  AS_WriteOneByte(AS5601_CONFIG_REG_PUSHTHR, _push_thr_value);
}
/*
 * @brief: установить новое значение порога срабатывания кнопки энкодера PUSHTHR(7:0) с подтверждением
 * @param _push_thr_value:
 *  0 - 255
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5601_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5601::setPushbuttonThresholdVerify(byte _push_thr_value) {
  setPushbuttonThreshold(_push_thr_value);
  return (getPushbuttonThreshold() == _push_thr_value) ? AS5601_DEFAULT_REPORT_OK : AS5601_DEFAULT_REPORT_ERROR;
}
/**************************/
/**** OUTPUT REGISTERS ****/
/**************************/
/* 
 * @brief: получить чистое значение угла RAW ANGLE(11:0)
 * @return:
 *  0 - 4095
 */
word AS5601::getRawAngle(void) {
  AS_SendFirstRegister(AS5601_OUT_REG_RAW_ANGLE_H);
  return AS_RequestPairRegisters();
}
/* 
 * @brief: получить чистое значение угла из регистра RAW ANGLE(11:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 4095
 */
void AS5601::getRawAngle(word &_raw_angle) {
  AS_SendFirstRegister(AS5601_OUT_REG_RAW_ANGLE_H);
  _raw_angle = AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение угла в градусах
 * @return:
 *  0.00 - 360.00
 */
float AS5601::getDegreesAngle(void) {
  return ((float)getRawAngle() * 360) / 4096;
}
/* 
 * @brief: получить значение угла в градусах
 * @note: метод работает через ссылку
 * @return:
 *  0.00 - 360.00
 */
void AS5601::getDegreesAngle(float &_degrees_angle) {
  _degrees_angle = ((float)getRawAngle() * 360) / 4096;
}
/* 
 * @brief: получить значение угла в радианах
 * @return:
 *  0.00 - 6.28
 */
float AS5601::getRadiansAngle(void) {
  return (getDegreesAngle() * PI) / 180;
}
/* 
 * @brief: получить значение угла в радианах
 * @note: метод работает через ссылку
 * @return:
 *  0.00 - 6.28
 */
void AS5601::getRadiansAngle(float &_radians_angle) {
  _radians_angle = (getDegreesAngle() * PI) / 180;
}
/* 
 * @brief: получить угол с учетом гистерезиса 10 LSB ANGLE(11:0)
 * @return:
 *  0 - 4095
 */
word AS5601::getAngle(void) {
  AS_SendFirstRegister(AS5601_OUT_REG_ANGLE_H);
  return AS_RequestPairRegisters();
}
/* 
 * @brief: получить угол с учетом гистерезиса 10 LSB ANGLE(11:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 4095
 */
void AS5601::getAngle(word &_10lsb_angle) {
  AS_SendFirstRegister(AS5601_OUT_REG_ANGLE_H);
  _10lsb_angle = AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение угла в градусах 10LSB
 * @return:
 *  0.00 - 360.00
 */
float AS5601::getDegreesAngle10LSB(void) {
  return ((float)getAngle() * 360) / 4096;
}
/* 
 * @brief: получить значение угла в градусах 10LSB
 * @note: метод работает через ссылку
 * @return:
 *  0.00 - 360.00
 */
void AS5601::getDegreesAngle10LSB(float &_10lsb_degrees_angle) {
  _10lsb_degrees_angle = ((float)getAngle() * 360) / 4096;
}
/* 
 * @brief: получить значение угла в радианах 10LSB
 * @return:
 *  0.00 - 6.28
 */
float AS5601::getRadiansAngle10LSB(void) {
  return (getDegreesAngle10LSB() * PI) / 180;
}
/* 
 * @brief: получить значение угла в радианах 10LSB
 * @note: метод работает через ссылку
 * @return:
 *  0.00 - 6.28
 */
void AS5601::getRadiansAngle10LSB(float &_10lsb_radians_angle) {
  _10lsb_radians_angle = (getDegreesAngle10LSB() * PI) / 180;
}
/**************************/
/**** STATUS REGISTERS ****/
/**************************/
/*
 * @brief: получить значение регистра STATUS(5:3)
 * @return:
 *  AS5601_STATUS_REPORT_MD0_ML0_MH_0 - MD = 0, ML = 0, MH = 0
 *  AS5601_STATUS_REPORT_MD0_ML1_MH_0 - MD = 0, ML = 1, MH = 0
 *  AS5601_STATUS_REPORT_MD1_ML0_MH_0 - MD = 1, ML = 0, MH = 0
 *  AS5601_STATUS_REPORT_MD1_ML0_MH_1 - MD = 1, ML = 0, MH = 1
 *  AS5601_STATUS_REPORT_MD1_ML1_MH_0 - MD = 1, ML = 1, MH = 0
 */
AS5601StatusReports AS5601::getStatus(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  return (AS5601StatusReports)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MH_3) & 0x07); // 0x07 = 0b00000111
}
/*
 * @brief: получить значение регистра STATUS(5:3)
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_STATUS_REPORT_MD0_ML0_MH_0 - MD = 0, ML = 0, MH = 0
 *  AS5601_STATUS_REPORT_MD0_ML1_MH_0 - MD = 0, ML = 1, MH = 0
 *  AS5601_STATUS_REPORT_MD1_ML0_MH_0 - MD = 1, ML = 0, MH = 0
 *  AS5601_STATUS_REPORT_MD1_ML0_MH_1 - MD = 1, ML = 0, MH = 1
 *  AS5601_STATUS_REPORT_MD1_ML1_MH_0 - MD = 1, ML = 1, MH = 0
 */
void AS5601::getStatus(AS5601StatusReports &_status) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  _status = (AS5601StatusReports)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MH_3) & 0x07); // 0x07 = 0b00000111
}
/*
 * @brief: определить наличие магнита MD:5
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - магнита не обнаружен
 *  AS5601_DEFAULT_REPORT_OK - магнит обнаружен
 */
bool AS5601::isMagnetDetected(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  return (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MD_5) & 0x01);
}
/*
 * @brief: определить наличие магнита MD:5
 * @note: метод работает через ссылку
 * @return: 
 *  AS5601_DEFAULT_REPORT_ERROR - магнита не обнаружен
 *  AS5601_DEFAULT_REPORT_OK - магнит обнаружен
 */
void AS5601::isMagnetDetected(bool &_magnet_detect) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  _magnet_detect = (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MD_5) & 0x01);
}
/*
 * @brief: определить слишком слабый магнит ML:4
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - магнит не слишком слабый
 *  AS5601_DEFAULT_REPORT_OK - магнит слишком слабый
 */
bool AS5601::isMagnetTooWeak(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  return (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_ML_4) & 0x01);
}
/*
 * @brief: определить слишком слабый магнит ML:4
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - магнит не слишком слабый
 *  AS5601_DEFAULT_REPORT_OK - магнит слишком слабый
 */
void AS5601::isMagnetTooWeak(bool &_magnet_weak) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  _magnet_weak = (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_ML_4) & 0x01);
}
/*
 * @brief: определить слишком сильный магнит MH:3
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - магнит не слишком сильный
 *  AS5601_DEFAULT_REPORT_OK - магнит слишком сильный
 */
bool AS5601::isMagnetTooStrong(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  return (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MH_3) & 0x01);
}
/*
 * @brief: определить слишком сильный магнит MH:3
 * @note: метод работает через ссылку
 * @return:
 *  AS5601_DEFAULT_REPORT_ERROR - магнит не слишком сильный
 *  AS5601_DEFAULT_REPORT_OK - магнит слишком сильный
 */
void AS5601::isMagnetTooStrong(bool &_magnet_strong) {
  AS_SendFirstRegister(AS5601_STATUS_REG);
  _magnet_strong = (bool)((AS_RequestSingleRegister() >> AS5601_STATUS_BIT_MH_3) & 0x01);
}
/*
 * @brief: получить значение автоматического усиления AGC(7:0)
 * @return:
 *  0 - 255, при VCC = 5V
 *  0 - 128, при VCC = 3.3V
 */
byte AS5601::getAutomaticGainControl(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG_AGC);
  return AS_RequestSingleRegister();
}
/*
 * @brief: получить значение автоматического усиления AGC(7:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 255, при VCC = 5V
 *  0 - 128, при VCC = 3.3V
 */
void AS5601::getAutomaticGainControl(byte &_agc) {
  AS_SendFirstRegister(AS5601_STATUS_REG_AGC);
  _agc = AS_RequestSingleRegister();
}
/* 
 * @brief: получить значение магнитуды MAGNITUDE(11:0)
 * @return:
 *  0 - 4095
 */
word AS5601::getMagnitude(void) {
  AS_SendFirstRegister(AS5601_STATUS_REG_MAGNITUDE_H);
  return AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение магнитуды MAGNITUDE(11:0)
 * @note: метод работает через ссылку
 * @return:
 *  0 - 4095
 */
void AS5601::getMagnitude(word &_magnitude) {
  AS_SendFirstRegister(AS5601_STATUS_REG_MAGNITUDE_H);
  _magnitude = AS_RequestPairRegisters();
}
/************************/
/**** BURN REGISTERS ****/
/************************/
/* 
 * @brief: записать НАВСЕГДА установленное значение в регистре ZPOS(11:0)
 * @note: ВЫПОЛНИТЬ ЭТУ КОМАНДУ МОЖНО ТОЛЬКО 3(ТРИ) РАЗА ДЛЯ ОДНОГО ДАТЧИКА 
 *  ПРИ НАЛИЧИИ МАГНИТА (MD:5 = 1) И ПРИ НАЛИЧИИ РЕСУРСА В ZMCO(1:0)!
 * @param _use_special_verify:
 *  AS5601_FLAG_SPECIAL_VERIFY_DISABLE
 *  AS5601_FLAG_SPECIAL_VERIFY_ENABLE
 * @return:
 *  AS5601_BURN_REPROT_SENSOR_NOT_CONNECTED
 *  AS5601_BURN_REPROT_MAGNET_NOT_FOUND
 *  AS5601_BURN_REPROT_WRITE_OK_WITH_VERIFY
 *  AS5601_BURN_REPROT_WRITE_WRONG
 *  AS5601_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY
 *  AS5601_BURN_REPROT_ZPOS_NOT_SET
 *  AS5601_BURN_REPROT_RESOURCE_ZMCO_ENDED
 */
AS5601BurnReports AS5601::burnZeroPosition(AS5601SpecialVerifyFlags _use_special_verify) {
  AS5601BurnReports result = AS5601_BURN_REPROT_SENSOR_NOT_CONNECTED;
  
  if(isConnected()) { // Если датчик подключен
    // Собираем значениях из критически выжных регистров
    byte burn_count = getBurnPositionsCount();
    word z_pos = getZeroPosition();
    if(burn_count < AS5601_MAX_VALUE_ZMCO) { // Если ресурс для записи не исчерпан
      if(z_pos > 0) { // Если значение начального положения не 0
        // Наличие магнита проверяем НА ПОСЛЕДНЕМ ШАГЕ, перед отправлением команды на запись!
        if(isMagnetDetected()) { // Если магнит обнаружен
          AS_WriteOneByte(AS5601_BURN_REG, AS5601_CMD_BURN_ANGLE); // Отправляем команду записи
          if(_use_special_verify) { // Если используется проверка записанного
            loadSavedValues(); // Загружаем из памяти ранее записанные данные
            // Получаем загруженные данные для сравнения
            word z_pos_now = getZeroPosition();
            if(z_pos == z_pos_now) { // Если записываемые данные совпадают с сохраненными
              result = AS5601_BURN_REPROT_WRITE_OK_WITH_VERIFY;
            }else {
              result = AS5601_BURN_REPROT_WRITE_WRONG;
            }
          }else {
            result = AS5601_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY;
          }
        }else {
          result = AS5601_BURN_REPROT_MAGNET_NOT_FOUND;
        }
      }else {
        result = AS5601_BURN_REPROT_ZPOS_NOT_SET;
      }
    }else {
      result = AS5601_BURN_REPROT_RESOURCE_ZMCO_ENDED;
    }
  }

  return result;
}
/* 
 * @brief: записать НАВСЕГДА установленное значение в регистре CONF(13:0)
 * @note: ВЫПОЛНИТЬ ЭТУ КОМАНДУ МОЖНО ТОЛЬКО 1(ОДИН) РАЗ ДЛЯ ОДНОГО ДАТЧИКА 
 *  ПРИ НАЛИЧИИ МАГНИТА (MD:5 = 1)!
 * @param _use_special_verify:
 *  AS5601_FLAG_SPECIAL_VERIFY_DISABLE
 *  AS5601_FLAG_SPECIAL_VERIFY_ENABLE
 * @return:
 *  AS5601_BURN_REPROT_SENSOR_NOT_CONNECTED
 *  AS5601_BURN_REPROT_MAGNET_NOT_FOUND
 *  AS5601_BURN_REPROT_WRITE_OK_WITH_VERIFY
 *  AS5601_BURN_REPROT_WRITE_WRONG
 *  AS5601_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY
 */
AS5601BurnReports AS5601::burnConfiguration(AS5601SpecialVerifyFlags _use_special_verify) {
  AS5601BurnReports result = AS5601_BURN_REPROT_SENSOR_NOT_CONNECTED;
  
  if(isConnected()) { // Если датчик подключен
    word conf = getRawConfigurationValue();
    // Проверяем наличие магнита перед отправлением команды на запись!
    if(isMagnetDetected()) { // Если магнит обнаружен
      AS_WriteOneByte(AS5601_BURN_REG, AS5601_CMD_BURN_SETTINGS); // Отправляем команду записи настроек
      if(_use_special_verify) { // Если используется проверка записанного
        loadSavedValues(); // Загружаем из памяти ранее записанные данные
        // Получаем загруженные данные для сравнения
        word conf_now = getRawConfigurationValue();
        if(conf == conf_now) { // Если записываемые данные совпадают с сохраненными
          result = AS5601_BURN_REPROT_WRITE_OK_WITH_VERIFY;
        }else {
          result = AS5601_BURN_REPROT_WRITE_WRONG;
        }
      }else {
        result = AS5601_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY;
      }
    }else {
      result = AS5601_BURN_REPROT_MAGNET_NOT_FOUND;
    }
  }
  
  return result;
}
