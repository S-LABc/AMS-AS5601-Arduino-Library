/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с бесконтактным магнитным энкодером
 * AS5601 от компании AMS https://ams.com/ams-start
 * 
 * Документация к датчику:
 ** https://ams.com/documents/20143/36005/AS5601_DS000395_3-00.pdf
 ** https://ams.com/en/as5601
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.4 / License MIT / Скляр Роман S-LAB
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Настройки шины I2C датчика ===*/
const uint32_t AS5601_I2C_CLOCK_100KHZ = 100000;
const uint32_t AS5601_I2C_CLOCK_400KHZ = 400000;
const uint32_t AS5601_I2C_CLOCK_1MHZ   = 1000000;
const uint8_t AS5601_I2C_ADDRESS       = 0x36;

/*=== Адреса регистров датчика ===*/
/* Configuration Registers */
const uint8_t AS5601_CONFIG_REG_ZMCO = 0x00;
const uint8_t AS5601_CONFIG_REG_ZPOS_H = 0x01;
const uint8_t AS5601_CONFIG_REG_ZPOS_L = 0x02;
const uint8_t AS5601_CONFIG_REG_CONF_H = 0x07;
const uint8_t AS5601_CONFIG_REG_CONF_L = 0x08;
// Значимые биты регистра CONF_H
enum AS5601ConfHighRegisterBits {
  AS5601_CONF_BIT_SF_0,
  AS5601_CONF_BIT_SF_1,
  AS5601_CONF_BIT_FTH_0,
  AS5601_CONF_BIT_FTH_1,
  AS5601_CONF_BIT_FTH_2,
  AS5601_CONF_BIT_WD,
};
// Значимые биты регистра CONF_L
enum AS5601ConfLowRegisterBits {
  AS5601_CONF_BIT_PM_0,
  AS5601_CONF_BIT_PM_1,
  AS5601_CONF_BIT_HYST_0,
  AS5601_CONF_BIT_HYST_1,
};
const uint8_t AS5601_CONFIG_REG_ABN = 0x09;
const uint8_t AS5601_CONFIG_REG_PUSHTHR = 0x0A;
/* Output Registers */
const uint8_t AS5601_OUT_REG_RAW_ANGLE_H = 0x0C;
const uint8_t AS5601_OUT_REG_RAW_ANGLE_L = 0x0D;
const uint8_t AS5601_OUT_REG_ANGLE_H = 0x0E;
const uint8_t AS5601_OUT_REG_ANGLE_L = 0x0F;
/* Status Registers */
const uint8_t AS5601_STATUS_REG = 0x0B;
const uint8_t AS5601_STATUS_REG_AGC = 0x1A;
const uint8_t AS5601_STATUS_REG_MAGNITUDE_H = 0x1B;
const uint8_t AS5601_STATUS_REG_MAGNITUDE_L = 0x1C;
// Значимые биты регистра STATUS
enum AS5601StatusRegisterBits {
  AS5601_STATUS_BIT_MH_3 = 3,
  AS5601_STATUS_BIT_ML_4,
  AS5601_STATUS_BIT_MD_5,
};
/* Burn Commands */
const uint8_t AS5601_BURN_REG = 0xFF;
// Команды регистра BURN
const uint8_t AS5601_CMD_BURN_ANGLE = 0x80;
const uint8_t AS5601_CMD_BURN_SETTINGS = 0x40;
// Zero Position and Resolution Programming Procedure (Step 7)
const uint8_t AS5601_CMD_BURN_LOAD_OTP_CONTENT_0 = 0x01;
const uint8_t AS5601_CMD_BURN_LOAD_OTP_CONTENT_1 = 0x11;
const uint8_t AS5601_CMD_BURN_LOAD_OTP_CONTENT_2 = 0x10;

/*=== Вспомогательные значения ===*/
// Предельное значение регистра CONF_ZMCO
const uint8_t AS5601_MAX_VALUE_ZMCO = 0x03;
// Ответы стандартного вида успех/ошибка
const uint8_t AS5601_DEFAULT_REPORT_ERROR = 0;
const uint8_t AS5601_DEFAULT_REPORT_OK = 1;
// Состояния сторожевого таймера
const uint8_t AS5601_WATCHDOG_OFF = 0;
const uint8_t AS5601_WATCHDOG_ON = 1;
// Режимы питания
enum AS5601PowerModes {
  AS5601_NOM_POWER_MODE,
  AS5601_LOW_POWER_MODE_1,
  AS5601_LOW_POWER_MODE_2,
  AS5601_LOW_POWER_MODE_3,
};
// Режимы гистерезиса
enum AS5601Hysteresis {
  AS5601_HYSTERESIS_OFF,
  AS5601_HYSTERESIS_1_LSB,
  AS5601_HYSTERESIS_2_LSB,
  AS5601_HYSTERESIS_3_LSB,
};
// Коэффициенты медленной фильтрации
enum AS5601SlowFilter {
  AS5601_SLOW_FILTER_16X,
  AS5601_SLOW_FILTER_8X,
  AS5601_SLOW_FILTER_4X,
  AS5601_SLOW_FILTER_2X,
};
// Пороги быстрой фильтрации
enum AS5601FastFilterThreshold {
  AS5601_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY,
  AS5601_FAST_FILTER_THRESHOLD_6_LSB,
  AS5601_FAST_FILTER_THRESHOLD_7_LSB,
  AS5601_FAST_FILTER_THRESHOLD_9_LSB,
  AS5601_FAST_FILTER_THRESHOLD_18_LSB,
  AS5601_FAST_FILTER_THRESHOLD_21_LSB,
  AS5601_FAST_FILTER_THRESHOLD_24_LSB,
  AS5601_FAST_FILTER_THRESHOLD_10_LSB,
};
// Количество шагов на оборот
enum AS5601OutputPositions {
  AS5601_OUTPUT_POSITIONS_8,
  AS5601_OUTPUT_POSITIONS_16,
  AS5601_OUTPUT_POSITIONS_32,
  AS5601_OUTPUT_POSITIONS_64,
  AS5601_OUTPUT_POSITIONS_128,
  AS5601_OUTPUT_POSITIONS_256,
  AS5601_OUTPUT_POSITIONS_512,
  AS5601_OUTPUT_POSITIONS_1024,
  AS5601_OUTPUT_POSITIONS_2048,
};
// Ответы метода getStatus
enum AS5601StatusReports {
  AS5601_STATUS_REPORT_MD0_ML0_MH_0,
  AS5601_STATUS_REPORT_MD0_ML1_MH_0 = 2,
  AS5601_STATUS_REPORT_MD1_ML0_MH_0 = 4,
  AS5601_STATUS_REPORT_MD1_ML0_MH_1,
  AS5601_STATUS_REPORT_MD1_ML1_MH_0,
};
// Флаги для использования с burn
enum AS5601SpecialVerifyFlags {
  AS5601_FLAG_SPECIAL_VERIFY_DISABLE,
  AS5601_FLAG_SPECIAL_VERIFY_ENABLE,
};
// Ответы методов burnZeroPosition, burnConfiguration
enum AS5601BurnReports {
  AS5601_BURN_REPROT_SENSOR_NOT_CONNECTED,
  AS5601_BURN_REPROT_MAGNET_NOT_FOUND,
  AS5601_BURN_REPROT_WRITE_OK_WITH_VERIFY,
  AS5601_BURN_REPROT_WRITE_WRONG,
  AS5601_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY,
  AS5601_BURN_REPROT_ZPOS_NOT_SET,
  AS5601_BURN_REPROT_RESOURCE_ZMCO_ENDED,
};


class AS5601 {
  private:
    TwoWire* _wire_; // Объект для доступа к методам I2C Wire.h

  protected:
    virtual void AS_SendFirstRegister(uint8_t _reg_addr); // Отправить адрес регистра
    virtual uint8_t AS_RequestSingleRegister(void); // Запрос значения регистра размером 1 байт
    virtual uint16_t AS_RequestPairRegisters(void); // Запрос значения регистра размером 2 байта
	
    virtual void AS_WriteOneByte(uint8_t _reg, uint8_t _payload); // Запись одного байта в регистр размером 1 байт
    virtual void AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload); // Запись 2х байтов в регистр размером 2 байта
	
  public:
    AS5601(TwoWire* _twi); // Конструктор

    virtual void begin(void); // Вызов Wire.begin()
#if defined(ESP8266) || defined(ESP32)
    virtual void begin(int8_t _sda_pin, int8_t _scl_pin); // Вызов Wire.begin(SDA, SCL) с указанием выводов
#endif
    virtual void setClock(uint32_t _freq_hz = AS5601_I2C_CLOCK_400KHZ); // Настройка частоты на 100кГц, 400кГц, 1МГц, или пользовательское значение (по умолчанию 400кГц)
	
    virtual void loadSavedValues(void); // Метод производителя для загрузки значений из памяти в регистры ZPOS, CONF
	
    virtual bool isConnected(void); // Проверка наличия датчика на шине I2C
	
    /* Configuration Registers */
    virtual byte getBurnPositionsCount(void); // Получить количество сохранений(burn) значений в ZPOS (с завода ZMCO = 0). 0 - 3
	
    virtual word getZeroPosition(void); // Получить значение начального положения ZPOS. 0 - 4095
    virtual void setZeroPosition(word _zero_position); // Установить новое начальное положение ZPOS
    virtual bool setZeroPositionVerify(word _zero_position); // Тоже самое, но с подтверждением
    virtual void setZeroPositionViaRawAngle(void); // Установить новое начальное положение ZPOS используя нынешнее положение магнита (getRawAngle)
    virtual bool setZeroPositionViaRawAngleVerify(void); // Тоже самое, но с подтверждением
    virtual void setZeroPositionViaAngle10LSB(void); // Установить новое начальное положение ZPOS используя нынешнее положение магнита (getAngle)
    virtual bool setZeroPositionViaAngle10LSBVerify(void); // Тоже самое, но с подтверждением
	
    virtual word getRawConfigurationValue(void); // Получить значение регистра конфигураций CONF
    virtual void setRawConfigurationValue(word _configuration_value); // Установить новое значение регистра конфигураций CONF
    virtual bool setRawConfigurationValueVerify(word _configuration_value); // Тоже самое, но с подтверждением
    /** Управление Power Mode битами PM **/
    virtual AS5601PowerModes getPowerMode(void); // Получить текущий режим питания
    virtual void setPowerMode(AS5601PowerModes _power_mode); // Установить новый режим питания
    virtual bool setPowerModeVerify(AS5601PowerModes _power_mode); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableNomPowerMode(void); // Включить нормальный режим питания
    virtual bool enableNomPowerModeVerify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode1(void); // Включить пониженный режим питания 1
    virtual bool enableLowPowerMode1Verify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode2(void); // Включить пониженный режим питания 2
    virtual bool enableLowPowerMode2Verify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode3(void); // Включить пониженный режим питания 3
    virtual bool enableLowPowerMode3Verify(void); // Тоже самое, но с подтверждением
    /** Управление Hysteresis битами HYST **/
    virtual AS5601Hysteresis getHysteresis(void); // Получить параметры гистерезиса
    virtual void setHysteresis(AS5601Hysteresis _hysteresis); // Установить новые параметры гистерезиса
    virtual bool setHysteresisVerify(AS5601Hysteresis _hysteresis); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void disableHysteresis(void); // Отключить гистерезис
    virtual bool disableHysteresisVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis1LSB(void); // Включить гистерезис 1 LSB
    virtual bool enableHysteresis1LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis2LSB(void); // Включить гистерезис 2 LSB
    virtual bool enableHysteresis2LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis3LSB(void); // Включить гистерезис 3 LSB
    virtual bool enableHysteresis3LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Slow Filter битами SF **/
    virtual AS5601SlowFilter getSlowFilter(void); // Получить коэффициент медленной фильтрации
    virtual void setSlowFilter(AS5601SlowFilter _slow_filter); // Установить новый коэффициент медленной фильтрации
    virtual bool setSlowFilterVerify(AS5601SlowFilter _slow_filter); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableSlowFilter16x(void); // Включить коэффициент 16х
    virtual bool enableSlowFilter16xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter8x(void); // Включить коэффициент 8х
    virtual bool enableSlowFilter8xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter4x(void); // Включить коэффициент 4х
    virtual bool enableSlowFilter4xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter2x(void); // Включить коэффициент 2х
    virtual bool enableSlowFilter2xVerify(void); // Тоже самое, но с подтверждением
    /** Управление Fast Filter Threshold битами FTH **/
    virtual AS5601FastFilterThreshold getFastFilterThreshold(void); // Получить порог быстрой фильтрации
    virtual void setFastFilterThreshold(AS5601FastFilterThreshold _fast_filter_thredhold); // Установить порог быстрой фильтрации
    virtual bool setFastFilterThresholdVerify(AS5601FastFilterThreshold _fast_filter_thredhold); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableSlowFilterOnly(void); // Включить только медленную фильтрацию
    virtual bool enableSlowFilterOnlyVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold6LSB(void); // Включить быструю фильтрацию 6 LSB
    virtual bool enableFastFilterThreshold6LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold7LSB(void); // Включить быструю фильтрацию 7 LSB
    virtual bool enableFastFilterThreshold7LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold9LSB(void); // Включить быструю фильтрацию 9 LSB
    virtual bool enableFastFilterThreshold9LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold18LSB(void); // Включить быструю фильтрацию 18 LSB
    virtual bool enableFastFilterThreshold18LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold21LSB(void); // Включить быструю фильтрацию 21 LSB
    virtual bool enableFastFilterThreshold21LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold24LSB(void); // Включить быструю фильтрацию 24 LSB
    virtual bool enableFastFilterThreshold24LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold10LSB(void); // Включить быструю фильтрацию 10 LSB
    virtual bool enableFastFilterThreshold10LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Watchdog битом WD **/
    virtual bool isWatchdog(void); // Определить состояние сторожевого таймера
    // Отдельные способы
    virtual void enableWatchdog(void); // Включить сторожевой таймер
    virtual bool enableWatchdogVerify(void); // Тоже самое, но с подтверждением
    virtual void disableWatchdog(void); // Выключить сторожевой таймер
    virtual bool disableWatchdogVerify(void); // Тоже самое, но с подтверждением

    virtual AS5601OutputPositions getQuadratureOutputPositions(void); // Получить значение количества шагов на оборот. 0 - 8, свыше 8 количество будет 2048
    virtual void setQuadratureOutputPositions(AS5601OutputPositions _output_positions); // Установить новое значение количества выходных позиций энкодера. 0 - 8, свыше 8 количество будет 2048
    virtual bool setQuadratureOutputPositionsVerify(AS5601OutputPositions _output_positions); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableOutputPositions8(void); // Включить режим 8 шагов на оборот
    virtual bool enableOutputPositions8Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions16(void); // Включить режим 16 шагов на оборот
    virtual bool enableOutputPositions16Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions32(void); // Включить режим 32 шага на оборот
    virtual bool enableOutputPositions32Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions64(void); // Включить режим 64 шага на оборот
    virtual bool enableOutputPositions64Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions128(void); // Включить режим 128 шагов на оборот
    virtual bool enableOutputPositions128Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions256(void); // Включить режим 256 шагов на оборот
    virtual bool enableOutputPositions256Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions512(void); // Включить режим 512 шагов на оборот
    virtual bool enableOutputPositions512Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions1024(void); // Включить режим 1024 шага на оборот
    virtual bool enableOutputPositions1024Verify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputPositions2048(void); // Включить режим 2048 шагов на оборот
    virtual bool enableOutputPositions2048Verify(void); // Тоже самое, но с подтверждением
    
    virtual byte getPushbuttonThreshold(void); // Получить значение порога срабатывания кнопки. 0 - 255
    virtual void setPushbuttonThreshold(byte _push_thr_value); // Установить новое значение порога срабатывания кнопки. 0 - 255
    virtual bool setPushbuttonThresholdVerify(byte _push_thr_value); // Тоже самое, но с подтверждением
    
    /* Output Registers */
    virtual word getRawAngle(void); // Получить угол в чистом виде. 0 - 4095
    virtual float getDegreesAngle(void); // Получить угол в градусах. 0.00 - 360.00. Основан на значениях от getRawAngle
    virtual float getRadiansAngle(void); // Получить угол в радианах 0.00 - 6.29. Основан на значениях от getRawAngle
    
    virtual word getAngle(void); // Получить угол с учетом гистерезиса 10 LSB. 0 - 4095
    virtual float getDegreesAngle10LSB(void); // Получить угол в градусах. 0.00 - 360.00. Основан на значениях от getAngle
    virtual float getRadiansAngle10LSB(void); // Получить угол в радианах 0.00 - 6.29. Основан на значениях от getAngle
	
    /* Status Registers */
    virtual AS5601StatusReports getStatus(void); // Получить значение регистра STATUS
    
    virtual bool isMagnetDetected(void); // Определить наличие магнита
    virtual bool isMagnetTooWeak(void); // Определить является ли магнит очень СЛАБЫМ
    virtual bool isMagnetTooStrong(void); // Определить является ли магнит очень СИЛЬНЫМ
	
    virtual byte getAutomaticGainControl(void); // Получить значение автоусиления. При 5В 0 - 255, при 3.3В 0 - 128
	
    virtual word getMagnitude(void); // Получить значение магнитуды. 0 - 4095
    
    /* Burn Commands */
    virtual AS5601BurnReports burnZeroPosition(AS5601SpecialVerifyFlags _use_special_verify = AS5601_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда ZPOS. CMD_BURN_ANGLE [3 РАЗА МАКСИМУМ!]
    virtual AS5601BurnReports burnConfiguration(AS5601SpecialVerifyFlags _use_special_verify = AS5601_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда CONF. CMD_BURN_SETTINGS [1 РАЗ МАКСИМУМ!]
};
