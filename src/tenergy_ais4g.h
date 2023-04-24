/***********************************************************************
 * File         :     tenergy_ais4g.h
 * Description  :     Class for Hardware config and function for tenergy_ais4g module
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     20 Apir 2023
 * Revision     :     1.0.1
 * Rev1.0.0     :     Original
 * Rev1.0.1     :     Add sht40 temperature and humidity sensor interface (2023-04-22)
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@innovation.co.th
 * TEL          :     089-140-7205
 ***********************************************************************/

#ifndef TENERGY_AIS4G_H
#define TENERGY_AIS4G_H
#include "Ticker.h"
#include "Wire.h"

enum {
    CELSIUS,
    FAHRENHEIT
};

class tenergy_ais4g
{
private:
#define version_c "1.0.1"

public:
/**************************************/
/*           GPIO define              */
/**************************************/
#define RXD2 16
#define TXD2 17
#define RXD3 27
#define TXD3 26
#define SW1 34
#define SW2 35
#define RELAY1 25
#define RELAY2 32
#define LED_ONBOARD 15
#define SLID_SW 05
#define BUZZER 04
#define SW_IO0 0
#define SW_E18 18

    tenergy_ais4g(TwoWire *wire = &Wire);
    void Relay1(bool state);
    void Relay2(bool state);
    void BuildinLED(bool state);
    void buzzer_beep(int times);
    bool Sw_IO0(void);
    bool Sw_E18(void);
    bool Slid_sw(void);
    void library_version(void);

private:
    uint8_t _resolution_bit;
    uint16_t crc16_update(uint16_t crc, uint8_t a);

    /*sht40*/
    TwoWire *wire = NULL;
    uint8_t dev_addr = 0x00;
    float t_degC;
    float rh_pRH;
    bool sht40_read();

public:
    void TickBuildinLED(float second);
    bool PWM_Setup(uint8_t channel, double freq, uint8_t resolution_bit, uint8_t pin);
    bool PWM_Drive(uint8_t channel, uint8_t percentage);
    bool Frequency_Out(uint8_t pin, double freq);
    uint16_t TimeStamp_minute_encode(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mi);
    uint16_t TimeStamp_24hr_encode(uint16_t h, uint16_t mi);
    void TimeStamp_hour_minute_decode(uint16_t timestemp, uint16_t &h, uint16_t &mi);

private:
    uint16_t ec_modbusRTU(uint8_t id);
    bool ec_modbusRTU_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);

public:
    /* PZEM-016 Modbus RTU AC power meter module */
    bool PZEM_016(uint8_t id, float &volt, float &amp, float &power, uint32_t &energy, float &freq, float &pf);
    float PZEM_016_Volt(uint8_t id);
    float PZEM_016_Amp(uint8_t id);
    float PZEM_016_Power(uint8_t id);
    int16_t PZEM_016_Energy(uint8_t id);
    float PZEM_016_Freq(uint8_t id);
    float PZEM_016_PF(uint8_t id);
    bool PZEM_016_ResetEnergy(uint8_t id);
    int8_t PZEM_016_SetAddress(uint8_t id, uint8_t new_id);
    int8_t PZEM_016_SearchAddress(void);
    bool PZEM_016_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);

    /* PZEM-003 Modbus RTU DC power meter module */
    bool PZEM_003(uint8_t id, float &volt, float &amp, float &power, uint32_t &energy);
    float PZEM_003_Volt(uint8_t id);
    float PZEM_003_Amp(uint8_t id);
    float PZEM_003_Power(uint8_t id);
    int16_t PZEM_003_Energy(uint8_t id);
    bool PZEM_003_ResetEnergy(uint8_t id);
    int8_t PZEM_003_SetAddress(uint8_t id, uint8_t new_id);
    int8_t PZEM_003_SearchAddress(void);
    bool PZEM_003_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);

    /* WTR10-E Modbus RTU Temperature and Humidity sensor module */
    bool WTR10_E_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    bool WTR10_E(uint8_t id, float &temp, float &humi);
    float WTR10_E_tempeature(uint8_t id);
    float WTR10_E_humidity(uint8_t id);

    /* XY-MD02 Modbus RTU Temperature and Humidity sensor module */
    bool XY_MD02_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    bool XY_MD02(uint8_t id, float &temp, float &humi);
    float XY_MD02_tempeature(uint8_t id);
    float XY_MD02_humidity(uint8_t id);
    int8_t XY_MD02_searchAddress(void);
    int8_t XY_MD02_SetAddress(uint8_t id, uint8_t new_id);

    /* SOIL MOISTURE PR-3000-H-N01 sensor (RS485) module */
    bool PR3000_H_N01_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    bool PR3000_H_N01(float &temp, float &humi);
    float PR3000_H_N01_tempeature();
    float PR3000_H_N01_humidity();

    /* RS485 Water Flow Meter RS485 MODBUS output  */
    bool WATER_FLOW_METER_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t WATER_FLOW_METER_searchAddress(void);
    int8_t WATER_FLOW_METER_SetAddress(uint8_t id, uint8_t new_id);
    float WATER_FLOW_METER(uint8_t id);

    /* PYR20-Solar Radiation/Pyranometer Sensor, RS485, Modbus */
    bool PYR20_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t PYR20_searchAddress(void);
    int8_t PYR20_SetAddress(uint8_t id, uint8_t new_id);
    int16_t PYR20_read(uint8_t id);

    /* tiny32 ModbusRTU communication*/
    bool ModbusRTU_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t ModbusRTU_searchAddress(void);
    int8_t ModbusRTU_setAddress(uint8_t id, uint8_t new_id);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7, float &val8, float &val9, float &val10);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7, float &val8, float &val9);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7, float &val8);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4, float &val5);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3, float &val4);
    bool ModbusRTU(uint8_t id, float &val1, float &val2, float &val3);
    bool ModbusRTU(uint8_t id, float &val1, float &val2);
    bool ModbusRTU(uint8_t id, float &val1);

    /* Enenergic ModbusRTU PowerMeter*/
    bool ENenergic_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t ENenergic_searchAddress(void);
    int8_t ENenergic_setAddress(uint8_t id, uint8_t new_id);
    float ENenergic_getTemperature(uint8_t id);
    bool ENenergic_Volt_L_N(uint8_t id, float &L1_N, float &L2_N, float &L3_N);
    bool ENenergic_Volt_L_L(uint8_t id, float &L1_L2, float &L2_L3, float &L3_L1);
    bool ENenergic_Current_L(uint8_t id, float &L1, float &L2, float &L3);
    float ENenergic_NeutralCurrent(uint8_t id);
    float ENenergic_Freq(uint8_t id);
    bool ENenergic_PhaseVolt_Angle(uint8_t id, float &L1, float &L2, float &L3);
    bool ENenergic_PhaseCurrent_Angle(uint8_t id, float &L1, float &L2, float &L3);

    /* Schneider EasyLogic PM2xxx Digital Power Meter */
    bool SchneiderPM2xxx_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t SchneiderPM2xxx_searchAddress(void);

    bool SchneiderPM2xxx_MeteringSetup(void);

    float SchneiderPM2xxx_CurrentA(uint8_t id);
    float SchneiderPM2xxx_CurrentB(uint8_t id);
    float SchneiderPM2xxx_CurrentC(uint8_t id);
    float SchneiderPM2xxx_CurrentN(uint8_t id);
    float SchneiderPM2xxx_CurrentG(uint8_t id);
    float SchneiderPM2xxx_CurrentAvg(uint8_t id);
    float SchneiderPM2xxx_CurrentUnblanceA(uint8_t id);
    float SchneiderPM2xxx_CurrentUnblanceB(uint8_t id);
    float SchneiderPM2xxx_CurrentUnblanceC(uint8_t id);
    float SchneiderPM2xxx_CurrentUnblanceWorst(uint8_t id);

    float SchneiderPM2xxx_Voltage_AB(uint8_t id);
    float SchneiderPM2xxx_Voltage_BC(uint8_t id);
    float SchneiderPM2xxx_Voltage_CA(uint8_t id);
    float SchneiderPM2xxx_Voltage_LL_Avg(uint8_t id);
    float SchneiderPM2xxx_Voltage_AN(uint8_t id);
    float SchneiderPM2xxx_Voltage_BN(uint8_t id);
    float SchneiderPM2xxx_Voltage_CN(uint8_t id);
    float SchneiderPM2xxx_Voltage_LN_Avg(uint8_t id);

    float SchneiderPM2xxx_VoltageUnblance_AB(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_BC(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_CA(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_LL_Worst(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_AN(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_BN(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_CN(uint8_t id);
    float SchneiderPM2xxx_VoltageUnblance_LN_Worst(uint8_t id);

    float SchneiderPM2xxx_ActivePowerA(uint8_t id);
    float SchneiderPM2xxx_ActivePowerB(uint8_t id);
    float SchneiderPM2xxx_ActivePowerC(uint8_t id);
    float SchneiderPM2xxx_ActivePowerTotal(uint8_t id);

    float SchneiderPM2xxx_ReactivePowerA(uint8_t id);
    float SchneiderPM2xxx_ReactivePowerB(uint8_t id);
    float SchneiderPM2xxx_ReactivePowerC(uint8_t id);
    float SchneiderPM2xxx_ReactivePowerTotal(uint8_t id);

    float SchneiderPM2xxx_ApparentPowerA(uint8_t id);
    float SchneiderPM2xxx_ApparentPowerB(uint8_t id);
    float SchneiderPM2xxx_ApparentPowerC(uint8_t id);
    float SchneiderPM2xxx_ApparentPowerTotal(uint8_t id);

    float SchneiderPM2xxx_PowerFactorA(uint8_t id);
    float SchneiderPM2xxx_PowerFactorB(uint8_t id);
    float SchneiderPM2xxx_PowerFactorC(uint8_t id);
    float SchneiderPM2xxx_PowerFactorTotal(uint8_t id);

    float SchneiderPM2xxx_Freq(uint8_t id);

    /* EASTRON 120CT Modbus Powermeter */
    bool SDM120CT_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t SDM120CT_searchAddress(void);
    int8_t SDM120CT_setAddress(uint8_t id, uint8_t new_id);
    float SDM120CT_Volt(uint8_t id);
    float SDM120CT_Freq(uint8_t id);
    float SDM120CT_Power(uint8_t id);
    float SDM120CT_Current(uint8_t id);
    float SDM120CT_AP_Power(uint8_t id);   // Apparent power
    float SDM120CT_Reac_Power(uint8_t id); // Reactive power
    float SDM120CT_Total_Energy(uint8_t id);
    float SDM120CT_POWER_FACTOR(uint8_t id);

    /* Wind speed sensor [PR-3000FSJT-N01] */
    bool WIND_RSFSN01_begin(uint8_t rx = RXD2, uint8_t tx = TXD2);
    int8_t WIND_RSFSN01_searchAddress(void);
    int8_t WIND_RSFSN01_setAddress(uint8_t id, uint8_t new_id);
    float WIND_RSFSN01_SPEED(uint8_t id);

    /* Sim300cz GPS module */
    bool SIM300CZ_begin(uint8_t rx = RXD3, uint8_t tx = TXD3);
    bool SIM300CZ_cmd(char *cmd);
    bool SIM300CZ_sms(char *number, char *sms);

    /* sht40 temperature & humidity sensor */
    void sht40_begin(uint8_t address = 0x44);
    float sht40_readTemperature(int units = CELSIUS);
    float sht40_readHumidity();
    void sht40_end();
};
#endif