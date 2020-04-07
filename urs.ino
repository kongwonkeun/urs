//
//
//

#include <Arduino.h>
#include <ArduinoBLE.h>

//================================
//
//
#define CONSOLE_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 115200
#define BLUETOOTH_AT_CMD_BAUDRATE 38400
#define TICK_INTERVAL 1 // millisec
#define TICK_FIRE 10 // 30 millisec --> 10 millisec
#define TICK_STOP_CHECK 1000 // 1 sec
#define INT_PIN 6 // digital pin 6
#define ADC_BITS 12 // adc resolution 12 bits (0 ~ 4095)
#define ADC_CH A0 // adc channel
#define ADC_REF AR_DEFAULT // adc reference
#define ROLLER_CIRCUMFERENCE 34.56 // 11cm * pi --> 34.56 cm
#define WHEEL_CIRCUMFERENCE 210.0 //

//================================
//
//
void MY_INFO(const char *info);
void MY_ECHO();
void MY_SEND(const char *data);
void MY_NOTI(const char *info);
void MY_WAIT();

void TC5_Handler(void);
void EI5_Handler(void);

void timer_setup(uint32_t interval);
bool timer_is_syncing();
void timer_start();
void timer_stop();
void timer_reset();

void adc_setup(uint32_t bits);
uint32_t get_dir();

void ext_int_setup();
uint32_t get_speed();

uint32_t map(unsigned long x, unsigned long x_min, unsigned long x_max, unsigned long y_min, unsigned long y_max);
void b2str(char key, unsigned char data, char *str);
void w2str(char key, unsigned int  data, char *str);
void b2str(unsigned char data, char *str);
void w2str(unsigned int  data, char *str);

void di_service_setup();
void di_service();
void csc_service_setup();
void csc_generate_measurement_data();
void csc_tick();
void csc_service();
void uart_service_setup();
void uart_service();

//================================
//
//
bool x_serial_con = false;
bool x_serial_bt = false;
bool x_serial_ble = false;

uint32_t m_dir = 0; // centimeter
uint32_t m_max_adc_value = 4095; // max value of 12 bits
uint32_t m_max_volt = 3300; // milli-volt
bool m_polling = true;

uint32_t m_ticks = 0;
uint32_t m_millisec = 0;
uint32_t m_sec = 0;

uint32_t m_rotations = 0;
uint32_t m_cps = 0; // centimeter per sec
uint32_t m_last_cps = 0;
uint32_t m_last_rot_time = 0;

//================================
//
//
void setup()
{
    // console
    Serial.begin(CONSOLE_BAUDRATE);
    MY_WAIT();
    if (Serial) {
        x_serial_con = true;
    }
    MY_INFO("\n----1111----");
    MY_INFO("\n----polling rate: 10ms----");
    MY_INFO("\n----fw ver: 1.2.1----");

    // bluetooth
    Serial1.begin(BLUETOOTH_BAUDRATE);
    MY_WAIT();
    if (Serial1) {
        x_serial_bt = true;
    }
    MY_INFO("\n----2222----");

    // ble services
    if (BLE.begin()) {
        BLE.setLocalName("ultiracer");
        BLE.setDeviceName("ultiracer");
        BLE.setAppearance(1154); // csc: cycling speed sensor
        csc_service_setup();
        di_service_setup();

        MY_INFO("\n----3333----");
        x_serial_ble = true;
    }
    MY_INFO("\n----4444----\n");
    MY_WAIT();
    MY_INFO("\n");

    // external interrupt handler
    ext_int_setup(); // external interrupt handler is EI5_Handler

    // adc setup
    adc_setup(ADC_BITS);

    // tick timer
    timer_setup(TICK_INTERVAL);
    timer_start(); // tick handler is TC5_Handler

}

//================================
//
//
void loop()
{
    MY_ECHO();

    BLEDevice central = BLE.central();

    if (central) {
        m_polling = false;
        MY_INFO("\n----5555----");
        while (central.connected()) {
            csc_service();
        }
        MY_INFO("\n----6666----");
        m_polling = true;
    }

}

//================================
//
//
void TC5_Handler(void)
{
    m_ticks++;
    m_millisec++;

    if (!(m_ticks % TICK_FIRE) && m_polling) {
        unsigned char d;
        unsigned int w;
        char s[10];

        w = get_speed();
        w2str('V', w, s);
        MY_SEND(s);

        d = get_dir();
        b2str('D', d, s);
        MY_SEND(s);
    }
    
    if (!(m_millisec % TICK_STOP_CHECK)) {
        //MY_INFO("T"); // sec tick
        if (m_last_cps == m_cps) {
            m_cps = 0;
        }
        m_last_cps = m_cps;
        csc_tick();
    }
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

void EI5_Handler(void)
{
    //MY_INFO("R");
    m_rotations++;
    if (m_last_rot_time < m_millisec) {
        m_cps = (ROLLER_CIRCUMFERENCE * 1000) / (m_millisec - m_last_rot_time);
    }
    m_last_rot_time = m_millisec;
}

//================================
//
//
void timer_setup(uint32_t interval)
{
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN
        | GCLK_CLKCTRL_GEN_GCLK0
        | GCLK_CLKCTRL_ID(GCM_TC4_TC5)
    );
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    //TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    //TC5->COUNT16.CC[0].reg  = (uint16_t)((SystemCoreClock / 1000) * interval); // 1MHz at Reset
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; // prescaler = 1024
    TC5->COUNT16.CC[0].reg  = (uint16_t)((46875 / 1000) * interval); // 48MHz/1024 = 46875 (1sec)
    while (timer_is_syncing()) {}

    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (timer_is_syncing()) {}
}

bool timer_is_syncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void timer_start()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (timer_is_syncing()) {}
}

void timer_stop()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (timer_is_syncing()) {}
}

void timer_reset()
{
    TC5->COUNT16.CTRLA.reg &= TC_CTRLA_SWRST;
    while (timer_is_syncing()) {}
    while (TC5->COUNT16.CTRLA.bit.SWRST) {}
}

//================================
//
//
void adc_setup(uint32_t bits)
{
    analogReference(ADC_REF);
    analogReadResolution(bits);
}

uint32_t get_dir()
{
    uint32_t d;
    uint32_t v;
    uint32_t c;

    d = analogRead(ADC_CH);
    v = map(d, 0, m_max_adc_value, 0, m_max_volt);
    c = 1000;
    m_dir = (c * 23) / (v + 50);
    return m_dir;
}

//================================
//
//
void ext_int_setup()
{
    pinMode(INT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), EI5_Handler, RISING);
}

uint32_t get_speed()
{
    return m_cps;
}

//================================
//
//
uint32_t map(unsigned long x, unsigned long x_min, unsigned long x_max, unsigned long y_min, unsigned long y_max)
{
    uint32_t y = ((x - x_min) * (y_max - y_min) / (x_max - x_min)) + y_min;
    return y;
}

void b2str(char key, unsigned char data, char *str)
{
    unsigned int num = data;
    str[0] = key;
    str[1] = 0x30 + (num  / 100);
    str[2] = 0x30 + ((num % 100) / 10);
    str[3] = 0x30 + (num  % 10);
    str[4] = 0;
}

void b2str(unsigned char data, char *str)
{
    unsigned int num = data;
    str[0] = 0x30 + (num  / 100);
    str[1] = 0x30 + ((num % 100) / 10);
    str[2] = 0x30 + (num  % 10);
    str[3] = 0;
}

void w2str(char key, unsigned int data, char *str)
{
    unsigned int  num = data;
    str[0] = key;
    str[1] = 0x30 + (num  / 10000);
    str[2] = 0x30 + ((num % 10000) / 1000);
    str[3] = 0x30 + ((num % 1000) / 100);
    str[4] = 0x30 + ((num % 100) / 10);
    str[5] = 0x30 + (num  % 10);
    str[6] = 0;
}

void w2str(unsigned int data, char *str)
{
    unsigned int  num = data;
    str[0] = 0x30 + (num  / 10000);
    str[1] = 0x30 + ((num % 10000) / 1000);
    str[2] = 0x30 + ((num % 1000) / 100);
    str[3] = 0x30 + ((num % 100) / 10);
    str[4] = 0x30 + (num  % 10);
    str[5] = 0;
}

//================================
//
//
BLEService b_di_service("180A");
BLEStringCharacteristic b_manufacturer_name_characteristic("2A29", BLERead, 14);
BLEStringCharacteristic b_model_number_characteristic("2A24", BLERead, 5);
BLEStringCharacteristic b_serial_number_characteristic("2A25", BLERead, 9);

void di_service_setup()
{
    BLE.setAdvertisedService(b_di_service);
    b_di_service.addCharacteristic(b_manufacturer_name_characteristic);
    b_di_service.addCharacteristic(b_model_number_characteristic);
    b_di_service.addCharacteristic(b_serial_number_characteristic);
    BLE.addService(b_di_service);
    b_manufacturer_name_characteristic.writeValue("RealDesignTech");
    b_model_number_characteristic.writeValue("RDT-1");
    b_serial_number_characteristic.writeValue("SN-000001");
    BLE.advertise();
}

void di_service() {}

BLEService b_csc_service("1816");
BLECharacteristic b_csc_measurement("2A5B", BLENotify, 7, true); // 11 -> 7
BLEWordCharacteristic b_csc_feature("2A5C", BLERead);
BLEByteCharacteristic b_csc_sensor_location("2A5D", BLERead);

uint8_t  b_csc_measurement_data[12];
uint32_t b_csc_cumulative_wheel_revolution;
uint16_t b_csc_last_wheel_event_time;
uint16_t b_csc_cumulative_crank_revolution;
uint16_t b_csc_last_crank_event_time;
bool b_csc_notify = false;

void csc_service_setup()
{
    BLE.setAdvertisedService(b_csc_service);
    b_csc_service.addCharacteristic(b_csc_measurement);
    b_csc_service.addCharacteristic(b_csc_feature);
    b_csc_service.addCharacteristic(b_csc_sensor_location);
    BLE.addService(b_csc_service);
    csc_generate_measurement_data();
    b_csc_measurement.writeValue(b_csc_measurement_data, 1); //
    b_csc_feature.writeValue(0x0001); // wheel revolution supported
    b_csc_sensor_location.writeValue(12); // rear wheel
    BLE.advertise();
}

void csc_generate_measurement_data()
{
    b_csc_measurement_data[0] = 0x01; // flags is wheel revolution data support
    b_csc_measurement_data[1] = (b_csc_cumulative_wheel_revolution >> 24) & 0xff;
    b_csc_measurement_data[2] = (b_csc_cumulative_wheel_revolution >> 16) & 0xff;
    b_csc_measurement_data[3] = (b_csc_cumulative_wheel_revolution >> 8) & 0xff;
    b_csc_measurement_data[4] = (b_csc_cumulative_wheel_revolution) & 0xff;
    b_csc_measurement_data[5] = (b_csc_last_wheel_event_time >> 8) & 0xff;
    b_csc_measurement_data[6] = (b_csc_last_wheel_event_time) & 0xff;
    b_csc_measurement_data[7] = 0;
    
    // if support crank revolution then
    //b_csc_measurement_data[7]  = (b_csc_cumulative_crank_revolution >> 8) & 0xff;
    //b_csc_measurement_data[8]  = (b_csc_cumulative_crank_revolution) & 0xff;
    //b_csc_measurement_data[9]  = (b_csc_last_crank_event_time >> 8) & 0xff;
    //b_csc_measurement_data[10] = (b_csc_last_crank_event_time) & 0xff;
    //b_csc_measurement_data[11] = 0;
}

void csc_tick()
{
    if (!b_csc_notify) {
        b_csc_notify = true;
    }
}

void csc_service()
{
    if (b_csc_notify) {
        b_csc_cumulative_wheel_revolution = m_rotations * ROLLER_CIRCUMFERENCE / WHEEL_CIRCUMFERENCE;
        b_csc_last_wheel_event_time = (m_last_rot_time * 1024 / 1000) % 65536;
        csc_generate_measurement_data();
        b_csc_measurement.writeValue(b_csc_measurement_data, 7);
        //MY_INFO("+");
        b_csc_notify = false;
    }
}

BLEService b_uart_service("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLEStringCharacteristic b_uart_tx_characteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e", BLEWrite, 20); // client tx
BLEStringCharacteristic b_uart_rx_characteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e", BLERead | BLENotify, 20); // client rx

void uart_service_setup()
{
    BLE.setAdvertisedService(b_uart_service);
    b_uart_service.addCharacteristic(b_uart_tx_characteristic);
    b_uart_service.addCharacteristic(b_uart_rx_characteristic);
    BLE.addService(b_uart_service);
    BLE.advertise();
}

void uart_service()
{
    b_uart_rx_characteristic.writeValue("hi");
}

//================================
//
//
void MY_INFO(const char *info)
{
    if (x_serial_con) Serial.write(info);
    //if (x_serial_bt) Serial1.write(info);
}

void MY_ECHO()
{
    // bluetooth input handler
    if (x_serial_bt && Serial1.available()) {
        char c = Serial1.read();
        if (x_serial_con) Serial.write(c);
        Serial1.write(c);
    }

    // console input handler
    if (x_serial_con && Serial.available()) {
        char c = Serial.read();
        if (x_serial_bt) Serial1.write(c);
        Serial.write(c);
    }
}

void MY_SEND(const char *data)
{
    if (x_serial_bt) Serial1.write(data);
}

void MY_NOTI(const char *info) {}

void MY_WAIT()
{
    delay(1000);
}

//
//
//
