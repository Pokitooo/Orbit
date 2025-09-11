#include <Arduino.h>
#include "Arduino_Extended.h"
#include "lib_xcore"
#include <STM32LowPower.h>
#include "vt_linalg"
#include "vt_kalman"
#include "File_Utility.h"

#include <Wire.h>
#include <SPI.h>

#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "orbit_peripheral_def.h"
#include "orbit_pin_def.h"
#include "orbit_state_def.h"

// Device specific
#define THIS_FILE_PREFIX "ORBIT_LOGGER_"
#define THIS_FILE_EXTENSION "CSV"

using time_type = uint32_t;
using smart_delay = vt::smart_delay<time_type>;
using on_off_timer = vt::on_off_timer<time_type>;
using task_type = vt::task_t<vt::smart_delay, time_type>;

template <size_t N>
using dispatcher_type = vt::task_dispatcher<N, vt::smart_delay, time_type>;

on_off_timer::interval_params buzzer_intervals(nova::config::BUZZER_ON_INTERVAL,
                                               nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL));
bool enable_buzzer = true;

// i2c
TwoWire i2c1(PIN_SDA, PIN_SCL);
Adafruit_BME280 bme1;
Adafruit_H3LIS331 imu = Adafruit_H3LIS331();

// UARTS
HardwareSerial gnssSerial(PIN_RX, PIN_TX);
TinyGPSPlus lc86;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);
using sd_t = SdFat32;
using file_t = File32;
FsUtil<sd_t, file_t> sd_util;

// LoRa

// LoRa State
enum class LoRaState
{
    IDLE = 0,
    TRANSMITTING,
    RECEIVING
};

int status_lora;
volatile bool tx_flag = false;
volatile bool rx_flag = false;

volatile LoRaState lora_state = LoRaState::IDLE;
uint32_t lora_tx_end_time;
float lora_rssi;

SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
    float center_freq = 925.00'000f; // MHz
    float bandwidth = 125.f;         // kHz
    uint8_t spreading_factor = 9;    // SF: 6 to 12
    uint8_t coding_rate = 8;         // CR: 5 to 8
    uint8_t sync_word = 0x12;        // Private SX1262
    int8_t power = 22;               // up to 22 dBm for SX1262
    uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// ADC
constexpr size_t ADC_BITS(10);
constexpr float ADC_DIVIDER = ((1 << ADC_BITS) - 1);
constexpr float VREF = 3300.f;              // V
constexpr float VREF_VMON = 1938.46153846f; // V
constexpr float R_SHUNT = 2.e-3f;           // Ohm

// Servo
struct SERVO
{
    uint32_t pin;
    SERVO(uint32_t pin) : pin(pin)
    {
        pinMode(pin, OUTPUT);
    }

    void write(int angle)
    {
        const int cpw = map(angle, 0, 180, 500, 2500);
        for (size_t i = 0; i < max(5, (angle + 17) / 18); ++i)
        {
            digitalWrite(pin, HIGH);
            delayMicroseconds(cpw);
            digitalWrite(pin, LOW);
            delayMicroseconds(20000);
        }
    }
};

// SERVO servo_a(servoPinA);
// SERVO servo_b(servoPinB);

// DATA

struct Data
{
    // 40 bits
    uint32_t timestamp;
    uint8_t counter;

    nova::state_t ps;
    nova::pyro_state_t pyro_a{};
    nova::pyro_state_t pyro_b{};

    // 160 bits
    double gps_latitude;
    double gps_longitude;
    float gps_altitude;

    // 96 bits
    float altitude;
    float temp;
    float press;

    // 384 bits
    struct
    {
        vec3_u<double> acc;
    } imu;

    float currentServo;
    float currentEXT;
    float voltageMon;

    uint8_t last_ack{};
    uint8_t last_nack{};

} data;

struct peripherals_t
{
    union
    {
        struct
        {
            uint8_t imu;
            uint8_t bme;
            uint8_t sd;
        };
    };

    template <traits::has_ostream OStream>
    OStream &operator>>(OStream &ostream)
    {
        ostream << "IMU imu42688: " << imu << stream::crlf;
        ostream << "BME280: " << bme << stream::crlf;
        ostream << "SD: " << sd << stream::crlf;
        return ostream;
    }
};
peripherals_t pvalid;

// Software filters
constexpr size_t FILTER_ORDER = 4;
constexpr double dt_base = 0.1;
constexpr double covariance = 0.01;
constexpr double alpha = 0.;
constexpr double beta = 0.;
constexpr double G = 9.81;

struct software_filters
{
    vt::kf_pos<FILTER_ORDER> bme_pres{dt_base, covariance, alpha, beta};

    struct
    {
        vec3_u<vt::kf_pos<4>> acc{dt_base, covariance, alpha, beta};
        vec3_u<vt::kf_pos<4>> gyro{dt_base, covariance, alpha, beta};
    } imu_1;

    vt::kf_pos<FILTER_ORDER> altitude{dt_base, covariance, alpha, beta};
    vt::kf_acc<FILTER_ORDER> acceleration{dt_base, covariance, alpha, beta};
} filters;

struct bme_ref_t
{
    Adafruit_BME280 &bmef;
    float &temp;
    float &pres;
    float &alt;
    vt::kf_pos<FILTER_ORDER> &kf;

    bme_ref_t(Adafruit_BME280 &t_bmef, float &t_temp, float &t_pres, float &t_alt, vt::kf_pos<FILTER_ORDER> &t_kf)
        : bmef{t_bmef}, temp{t_temp}, pres{t_pres}, alt{t_alt}, kf{t_kf} {}
};

bme_ref_t bme_ref = {bme1, data.temp, data.press, data.altitude, filters.bme_pres};

// State
bool sdstate;

// Software control
dispatcher_type<32> dispatcher;
bool launch_override = false;
struct
{
    float altitude_offset{};
    float apogee;
} ground_truth;

// Communication data
String constructed_data;
String tx_data;
time_type tx_interval = nova::config::TX_IDLE_INTERVAL;
time_type log_interval = nova::config::LOG_IDLE_INTERVAL;

HardwareTimer timer_buz(TIM3);

// variables
volatile bool wake_flag = false;

extern void led_control();

extern void read_m10q();

extern void read_gnss();

extern void read_bme(bme_ref_t *bme);

extern void read_imu();

extern void synchronize_kf();

// extern void read_current();

extern void construct_data();

extern void transmit_receive_data();

extern void save_data(time_type *interval_ms);

extern void print_data();

extern void fsm_eval();

extern void set_rxflag();

template <typename SdType, typename FileType>
extern void init_storage(FsUtil<SdType, FileType> &sd_util_instance);

extern void handle_command(String rx_message);

void setup()
{
    Serial.begin(460800);
    delay(2000);

    i2c1.begin();
    i2c1.setClock(300000u);

    spi1.begin();

    // GPIO
    pinMode(ledPin1, OUTPUT); // LED
    pinMode(ledPin2, OUTPUT); // LED
    pinMode(ledPin3, OUTPUT); // LED
    pinMode(ledPin4, OUTPUT); // LED

    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
    delay(200); // 200 ms ON

    // Turn all LEDs OFF
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    // variable
    static bool state;

    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH); // deselect LoRa

    // pinMode(PIN_NSS_SD, OUTPUT);
    // digitalWrite(PIN_NSS_SD, HIGH); // idle SD CS high

    // SD
    pvalid.sd = sd_util.sd().begin(sd_config);
    if (pvalid.sd)
    {
        init_storage(sd_util);
        Serial.println("SD SUCESS");
        digitalWrite(ledPin1, 1);
    }

    // LoRa
    uint16_t lora_state = lora.begin(params.center_freq,
                                     params.bandwidth,
                                     params.spreading_factor,
                                     params.coding_rate,
                                     params.sync_word,
                                     params.power,
                                     params.preamble_length,
                                     0,
                                     false);
    state = state || lora.explicitHeader();
    state = state || lora.setCRC(true);
    state = state || lora.autoLDRO();

    lora.setPacketReceivedAction(set_rxflag);

    if (lora_state == RADIOLIB_ERR_NONE)
    {
        Serial.println("SX1262 initialized successfully!");
        digitalWrite(ledPin2, 1);
    }
    else
    {
        Serial.print("Initialization failed! Error: ");
        Serial.println(lora_state);
        while (true)
            ;
    }

    // imu42688
    pvalid.imu = imu.begin_I2C(0x18, &i2c1);
    if (pvalid.imu)
    {
        imu.setRange(H3LIS331_RANGE_400_G); // 6, 12, or 24 G
        imu.setDataRate(LIS331_DATARATE_1000_HZ);
        digitalWrite(ledPin3, 1);
    }

    // lc86g UARTS
    gnssSerial.begin(115200);

    // bme280(0x76)
    float gnd = 0.f;

    pvalid.bme = bme1.begin(0x76, &i2c1);
    if (pvalid.bme)
    {
        bme1.SAMPLING_X16;
        for (size_t i = 0; i < 20; ++i)
        {
            read_bme(&bme_ref);
        }
        gnd += data.altitude;
    }
    ground_truth.altitude_offset = gnd;

    auto pyro_cutoff = []
    {
        static smart_delay sd_a(nova::config::PYRO_ACTIVATE_INTERVAL, millis);
        static smart_delay sd_b(nova::config::PYRO_ACTIVATE_INTERVAL, millis);
        static bool flag_a = false, flag_b = false;

        if (data.pyro_a == nova::pyro_state_t::FIRING)
        {
            if (!flag_a)
            {
                sd_a.reset();
                flag_a = true;
            }
            else
            {
                sd_a([]
                     {
                    data.pyro_a = nova::pyro_state_t::FIRED;
                    flag_a = false; });
            }
        }

        if (data.pyro_b == nova::pyro_state_t::FIRING)
        {
            if (!flag_b)
            {
                sd_b.reset();
                flag_b = true;
            }
            else
            {
                sd_b([]
                     {
                    data.pyro_b = nova::pyro_state_t::FIRED;
                    flag_b = false; });
            }
        }
    };

    // Tasks
    dispatcher << task_type(led_control, 1000ul, millis, 0)
               << task_type(pyro_cutoff, 0)

               << task_type(fsm_eval, 25ul, millis, 0)

               << (task_type(read_imu, 25ul, millis, 1), pvalid.imu)
               << (task_type(read_bme, &bme_ref, 50ul, millis, 1), pvalid.bme)
               << task_type(synchronize_kf, 25ul, millis, 1)

               << task_type(read_gnss, 100ul, millis, 2)

               << task_type(transmit_receive_data, 1000ul, millis, 252)
               //    << task_type(print_data, 1000ul, millis, 253)
               << task_type(construct_data, 25ul, millis, 254)
               << (task_type(save_data, &log_interval, 255), pvalid.sd);

    // Low power mode
    // LowPower.begin();
    dispatcher.reset();
}

void loop()
{
    dispatcher();
}

void led_control()
{
    digitalWrite(ledPin4, !digitalRead(ledPin4));
}

void read_gnss()
{

    while (gnssSerial.available())
    {
        lc86.encode(gnssSerial.read());
        data.timestamp = lc86.time.value();
    }

    if (lc86.location.isUpdated())
    {
        data.gps_latitude = lc86.location.lat();
        data.gps_longitude = lc86.location.lng();
        data.gps_altitude = lc86.altitude.meters();
    }
}

void read_bme(bme_ref_t *bme)
{
    static uint32_t t_prev = millis();

    if (const float t = bme1.readTemperature(); t > 0.)
    {
        bme->temp = t;
    }

    if (const float p = bme1.readPressure() / 100.0F; p > 0.)
    {
        bme->kf.update_dt(millis() - t_prev);
        bme->kf.kf.predict().update(p);
        bme->pres = static_cast<float>(bme->kf.kf.state);
    }

    bme->alt = pressure_altitude(data.press);

    t_prev = millis();
}

void read_imu()
{
    static uint32_t t_prev = millis();

    imu.read(); // get X Y and Z data at once

    /* Or....get a new sensor event, normalized */
    sensors_event_t event;
    imu.getEvent(&event);

    data.imu.acc.x = event.acceleration.x;
    data.imu.acc.y = event.acceleration.y;
    data.imu.acc.z = event.acceleration.z;

    for (size_t i = 0; i < 3; ++i)
    {
        const uint32_t dt = millis() - t_prev;
        filters.imu_1.acc.values[i].update_dt(dt);
        filters.imu_1.acc.values[i].kf.predict().update(data.imu.acc.values[i]);
        data.imu.acc.values[i] = filters.imu_1.acc.values[i].kf.state;
    }

    t_prev = millis();
}

void synchronize_kf()
{
    static uint32_t t_prev = millis();

    const double total_acc = algorithm::root_sum_square(data.imu.acc.x, data.imu.acc.y, data.imu.acc.z);

    filters.altitude.update_dt(millis() - t_prev);
    filters.acceleration.update_dt(millis() - t_prev);
    filters.altitude.kf.predict().update(data.altitude);
    filters.acceleration.kf.predict().update(total_acc - G);

    t_prev = millis();
}

void construct_data()
{
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        << data.counter
        << data.timestamp

        << nova::state_string(data.ps)
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.altitude, 4)
        << String(ground_truth.apogee, 4)

        << nova::pyro_state_string(data.pyro_a)
        << nova::pyro_state_string(data.pyro_b)

        << data.temp
        << data.press

        << data.imu.acc.x << data.imu.acc.y << data.imu.acc.z

        << data.last_ack
        << data.last_nack;

    tx_data = "";
    csv_stream_crlf(tx_data)
        << "<3>" // DEVICE NO
        << params.center_freq
        << data.counter

        << nova::state_string(data.ps)
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.altitude, 4)
        << String(ground_truth.apogee, 4)

        << data.voltageMon

        << data.last_ack
        << data.last_nack;
}

template <typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance)
{
    sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
    sd_util_instance.template open_one<FsMode::WRITE>();
}

void save_data(time_type *interval_ms)
{
    static time_type prev = *interval_ms;
    static smart_delay sd(*interval_ms, millis);
    static smart_delay sd_save(1000, millis);

    if (prev != *interval_ms)
    {
        sd.set_interval(*interval_ms);
        prev = *interval_ms;
    }

    sd([&]() -> void
       {
           sd_util.file() << constructed_data;
           //  Serial.println("Data written and flushed.");
       });

    sd_save([&]() -> void
            { sd_util.flush_one(); });
}

void transmit_receive_data()
{
    lora.startTransmit(tx_data);
    return;

    static smart_delay nb_trx(2000ul, millis);

    // Tx Loop
    nb_trx([&]() -> void
           {
    lora_state = LoRaState::TRANSMITTING;
    lora.startTransmit(tx_data);
    lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
    Serial.println("[TRANSMITTING...]"); 
    ++data.counter; });

    // Set Tx Done
    if (millis() > lora_tx_end_time &&
        lora_state != LoRaState::RECEIVING)
    {
        tx_flag = true;
        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }

    // Set Rx Done
    if (lora.getPacketLength() > 0 &&
        lora.getRSSI() != lora_rssi)
    {
        rx_flag = true;
    }

    // On Transmit
    if (tx_flag)
    {
        Serial.print("[TRANSMITTED] ");
        Serial.println(tx_data);
        tx_flag = false;
    }

    // On Receive
    if (rx_flag)
    {
        String rx_string;
        lora.readData(rx_string);
        lora_rssi = lora.getRSSI();
        Serial.print("[RECEIVED]    ");
        Serial.println(rx_string);
        rx_flag = false;

        handle_command(rx_string);

        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }
}

void handle_command(String rx_message)
{
    if (rx_message.substring(0, 4) != "cmd ")
    {
        // Return if cmd header is invalid
        Serial.print("Receive: ");
        Serial.println(rx_message);
        ++data.last_nack;
        return;
    }

    String command = rx_message.substring(4);
    command.trim();

    ++data.last_ack;

    if (command == "ping" || command == "wake" || command == "on")
    {
        // <--- Maybe a wakeup command --->
    }
    else if (command == "arm")
    {
        // <--- Arming the rocket --->
        data.ps = nova::state_t::ARMED;
        tx_interval = nova::config::TX_ARMED_INTERVAL;
        log_interval = nova::config::LOG_ARMED_INTERVAL;
        data.pyro_a = nova::pyro_state_t::ARMED;
        data.pyro_b = nova::pyro_state_t::ARMED;
    }
    else if (command == "disarm")
    {
        data.ps = nova::state_t::IDLE_SAFE;
        tx_interval = nova::config::TX_IDLE_INTERVAL;
        log_interval = nova::config::LOG_IDLE_INTERVAL;
        data.pyro_a = nova::pyro_state_t::DISARMED;
        data.pyro_b = nova::pyro_state_t::DISARMED;
    }
    else if (command == "pad")
    {
        // <--- Prelaunch operation --->
        // Must be armed first!
        if (data.ps == nova::state_t::ARMED)
        {
            data.ps = nova::state_t::PAD_PREOP;
            tx_interval = nova::config::TX_PAD_PREOP_INTERVAL;
            log_interval = nova::config::LOG_PAD_PREOP_INTERVAL;
        }
    }
    else if (command == "servo-a-set")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_a.write(nova::config::SERVO_A_SET);
        }
    }
    else if (command == "servo-a-lock")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_a.write(nova::config::SERVO_A_LOCK);
        }
    }
    else if (command == "servo-a-deploy")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_a.write(nova::config::SERVO_A_DEPLOY);
            data.pyro_a = nova::pyro_state_t::FIRING;
        }
    }
    else if (command == "servo-a-set")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_b.write(nova::config::SERVO_B_SET);
        }
    }
    else if (command == "servo-b-lock")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_b.write(nova::config::SERVO_B_LOCK);
        }
    }
    else if (command == "servo-b-deploy")
    {
        if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
        {
            // servo_b.write(nova::config::SERVO_B_DEPLOY);
            data.pyro_b = nova::pyro_state_t::FIRING;
        }
    }
    else if (command == "launch-override")
    {
        launch_override = true;
    }
    else if (command == "recover")
    {
        // <--- Rocket landing confirmed --->
        data.ps = nova::state_t::RECOVERED_SAFE;
        if (data.pyro_a != nova::pyro_state_t::FIRED)
            data.pyro_a = nova::pyro_state_t::DISARMED;
        if (data.pyro_b != nova::pyro_state_t::FIRED)
            data.pyro_b = nova::pyro_state_t::DISARMED;
    }
    else if (command == "zero")
    {
        // <--- Zero barometric altitude --->
        read_bme(&bme_ref);
        ground_truth.altitude_offset = data.press;
    }
    else if (command == "sleep")
    {
        // <--- Put the device into deep sleep mode (power saving) --->
        // PINS_OFF();
        // LowPower.deepSleep();
    }
    else if (command == "shutdown")
    {
        // <--- Shutdown the device --->
        PINS_OFF();

        if (sdstate)
        {
            sd_util.close_one();
        }

        // LowPower.deepSleep();

        __NVIC_SystemReset();
    }
    else if (command == "reboot" || command == "restart")
    {
        // <--- Reboot/reset the device --->
        if (sdstate)
        {
            sd_util.close_one();
        }
        __NVIC_SystemReset();
    }
    else
    {
        Serial.println(command);
        // <--- Unknown command: send back nack --->
        ++data.last_nack;
        --data.last_ack;
    }
}

void fsm_eval()
{

    static bool state_satisfaction = false;
    int32_t static launched_time = 0;
    static algorithm::Sampler sampler[2];

    const double alt_x = filters.altitude.kf.state_vector[0] - ground_truth.altitude_offset;
    const double vel_x = filters.altitude.kf.state_vector[1];
    const double acc = filters.acceleration.kf.state_vector[2];

    switch (data.ps)
    {
    case nova::state_t::STARTUP:
    {
        // Next: always transfer
        data.ps = nova::state_t::IDLE_SAFE;
        break;
    }
    case nova::state_t::IDLE_SAFE:
    {
        //  <--- Next: wait for uplink --->
        data.ps = nova::state_t::ARMED;
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL);
        break;
    }
    case nova::state_t::ARMED:
    {
        // <--- Next: wait for uplink --->
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ARMED_INTERVAL);
        data.ps = nova::state_t::PAD_PREOP;

        if (launch_override)
        {
            data.ps = nova::state_t::PAD_PREOP;
            tx_interval = nova::config::TX_PAD_PREOP_INTERVAL;
            log_interval = nova::config::LOG_PAD_PREOP_INTERVAL;
        }

        break;
    }
    case nova::state_t::PAD_PREOP:
    {
        // !!!!! Next: DETECT launch !!!!!
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_PAD_PREOP_INTERVAL);

        static on_off_timer tim(nova::config::alg::LAUNCH_TON / 2, nova::config::alg::LAUNCH_TON / 2, millis);

        if (!state_satisfaction)
        {
            sampler[0].add(acc >= nova::config::alg::LAUNCH_ACC);
            sampler[1].add(acc >= nova::config::alg::LAUNCH_ACC);

            tim.on_rising([&]
                          {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

            tim.on_falling([&]
                           {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
        }

        state_satisfaction |= launch_override;

        if (state_satisfaction)
        {
            launched_time = millis();
            data.ps = nova::state_t::POWERED;
            state_satisfaction = false;
            sampler[0].reset();
            sampler[1].reset();
            tx_interval = nova::config::TX_ASCEND_INTERVAL;
            log_interval = nova::config::LOG_ASCEND_INTERVAL;
        }

        break;
    }
    case nova::state_t::POWERED:
    {
        // !!!!! Next: DETECT motor burnout !!!!!
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ASCEND_INTERVAL);

        static on_off_timer tim(nova::config::alg::BURNOUT_TON / 2, nova::config::alg::BURNOUT_TON / 2, millis);

        if (!state_satisfaction)
        {
            sampler[0].add(acc < nova::config::alg::LAUNCH_ACC);
            sampler[1].add(acc < nova::config::alg::LAUNCH_ACC);

            tim.on_rising([&]
                          {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[0].reset(); });

            tim.on_falling([&]
                           {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[1].reset(); });
        }

        state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MAX;

        if (state_satisfaction)
        {
            data.ps = nova::state_t::COASTING;
            state_satisfaction = false;
            sampler[0].reset();
            sampler[1].reset();
        }

        break;
    }
    case nova::state_t::COASTING:
    {
        // !!!!! Next: DETECT apogee !!!!!
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ASCEND_INTERVAL);

        static on_off_timer tim(nova::config::alg::APOGEE_SLOW_TON / 2, nova::config::alg::APOGEE_SLOW_TON / 2, millis);

        if (!state_satisfaction)
        {
            sampler[0].add(vel_x <= nova::config::alg::APOGEE_VEL);
            sampler[1].add(vel_x <= nova::config::alg::APOGEE_VEL);

            tim.on_rising([&]
                          {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MIN;
          }
          sampler[0].reset(); });

            tim.on_falling([&]
                           {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MIN;
          }
          sampler[1].reset(); });

            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MAX;
        }

        if (state_satisfaction)
        {
            data.ps = nova::state_t::DROGUE_DEPLOY;
            state_satisfaction = false;
            sampler[0].reset();
            sampler[1].reset();
        }

        break;
    }
    case nova::state_t::DROGUE_DEPLOY:
    {
        // Next: activate and always transfer
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

        static bool fired = false;

        if (!fired)
        {
            // servo_a.write(nova::config::SERVO_A_DEPLOY); // Deploy
            data.pyro_a = nova::pyro_state_t::FIRING;
            fired = true;
        }
        else if (data.pyro_a == nova::pyro_state_t::FIRED)
        {
            data.ps = nova::state_t::DROGUE_DESCEND;
        }

        break;
    }
    case nova::state_t::DROGUE_DESCEND:
    {
        // !!!!! Next: DETECT main deployment altitude !!!!!
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

        static on_off_timer tim(nova::config::alg::MAIN_DEPLOYMENT_TON / 2, nova::config::alg::MAIN_DEPLOYMENT_TON / 2, millis);

        if (!state_satisfaction)
        {
            sampler[0].add(alt_x <= nova::config::alg::MAIN_ALTITUDE);
            sampler[1].add(alt_x <= nova::config::alg::MAIN_ALTITUDE);

            tim.on_rising([&]
                          {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

            tim.on_falling([&]
                           {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
        }

        if (state_satisfaction)
        {
            data.ps = nova::state_t::MAIN_DEPLOY;
            state_satisfaction = false;
            sampler[0].reset();
            sampler[1].reset();
            tx_interval = nova::config::TX_DESCEND_INTERVAL;
            log_interval = nova::config::LOG_DESCEND_INTERVAL;
        }

        break;
    }
    case nova::state_t::MAIN_DEPLOY:
    {
        // Next: activate and always transfer
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

        static bool fired = false;

        if (!fired)
        {
            // servo_b.write(nova::config::SERVO_B_DEPLOY);
            data.pyro_b = nova::pyro_state_t::FIRING;
            fired = true;
        }
        else if (data.pyro_b == nova::pyro_state_t::FIRED)
        {
            data.ps = nova::state_t::MAIN_DESCEND;
        }

        break;
    }
    case nova::state_t::MAIN_DESCEND:
    {
        // !!!!! Next: DETECT landing !!!!!
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

        static on_off_timer tim(nova::config::alg::LANDING_TON / 2, nova::config::alg::LANDING_TON / 2, millis);

        if (!state_satisfaction)
        {
            const bool stable = algorithm::is_zero(vel_x, 0.5);
            sampler[0].add(stable);
            sampler[1].add(stable);

            tim.on_rising([&]
                          {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

            tim.on_falling([&]
                           {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
        }

        if (state_satisfaction)
        {
            data.ps = nova::state_t::LANDED;
            state_satisfaction = false;
            sampler[0].reset();
            sampler[1].reset();
            tx_interval = nova::config::TX_IDLE_INTERVAL;
            log_interval = nova::config::LOG_IDLE_INTERVAL;
        }

        break;
    }
    case nova::state_t::LANDED:
    {
        // <--- Next: wait for uplink --->
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

        break;
    }
    case nova::state_t::RECOVERED_SAFE:
    {
        // Sink state (requires reboot)
        buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL);

        do_nothing();
        break;
    }
    default:
        __builtin_unreachable();
    }

    if (alt_x > ground_truth.apogee)
    {
        ground_truth.apogee = alt_x;
    }
}

void print_data()
{

    Serial.println("====== DATA ======");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);

    Serial.print("Counter: ");
    Serial.println(data.counter);

    Serial.println("---- STATES ----");
    Serial.print("PS: ");
    Serial.println(nova::state_string(data.ps));
    Serial.print("Pyro A: ");
    Serial.println(nova::pyro_state_string(data.pyro_a));
    Serial.print("Pyro B: ");
    Serial.println(nova::pyro_state_string(data.pyro_b));

    Serial.println("---- GPS ----");
    Serial.print("Latitude: ");
    Serial.println(data.gps_latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(data.gps_longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(data.gps_altitude, 4);

    Serial.println("---- ENV ----");
    Serial.print("Temperature: ");
    Serial.println(data.temp, 2);
    Serial.print("Altitude: ");
    Serial.println(data.altitude, 2);
    Serial.print("Pressure: ");
    Serial.println(data.press, 2);

    Serial.println("---- IMU ACC ----");
    Serial.print("X: ");
    Serial.print(data.imu.acc.x, 3);
    Serial.print("  Y: ");
    Serial.print(data.imu.acc.y, 3);
    Serial.print("  Z: ");
    Serial.println(data.imu.acc.z, 3);

    Serial.println("---- COMM ----");
    Serial.print("Last ACK: ");
    Serial.println(data.last_ack);
    Serial.print("Last NACK: ");
    Serial.println(data.last_nack);

    Serial.print("currentServo = ");
    Serial.println(data.currentServo, 2); // print with 2 decimal places

    Serial.print("currentEXT = ");
    Serial.println(data.currentEXT, 2);
    Serial.print("voltageMon = ");
    Serial.println(data.voltageMon, 2);

    Serial.println("==================\n");
}

void set_rxflag()
{
    rx_flag = true;
}
