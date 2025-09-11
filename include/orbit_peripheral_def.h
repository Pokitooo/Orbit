#ifndef NOVA_PERIPHERAL_DEF_H
#define NOVA_PERIPHERAL_DEF_H

#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

#define SEALEVELPRESSURE_HPA (1013.25)


//Kalman filters
template<typename T>
union vec3_u
{
  struct
  {
    T values[3]{};
  };

  struct
  {
    T x;
    T y;
    T z;
  };

  template <typename... Args>
  explicit vec3_u(Args &&...args) : x{T(std::forward<Args>(args)...)},
                                    y{T(std::forward<Args>(args)...)},
                                    z{T(std::forward<Args>(args)...)} {}
};

namespace nova::config {
    constexpr uint32_t PYRO_ACTIVATE_INTERVAL = 1000ul;

    constexpr auto HZ_TO_INTERVAL_MS          = [](const double FREQUENCY_HZ) -> uint32_t {
        return static_cast<uint32_t>(1000. / FREQUENCY_HZ);
    };

    constexpr auto INTERVAL_MS_TO_HZ = [](const uint32_t INTERVAL) -> uint32_t {
        return 1000 / INTERVAL;
    };

    constexpr uint8_t INA236_ADDRESS = 0x40;

    enum INA236ADCRange {
        RANGE_80MV = 0,
        RANGE_20MV = 1
    };

    constexpr uint32_t TIME_TO_APOGEE_MIN  = 14.69 * 1000ul;
    constexpr uint32_t TIME_TO_APOGEE_MAX  = 15.69 * 1000ul; //Sim 14.69
    constexpr uint32_t TIME_TO_BURNOUT_MIN = 1.85 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MAX = 2.85 * 1000ul; //Sim 1.85

    namespace alg {
        constexpr uint32_t LAUNCH_TON          = 150ul;          // 150 ms
        constexpr uint32_t BURNOUT_TON         = 500ul;          // 500 ms
        constexpr uint32_t APOGEE_SLOW_TON     = 1000ul;         // 2000 ms
        constexpr uint32_t MAIN_DEPLOYMENT_TON = 200ul;          // 200 ms
        constexpr uint32_t LANDING_TON         = 5000ul;         // 200 ms
        constexpr double LAUNCH_ACC            = 30.0;       // 62.7 m/s^2
        constexpr double APOGEE_VEL            = 5.0;           // m/s
        constexpr double MAIN_ALTITUDE         = 150.f;  // m
    }  // namespace alg

    constexpr unsigned long RFD900X_BAUD         = 460800;
    constexpr unsigned long RPI_BAUD             = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT     = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ          = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE         = 512ul;

    constexpr uint32_t TX_IDLE_INTERVAL          = HZ_TO_INTERVAL_MS(1);  // 1 Hz
    constexpr uint32_t TX_ARMED_INTERVAL         = HZ_TO_INTERVAL_MS(2);  // 2 Hz
    constexpr uint32_t TX_PAD_PREOP_INTERVAL     = HZ_TO_INTERVAL_MS(4);  // 4 Hz
    constexpr uint32_t TX_ASCEND_INTERVAL        = HZ_TO_INTERVAL_MS(5);  // 5 Hz
    constexpr uint32_t TX_DESCEND_INTERVAL       = HZ_TO_INTERVAL_MS(4);  // 4 Hz

    constexpr uint32_t LOG_IDLE_INTERVAL         = HZ_TO_INTERVAL_MS(1);   // 1 Hz
    constexpr uint32_t LOG_ARMED_INTERVAL        = HZ_TO_INTERVAL_MS(2);   // 2 Hz
    constexpr uint32_t LOG_PAD_PREOP_INTERVAL    = HZ_TO_INTERVAL_MS(10);  // 10 Hz
    constexpr uint32_t LOG_ASCEND_INTERVAL       = HZ_TO_INTERVAL_MS(20);  // 20 Hz
    constexpr uint32_t LOG_DESCEND_INTERVAL      = HZ_TO_INTERVAL_MS(10);  // 10 Hz

    constexpr uint32_t BUZZER_ON_INTERVAL        = 50ul;                    // 50 ms
    constexpr uint32_t BUZZER_IDLE_INTERVAL      = HZ_TO_INTERVAL_MS(1);    // 1 Hz
    constexpr uint32_t BUZZER_ARMED_INTERVAL     = HZ_TO_INTERVAL_MS(2);    // 2 Hz
    constexpr uint32_t BUZZER_PAD_PREOP_INTERVAL = HZ_TO_INTERVAL_MS(10);   // 10 Hz
    constexpr uint32_t BUZZER_ASCEND_INTERVAL    = HZ_TO_INTERVAL_MS(0.2);  // 0.2 Hz
    constexpr uint32_t BUZZER_DESCEND_INTERVAL   = HZ_TO_INTERVAL_MS(1);    // 1 Hz

    constexpr int SERVO_A_DEPLOY = 180;
    constexpr int SERVO_A_LOCK = 60;
    constexpr int SERVO_A_SET = 0;

    constexpr int SERVO_B_DEPLOY = 70;
    constexpr int SERVO_B_LOCK = 180;
    constexpr int SERVO_B_SET = 110;

    constexpr auto BUZZER_OFF_INTERVAL           = [](const uint32_t BUZZER_TOTAL_INTERVAL) -> uint32_t {
        return BUZZER_TOTAL_INTERVAL - BUZZER_ON_INTERVAL;
    };

    namespace details::assertions {
        static_assert(TIME_TO_APOGEE_MAX >= TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
        static_assert(TIME_TO_BURNOUT_MAX >= TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
    }  // namespace details::assertions
}  // namespace luna::config

enum RadioMode {
  RADIO_MODE_IDLE,
  RADIO_MODE_TX,
  RADIO_MODE_RX
};
volatile RadioMode currentMode = RADIO_MODE_IDLE;

#endif