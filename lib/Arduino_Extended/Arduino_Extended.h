#ifndef ARDUINO_EXTENDED_H
#define ARDUINO_EXTENDED_H

#include <cmath>
#include <Arduino.h>
#include <Wire.h>
#define BITS_AT(val, pos) ((val >> pos) & 1)

namespace traits {
    template<typename S>
    concept has_ostream = requires(S s, const char *str) {
        { s << str } -> std::same_as<S &>;
    };

    template<typename S>
    concept arduino_stream_derived = std::derived_from<S, Stream>;

    template<typename S>
    concept stream_has_flush = requires(S s) {
        { s.flush() } -> std::same_as<void>;
    };

    template<typename S>
    concept ostream_flush = has_ostream<S> && stream_has_flush<S>;
}  // namespace traits

namespace stream {
    inline const char *crlf = "\r\n";

    namespace detail {
        class flush_type {};
    }  // namespace detail

    constexpr detail::flush_type flush = detail::flush_type();
}  // namespace stream

template<traits::arduino_stream_derived S, typename T>
S &operator<<(S &stream, T &&v) {
    stream.print(std::forward<T>(v));
    return stream;
}

template<typename T>
String &operator<<(String &string, T &&v) {
    string += std::forward<T>(v);
    return string;
}

struct FakeOStream {
    template<typename T>
    constexpr FakeOStream &operator<<(T &&) { return *this; }
};

namespace detail {
    template<typename T>
        requires traits::has_ostream<T>
    struct flush_ostream {
        static constexpr bool value = false;
    };

    template<traits::ostream_flush T>
    struct flush_ostream<T> {
        static constexpr bool value = true;
    };

    template<traits::has_ostream OStream,
             size_t ReserveSize = 0,
             bool NewLine       = false,
             bool AutoFlush     = true>
    class csv_stream {
    private:
        OStream *m_stream = {};
        String m_string   = {};

    public:
        explicit csv_stream(OStream &stream)
            : m_stream(&stream) {
            m_string.reserve(ReserveSize);
        }

        csv_stream(const csv_stream &other)     = delete;
        csv_stream(csv_stream &&other) noexcept = delete;

        template<typename T>
        csv_stream &operator<<(T &&value) {
            m_string += std::forward<T>(value);
            m_string += ",";
            return *this;
        }

        ~csv_stream() {
            // End of message

            // Remove trailing comma
            m_string.remove(m_string.length() - 1);

            if constexpr (NewLine) {
                m_string += stream::crlf;
            }

            // Flush to stream
            *m_stream << m_string;

            if constexpr (AutoFlush && flush_ostream<OStream>::value) {
                m_stream->flush();
            }
        }
    };
}  // namespace detail

/**
 * @tparam OStream Output Stream Type with "<<" stream operator
 * @tparam ReserveSize Internal string reserve size
 * @tparam NewLine Whether to insert CRLF at the end or not
 * @param stream Ouptut stream OStream object
 * @return Csv stream object
 */
template<traits::has_ostream OStream, size_t ReserveSize = 0, bool NewLine = false, bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush> csv_stream(OStream &stream) {
    return detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>(stream);
}

template<traits::has_ostream OStream, size_t ReserveSize = 0, bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, true, AutoFlush> csv_stream_crlf(OStream &stream) {
    return csv_stream<OStream, ReserveSize, true, AutoFlush>(stream);
}

// IO Pin as stream

inline uint32_t to_digital(const PinName pin_name) {
    return pinNametoDigitalPin(pin_name);
}

namespace detail {
    struct IOFunction {};

    template<typename T>
    concept io_func = std::derived_from<T, IOFunction>;
}  // namespace detail

namespace io_function {
    struct pull_high : detail::IOFunction {
        PinName m_pin;

        constexpr explicit pull_high(const PinName pin) : m_pin{pin} {}
    };

    struct pull_low : detail::IOFunction {
        PinName m_pin;

        constexpr explicit pull_low(const PinName pin) : m_pin{pin} {}
    };

    struct toggle : detail::IOFunction {
        PinName m_pin;

        constexpr explicit toggle(const PinName pin) : m_pin{pin} {}
    };

    struct set : detail::IOFunction {
        PinName m_pin;
        uint32_t m_val;

        constexpr explicit set(const PinName pin, const uint32_t val) : m_pin{pin}, m_val{val} {}
    };

    struct pwm : detail::IOFunction {
        uint32_t m_pin;
        uint32_t m_val;

        explicit pwm(const PinName pin, const uint32_t val)
            : m_pin{to_digital(pin)},
              m_val{val} {}
    };

}  // namespace io_function

namespace detail {
    struct SampleStatus {
        int status = {};
        int value  = {};

        [[nodiscard]] constexpr bool stable() const { return status == 1; }
    };

    // GPIO as ostream
    struct gpio_write_t {
        template<io_func IoFuncType>
        gpio_write_t &operator<<(IoFuncType func) {
            if constexpr (std::is_same_v<IoFuncType, io_function::pull_high>) {
                digitalWriteFast(func.m_pin, HIGH);
            } else if constexpr (std::is_same_v<IoFuncType, io_function::pull_low>) {
                digitalWriteFast(func.m_pin, LOW);
            } else if constexpr (std::is_same_v<IoFuncType, io_function::toggle>) {
                digitalToggleFast(func.m_pin);
            } else if constexpr (std::is_same_v<IoFuncType, io_function::set>) {
                digitalWriteFast(func.m_pin, func.m_val);
            } else if constexpr (std::is_same_v<IoFuncType, io_function::pwm>) {
                analogWrite(func.m_pin, func.m_val);
            }
            return *this;
        }
    };

    struct gpio_read_t {
        int operator()(const PinName pin) const {
            return digitalReadFast(pin);
        }

        // 480 MHz: 20 MHz sample rate (20 samples per microsecond)
        // 24 clock cycles per sample
        template<size_t Sample>
        [[nodiscard]] int sample(const PinName pin) const {
            static_assert(Sample > 0, "Sample count must be non-zero.");

            if constexpr (Sample == 1) {
                return this->operator()(pin);
            } else {
                size_t count[2] = {};
                for (size_t i = 0; i < Sample - 1; ++i) {
                    ++count[this->operator()(pin)];
                }
                return count[1] > count[0];
            }
        }
    };

    // GPIO Config as stream
    template<int Val>
    class digital_out_configuration_t {
    public:
        digital_out_configuration_t &operator<<(const PinName pin) {
            pinMode(to_digital(pin), OUTPUT);
            digitalWriteFast(pin, Val);
            return *this;
        }
    };

    class digital_in_configuration_t {
    public:
        digital_in_configuration_t &operator<<(const PinName pin) {
            pinMode(to_digital(pin), INPUT);
            return *this;
        }
    };
}  // namespace detail

inline detail::gpio_write_t gpio_write;
inline detail::gpio_read_t gpio_read;
inline detail::digital_out_configuration_t<LOW> dout_low;
inline detail::digital_out_configuration_t<HIGH> dout_high;
inline detail::digital_in_configuration_t din_config;

// I2C Scanner
inline void i2c_detect(Stream &output_stream,
                       TwoWire &i2c_wire,
                       const uint8_t addr_from,
                       const uint8_t addr_to) {
    char buf[10];
    output_stream.println("I2C Detector");
    output_stream.print("   ");
    for (uint8_t i = 0; i < 16; i++) {
        sprintf(buf, "%3x", i);
        output_stream.print(buf);
    }

    for (uint8_t addr = 0; addr < 0x80; addr++) {
        if (addr % 16 == 0) {
            sprintf(buf, "\n%02x:", addr & 0xF0);
            output_stream.print(buf);
        }
        if (addr >= addr_from && addr <= addr_to) {
            i2c_wire.beginTransmission(addr);
            if (const uint8_t resp = i2c_wire.endTransmission(); resp == 0) {
                // device found
                //stream.printf(" %02x", addr);
                sprintf(buf, " %02x", addr);
                output_stream.print(buf);
            } else if (resp == 4) {
                // other resp
                output_stream.print(" XX");
            } else {
                // resp = 2: received NACK on transmit of addr
                // resp = 3: received NACK on transmit of data
                output_stream.print(" --");
            }
        } else {
            // addr not scanned
            output_stream.print("   ");
        }
    }
    output_stream.println("\n");
}

template<typename FP>
FP pressure_altitude(const FP pressure_hpa) {
    constexpr double h0  = 44307.69396;
    constexpr double p0  = 1013.25;
    const double h_ratio = pressure_hpa / p0;
    const double v       = 1 - std::pow(h_ratio, 0.190284);
    return h0 * v;
}

constexpr uint8_t *byte_cast(void *ptr) {
    return static_cast<uint8_t *>(ptr);
}

inline __attribute__((__always_inline__)) void do_nothing() {}

#include "smart_delay.h"
#include "tasks.h"
#include "avionics_algorithm.h"

#endif  //ARDUINO_EXTENDED_H