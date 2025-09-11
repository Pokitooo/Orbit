#ifndef AVIONICS_ALGORITHM_H
#define AVIONICS_ALGORITHM_H

#include <Arduino.h>
#include "Arduino_Extended.h"

namespace algorithm {
    namespace traits {
        template<typename F>
        concept GettableX = requires(F f) {
            { f.x() } -> std::same_as<double>;
        };

        template<typename T>
        concept lt_comparable = requires(T t1, T t2) {
            { t1 < t2 } -> std::same_as<bool>;
        };

        template<typename T>
        concept gt_comparable = requires(T t1, T t2) {
            { t1 < t2 } -> std::same_as<bool>;
        };

        template<typename T>
        concept eq_comparable = requires(T t1, T t2) {
            { t1 == t2 } -> std::same_as<bool>;
        };

        template<typename T>
        concept comparable = eq_comparable<T> || gt_comparable<T> || lt_comparable<T>;
    }  // namespace traits

    template<traits::gt_comparable T>
    constexpr T max(T t1, T t2) {
        return t1 > t2 ? t1 : t2;
    }

    template<traits::gt_comparable T>
    constexpr T min(T t1, T t2) {
        return t1 < t2 ? t1 : t2;
    }

    namespace detail {
        constexpr double sum_square_helper() {
            return 0;
        }

        template<typename T, typename... Ts>
        constexpr double sum_square_helper(const T &v, const Ts &...vs) {
            return v * v + sum_square_helper(vs...);
        }
    }  // namespace detail

    template<typename... Ts>
    constexpr double root_sum_square(const Ts &...vs) {
        return std::sqrt(detail::sum_square_helper(vs...));
    }

    constexpr bool is_zero(const float value, const float epsilon = 1e-4f) {
        return value > 0.0f ? value < epsilon : value > -epsilon;
    }

    constexpr bool is_zero(const double value, const double epsilon = 1e-4) {
        return value > 0.0 ? value < epsilon : value > -epsilon;
    }

    class ExponentialMovingAverage {
    private:
        double m_alpha;
        double m_ema   = {};
        uint8_t m_init = {};

    public:
        explicit constexpr ExponentialMovingAverage(const double alpha)
            : m_alpha{alpha} {
        }

        ExponentialMovingAverage &operator<<(const double v) {
            if (!m_init) {
                m_ema  = v;
                m_init = 1;
            } else {
                m_ema = (1.0 - m_alpha) * m_ema + m_alpha * v;
            }
            return *this;
        }

        template<traits::GettableX KF_Type>
        ExponentialMovingAverage &operator<<(const KF_Type &kf_1d_object) {
            this->operator<<(kf_1d_object.x());
            return *this;
        }

        [[nodiscard]] constexpr double get() const {
            return m_ema;
        }

        void operator>>(double &targ) const {
            targ = get();
        }

        void reset() {
            m_ema  = 0.0;
            m_init = 0;
        }
    };

    class KalmanFilter_1D {
    private:
        double m_x;  // Estimated state
        double m_P;  // Estimated error covariance
        double m_Q;  // Process noise covariance
        double m_R;  // Measurement noise covariance
        double m_K;  // Kalman gain

    public:
        constexpr KalmanFilter_1D() : KalmanFilter_1D(initial_x, initial_P, initial_noise, initial_noise) {}

        constexpr KalmanFilter_1D(const double initial_x, const double initial_P, const double Q, const double R)
            : m_x(initial_x), m_P(initial_P), m_Q(Q), m_R(R), m_K(0.0) {
        }

        KalmanFilter_1D &predict(const double = 0.0) {
            m_P = m_P + m_Q;
            return *this;
        }

        KalmanFilter_1D &update(const double z) {
            m_K = m_P / (m_P + m_R);
            m_x = m_x + m_K * (z - m_x);
            m_P = (1 - m_K) * m_P;
            return *this;
        }

        KalmanFilter_1D &operator<<(const double z) {
            return predict().update(z);
        }

        KalmanFilter_1D &operator<<(const ExponentialMovingAverage &ema) {
            this->operator<<(ema.get());
            return *this;
        }

        [[nodiscard]] constexpr double x() const {
            return m_x;
        }

        [[nodiscard]] constexpr double P() const {
            return m_P;
        }

        void operator>>(double &targ) const {
            targ = x();
        }

        static constexpr double initial_x     = 0.0;
        static constexpr double initial_P     = 1.0;
        static constexpr double initial_noise = 0.1;
    };

    template<size_t N>
    class Derivative {
    private:
        double m_dt;
        double m_values[2][N + 1] = {};  // Ping-pong buffer (double buffering)
        uint8_t m_line            = 0;
        uint8_t m_cnt             = 0;

    public:
        explicit constexpr Derivative(const double dt) : m_dt{dt} {
        }

        Derivative &operator<<(const double z) {
            if (m_cnt == 0) {
                m_values[m_line][0] = z;
                m_cnt               = 1;
            } else {
                m_line              = 1 - m_line;  // Swap buffer line
                m_values[m_line][0] = z;
                for (size_t i = 1; i <= N; ++i) {
                    m_values[m_line][i] = (m_values[m_line][i - 1] - m_values[1 - m_line][i - 1]) / m_dt;
                }
            }

            return *this;
        }

        [[nodiscard]] constexpr double operator[](size_t i) const {
            return this->order(i);
        }

        [[nodiscard]] constexpr double order(size_t i) const {
            return i <= N ? m_values[m_line][i] : 0.0;
        }

        [[nodiscard]] constexpr size_t size() const {
            return N + 1;
        }
    };

    template<size_t N>
    class Integral {
    private:
        double m_values[2][N + 1] = {};  // Ping-pong buffer (double buffering)
        double m_dt;
        uint8_t m_line;
        uint8_t m_cnt;

    public:
        explicit constexpr Integral(const double dt) : m_dt{dt}, m_line{0}, m_cnt{0} {
        }

        Integral &operator<<(const double z) {
            if (m_cnt == 0) {
                m_values[m_line][0] = z;
                m_cnt               = 1;
            } else {
                m_line              = 1 - m_line;  // Swap buffer line
                m_values[m_line][0] = z;
                for (size_t i = 1; i <= N; ++i) {
                    m_values[m_line][i] = m_values[1 - m_line][i] + m_values[1 - m_line][i - 1] * m_dt;
                }
            }

            return *this;
        }

        [[nodiscard]] constexpr double operator[](const size_t i) const {
            return this->order(i);
        }

        [[nodiscard]] constexpr double order(const size_t i) const {
            return i <= N ? m_values[m_line][i] : 0.0;
        }

        [[nodiscard]] constexpr size_t size() const {
            return N + 1;
        }
    };

    template<size_t TrueRatio, size_t FalseRatio, bool Strict = false>
    constexpr bool vote_sample(const size_t true_count, const size_t false_count) {
        if constexpr (Strict) {
            return true_count * FalseRatio > false_count * TrueRatio;
        } else {
            return true_count * FalseRatio >= false_count * TrueRatio;
        }
    }

    class Sampler {
    private:
        size_t true_count  = {};
        size_t false_count = {};

    public:
        constexpr void add(const bool pred) {
            true_count += pred;
            false_count += 1 - pred;
        }

        constexpr void reset() {
            true_count  = 0;
            false_count = 0;
        }

        template<size_t TrueRatio, size_t FalseRatio, bool Strict = false>
        [[nodiscard]] constexpr bool vote() const {
            return vote_sample<TrueRatio, FalseRatio, Strict>(true_count, false_count);
        }

        [[nodiscard]] constexpr size_t count() const {
            return true_count + false_count;
        }
    };
}  // namespace algorithm

#endif  //AVIONICS_ALGORITHM_H
