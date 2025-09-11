#ifndef LUNA_FIRMWARE_SMART_DELAY_H
#define LUNA_FIRMWARE_SMART_DELAY_H

#include <cstdint>
#include <concepts>

namespace vt {
  namespace traits {
    template<typename F>
    concept procedure = requires(F f) {
      { f() } -> std::same_as<void>;
    };
  }  // namespace traits

  template<std::integral TimeType>
  class smart_delay {
  public:
    using time_func_t = TimeType();

  private:
    time_func_t *m_func            = {};
    uint32_t     m_target_interval = {};
    uint32_t     m_prev_time       = {};
    uint32_t     m_true_interval   = {};

    struct proc_else {
      bool value;

      template<traits::procedure Proc>
      void otherwise(Proc &&proc) {
        if (!value) {
          proc();
        }
      }
    };

  public:
    smart_delay(TimeType interval, time_func_t *time_func) : m_func{time_func}, m_target_interval{interval}, m_true_interval{interval} {
      if (time_func != nullptr)
        m_prev_time = time_func();
    }

    smart_delay(const smart_delay &)     = default;
    smart_delay(smart_delay &&) noexcept = default;

    smart_delay &operator=(const smart_delay &other) {
      if (this == &other) {
        return *this;
      }

      m_func            = other.m_func;
      m_target_interval = other.m_target_interval;
      m_prev_time       = other.m_prev_time;
      m_true_interval   = other.m_true_interval;

      return *this;
    }

    smart_delay &operator=(smart_delay &&other) noexcept {
      m_func            = std::move(other.m_func);
      m_target_interval = std::move(other.m_target_interval);
      m_prev_time       = std::move(other.m_prev_time);
      m_true_interval   = std::move(other.m_true_interval);

      return *this;
    }

    template<traits::procedure Proc>
    proc_else operator()(Proc &&proc) {
      const bool v = this->operator bool();
      if (v) {
        proc();
      }
      return {v};
    }

    bool triggered() {
      return this->operator bool();
    }

    bool passed() {
      return this->operator bool();
    }

    explicit operator bool() {
      if (m_func == nullptr) {
        return false;
      }

      if (m_target_interval == 0) {
        return true;
      }

      // Adaptive interval adjustment
      TimeType curr_time = m_func();
      if (curr_time - m_prev_time >= m_true_interval) {
        // absolute delta_e
        TimeType delta_e = m_target_interval > m_true_interval
                             ? m_target_interval - m_true_interval
                             : m_true_interval - m_target_interval;
        m_true_interval  = delta_e < m_true_interval
                             ? m_true_interval - delta_e
                             : m_true_interval + delta_e;
        m_prev_time      = curr_time;

        return true;
      }

      return false;
    }

    void reset() {
      if (m_func != nullptr) {
        m_prev_time = m_func();
      }
    }

    constexpr TimeType interval() const {
      return m_true_interval;
    }

    void set_interval(const TimeType new_interval) {
      m_target_interval = new_interval;
      m_true_interval   = new_interval;
      this->reset();
    }
  };

  /**
     * A flip-flop smart delay timer
     *
     * @tparam TimeType
     */
  template<std::integral TimeType>
  class on_off_timer {
  public:
    using time_func_t = TimeType();
    using SmartDelay  = smart_delay<TimeType>;

    struct interval_params {
      TimeType t_on;
      TimeType t_off;
    };

  private:
    SmartDelay sd_on;
    SmartDelay sd_off;
    bool       is_on = false;

  public:
    on_off_timer(TimeType interval_on, TimeType interval_off, time_func_t *time_func)
        : sd_on{SmartDelay(interval_on, time_func)}, sd_off{SmartDelay(interval_off, time_func)} {}

    template<traits::procedure Proc>
    on_off_timer &on_rising(Proc &&proc) {
      if (!is_on) {  // if 0
        sd_off([&]() -> void {
          proc();
          is_on = true;
          sd_on.reset();
        });
      }

      return *this;
    }

    template<traits::procedure Proc>
    on_off_timer &on_falling(Proc &&proc) {
      if (is_on) {  // if 1
        sd_on([&]() -> void {
          proc();
          is_on = false;
          sd_off.reset();
        });
      }

      return *this;
    }

    TimeType interval_on() {
      return sd_on.interval();
    }

    TimeType interval_off() {
      return sd_off.interval();
    }

    void set_interval_on(const TimeType new_interval) {
      sd_on.set_interval(new_interval);
    }

    void set_interval_off(const TimeType new_interval) {
      sd_off.set_interval(new_interval);
    }

    void reset() {
      sd_on.reset();
      sd_off.reset();
    }
  };
}  // namespace vt

#endif  //LUNA_FIRMWARE_SMART_DELAY_H
