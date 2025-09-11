#ifndef LUNA_FIRMWARE_TASKS_H
#define LUNA_FIRMWARE_TASKS_H

#include <cstdint>
#include <concepts>
#include <type_traits>

namespace vt {
  namespace functional {
    template<typename T>
    constexpr auto ptr_to_void_cast(void (*func)(T *)) -> void (*)(void *) {
      return reinterpret_cast<void (*)(void *)>(func);
    }
  }  // namespace functional

  namespace traits {
    template<typename C, typename T>
    concept smart_delay_variant = requires(T value, T (*func_ptr)()) {
      { C(value, func_ptr) } -> std::same_as<C>;
      { static_cast<bool>(std::declval<C>()) } -> std::same_as<bool>;
      { std::declval<C>().reset() } -> std::same_as<void>;
    } && std::integral<T>;
  }  // namespace traits

  namespace dummy {
    template<typename TimeType>
    constexpr TimeType get_time() { return 0; }
  }  // namespace dummy

  template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType>
    requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
  class task_dispatcher;

  template<template<typename> class SmartDelay, std::integral TimeType>
    requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
  class task_t {
  public:
    template<typename T>
    using func_ptr_Targ = void (*)(T *);

    template<typename T>
    using func_ptr_Tref = void (*)(T &);

    using func_ptr      = void (*)();
    using func_ptr_arg  = func_ptr_Targ<void>;
    using time_func_t   = TimeType();
    using priority_t    = uint8_t;

    struct addition_struct {
      task_t task;
      bool   pred;
    };

    template<size_t MaxTasks, template<typename> class, std::integral>
      requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    friend class task_dispatcher;

  private:
    using smart_delay_t = SmartDelay<TimeType>;

    func_ptr_arg  m_func;
    void         *m_arg;
    smart_delay_t m_sd;
    priority_t    m_priority;

  public:
    task_t()
        : m_func{nullptr}, m_arg(nullptr),
          m_sd{smart_delay_t(0, nullptr)}, m_priority{0} {}

    task_t(const task_t &)     = default;
    task_t(task_t &&) noexcept = default;

    // Function taking argument pointer
    template<typename Arg>
    task_t(func_ptr_Targ<Arg> task_func, Arg *arg, const TimeType interval, const time_func_t time_func, const priority_t priority = 0)
        : m_func{functional::ptr_to_void_cast(task_func)}, m_arg(arg),
          m_sd{smart_delay_t(interval, time_func)}, m_priority{priority} {}

    // Function taking argument pointer (no delay)
    template<typename Arg>
    task_t(func_ptr_Targ<Arg> task_func, Arg *arg, const priority_t priority = 0)
        : m_func{functional::ptr_to_void_cast(task_func)}, m_arg(arg),
          m_sd{smart_delay_t(0, dummy::get_time<TimeType>)}, m_priority{priority} {}

    // Function taking nothing
    task_t(const func_ptr task_func, const TimeType interval, const time_func_t time_func, const priority_t priority = 0)
        : m_func{reinterpret_cast<func_ptr_arg>(task_func)}, m_arg(nullptr),
          m_sd{smart_delay_t(interval, time_func)}, m_priority{priority} {}

    // Function taking nothing (no delay)
    explicit task_t(const func_ptr task_func, const priority_t priority = 0)
        : m_func{reinterpret_cast<func_ptr_arg>(task_func)}, m_arg(nullptr),
          m_sd{smart_delay_t(0, dummy::get_time<TimeType>)}, m_priority{priority} {}

    task_t &operator=(const task_t &other) {
      if (this == &other) {
        return *this;
      }

      m_func     = other.m_func;
      m_arg      = other.m_arg;
      m_sd       = other.m_sd;
      m_priority = other.m_priority;

      return *this;
    }

    task_t &operator=(task_t &&other) noexcept {
      m_func     = std::move(other.m_func);
      m_arg      = std::move(other.m_arg);
      m_sd       = std::move(other.m_sd);
      m_priority = std::move(other.m_priority);

      return *this;
    }

    void operator()() {
      if (m_func != nullptr && m_sd) {
        m_func(m_arg);
      }
    }

    void reset() {
      m_sd.reset();
    }

    addition_struct operator,(const bool pred) const {
      return {std::move(*this), pred};
    }

    constexpr TimeType interval() const {
      return m_sd.interval();
    }
  };

  template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType>
    requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
  class task_dispatcher {
    static_assert(MaxTasks > 0, "Scheduler size cannot be zero.");

  private:
    using Task               = task_t<SmartDelay, TimeType>;

    size_t m_size            = {};
    Task   m_tasks[MaxTasks] = {};

  public:
    task_dispatcher()                            = default;
    task_dispatcher(const task_dispatcher &)     = default;
    task_dispatcher(task_dispatcher &&) noexcept = default;

    task_dispatcher &operator+=(typename Task::addition_struct &&task_struct) {
      return this->operator<<(std::forward<Task>(task_struct));
    }

    task_dispatcher &operator<<(typename Task::addition_struct &&task_struct) {
      if (task_struct.pred) {
        this->operator<<(std::move(task_struct.task));
      }
      return *this;
    }

    task_dispatcher &operator+=(Task &&task) {
      return this->operator<<(std::forward<Task>(task));
    }

    task_dispatcher &operator<<(Task &&task) {
      if (m_size < MaxTasks) {
        size_t i;
        for (i = 0; i < m_size && m_tasks[i].m_priority < task.m_priority; ++i);
        for (size_t j = 0; j < m_size - i; ++j) {
          m_tasks[m_size - j] = std::move(m_tasks[m_size - j - 1]);
        }
        m_tasks[i] = std::move(task);
        ++m_size;
      }

      return *this;
    }

    void operator()() {
      for (size_t i = 0; i < m_size; ++i) {
        m_tasks[i]();
      }
    }

    void reset() {
      for (size_t i = 0; i < m_size; ++i) {
        m_tasks[i].reset();
      }
    }

    void clear() {
      m_size = 0;
    }
  };
}  // namespace vt

#endif  //LUNA_FIRMWARE_TASKS_H
