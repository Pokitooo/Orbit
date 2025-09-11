#ifndef NOVA_PIN_DEF_H
#define NOVA_PIN_DEF_H

#include <Arduino.h>
#include <Arduino_Extended.h>
// Pins defination

// SPI
constexpr uint32_t PIN_SPI_MOSI1 = PB5;
constexpr uint32_t PIN_SPI_MISO1 = PB4;
constexpr uint32_t PIN_SPI_SCK1 = PA5;

// NSS
constexpr uint32_t PIN_NSS_SD = PA7;

// LORA
constexpr uint32_t LORA_DIO1 = PB9;
constexpr uint32_t LORA_NSS = PB12;
constexpr uint32_t LORA_BUSY = PB8;
constexpr uint32_t LORA_NRST = PB10;

// UARTS
constexpr uint32_t PIN_RX = PA3;
constexpr uint32_t PIN_TX = PA2;

// i2c
constexpr uint32_t PIN_SDA = PB7;
constexpr uint32_t PIN_SCL = PB6;

// GPIO
constexpr uint32_t ledPin1 = PB15;
constexpr uint32_t ledPin2 = PB14;
constexpr uint32_t ledPin3 = PB13;
constexpr uint32_t ledPin4 = PC13;

// constexpr uint32_t servoPinA = PA1;
// constexpr uint32_t servoPinB = PA2;

// CURRENT ADC
// constexpr uint32_t VOUT_EXT = PB1;
// constexpr uint32_t VOUT_Servo = PA3;
// constexpr uint32_t VMON = PB4;

constexpr auto PINS_OFF = []
{
    // gpio_write << io_function::pull_low(ledPin);
};
#endif