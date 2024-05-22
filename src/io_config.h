#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include <array>
#include <stdint.h>
#include <Arduino.h>

// #define SIMULATION_MODE

/* num define */
#define NUM_DIGITAL_OUT 4
#define NUM_ANALOG_IN 4
#define NUM_AS5600 4
#define NUM_AS5600_CH 1
#define NUM_AS5600_ALL_CH (NUM_AS5600 * NUM_AS5600_CH)
#define NUM_MCP4922 2
#define NUM_MCP4922_CH 2
#define NUM_MCP4922_ALL_CH (NUM_MCP4922 * NUM_MCP4922_CH)

/**
 * @brief pin define
 * @note see https://os.mbed.com/platforms/ST-Nucleo-F767ZI/
 */
// I2C
#define I2C1_SDA_PIN PB9
#define I2C1_SCL_PIN PB8
#define I2C2_SDA_PIN PB11
#define I2C2_SCL_PIN PB10
#define I2C3_SDA_PIN PC9
#define I2C3_SCL_PIN PA8
#define I2C4_SDA_PIN PD13
#define I2C4_SCL_PIN PD12
// SPI
#define SPI1_SCL_PIN  PB3
#define SPI1_MISO_PIN PB4
#define SPI1_MOSI_PIN PB5
#define SPI1_CS_PIN   PA4
#define SPI4_SCL_PIN  PE2
#define SPI4_MISO_PIN PE13 // unused
#define SPI4_MOSI_PIN PE6
#define SPI4_CS_PIN   PD3
#define SPI5_SCL_PIN  PF7
#define SPI5_MISO_PIN PF8 // unused
#define SPI5_MOSI_PIN PF9
#define SPI5_CS_PIN   PE3
// Analog In
#define ANALOG_IN0_PIN PA2
#define ANALOG_IN1_PIN PA0
#define ANALOG_IN2_PIN PC0
#define ANALOG_IN3_PIN PF10
// Digital Out
#define DIGITAL_OUT0_PIN PA10
#define DIGITAL_OUT1_PIN PD11
#define DIGITAL_OUT2_PIN PC1
#define DIGITAL_OUT3_PIN PE5
// Ethernet
#define ETHERNET_CLOCK      PA1 // unused
#define ETHERNET_MDIO       PA2  // unused
#define ETHERNTT_MDC        PC1 // unused
#define ETHERNET_RX_VAILD   PA7 // unused
#define ETHERNET_RX0        PC4 // unused
#define ETHERNET_RX1        PC5 // unused
#define ETHERNET_TX_ENABLE  PG11 // unused
#define ETHERNET_TX0        PG13 // unused
#define ETHERNET_TX1        PB13 // unused
// Serial(USART3)
#define USB_SERIAL_TX       PD8
#define USB_SERIAL_RX       PD9



#endif // IO_CONFIG_H