#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "spi.h"

#if defined STM32F100xB
#include "stm32f1xx_hal.h"
#elif defined STM32F407xx
#include "stm32f4xx_hal.h"
#endif

#include "nrf24l01def.h"

typedef struct {
   volatile uint8_t enabled;
   volatile uint8_t address[5];
   volatile uint8_t pload_len;
   volatile uint8_t dynpd_en;
   volatile uint8_t ack_en;
} nrf24l01_datapipe; 

typedef struct {
  
    SPI_HandleTypeDef*  spi;
    uint32_t            spi_timeout;
    GPIO_TypeDef*       csn_port;
    uint16_t            csn_pin;
    GPIO_TypeDef*       ce_port;
    uint16_t            ce_pin;
    GPIO_TypeDef*       irq_port;
    uint16_t            irq_pin;
  
} nrf24l01_hwconfig;

typedef struct {
    nrf24l01_datapipe  pipes[6];
    uint8_t            tx_addr[5];  
    NRF_ADDR_WIDTH     addr_width;
    NRF_DATA_RATE      data_rate;
    NRF_TX_PWR         tx_power;
    uint8_t            crc_en;
    NRF_CRC_WIDTH      crc_width;
    
    
    uint8_t            tx_pload_len;      //ignored when dymamic payload activated
    uint8_t            rf_channel;
    uint8_t            retransmit_count;
    uint8_t            retransmit_delay;

} nrf24l01_config;


typedef struct {
 volatile uint8_t   config;
 volatile uint8_t   en_aa;
 volatile uint8_t   en_rxaddr;
 volatile uint8_t   setup_aw;
 volatile uint8_t   setup_retr;
 volatile uint8_t   rf_ch;
 volatile uint8_t   rf_setup;
 volatile uint8_t   status;
 volatile uint8_t   observe_tx;
 volatile uint8_t   rpd;
 volatile uint8_t   rx_addr_p0[5];
 volatile uint8_t   rx_addr_p1[5];
 volatile uint8_t   rx_addr_p2;
 volatile uint8_t   rx_addr_p3;
 volatile uint8_t   rx_addr_p4;
 volatile uint8_t   rx_addr_p5;
 volatile uint8_t   tx_addr[5];
 volatile uint8_t   rx_pw_p0;
 volatile uint8_t   rx_pw_p1;
 volatile uint8_t   rx_pw_p2;
 volatile uint8_t   rx_pw_p3;
 volatile uint8_t   rx_pw_p4;
 volatile uint8_t   rx_pw_p5;
 volatile uint8_t   fifo_status;
 volatile uint8_t   dynpd;
 volatile uint8_t   feature;

} nrf24l01_registers;

 


typedef struct {
    nrf24l01_config         config;
    nrf24l01_hwconfig       hwconfig;
    nrf24l01_registers      registers;
    volatile uint8_t        tx_busy;
    volatile NRF_RESULT     tx_result;
    volatile uint8_t        rx_busy;
    volatile NRF_TXRX_STATE state;
    //uint8_t        cmdbuffer[8];
    uint8_t        rtxbuf[34];      /* Must be sufficient size according to payload_length */
    uint8_t        rxpayload[32];
} nrf24l01;


/* Initialization routine */
NRF_RESULT nrf_init(nrf24l01* dev);

/* EXTI Interrupt Handler
 *
 * You must call this function on Falling edge trigger detection interrupt handler,
 * typically, from HAL_GPIO_EXTI_Callback  */
void nrf_irq_handler(nrf24l01* dev);

/* Asynchronous Data Receiving (__weak)
 *
 * Override this function to handle received data asynchronously,
 * default implementation is used in favor of nrf_receive_packet for blocking data receiving */
void nrf_packet_received_callback(nrf24l01* dev, uint8_t* data);

/* Blocking Data Receiving
 *
 * Blocks until the data has arrived, then returns a pointer to received data.
 * Please note, once nrf_packet_received_callback routine is overridden, this one will stop working. */
const uint8_t* nrf_receive_packet(nrf24l01* dev);

/* Blocking Data Sending
 *
 * If the AA is enabled (default), this method will return:
 *   NRF_OK - the data has been acknowledged by other party
 *   NRF_ERROR - the data has not been received (maximum retransmissions has occurred)
 * If the AA is disabled, returns NRF_OK once the data has been transmitted
 *   (with no guarantee the data was actually received). */
NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data);

/* Blocking Data Sending, with NO_ACK flag
 *
 * Disables the AA for this packet, thus this method always returns NRF_OK */
NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data);

/* Non-Blocking Data Sending */
NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data);

/* LOW LEVEL STUFF (you don't have to look in here...)*/
NRF_RESULT nrf_send_command(nrf24l01* dev, uint8_t cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len);
/* CMD */
NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data);
NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_flush_rx(nrf24l01* dev);
NRF_RESULT nrf_flush_tx(nrf24l01* dev);

/* RF_SETUP */
NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate);
NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr);
NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate);

/* STATUS */
NRF_RESULT nrf_clear_interrupts(nrf24l01* dev);

/* RF_CH */
NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch);

/* SETUP_RETR */
NRF_RESULT nrf_set_retransmit_count(nrf24l01* dev, uint8_t count);
NRF_RESULT nrf_set_retransmit_delay(nrf24l01* dev, uint8_t delay);

/* SETUP_AW */
NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width);

/* EN_RXADDR */
NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes);

/* EN_AA */
NRF_RESULT nrf_set_auto_ack(nrf24l01* dev, uint8_t pipe_num, uint8_t ack_state);
// TODO disable AA?

/* CONFIG */
NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate);
NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width);
NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up);
NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_TXRX_STATE rx);
NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate);


/* RX_ADDR_P0 */
//NRF_RESULT nrf_set_rx_address_p0(nrf24l01* dev, const uint8_t*  address); // 5bytes of address
 NRF_RESULT nrf_set_pipe_address(nrf24l01* dev, uint8_t pipe_num ,const uint8_t* address);
/* RX_ADDR_P1 */
//NRF_RESULT nrf_set_rx_address_p1(nrf24l01* dev, const uint8_t*  address); // 5bytes of address

/* TX_ADDR */
NRF_RESULT nrf_set_tx_address(nrf24l01* dev, const uint8_t*  address); // 5bytes of address

/* RX_PW_P0 */
NRF_RESULT nrf_set_rx_payload_width(nrf24l01* dev, uint8_t pipe_num, uint8_t width);

/* RX_PW_P1 */
//NRF_RESULT nrf_set_rx_payload_width_p1(nrf24l01* dev, uint8_t width);



NRF_RESULT nrf_set_pipe_dyn_payload(nrf24l01* dev, uint8_t pipe_num, uint8_t dynpd_state);
NRF_RESULT nrf_set_feature(nrf24l01* dev, uint8_t en_dpl, uint8_t en_ack_pay, uint8_t en_dyn_ack);
NRF_RESULT nrf_read_observe_tx(nrf24l01* dev);
uint8_t nrf_carrier_detect(nrf24l01* dev);
