#define ON   1
#define OFF  0


#define NRF_DATARATE              NRF_DATA_RATE_250KBPS   //NRF_DATA_RATE_250KBPS = 1, NRF_DATA_RATE_1MBPS   = 0, NRF_DATA_RATE_2MBPS   = 2
#define NRF_TXPOWER               NRF_TX_PWR_0dBm
#define NRF_CRC_ON                ON
#define NRF_CRC_WITH              NRF_CRC_WIDTH_1B
#define NRF_ADR_WITH              NRF_ADDR_WIDTH_5
#define NRF_PAYLOADLEN            0x10
#define NRF_RETRANSCOUNT          0x0F           //15
#define NRF_RETRANSDELAY          0x04          //1000 uS
#define NRF_CHANNEL               119           //2.519Ghz
#define NRF_SPI_TIMEOUT           10
#define NRF_ENABLE_DYNP           OFF


#define NRF_RX_PIPE0_EN           ON
#define NRF_RX_PIPE0_ADDR         (uint8_t[]){0x6C, 0x75, 0x6E, 0x61, 0x31}        //predefined addresses. to change actual address refer to device structure field
#define NRF_RX_PIPE0_ACK          ON
#define NRF_RX_PIPE0_DYNPD        OFF


#define NRF_RX_PIPE1_EN           ON
#define NRF_RX_PIPE1_ADDR         (uint8_t[]){0x6C, 0x75, 0x6E, 0x61, 0x32}
#define NRF_RX_PIPE1_ACK          ON
#define NRF_RX_PIPE1_DYNPD        OFF

#define NRF_RX_PIPE2_EN           OFF
#define NRF_RX_PIPE2_ADDR         0x32
#define NRF_RX_PIPE2_ACK          OFF
#define NRF_RX_PIPE2_DYNPD        OFF


#define NRF_RX_PIPE3_EN           OFF
#define NRF_RX_PIPE3_ADDR         0x33
#define NRF_RX_PIPE3_ACK          OFF
#define NRF_RX_PIPE3_DYNPD        OFF


#define NRF_RX_PIPE4_EN           OFF
#define NRF_RX_PIPE4_ADDR         0x34
#define NRF_RX_PIPE4_ACK          OFF
#define NRF_RX_PIPE4_DYNPD        OFF


#define NRF_RX_PIPE5_EN           OFF
#define NRF_RX_PIPE5_ADDR         0x35
#define NRF_RX_PIPE5_ACK          OFF
#define NRF_RX_PIPE5_DYNPD        OFF

#define NRF_TX_ADDRESS            (uint8_t[]){0x6C, 0x75, 0x6E, 0x61, 0x31}  



#if     defined STM32F100xB
        #define NRF_TX_SPI_PORT   &hspi1
#elif   defined STM32F407xx
        #define NRF_TX_SPI_PORT   &hspi3
#endif           