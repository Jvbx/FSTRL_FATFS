#define  BIT7         (1 << 7)
#define  BIT6         (1 << 6)
#define  BIT5         (1 << 5)
#define  BIT4         (1 << 4)
#define  BIT3         (1 << 3)
#define  BIT2         (1 << 2)
#define  BIT1         (1 << 1)
#define  BIT0         (1 << 0)

typedef enum {
    NRF_CONFIG      = 0x00,
  
          #define  NRF_CONFIG_FORBIDDEN_BIT   BIT7
          #define  NRF_CONFIG_MASK_RX_DR      BIT6
          #define  NRF_CONFIG_MASK_TS_DR      BIT5
          #define  NRF_CONFIG_MASK_MAX_RT     BIT4
          #define  NRF_CONFIG_EN_CRC          BIT3
          #define  NRF_CONFIG_CRCO            BIT2
          #define  NRF_CONFIG_PWR_UP          BIT1
          #define  NRF_CONFIG_PRIM_RX         BIT0 
    
    NRF_EN_AA       = 0x01,
    NRF_EN_RXADDR   = 0x02,
  
  
  
    NRF_SETUP_AW    = 0x03,
  
          #define  NRF_SETUP_AW_RESERVED      (BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|)
          #define  NRF_SETUP_AW_AW            (BIT1|BIT0)                         
  
    NRF_SETUP_RETR  = 0x04,
          
          #define NRF_SETUP_RETR_ARD          (BIT7|BIT6|BIT5|BIT4)          
          #define NRF_SETUP_RETR_ARC          (BIT3|BIT2|BIT1|BIT0)
  
    NRF_RF_CH       = 0x05,

          #define  NRF_RF_CH_FORBIDDEN_BIT    BIT7
          #define  NRF_RF_CH_RF_CHANNEL       (BIT6|BIT5|BIT6|BIT3|BIT2|BIT1|BIT0|)
         
    NRF_RF_SETUP    = 0x06,
  
          #define  NRF_RF_SETUP_CONT_WAVE     BIT7     
          #define  NRF_RF_SETUP_RESERVED      BIT6
          #define  NRF_RF_SETUP_RF_DR_LOW     BIT5
          #define  NRF_RF_SETUP_PLL_LOCK      BIT4
          #define  NRF_RF_SETUP_RF_DR_HIGH    BIT3
          #define  NRF_RF_SETUP_RF_PWR        (BIT2|BIT1)
          #define  NRF_RF_SETUP_OBSOLETE      BIT0
          
    NRF_STATUS      = 0x07,
    NRF_OBSERVE_TX  = 0x08,
    NRF_RPD         = 0x09,
    NRF_RX_ADDR_P0  = 0x0A,
    NRF_RX_ADDR_P1  = 0x0B,
    NRF_RX_ADDR_P2  = 0x0C,
    NRF_RX_ADDR_P3  = 0x0D,
    NRF_RX_ADDR_P4  = 0x0E,
    NRF_RX_ADDR_P5  = 0x0F,
    NRF_TX_ADDR     = 0x10,
    NRF_RX_PW_P0    = 0x11,
    NRF_RX_PW_P1    = 0x12,
    NRF_RX_PW_P2    = 0x13,
    NRF_RX_PW_P3    = 0x14,
    NRF_RX_PW_P4    = 0x15,
    NRF_RX_PW_P5    = 0x16,
    NRF_FIFO_STATUS = 0x17,
    NRF_DYNPD       = 0x1C,
    NRF_FEATURE     = 0x1D
} NRF_REGISTER;



 //CONFIG Bits



//RF_SETUP Bits



/* Commands */
typedef enum {
    NRF_CMD_R_REGISTER         = 0x00,
    NRF_CMD_W_REGISTER         = 0x20,
    NRF_CMD_R_RX_PAYLOAD       = 0x61,
    NRF_CMD_W_TX_PAYLOAD       = 0xA0,
    NRF_CMD_FLUSH_TX           = 0xE1,
    NRF_CMD_FLUSH_RX           = 0xE2,
    NRF_CMD_REUSE_TX_PL        = 0xE3,
    NRF_CMD_R_RX_PL_WID        = 0x60,
    NRF_CMD_W_ACK_PAYLOAD      = 0xA8,
    NRF_CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    NRF_CMD_NOP                = 0xFF,
    NRF_CMD_ACTIVATE           = 0x50
} NRF_COMMAND;

typedef enum {
    NRF_DATA_RATE_250KBPS = 1,
    NRF_DATA_RATE_1MBPS   = 0,
    NRF_DATA_RATE_2MBPS   = 2
} NRF_DATA_RATE;

typedef enum {
    NRF_TX_PWR_M18dBm = 0,
    NRF_TX_PWR_M12dBm = 1,
    NRF_TX_PWR_M6dBm  = 2,
    NRF_TX_PWR_0dBm   = 3
} NRF_TX_PWR;

typedef enum {
    NRF_ADDR_WIDTH_3 = 1,
    NRF_ADDR_WIDTH_4 = 2,
    NRF_ADDR_WIDTH_5 = 3
} NRF_ADDR_WIDTH;

typedef enum { NRF_CRC_WIDTH_1B = 0, NRF_CRC_WIDTH_2B = 1 } NRF_CRC_WIDTH;

typedef enum { NRF_STATE_RX = 1, NRF_STATE_TX = 0 } NRF_TXRX_STATE;

typedef enum { NRF_OK, NRF_ERROR, NRF_INVALID_ARGUMENT, NRF_TIMEOUT } NRF_RESULT;


#define NRF_RX_FIFO_INT         (1 << 6)
#define NRF_TX_SEND_INT         (1 << 5)
