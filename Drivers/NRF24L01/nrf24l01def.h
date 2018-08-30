#pragma once

#define ON   1
#define OFF  0

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
  
          #define  NRF_CONFIG_RESERVED      BIT7
          #define  NRF_CONFIG_MASK_RX_DR    BIT6
          #define  NRF_CONFIG_MASK_TS_DR    BIT5
          #define  NRF_CONFIG_MASK_MAX_RT   BIT4
          #define  NRF_CONFIG_EN_CRC        BIT3
          #define  NRF_CONFIG_CRCO          BIT2
          #define  NRF_CONFIG_PWR_UP        BIT1
          #define  NRF_CONFIG_PRIM_RX       BIT0 
    
    NRF_EN_AA       = 0x01,
         
          #define  NRF_EN_AA_RESERVED      (BIT7|BIT6)
          #define  NRF_EN_AA_P5             BIT5
          #define  NRF_EN_AA_P4             BIT4
          #define  NRF_EN_AA_P3             BIT3
          #define  NRF_EN_AA_P2             BIT2
          #define  NRF_EN_AA_P1             BIT1
          #define  NRF_EN_AA_P0             BIT0
  
    NRF_EN_RXADDR   = 0x02,
    
          #define  NRF_ERX_RESERVED        (BIT7|BIT6)
          #define  NRF_ERX_P5               BIT5
          #define  NRF_ERX_P4               BIT4
          #define  NRF_ERX_P3               BIT3
          #define  NRF_ERX_P2               BIT2
          #define  NRF_ERX_P1               BIT1
          #define  NRF_ERX_P0               BIT0
  
  
    NRF_SETUP_AW    = 0x03,
  
          #define  NRF_SETUP_AW_RESERVED      (BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|)
          #define  NRF_SETUP_AW_AW            (BIT1|BIT0)                         
  
    NRF_SETUP_RETR  = 0x04,
          
          #define NRF_SETUP_RETR_ARD          (BIT7|BIT6|BIT5|BIT4)          
          #define NRF_SETUP_RETR_ARC          (BIT3|BIT2|BIT1|BIT0)
  
    NRF_RF_CH       = 0x05,

          #define  NRF_RF_CH_FORBIDDEN_BIT    BIT7
          #define  NRF_RF_CH_RF_CHANNEL      (BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|)
         
    NRF_RF_SETUP    = 0x06,
          //RF_SETUP Bits 
          #define  NRF_RF_SETUP_CONT_WAVE     BIT7     
          #define  NRF_RF_SETUP_RESERVED      BIT6
          #define  NRF_RF_SETUP_RF_DR_LOW     BIT5
          #define  NRF_RF_SETUP_PLL_LOCK      BIT4
          #define  NRF_RF_SETUP_RF_DR_HIGH    BIT3
          #define  NRF_RF_SETUP_RF_PWR       (BIT2|BIT1)
          #define  NRF_RF_SETUP_OBSOLETE      BIT0
          
    NRF_STATUS      = 0x07,
    
          #define  NRF_STATUS_RESERVED        BIT7
          #define  NRF_STATUS_RX_DR           BIT6
          #define  NRF_STATUS_TX_DS           BIT5
          #define  NRF_STATUS_MAX_RT          BIT4
          #define  NRF_STATUS_RX_P_NO        (BIT3|BIT2|BIT1)
          #define  NRF_STATUS_TX_FULL         BIT0
          
          
    NRF_OBSERVE_TX  = 0x08,
    
          #define  NRF_OBSERVE_TX_PLOS_CNT    (BIT7|BIT6|BIT5|BIT4)
          #define  NRF_OBSERVE_TX_ARC_CNT     (BIT3|BIT2|BIT1|BIT0)
   
   
    NRF_RPD         = 0x09,
    
          #define  NRF_RPD_RESERVED           (BIT7|BIT6|BIT5|BIT4)
          #define  NRF_RPD_RPD                 BIT0
          
    NRF_RX_ADDR_P0  = 0x0A,     //pipe 0 rx address. 5 bytes max
    NRF_RX_ADDR_P1  = 0x0B,     //pipe 1 rx address. 5 bytes max
    NRF_RX_ADDR_P2  = 0x0C,     //pipe 2 rx address. 1 byte only. high bytes are equal as correspondent bytes of pipe1. 3-5 pipes are the same. and london is the capital of great britain
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
    
          #define  NRF_RX_PW_RESERVED         (BIT7|BIT6)
          #define  NRF_RX_PW_Px               (BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)        // 0 - pipe not used, 1..32 - number of payload byted

    NRF_FIFO_STATUS = 0x17,
    
          #define  NRF_FIFO_STATUS_RESERVED   (BIT7|BIT3|BIT2) 
          #define  NRF_FIFO_STATUS_TX_REUSE    BIT6
          #define  NRF_FIFO_STATUS_TX_FULL     BIT5
          #define  NRF_FIFO_STATUS_TX_EMPYY    BIT4
          #define  NRF_FIFO_STATUS_RX_FULL     BIT1
          #define  NRF_FIFO_STATUS_RX_EMPTY    BIT0
    
    NRF_DYNPD       = 0x1C,
    
          #define  NRF_DYNPD_RESERVED         (BIT7|BIT6)
          #define  NRF_DYNPD_DPL_P5            BIT5
          #define  NRF_DYNPD_DPL_P4            BIT4
          #define  NRF_DYNPD_DPL_P3            BIT3
          #define  NRF_DYNPD_DPL_P2            BIT2
          #define  NRF_DYNPD_DPL_P1            BIT1
          #define  NRF_DYNPD_DPL_P0            BIT0
    
    NRF_FEATURE     = 0x1D
    
          #define  NRF_FEATURE_RESERVED       (BIT7|BIT6|BIT5|BIT4|BIT3)
          #define  NRF_FEATURE_EN_DPL          BIT2
          #define  NRF_FEATURE_EN_ACK_PAY      BIT1
          #define  NRF_FEATURE_EN_DYN_ACK      BIT0
} NRF_REGISTER;



 //CONFIG Bits





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



