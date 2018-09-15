#include "nrf24l01.h"
#include "nrf24l01_usercfg.h"

static void nrf24_ce_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->hwconfig.ce_port, dev->hwconfig.ce_pin, GPIO_PIN_SET);
}

static void nrf24_ce_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->hwconfig.ce_port, dev->hwconfig.ce_pin, GPIO_PIN_RESET);
}

static void nrf24_csn_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->hwconfig.csn_port, dev->hwconfig.csn_pin, GPIO_PIN_SET);
}

static void nrf24_csn_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->hwconfig.csn_port, dev->hwconfig.csn_pin, GPIO_PIN_RESET);
}


NRF_RESULT nrf_init(nrf24l01* dev) {
 
NRF_RESULT res; 
    /*---all macrodefinitions could be found at nrf24l01_defines.h (RF chip specific) and nrf24l01_usercfg.h - configurable settings ---*/
    /*--- Hardware configuration - spi port and irq, cs, ce pins...---*/
        dev->hwconfig.spi                = NRF_TX_SPI_PORT;
        dev->hwconfig.spi_timeout        = NRF_SPI_TIMEOUT; // milliseconds
        dev->hwconfig.ce_port            = NRF_CE_GPIO_Port;
        dev->hwconfig.ce_pin             = NRF_CE_Pin;
        dev->hwconfig.csn_port           = NRF_CSN_GPIO_Port;
        dev->hwconfig.csn_pin            = NRF_CSN_Pin;
        dev->hwconfig.irq_port           = NRF_IRQ_GPIO_Port;
        dev->hwconfig.irq_pin            = NRF_IRQ_Pin;  
  
    /*--- RF channel settings - datarate, power, channel, crc, retransmit timings---*/
        dev->config.data_rate          = NRF_DATARATE;
        dev->config.tx_power           = NRF_TXPOWER;
        dev->config.tx_pload_len       = NRF_TX_PAYLOADLEN;
        dev->config.rf_channel         = NRF_CHANNEL;
        dev->config.crc_en             = NRF_CRC_STATE;
        dev->config.crc_width          = NRF_CRC_WIDTH_CFG;
        dev->config.addr_width         = NRF_ADR_WIDTH_CFG;
  
        dev->config.retransmit_count   = NRF_RETRANSCOUNT;   // maximum is 15 times
        dev->config.retransmit_delay   = NRF_RETRANSDELAY; // 1000us, LSB:250us
          
     
     /*--- datapipes addresses and settings from nrf24l01_usercfg.h---*/
       
       uint8_t  tmp_pipes_en[NRF_PIPE_COUNT]        =  {NRF_RX_PIPE0_EN,        NRF_RX_PIPE1_EN,        NRF_RX_PIPE2_EN,        NRF_RX_PIPE3_EN,        NRF_RX_PIPE4_EN,        NRF_RX_PIPE5_EN};
       uint8_t  tmp_pipes_ack[NRF_PIPE_COUNT]       =  {NRF_RX_PIPE0_ACK,       NRF_RX_PIPE1_ACK,       NRF_RX_PIPE2_ACK,       NRF_RX_PIPE3_ACK,       NRF_RX_PIPE4_ACK,       NRF_RX_PIPE5_ACK};
       uint8_t  tmp_pipes_dynpd[NRF_PIPE_COUNT]     =  {NRF_RX_PIPE0_DYNPD,     NRF_RX_PIPE1_DYNPD,     NRF_RX_PIPE2_DYNPD,     NRF_RX_PIPE3_DYNPD,     NRF_RX_PIPE4_DYNPD,     NRF_RX_PIPE5_DYNPD};
       uint8_t  tmp_pipes_ploadlen[NRF_PIPE_COUNT]  =  {NRF_RX_PIPE0_PLOADLEN,  NRF_RX_PIPE1_PLOADLEN,  NRF_RX_PIPE2_PLOADLEN,  NRF_RX_PIPE3_PLOADLEN,  NRF_RX_PIPE4_PLOADLEN,  NRF_RX_PIPE5_PLOADLEN};
       uint8_t* tmp_pipes_addr[NRF_PIPE_COUNT]      =  {NRF_RX_PIPE0_ADDR,      NRF_RX_PIPE1_ADDR,      NRF_RX_PIPE2_ADDR,      NRF_RX_PIPE3_ADDR,      NRF_RX_PIPE4_ADDR,      NRF_RX_PIPE5_ADDR};
     
       
       memcpy((uint8_t*)&dev->config.tx_addr,    NRF_TX_ADDRESS,      sizeof(dev->config.tx_addr));
       memcpy((uint8_t*)&dev->registers.tx_addr, dev->config.tx_addr, sizeof(dev->registers.tx_addr)); 
       
     /*--- initializing datapipes config fields with predefined values---*/
       
        for (uint8_t i = 0; i< NRF_PIPE_COUNT; i++){
          
          dev->config.pipes[i].enabled = tmp_pipes_en[i];
      if (dev->config.pipes[i].enabled == OFF) continue;
          dev->config.pipes[i].ack_en    = tmp_pipes_ack[i];
          dev->config.pipes[i].pload_len = tmp_pipes_ploadlen[i]; 
          dev->config.pipes[i].dynpd_en  = tmp_pipes_dynpd[i];
                                         
          switch (i) {
              case 0: { memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[i], sizeof(dev->config.pipes[i].address));
                        memcpy((uint8_t*)&dev->registers.rx_addr_p0, (uint8_t*)&dev->config.pipes[i].address, sizeof(dev->registers.rx_addr_p0));
                        break;}
              
              case 1: { memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[i], sizeof(dev->config.pipes[i].address));
                        memcpy((uint8_t*)&dev->registers.rx_addr_p1, (uint8_t*)&dev->config.pipes[i].address, sizeof(dev->registers.rx_addr_p1));                
                        break;} 
              
              case 2: { memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[1], sizeof(dev->config.pipes[i].address)-1);
                        dev->config.pipes[i].address[0] = *tmp_pipes_addr[i];
                        dev->registers.rx_addr_p2 = *tmp_pipes_addr[i];
                        break;}
              
              case 3: { 
                        memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[1], sizeof(dev->config.pipes[i].address)-1);
                        dev->config.pipes[i].address[0] = *tmp_pipes_addr[i];
                        dev->registers.rx_addr_p3 = *tmp_pipes_addr[i];
                        break;}
              
              case 4: { 
                        memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[1], sizeof(dev->config.pipes[i].address)-1);
                        dev->config.pipes[i].address[0] = *tmp_pipes_addr[i];
                        dev->registers.rx_addr_p4 = *tmp_pipes_addr[i];
                        break;}
              
              case 5: { 
                        memcpy((uint8_t*)&dev->config.pipes[i].address, tmp_pipes_addr[1], sizeof(dev->config.pipes[i].address)-1);
                        dev->config.pipes[i].address[0] = *tmp_pipes_addr[i];
                        dev->registers.rx_addr_p5 = *tmp_pipes_addr[i];
                        break;}
              
              default: break;
            } 
          } 
        
       
  
        //i have dohyja time on startup and relatively assignable amount of memory and core mghz to do such things. But anyway I feel sorry
        //maybe later...
          
       





/*-- abowe was only filling of the device structure config fitelds, below goes actual hardware initialization using those fields --*/

        nrf24_ce_reset(dev);
        nrf24_csn_set(dev);
        
 /*--- soft power up the transmitter---*/   
          
 if(nrf_power_up(dev, ON)!= NRF_OK) return NRF_ERROR;
   uint8_t retryleft = 0xFF;   //counter to avoid deadloop if transmitter decided to go home. Normally only couple of cycles enough
    do {
        retryleft--;  
        if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config)!= NRF_OK) return NRF_ERROR;
        if (retryleft == 0) {return NRF_TIMEOUT;}
       } while ((dev->registers.config & NRF_CONFIG_PWR_UP) == 0);  // wait for powerup
              
  
  res  = nrf_enable_crc(dev, dev->config.crc_en);
  res |= nrf_set_crc_width(dev, dev->config.crc_width);     
       
       
  res |= nrf_set_address_width(dev, dev->config.addr_width);     
  res |= nrf_set_tx_address(dev, dev->config.tx_addr); 
       

  res |= nrf_set_rx_pipes(dev, (dev->config.pipes[0].enabled << 0)|
                               (dev->config.pipes[1].enabled << 1)|
                               (dev->config.pipes[2].enabled << 2)|
                               (dev->config.pipes[3].enabled << 3)|
                               (dev->config.pipes[4].enabled << 4)|
                               (dev->config.pipes[5].enabled << 5)); //set pipes state
       
       
  
       
  for (uint8_t i = 0; i < NRF_PIPE_COUNT; i++){
    if (dev->config.pipes[i].enabled != ON) continue;
    res |= nrf_set_pipe_dyn_payload(dev, i, dev->config.pipes[i].dynpd_en);
    res |= nrf_set_auto_ack(dev, i, dev->config.pipes[i].ack_en);
    res |= nrf_set_pipe_address(dev, i, (uint8_t*)&dev->config.pipes[i].address);
    res |= nrf_set_rx_payload_width(dev, i, dev->config.pipes[i].pload_len);   
  }    
 
    res |= nrf_set_rf_channel(dev, dev->config.rf_channel);
    res |= nrf_set_data_rate(dev, dev->config.data_rate);    
    res |= nrf_set_tx_power(dev, dev->config.tx_power);
    res |= nrf_set_retransmit_count(dev, dev->config.retransmit_count);
    res |= nrf_set_retransmit_delay(dev, dev->config.retransmit_delay);

    res |= nrf_clear_interrupts(dev);  
    res |= nrf_enable_rx_data_ready_irq(dev, ON);
    res |= nrf_enable_tx_data_sent_irq(dev, ON);
    res |= nrf_enable_max_retransmit_irq(dev, ON); 

   
    res |= nrf_flush_rx(dev);
    res |= nrf_flush_tx(dev);
    res |= nrf_rx_tx_control(dev, NRF_STATE_RX);
  
    HAL_Delay(140);
    nrf24_ce_set(dev);
  
  
  if (res != NRF_OK) return NRF_ERROR;
  return NRF_OK;
}



NRF_RESULT nrf_send_command(nrf24l01* dev, uint8_t cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len) {
  
    dev->rtxbuf[0] = cmd;
    memcpy(dev->rtxbuf + 1, tx, len);
    nrf24_csn_reset(dev);

    if (HAL_SPI_TransmitReceive(dev->hwconfig.spi, dev->rtxbuf, dev->rtxbuf, 1 + len, dev->hwconfig.spi_timeout) != HAL_OK) { return NRF_ERROR; }
    
    nrf24_csn_set(dev);
    memcpy(rx, dev->rtxbuf+1, len);
    

    return NRF_OK;
}

void nrf_irq_handler(nrf24l01* dev) {
      
    if (nrf_read_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status) != NRF_OK) { return; }

    if ((dev->registers.status & NRF_STATUS_RX_DR)) { // RX FIFO Interrupt
        nrf24_ce_reset(dev);
        nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
        nrf_read_register(dev, NRF_FIFO_STATUS, (uint8_t*)&dev->registers.fifo_status);
        if ((dev->registers.fifo_status & NRF_FIFO_STATUS_RX_EMPTY) == 0) {
            nrf_read_rx_payload(dev, dev->rxpayload);
            dev->registers.status |= NRF_STATUS_RX_DR;                                 // potentially datalost section. shoul be rewritten
            nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
            // nrf_flush_rx(dev);
            nrf_packet_received_callback(dev, dev->rxpayload);
        }
        nrf24_ce_set(dev);
    }
    if ((dev->registers.status & NRF_STATUS_TX_DS)) { // TX Data Sent Interrupt
        dev->registers.status |= NRF_STATUS_TX_DS;      // clear the interrupt flag
        nrf24_ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        nrf24_ce_set(dev);
        nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
        dev->tx_result = NRF_OK;
        dev->tx_busy   = 0;
    }
    if ((dev->registers.status & NRF_CONFIG_MASK_MAX_RT)) { // MaxRetransmits reached
        dev->registers.status |= NRF_CONFIG_MASK_MAX_RT;

        nrf_flush_tx(dev);
        nrf_power_up(dev, OFF); // power down
        nrf_power_up(dev, ON); // power up

        nrf24_ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        nrf24_ce_set(dev);

        nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
        dev->tx_result = NRF_ERROR;
        dev->tx_busy   = 0;
    }
}

__weak void nrf_packet_received_callback(nrf24l01* dev, uint8_t* data) {
    // default implementation (__weak) is used in favor of nrf_receive_packet
    dev->rx_busy = 0;
}

NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t tx = 0;
    return nrf_send_command(dev, NRF_CMD_R_REGISTER | reg, &tx, data, 1);
}

NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t rx = 0;
    return nrf_send_command(dev, NRF_CMD_W_REGISTER | reg, data, &rx, 1); 
}

NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data) {
      
    uint8_t tx[dev->config.pipes[0].pload_len];
    return nrf_send_command(dev, NRF_CMD_R_RX_PAYLOAD, tx, data, dev->config.pipes[0].pload_len); 
}

NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data) {
    uint8_t rx[dev->config.tx_pload_len];
    return nrf_send_command(dev, NRF_CMD_W_TX_PAYLOAD, data, rx, dev->config.tx_pload_len);
}

NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data) {
    uint8_t rx[dev->config.tx_pload_len];
    return nrf_send_command(dev, NRF_CMD_W_TX_PAYLOAD_NOACK, data, rx, dev->config.tx_pload_len);
}

NRF_RESULT nrf_flush_tx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    return nrf_send_command(dev, NRF_CMD_FLUSH_TX, &tx, &rx, 0);
}

NRF_RESULT nrf_flush_rx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    return nrf_send_command(dev, NRF_CMD_FLUSH_RX, &tx, &rx, 0);
}

NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate) {
    //uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}
    if (rate & BIT0) { // low bit set
        dev->registers.rf_setup |= NRF_RF_SETUP_RF_DR_LOW;
    } else { // low bit clear
        dev->registers.rf_setup &= ~NRF_RF_SETUP_RF_DR_LOW;
    }

    if (rate & BIT1) { // high bit set
        dev->registers.rf_setup |= NRF_RF_SETUP_RF_DR_HIGH;    
    } else { // high bit clear
        dev->registers.rf_setup &= ~NRF_RF_SETUP_RF_DR_HIGH;
    }
    if (nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}
    dev->config.data_rate = rate;
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr) {
    if (nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->config.tx_power) != NRF_OK) {return NRF_ERROR;}
    dev->config.tx_power &= ~NRF_RF_SETUP_RF_PWR; // clear bits 1,2
    dev->config.tx_power |= (pwr << 1);           // set bits   1,2
    if (nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->config.tx_power) != NRF_OK) {return NRF_ERROR;}
    dev->config.tx_power = pwr;
    return nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup);
}

NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate) {
    if (nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}

    if (activate) {
        dev->registers.rf_setup |= NRF_RF_SETUP_CONT_WAVE;
    } else {
        dev->registers.rf_setup &= ~NRF_RF_SETUP_CONT_WAVE;
    }

    return nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup);
}

NRF_RESULT nrf_clear_interrupts(nrf24l01* dev) {
    if (nrf_read_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status) != NRF_OK) {return NRF_ERROR;}

    dev->registers.status |= (NRF_CONFIG_MASK_RX_DR|NRF_CONFIG_MASK_TS_DR|NRF_CONFIG_MASK_MAX_RT);  //setting bits 4,5,6

    return nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
}

NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch) {
    ch &= ~NRF_RF_CH_FORBIDDEN_BIT;       //set to zero manufacturer "reserved" bit
    if (ch > 125) return NRF_INVALID_ARGUMENT ; 
    dev->config.rf_channel = ch; // setting channel
    if (nrf_write_register(dev, NRF_RF_CH, (uint8_t*)&dev->config.rf_channel) != NRF_OK) {return NRF_ERROR;}
    return  nrf_read_register(dev, NRF_RF_CH, (uint8_t*)&dev->registers.rf_ch);
}

NRF_RESULT nrf_set_retransmit_count(nrf24l01* dev, uint8_t count) {
    count &= NRF_SETUP_RETR_ARC;       //0x0F;
    if (nrf_read_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->config.retransmit_count) != NRF_OK) {return NRF_ERROR;}
    dev->config.retransmit_count &= NRF_SETUP_RETR_ARC;
    dev->config.retransmit_count |= count; // setting count
    if (nrf_write_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->config.retransmit_count) != NRF_OK) {return NRF_ERROR;}
    return nrf_read_register(dev, NRF_SETUP_RETR,(uint8_t*)&dev->registers.setup_retr);     
}

NRF_RESULT nrf_set_retransmit_delay(nrf24l01* dev, uint8_t delay) {
    delay &= 0x0F;
    if (nrf_read_register(dev, NRF_SETUP_RETR,(uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}

    dev->registers.setup_retr &= 0x0F;       // clearing bits 4,5,6,7
    dev->registers.setup_retr |= delay << 4; // setting delay

    if (nrf_write_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}
    dev->config.retransmit_delay = delay;
    return NRF_OK;
}

NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width) {
    if ((width > 3)|(width <1))  return NRF_INVALID_ARGUMENT;
    if (nrf_write_register(dev, NRF_SETUP_AW, (uint8_t*)&width) != NRF_OK) {return NRF_ERROR;}
    dev->config.addr_width = width;
    return nrf_read_register(dev, NRF_SETUP_AW, (uint8_t*)&dev->registers.setup_aw);
}


NRF_RESULT nrf_togglefeatures(nrf24l01* dev)

{ uint8_t dt[1] = {NRF_CMD_ACTIVATE};

  nrf24_csn_set(dev);
  HAL_SPI_Transmit(dev->hwconfig.spi, dt, 1, 1000);
  HAL_Delay(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(dev->hwconfig.spi, dt, 1, 1000);
  nrf24_csn_reset(dev);
  return NRF_OK;
}


NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes) {
    dev->registers.en_rxaddr  = (pipes & 0x3F);
    return nrf_write_register(dev, NRF_EN_RXADDR, (uint8_t*)&dev->registers.en_rxaddr);
}

NRF_RESULT nrf_set_auto_ack(nrf24l01* dev, uint8_t pipe_num, uint8_t ack_state) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_read_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa) != NRF_OK) { return NRF_ERROR; }
    if (ack_state > 0){dev->registers.en_aa |= (1 << pipe_num);} else {dev->registers.en_aa &= ~(1 << pipe_num);}
    return nrf_write_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa);
 } 
 

NRF_RESULT nrf_set_pipe_dyn_payload(nrf24l01* dev, uint8_t pipe_num, uint8_t dynpd_state) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_read_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd) != NRF_OK) { return NRF_ERROR; }
    if (dynpd_state > 0) {
    if (nrf_set_auto_ack(dev, pipe_num, dynpd_state) != NRF_OK) return NRF_ERROR;
        dev->registers.dynpd |= (1 << pipe_num); 
    if (nrf_set_feature(dev, 0x01, 0x01, 0x01)!= NRF_OK) return NRF_ERROR;
    }
    else {
        dev->registers.dynpd &= ~(1 << pipe_num);
         }
    if (dev->registers.dynpd & 0x3F)  {if (nrf_set_feature(dev, 0x01, 0x01, 0x01) != NRF_OK) return NRF_ERROR;}; 
    return nrf_write_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd);
    
}


NRF_RESULT nrf_set_feature(nrf24l01* dev, uint8_t en_dpl, uint8_t en_ack_pay, uint8_t en_dyn_ack) {
    if (nrf_read_register(dev, NRF_FEATURE, (uint8_t*)&dev->registers.feature) != NRF_OK) {return NRF_ERROR;}
    
    if (en_dpl) {
        dev->registers.feature |= BIT2;
    } else {
        dev->registers.feature &= ~(BIT2);
    }
if (en_ack_pay) {
        dev->registers.feature |= BIT1;
    } else {
        dev->registers.feature &= ~(BIT1);
    }
if (en_dyn_ack) {
        dev->registers.feature |= BIT0;
    } else {
        dev->registers.feature &= ~(BIT0);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.feature);
  }

NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate) {
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (activate) {
        dev->registers.config |= BIT3;
    } else {
        dev->registers.config &= ~(BIT3);
    }

    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}

    if (width == NRF_CRC_WIDTH_2B) {
        dev->registers.config |= BIT2;
    } else {
        dev->registers.config &= ~(BIT2);
    }

    if (nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    dev->config.crc_width = width;
    return NRF_OK;
}

NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (power_up) {
        dev->registers.config |= BIT1;
    } else {
        dev->registers.config &= ~(BIT1);
    }

    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_TXRX_STATE rx) {
   
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (rx) {
        dev->registers.config |= BIT0; 
        dev->state = NRF_STATE_RX;
    } else {
        dev->registers.config &= ~(BIT0);
        dev->state = NRF_STATE_TX;
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate) {
  if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}

    if (!activate) {
        dev->registers.config |= BIT6;
    } else {
        dev->registers.config &= ~(BIT6);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (!activate) {
        dev->registers.config |= BIT5;
    } else {
        dev->registers.config &= ~(BIT5);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate) {
   
    if (nrf_read_register(dev, NRF_CONFIG,(uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (!activate) {
        dev->registers.config |= BIT4;
    } else {
        dev->registers.config &= ~(BIT4);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_set_pipe_address(nrf24l01* dev, uint8_t pipe_num ,const uint8_t* address) {              
    uint8_t rx[dev->config.addr_width + 2];
    uint8_t addrsize = dev->config.addr_width + 2;
  
    if (pipe_num < 2) {
        
        if (nrf_send_command(dev, NRF_CMD_W_REGISTER | (NRF_RX_ADDR_P0 + pipe_num), address, rx, addrsize) != NRF_OK) {return NRF_ERROR;}
        if (pipe_num == 0) {
                            memcpy((uint8_t*)&dev->registers.rx_addr_p0, address, addrsize);
                            memcpy((uint8_t*)&dev->config.pipes[pipe_num].address, (uint8_t*)&dev->registers.rx_addr_p0, addrsize);
                           } else {
                                  memcpy((uint8_t*)&dev->registers.rx_addr_p1, address, addrsize);
                                  memcpy((uint8_t*)&dev->config.pipes[pipe_num].address, (uint8_t*)&dev->registers.rx_addr_p1, addrsize);}
      } else {
        addrsize = 1;
        if (nrf_send_command(dev, NRF_CMD_W_REGISTER | (NRF_RX_ADDR_P0 + pipe_num), address, rx, addrsize) != NRF_OK) {return NRF_ERROR;}
            memcpy((uint8_t*)&dev->registers.rx_addr_p2 + pipe_num - 2, address, addrsize);
            memcpy((uint8_t*)&dev->config.pipes[pipe_num].address, address, addrsize);
      }
      
   return NRF_OK;
}

NRF_RESULT nrf_set_tx_address(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx, 5) != NRF_OK) {return NRF_ERROR;}
                        
    memcpy(dev->config.tx_addr, address, dev->config.addr_width);
    memcpy((uint8_t*)&dev->registers.tx_addr, (uint8_t*)&dev->config.tx_addr, dev->config.addr_width);
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_payload_width(nrf24l01* dev, uint8_t pipe_num, uint8_t width) {
    width &= 0x3F;
    if (nrf_write_register(dev, NRF_RX_PW_P0 + pipe_num, &width) != NRF_OK) {
        dev->config.pipes[pipe_num].pload_len = 0;
        return NRF_ERROR;
    }
    dev->config.pipes[pipe_num].pload_len = width;
    return NRF_OK;
}

//NRF_RESULT nrf_set_rx_payload_width_p1(nrf24l01* dev, uint8_t width) {
//    width &= 0x3F;
//    if (nrf_write_register(dev, NRF_RX_PW_P1, &width) != NRF_OK) {
//        dev->config.pipes[1].pload_len = 0;
//        return NRF_ERROR;
//    }
//    dev->config.pipes[1].pload_len = width;
//    return NRF_OK;
//}

NRF_RESULT nrf_read_observe_tx(nrf24l01* dev) {
   
    return nrf_read_register(dev, NRF_OBSERVE_TX,(uint8_t*)&dev->registers.observe_tx);
 }

uint8_t nrf_carrier_detect(nrf24l01* dev) {
   
  uint8_t res = nrf_read_register(dev, NRF_RPD,(uint8_t*)&dev->registers.rpd);
  if (res != NRF_OK) {return NRF_ERROR;}
  return (dev->registers.rpd); 
  
 }


NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data) {

    dev->tx_busy = 1;

    nrf24_ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    nrf24_ce_set(dev);
    HAL_Delay(15);
    nrf24_ce_reset(dev);
 
    while (dev->tx_busy == 1) {} // wait for end of transmittion

    return dev->tx_result;
}

NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data) {
    dev->tx_busy = 1;

    nrf24_ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload_noack(dev, data);
    nrf24_ce_set(dev);

    while (dev->tx_busy == 1) {} // wait for end of transmittion

    return dev->tx_result;
}

const uint8_t* nrf_receive_packet(nrf24l01* dev) {

    dev->rx_busy = 1;

    nrf24_ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_RX);
    nrf24_ce_set(dev);

    while (dev->rx_busy == 1) {} // wait for reception

    return dev->rxpayload;
}

NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data) {

    if (dev->tx_busy == 1) {
        nrf_flush_tx(dev);
    } else {
        dev->tx_busy = 1;
    }

    nrf24_ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    nrf24_ce_set(dev);

    return NRF_OK;
}
