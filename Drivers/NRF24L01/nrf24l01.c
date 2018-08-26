#include "nrf24l01.h"
#include "nrf24l01cfg.h"

static void nrf24_ce_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_SET);
}

static void nrf24_ce_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_RESET);
}

static void nrf24_csn_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_SET);
}

static void nrf24_csn_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_RESET);
}


NRF_RESULT nrf_init(nrf24l01* dev) {
 
NRF_RESULT res; 
  
    /*--- Hardware configuration - spi port, irq pins etc...---*/
        dev->config.spi                = NRF_TX_SPI_PORT;
        dev->config.spi_timeout        = NRF_SPI_TIMEOUT; // milliseconds
        dev->config.ce_port            = NRF_CE_GPIO_Port;
        dev->config.ce_pin             = NRF_CE_Pin;
        dev->config.csn_port           = NRF_CSN_GPIO_Port;
        dev->config.csn_pin            = NRF_CSN_Pin;
        dev->config.irq_port           = NRF_IRQ_GPIO_Port;
        dev->config.irq_pin            = NRF_IRQ_Pin;  
  
    /*--- RF channel settings - datarate, power, channel, crc, retransmit timings---*/
        dev->config.data_rate          = NRF_DATARATE;
        dev->config.tx_power           = NRF_TXPOWER;
        dev->config.rf_channel         = NRF_CHANNEL;
        dev->config.crc_width          = NRF_CRC_WITH;
        dev->config.addr_width         = NRF_ADR_WITH;
  
        dev->config.retransmit_count   = NRF_RETRANSCOUNT;   // maximum is 15 times
        dev->config.retransmit_delay   = NRF_RETRANSDELAY; // 4000us, LSB:250us
          
     
     /*--- datapipes addresses and settings---*/
        if (NRF_RX_PIPE0_EN) {
        dev->config.pipes[0].enabled = ON;
        dev->config.pipes[0].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[0].dynpd_en  = NRF_RX_PIPE0_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[0].address, NRF_RX_PIPE0_ADDR, sizeof(dev->config.pipes[0].address));
        memcpy((uint8_t*)&dev->registers.rx_addr_p0, (uint8_t*)&dev->config.pipes[0].address, sizeof(dev->registers.rx_addr_p0));  
        }
     
        if (NRF_RX_PIPE1_EN) {
        dev->config.pipes[1].enabled = ON;
        dev->config.pipes[1].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[1].dynpd_en  = NRF_RX_PIPE1_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[1].address, NRF_RX_PIPE1_ADDR, sizeof(dev->config.pipes[1].address));
        memcpy((uint8_t*)&dev->registers.rx_addr_p1, (uint8_t*)&dev->config.pipes[1].address, sizeof(dev->registers.rx_addr_p1));  
        }     
                
        
        
        if (NRF_RX_PIPE2_EN) {
        dev->config.pipes[2].enabled = ON;
        dev->config.pipes[2].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[2].dynpd_en  = NRF_RX_PIPE0_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[2].address, NRF_RX_PIPE1_ADDR, sizeof(dev->config.pipes[2].address));
        dev->config.pipes[2].address[5] = NRF_RX_PIPE2_ADDR;
        memcpy((uint8_t*)&dev->registers.rx_addr_p2, (uint8_t*)&dev->config.pipes[2].address, sizeof(dev->registers.rx_addr_p2));  
        }


        if (NRF_RX_PIPE3_EN) {
        dev->config.pipes[3].enabled = ON;
        dev->config.pipes[3].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[3].dynpd_en  = NRF_RX_PIPE3_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[3].address, NRF_RX_PIPE1_ADDR, sizeof(dev->config.pipes[2].address));
        dev->config.pipes[3].address[5] = NRF_RX_PIPE3_ADDR;
        memcpy((uint8_t*)&dev->registers.rx_addr_p3, (uint8_t*)&dev->config.pipes[3].address, sizeof(dev->registers.rx_addr_p3));  
        }

        if (NRF_RX_PIPE4_EN) {
        dev->config.pipes[4].enabled = ON;
        dev->config.pipes[4].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[4].dynpd_en  = NRF_RX_PIPE4_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[4].address, NRF_RX_PIPE1_ADDR, sizeof(dev->config.pipes[2].address));
        dev->config.pipes[4].address[5] = NRF_RX_PIPE4_ADDR;
        memcpy((uint8_t*)&dev->registers.rx_addr_p4, (uint8_t*)&dev->config.pipes[4].address, sizeof(dev->registers.rx_addr_p4));  
        }
        
        if (NRF_RX_PIPE5_EN) {
        dev->config.pipes[5].enabled = ON;
        dev->config.pipes[5].pload_len = NRF_PAYLOADLEN; 
        dev->config.pipes[5].dynpd_en  = NRF_RX_PIPE5_DYNPD;
        memcpy((uint8_t*)&dev->config.pipes[5].address, NRF_RX_PIPE1_ADDR, sizeof(dev->config.pipes[2].address));
        dev->config.pipes[5].address[5] = NRF_RX_PIPE5_ADDR;
        memcpy((uint8_t*)&dev->registers.rx_addr_p5, (uint8_t*)&dev->config.pipes[5].address, sizeof(dev->registers.rx_addr_p5));  
        }
        
        
        memcpy((uint8_t*)&dev->registers.tx_addr,    NRF_TX_ADDRESS,    sizeof(dev->registers.tx_addr));
        memcpy((uint8_t*)&dev->config.tx_addr,   (uint8_t*)&dev->registers.tx_addr,    sizeof(dev->registers.tx_addr));       
      
        //i have dohyja time on startup and relatively assignable amount of memory and core mghz to do such things. But anyway I feel sorry
        //maybe later...
       
        dev->registers.rx_addr_p2 = NRF_RX_PIPE2_ADDR;
        dev->registers.rx_addr_p3 = NRF_RX_PIPE3_ADDR;
        dev->registers.rx_addr_p4 = NRF_RX_PIPE4_ADDR;
        dev->registers.rx_addr_p5 = NRF_RX_PIPE5_ADDR;
        
        nrf24_ce_reset(dev);
        nrf24_csn_set(dev);

        if(nrf_power_up(dev, true)!= NRF_OK) return NRF_ERROR;
        
        uint8_t retryleft = 0xFF;   //counter to avoid deadloop if ic decided to go home. Normally only one cycle pass enough
         do {
            retryleft--;  
            if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config)!= NRF_OK) return NRF_ERROR;
            if (retryleft == 0) {return NRF_ERROR;}
         } while ((dev->registers.config & NRF_PWR_UP_BIT) == 0);  // wait for powerup
              
    
         
         
  res  = nrf_enable_crc(dev, NRF_CRC_ON);
  res |= nrf_set_crc_width(dev, dev->config.crc_width);

  
  res |= nrf_set_rx_pipes(dev, 0b00000011); //enable pipe0 and pipe1
         
         
  if (NRF_ENABLE_DYNP) {      
  res |= nrf_set_feature(dev, 0x01, 0x01, 0x01);    
  res |= nrf_enable_pipe_dyn_payload(dev, 0);
  res |= nrf_enable_pipe_dyn_payload(dev, 1);
  } else {  
  res |= nrf_enable_auto_ack(dev, 0); //enable pipe0 autoACK
  res |= nrf_enable_auto_ack(dev, 1); //enable pipe1 autoACK
  res |= nrf_set_rx_payload_width_p0(dev, dev->config.pipes[0].pload_len);
  res |= nrf_set_rx_payload_width_p1(dev, dev->config.pipes[1].pload_len);
  }
     
  res |= nrf_set_address_width(dev, dev->config.addr_width);
  res |= nrf_set_tx_address(dev, dev->config.tx_addr);
  res |= nrf_set_rx_address_p0(dev, (uint8_t*)&dev->config.pipes[0].address);
  res |= nrf_set_rx_address_p1(dev, (uint8_t*)&dev->config.pipes[1].address); 
     

  res |= nrf_set_rf_channel(dev, dev->config.rf_channel);
  res |= nrf_set_data_rate(dev, dev->config.data_rate);    
  res |= nrf_set_tx_power(dev, dev->config.tx_power);
  res |= nrf_set_retransmittion_count(dev, dev->config.retransmit_count);
  res |= nrf_set_retransmittion_delay(dev, dev->config.retransmit_delay);

  res |= nrf_clear_interrupts(dev);  
  res |= nrf_enable_rx_data_ready_irq(dev, 1);
  res |= nrf_enable_tx_data_sent_irq(dev, 1);
  res |= nrf_enable_max_retransmit_irq(dev, 1); 

   
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
    memcpy(dev->rtxbuf+1, tx, len);
    nrf24_csn_reset(dev);

    if (HAL_SPI_TransmitReceive(dev->config.spi, dev->rtxbuf, dev->rtxbuf, 1 + len, dev->config.spi_timeout) != HAL_OK) { return NRF_ERROR; }
    
    nrf24_csn_set(dev);
    memcpy(rx, dev->rtxbuf+1, len);
    

    return NRF_OK;
}

void nrf_irq_handler(nrf24l01* dev) {
      
    if (nrf_read_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status) != NRF_OK) { return; }

    if ((dev->registers.status & NRF_RX_FIFO_INT)) { // RX FIFO Interrupt
        nrf24_ce_reset(dev);
        nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
        nrf_read_register(dev, NRF_FIFO_STATUS, (uint8_t*)&dev->registers.fifo_status);
        if ((dev->registers.fifo_status & 1) == 0) {
            nrf_read_rx_payload(dev, dev->rxpayload);
            dev->registers.status |= NRF_RX_FIFO_INT;                                 // potentially datalost section. shoul be rewritten
            nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
            // nrf_flush_rx(dev);
            nrf_packet_received_callback(dev, dev->rxpayload);
        }
        nrf24_ce_set(dev);
    }
    if ((dev->registers.status & NRF_TX_SEND_INT)) { // TX Data Sent Interrupt
        dev->registers.status |= NRF_TX_SEND_INT;      // clear the interrupt flag
        nrf24_ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        nrf24_ce_set(dev);
        nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
        dev->tx_result = NRF_OK;
        dev->tx_busy   = 0;
    }
    if ((dev->registers.status & NRF_MAX_RETRANSMITS_INT)) { // MaxRetransmits reached
        dev->registers.status |= NRF_MAX_RETRANSMITS_INT;

        nrf_flush_tx(dev);
        nrf_power_up(dev, 0); // power down
        nrf_power_up(dev, 1); // power up

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
    if (rate & 1) { // low bit set
        dev->registers.rf_setup |= 1 << 5;
    } else { // low bit clear
        dev->registers.rf_setup &= ~(1 << 5);
    }

    if (rate & 2) { // high bit set
        dev->registers.rf_setup |= 1 << 3;
    } else { // high bit clear
        dev->registers.rf_setup &= ~(1 << 3);
    }
    if (nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}
    dev->config.data_rate = rate;
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr) {
    if (nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}
    dev->registers.rf_setup &= 0xF9;     // clear bits 1,2
    dev->registers.rf_setup |= (pwr << 1); // set bits 1,2
    if (nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}
    dev->config.tx_power = pwr;
    return NRF_OK;
}

NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate) {
    if (nrf_read_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup) != NRF_OK) {return NRF_ERROR;}

    if (activate) {
        dev->registers.rf_setup |= 0x80;
    } else {
        dev->registers.rf_setup &= 0x7F;
    }

    return nrf_write_register(dev, NRF_RF_SETUP, (uint8_t*)&dev->registers.rf_setup);
}

NRF_RESULT nrf_clear_interrupts(nrf24l01* dev) {
    if (nrf_read_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status) != NRF_OK) {return NRF_ERROR;}

    dev->registers.status |= 0b01110000; // setting bits 4,5,6

    return nrf_write_register(dev, NRF_STATUS, (uint8_t*)&dev->registers.status);
}

NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch) {
    ch &= 0x7F;
    if (nrf_read_register(dev, NRF_RF_CH, (uint8_t*)&dev->registers.rf_ch) != NRF_OK) { return NRF_ERROR; }

    dev->registers.rf_ch |= ch; // setting channel

    if (nrf_write_register(dev, NRF_RF_CH, (uint8_t*)&dev->registers.rf_ch) != NRF_OK) {return NRF_ERROR;}
    
    dev->config.rf_channel = ch;
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmittion_count(nrf24l01* dev, uint8_t count) {
    count &= 0x0F;
    if (nrf_read_register(dev, NRF_SETUP_RETR,(uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}

    dev->registers.setup_retr &= 0xF0;  // clearing bits 0,1,2,3
    dev->registers.setup_retr |= count; // setting count

    if (nrf_write_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}
    
    dev->config.retransmit_count = count;
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmittion_delay(nrf24l01* dev, uint8_t delay) {
    delay &= 0x0F;
    if (nrf_read_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}

    dev->registers.setup_retr &= 0x0F;       // clearing bits 1,2,6,7
    dev->registers.setup_retr |= delay << 4; // setting delay

    if (nrf_write_register(dev, NRF_SETUP_RETR, (uint8_t*)&dev->registers.setup_retr) != NRF_OK) {return NRF_ERROR;}
    dev->config.retransmit_delay = delay;
    return NRF_OK;
}

NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width) {
    
    if (nrf_read_register(dev, NRF_SETUP_AW, (uint8_t*)&dev->registers.setup_aw) != NRF_OK) {return NRF_ERROR;}

    dev->registers.setup_aw &= 0b00000011;  // clearing bits 0,1
    dev->registers.setup_aw |= width; // setting delay

    if (nrf_write_register(dev, NRF_SETUP_AW, (uint8_t*)&dev->registers.setup_aw) != NRF_OK) {return NRF_ERROR;}
    dev->config.addr_width = width;
    return NRF_OK;
}


NRF_RESULT nrf_togglefeatures(nrf24l01* dev)

{ uint8_t dt[1] = {NRF_CMD_ACTIVATE};

  nrf24_csn_set(dev);
  HAL_SPI_Transmit(dev->config.spi, dt, 1, 1000);
  HAL_Delay(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(dev->config.spi, dt, 1, 1000);
  nrf24_csn_reset(dev);
  return NRF_OK;
}
NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes) {
    dev->registers.en_rxaddr  = (pipes & 0x3F);
    return nrf_write_register(dev, NRF_EN_RXADDR, (uint8_t*)&dev->registers.en_rxaddr);
}

NRF_RESULT nrf_enable_auto_ack(nrf24l01* dev, uint8_t pipe_num) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_read_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa) != NRF_OK) { return NRF_ERROR; }
    dev->registers.en_aa |= 1 << pipe_num;
    return nrf_write_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa);
 } 

 NRF_RESULT nrf_disable_auto_ack(nrf24l01* dev, uint8_t pipe_num) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_read_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa) != NRF_OK) { return NRF_ERROR; }
    dev->registers.en_aa &= (1 << pipe_num);
    return nrf_write_register(dev, NRF_EN_AA, (uint8_t*)&dev->registers.en_aa);
 }

NRF_RESULT nrf_enable_pipe_dyn_payload(nrf24l01* dev, uint8_t pipe_num) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_enable_auto_ack(dev, pipe_num) != NRF_OK) return NRF_ERROR;
    if (nrf_read_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd) != NRF_OK) { return NRF_ERROR; }
    dev->registers.dynpd |= 1 << pipe_num;
    return nrf_write_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd);
    
}

NRF_RESULT nrf_disable_pipe_dyn_payload(nrf24l01* dev, uint8_t pipe_num) {
    if (pipe_num > 5) return  NRF_INVALID_ARGUMENT;
    if (nrf_read_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd) != NRF_OK) { return NRF_ERROR; }
    dev->registers.dynpd &= ~(1 << pipe_num);
    return nrf_write_register(dev, NRF_DYNPD, (uint8_t*)&dev->registers.dynpd);
    
}


NRF_RESULT nrf_set_feature(nrf24l01* dev, uint8_t en_dpl, uint8_t en_ack_pay, uint8_t en_dyn_ack) {
    if (nrf_read_register(dev, NRF_FEATURE, (uint8_t*)&dev->registers.feature) != NRF_OK) {return NRF_ERROR;}
    
    if (en_dpl) {
        dev->registers.feature |= 1 << 2;
    } else {
        dev->registers.feature &= ~(1 << 2);
    }
if (en_ack_pay) {
        dev->registers.feature |= 1 << 1;
    } else {
        dev->registers.feature &= ~(1 << 1);
    }
if (en_dyn_ack) {
        dev->registers.feature |= 1 << 0;
    } else {
        dev->registers.feature &= ~(1 << 0);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.feature);
  }

NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate) {
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (activate) {
        dev->registers.config |= 1 << 3;
    } else {
        dev->registers.config &= ~(1 << 3);
    }

    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}

    if (width == NRF_CRC_WIDTH_2B) {
        dev->registers.config |= 1 << 2;
    } else {
        dev->registers.config &= ~(1 << 2);
    }

    if (nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    dev->config.crc_width = width;
    return NRF_OK;
}

NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (power_up) {
        dev->registers.config |= 1 << 1;
    } else {
        dev->registers.config &= ~(1 << 1);
    }

    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_TXRX_STATE rx) {
   
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (rx) {
        dev->registers.config |= 1; 
        dev->state = NRF_STATE_RX;
    } else {
        dev->registers.config &= ~(1);
        dev->state = NRF_STATE_TX;
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate) {
  if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}

    if (!activate) {
        dev->registers.config |= 1 << 6;
    } else {
        dev->registers.config &= ~(1 << 6);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate) {
    
    if (nrf_read_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (!activate) {
        dev->registers.config |= 1 << 5;
    } else {
        dev->registers.config &= ~(1 << 5);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate) {
   
    if (nrf_read_register(dev, NRF_CONFIG,(uint8_t*)&dev->registers.config) != NRF_OK) {return NRF_ERROR;}
    if (!activate) {
        dev->registers.config |= 1 << 4;
    } else {
        dev->registers.config &= ~(1 << 4);
    }
    return nrf_write_register(dev, NRF_CONFIG, (uint8_t*)&dev->registers.config);
}

NRF_RESULT nrf_set_rx_address_p0(nrf24l01* dev, const uint8_t* address) {              
    uint8_t rx[5];
    if (nrf_send_command(dev, (uint8_t)NRF_CMD_W_REGISTER | (uint8_t)NRF_RX_ADDR_P0, address, rx, 5) != NRF_OK) {return NRF_ERROR;}
    memcpy((uint8_t*)&dev->registers.rx_addr_p0, address, sizeof(dev->registers.rx_addr_p0));
    memcpy((uint8_t*)&dev->config.pipes[0].address, (uint8_t*)&dev->registers.rx_addr_p0, sizeof(dev->registers.rx_addr_p0)); 
   return NRF_OK;
}

NRF_RESULT nrf_set_rx_address_p1(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_RX_ADDR_P1, address, rx, 5) != NRF_OK) {return NRF_ERROR;}
    memcpy((uint8_t*)&dev->registers.rx_addr_p1, address, sizeof(dev->registers.rx_addr_p1));
    memcpy((uint8_t*)&dev->config.pipes[0].address, (uint8_t*)&dev->registers.rx_addr_p1, sizeof(dev->registers.rx_addr_p1));     
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_address(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx, 5) != NRF_OK) {return NRF_ERROR;}
                        
    memcpy(dev->config.tx_addr, address, sizeof(dev->config.tx_addr));
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_payload_width_p0(nrf24l01* dev, uint8_t width) {
    width &= 0x3F;
    if (nrf_write_register(dev, NRF_RX_PW_P0, &width) != NRF_OK) {
        dev->config.pipes[0].pload_len = 0;
        return NRF_ERROR;
    }
    dev->config.pipes[0].pload_len = width;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_payload_width_p1(nrf24l01* dev, uint8_t width) {
    width &= 0x3F;
    if (nrf_write_register(dev, NRF_RX_PW_P1, &width) != NRF_OK) {
        dev->config.pipes[1].pload_len = 0;
        return NRF_ERROR;
    }
    dev->config.pipes[1].pload_len = width;
    return NRF_OK;
}

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
