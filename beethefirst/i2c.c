#include "i2c.h"

void i2c_init(void)
{

  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_3; // SDA1 function
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 0;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Funcnum = PINSEL_FUNC_3; // SCL1 function
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 1;
  PINSEL_ConfigPin(&PinCfg);

  I2C_Init(LPC_I2C1,100000);

  I2C_Cmd(LPC_I2C1,I2C_MASTER_MODE,ENABLE);
  I2C_IntCmd(LPC_I2C1,DISABLE);

  i2c_setup_Pot();
}

void i2c_setup_Pot(void)
{
  i2c_pot_set_current(0x2c,0x00,0xAA);  //X AXIS 1.5A
  i2c_pot_set_current(0x2c,0x10,0xAA);  //Y AXIS 1.5A
  i2c_pot_set_current(0x2c,0x60,0x72);  //Z AXIS 1A
  i2c_pot_set_current(0x2c,0x70,0xAA);  //E AXIS 1.5A
}

void i2c_pot_set_current(uint8_t add, uint8_t ch, uint8_t val)
{
  Status I2C_status;
  I2C_M_SETUP_Type i2c_msg;
  uint8_t data[2] = {};
  i2c_msg.sl_addr7bit = add;

  data[0] = 0x40;
  data[1] = 0xff;
  i2c_msg.tx_data = &data;
  i2c_msg.tx_length = 2;
  i2c_msg.rx_length = 0;

  I2C_status = I2C_MasterTransferData(LPC_I2C1,&i2c_msg,I2C_TRANSFER_POLLING);

  data[0] = 0xA0;
  i2c_msg.tx_data = &data;
  i2c_msg.tx_length = 2;
  i2c_msg.rx_length = 0;

  I2C_status = I2C_MasterTransferData(LPC_I2C1,&i2c_msg,I2C_TRANSFER_POLLING);

  data[0] = ch;
  data[1] = val;
  i2c_msg.tx_data = &data;
  i2c_msg.tx_length = 2;
  i2c_msg.rx_length = 0;

  I2C_status = I2C_MasterTransferData(LPC_I2C1,&i2c_msg,I2C_TRANSFER_POLLING);
}

/********************************************************************//**
 * @brief               Generate a start condition on I2C bus (in master mode only)
 * @param[in]   I2Cx: I2C peripheral selected, should be:
 *                              - LPC_I2C0
 *                              - LPC_I2C1
 *                              - LPC_I2C2
 * @return              value of I2C status register after generate a start condition
 *********************************************************************/
uint32_t i2c_send_Start (LPC_I2C_TypeDef *I2Cx)
{
  // Reset STA, STO, SI
  I2Cx->I2CONCLR = I2C_I2CONCLR_SIC|I2C_I2CONCLR_STOC|I2C_I2CONCLR_STAC;

  // Enter to Master Transmitter mode
  I2Cx->I2CONSET = I2C_I2CONSET_STA;

  // Wait for complete
  while (!(I2Cx->I2CONSET & I2C_I2CONSET_SI));
  I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
  return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}

/********************************************************************//**
 * @brief               Generate a stop condition on I2C bus (in master mode only)
 * @param[in]   I2Cx: I2C peripheral selected, should be:
 *                              - LPC_I2C0
 *                              - LPC_I2C1
 *                              - LPC_I2C2
 * @return              None
 *********************************************************************/
void i2c_send_stop (LPC_I2C_TypeDef *I2Cx)
{

  /* Make sure start bit is not active */
  if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
    {
      I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
    }

  I2Cx->I2CONSET = I2C_I2CONSET_STO|I2C_I2CONSET_AA;

  I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
}

/********************************************************************//**
 * @brief               Send a byte
 * @param[in]   I2Cx: I2C peripheral selected, should be:
 *                              - LPC_I2C0
 *                              - LPC_I2C1
 *                              - LPC_I2C2
 * @param[in]   databyte: number of byte
 * @return              value of I2C status register after sending
 *********************************************************************/
uint32_t i2c_send_byte (LPC_I2C_TypeDef *I2Cx, uint8_t databyte)
{
  uint32_t CodeStatus = I2Cx->I2STAT & I2C_STAT_CODE_BITMASK;

  if((CodeStatus != I2C_I2STAT_M_TX_START) &&
      (CodeStatus != I2C_I2STAT_M_TX_RESTART) &&
      (CodeStatus != I2C_I2STAT_M_TX_SLAW_ACK)  &&
      (CodeStatus != I2C_I2STAT_M_TX_DAT_ACK)  )
    {
      return CodeStatus;
    }

  /* Make sure start bit is not active */
  if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
    {
      I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
    }
  I2Cx->I2DAT = databyte & I2C_I2DAT_BITMASK;

  I2Cx->I2CONSET = I2C_I2CONSET_AA;

  I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;

  return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}
