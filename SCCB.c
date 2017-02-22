#include "ch.h"
#include "hal.h"

msg_t SCCB_Write(const uint8_t addr, const uint8_t reg, const uint8_t value) {
   msg_t status;
   uint8_t txbuf[2] = {reg, value};
   uint8_t rxbuf = 0;

   i2cAcquireBus(&I2CD1);
   status = i2cMasterTransmitTimeout(&I2CD1, addr, txbuf, 2, &rxbuf, 0, MS2ST(5));
   i2cReleaseBus(&I2CD1);

   return status;
}

msg_t SCCB_Read(const uint8_t addr, const uint8_t reg, uint8_t *value) {
  msg_t status;
  uint8_t rxbuf[1];
  uint8_t txbuf[1] = {reg};

  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, addr, txbuf, 1, rxbuf, 1, MS2ST(5));
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    return status;
  } else {
    *value = rxbuf[0];
    return status;
  }
}
