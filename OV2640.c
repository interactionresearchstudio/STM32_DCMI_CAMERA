#include "ch.h"
#include "hal.h"
#include "SCCB.h"
#include "OV2640.h"

uint8_t cam_write_reg(uint8_t reg, uint8_t value) {
  if (SCCB_Write(0x60 >> 1, reg, value) == 0) {
    return 0;
  } else {
    return 1;
  }
}

uint8_t cam_read_reg(uint8_t reg, uint8_t *value) {
  if (SCCB_Read(0x60 >> 1, reg, value) == 0) {
    return 0;
  } else {
    return 1;
  }
}

uint8_t cam_write_array(const struct regval_list *vals) {
  while ((vals->reg_num != 0xff) || (vals->value != 0xff)) {
        if (SCCB_Write(0x60 >> 1, vals->reg_num, vals->value) != 0) {
            return 1;
        }
        vals++;
    }
    return 0;
}

