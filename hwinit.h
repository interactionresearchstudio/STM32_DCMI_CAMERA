#ifndef HWINIT_H_
#define HWINIT_H_

#include "main.h" // For callback functions

static const DCMIConfig dcmicfg = {
   frameEndCb,
   dmaTxferEndCb,
   //0b00000000000000000000000011101010
   //DCMI_CR_CM | DCMI_CR_JPEG | DCMI_CR_PCKPOL | DCMI_CR_HSPOL | DCMI_CR_VSPOL           // CR settings
   DCMI_CR_JPEG | DCMI_CR_PCKPOL
};

void hwInit(void);

#endif /* HWINIT_H_ */
