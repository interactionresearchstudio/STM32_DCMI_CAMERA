/*
 * SCCB.h
 *
 *  Created on: 2 Apr 2015
 *      Author: z3168771
 */

#ifndef SCCB_H_
#define SCCB_H_

msg_t SCCB_Write(const uint8_t addr, const uint8_t reg, const uint8_t value);
msg_t SCCB_Read(const uint8_t addr, const uint8_t reg, uint8_t *value);

#endif /* SCCB_H_ */
