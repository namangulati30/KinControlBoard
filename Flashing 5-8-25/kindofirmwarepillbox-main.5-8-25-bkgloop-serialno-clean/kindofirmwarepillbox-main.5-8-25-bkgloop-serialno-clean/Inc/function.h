#ifndef _FUNCTION_H
#define _FUNCTION_H
#include "main.h"

extern uint8_t temp_x, temp_z, temp_y, temp_pill, Door, rot;
extern const uint16_t  ROT_STP;
extern uint32_t time_z, time_init,stepstart;
void stperInit(void);
void moveARM_X(uint16_t stp, uint8_t dir);
void moveARM_Xptp(uint8_t dir);
void moveARM_Z(uint8_t dir);
void moveARM_Z_pulse(uint16_t stp, uint8_t dir);
void rotateProt(uint32_t i, uint8_t dir);
void moveROT_Y(uint16_t i, uint8_t dir);
void chuteDoor (uint8_t dir);
void test1(void);
void test2(void);
void returnState(void);
uint8_t check_pillbox(void);
uint8_t active_pill_drop(void);
void Check_STPX_DropP(void);
void Check_STPX_PickP(void);

void frontdoor(uint8_t state);




extern uint8_t rotPcnt, rotYcnt,rottol_Prot, rottol_y;


void readLimSwitch(void);


void readNFCSensor (void);
#endif
