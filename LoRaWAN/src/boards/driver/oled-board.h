/*
    2018 Grope

Description: OLED driver implementation,used for display

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Derek Chen
*/

#ifndef __OLED_BOARD_H
#define __OLED_BOARD_H	

#include "stdlib.h"	    
#include "stm32l0xx_hal.h"	    			
		      



/**********Function declaration*************/ 
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Clear_Half(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowString_XY(uint8_t x,uint8_t y,const uint8_t *p);	 
void OLED_Clear_Line(uint8_t lineNumber);
void OLED_Fill_All(void);
void OLED_ShowString_Line(uint8_t LineNum, const uint8_t * p);
void OLED_Clean_All_Slowly(void);


#endif  //__OLED_BOARD_H
	 







 

