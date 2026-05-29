#ifndef __LCD_H
#define __LCD_H		

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define STM32


//CS - GND,LED VCC 3.3
#define LCD_X_SIZE	        240
#define LCD_Y_SIZE	        320

#define USE_HORIZONTAL  		1	//定义是否使用横屏 		0,不使用.1,使用.


#ifdef USE_HORIZONTAL//如果定义了横屏 
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif



#ifdef STM32
#define ILI9341_RS_H   HAL_GPIO_WritePin(TFT_T_RS_GPIO_Port, TFT_T_RS_Pin, GPIO_PIN_SET)
#define ILI9341_RS_L   HAL_GPIO_WritePin(TFT_T_RS_GPIO_Port, TFT_T_RS_Pin, GPIO_PIN_RESET)

#define ILI9341_RST_H  HAL_GPIO_WritePin(TFT_T_RST_GPIO_Port, TFT_T_RST_Pin, GPIO_PIN_SET)
#define ILI9341_RST_L  HAL_GPIO_WritePin(TFT_T_RST_GPIO_Port, TFT_T_RST_Pin, GPIO_PIN_RESET)

#define ILI9341_SDI_H  HAL_GPIO_WritePin(TFT_T_SDI_GPIO_Port, TFT_T_SDI_Pin, GPIO_PIN_SET)
#define ILI9341_SDI_L  HAL_GPIO_WritePin(TFT_T_SDI_GPIO_Port, TFT_T_SDI_Pin, GPIO_PIN_RESET)

#define ILI9341_SCK_H  HAL_GPIO_WritePin(TFT_T_SCK_GPIO_Port, TFT_T_SCK_Pin, GPIO_PIN_SET)
#define ILI9341_SCK_L  HAL_GPIO_WritePin(TFT_T_SCK_GPIO_Port, TFT_T_SCK_Pin, GPIO_PIN_RESET)

#define ILI9341_CS_H   HAL_GPIO_WritePin(TFT_T_CS_GPIO_Port, TFT_T_CS_Pin, GPIO_PIN_SET)
#define ILI9341_CS_L   HAL_GPIO_WritePin(TFT_T_CS_GPIO_Port, TFT_T_CS_Pin, GPIO_PIN_RESET)

#define ILI9341_LED_H  HAL_GPIO_WritePin(TFT_T_LED_GPIO_Port, TFT_T_LED_Pin, GPIO_PIN_SET)
#define ILI9341_LED_L  HAL_GPIO_WritePin(TFT_T_LED_GPIO_Port, TFT_T_LED_Pin, GPIO_PIN_RESET)

#endif
#ifdef MSPM0

#endif


#define uint_8   unsigned char
#define uint_16  unsigned short
#define uint_32  unsigned int
#define int_8    signed char
#define int_16   signed short
#define int_32   signed int
	
extern  uint_16 BACK_COLOR, POINT_COLOR;   //背景色，画笔色

void ILI9341_init(uint16_t Color);
void ILI9341_clear(uint_16 Color);
void ILI9341_address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2);
void ILI9341_fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void ILI9341_wr_data8(char da);
void ILI9341_wr_data(int da);
void ILI9341_wr_reg(char da);
void ILI9341_draw_point_big(uint16_t x,uint16_t y,uint16_t COLOR);
void ILI9341_draw_char(uint16_t x,uint16_t y,char num,uint8_t mode,uint16_t COLOR);
void ILI9341_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t COLOR);
void ILI9341_draw_string(uint16_t x,uint16_t y,const char *p,uint16_t COLOR);
void ILI9341_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t COLOR);	//画矩形	   
                                        

uint32_t mypow(uint8_t m,uint8_t n);
///以下几个函数是自己写的

/*************************************************************************/

//画笔颜色

#define RGB888_To_RGB565(R,G,B)  (uint16_t)((R & 0x1f)<<11|(G & 0x3f)<<5|(B & 0x1f))  //自选RGB888转RGB565

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F//粉紫
#define GRED 			 0XFFE0 //黄色
#define GBLUE			 0X07FF //浅浅蓝
#define RED           	 0xF800
//#define MAGENTA       	 0xF81F
#define CYAN          	 0x7FFF//浅浅浅蓝
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
#define ORANGE           0XFD20 //橙色
#define PURPLE           0X8010 
#define GREEN            0x0FE0
#define PURE_FLUO_PURPLE    0xF81F 
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//深浅浅蓝
#define GRAYBLUE       	 0X5458 //浅蓝
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅紫
#define LGRAY 			 0XC618 //近白(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //青色
#define LBBLUE           0X2B12 //半浅蓝(选择条目的反色)

extern unsigned char test[];
extern const unsigned char asc2_1608[1520];
					  		 
#endif  
	 
	 



