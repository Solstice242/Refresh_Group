#include "Basic.h"
#include "main.h"
#include "tim.h"
#include "dac.h"
#include "adc.h"
#include "ili9341_driver.h"
#include <stdlib.h>
#include "usart.h"
#include <string.h>
#include "tim.h"
#include <math.h>
#include <inttypes.h>
/*波形处理与屏幕显示（主功能）：
1.取点
2.滤波
3.画波形
3.屏幕显示(初始化+波形对应逻辑)*/
 char buf[100];
 volatile uint8_t ADC_flag=0;//采集完成标志
//屏幕显示
 char Display[4][10] = {
    "Freq:",
    "Amp:",
    "Level:",
    "Prase:"
};
uint16_t Mode[2] = {
   LGRAYBLUE ,
   BLUE   ,
};
//ADC数组
uint16_t adc_buffer[Sample_Point]; 
uint16_t adc_display[Display_Point];
uint16_t adc_Filter[Display_Point];
uint16_t last_wave[Display_Point];
float last_ADC_grain;
float last_V_DIV;
uint8_t first_draw = 1; 
uint16_t last_x,last_y=0;

/* ── ADC DMA完成回调 ── */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1) {
        ADC_flag = 1;
        HAL_TIM_Base_Stop(&htim1);
        HAL_ADC_Stop_DMA(&hadc1);
    }
}
void DMA_Finish(void)
 {
    if(ADC_flag == 1) {
        ADC_flag = 0;
        ADC_Project();
    }
}
/*-----------------------
         取前200个点
-------------------------*/
 void Extract_200(uint16_t* dma,uint16_t* dst)
{
 memcpy(dst,&dma[0],Display_Point*sizeof(uint16_t));
}

/*-----------------
滤波函数：
1.中值滤波-去毛刺：
inline关键词--编译器会尝试将函数体直接插入到每个调用点，避免函数调用的开销。适用于小函数。
2.EMA低通滤波-降噪声干扰
-------------------*/
//三点滤波
static inline uint16_t mid_3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (a > c) { uint16_t t = a; a = c; c = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    return b;
}

void ADC_Filter(uint16_t *src, uint16_t *dst, uint16_t len) {
    dst[0] = src[0];
    for (uint16_t i = 1; i < len - 1; i++) {
        dst[i] = mid_3(src[i-1], src[i], src[i+1]);
    }
    dst[len-1] = src[len-1];
}

/*EMA低通滤波:alpha-记忆权重
 关键公式：y[i] = alpha*y[i-1]（上次值） + (1-alpha)*x[i]（当前值）
*/
void ADC_Filter_EMA(uint16_t *src, uint16_t *dst, uint16_t len, float alpha)
{
    if(alpha < 0.0f) alpha = 0.0f;
    if(alpha > 0.95f) alpha = 0.95f;
    dst[0] = src[0];
    float acc = (float)src[0];
    for(uint16_t i = 1; i < len; i++) {
        acc = alpha * acc + (1.0f - alpha) * (float)src[i];
        dst[i] = (uint16_t)(acc + 0.5f);//加0.5-浮点数转整数，需要四舍五入而不是截断
    }
}
/*------------------
画波形：
输入：原始数据数组、垂直灵敏度、增益、颜色（先擦后画）
计算：y：满屏电压（垂直灵敏度）->每伏像素-->每个点的电压（增益）-->对应像素坐标
--------------------*/
void Screen_DrawWave_color(uint16_t *src,float v_div,float ADC_grain,uint16_t color)
{
    float full_screen_vol =  v_div * 8.0f;//满屏电压
    float pixels_per_vol = (float)Show_Height / full_screen_vol;//每伏电压像素
    for (int i = 0; i < Display_Point; i++)
    {
        int32_t val = src[i] - adc_zero;//去DC偏置
        float voltage;
        voltage = val * ADC_LSB;
        int16_t y = Show_Half - (int16_t)(voltage * pixels_per_vol/ADC_grain);
        int16_t x = Show_Left + (i * Show_Width) / Display_Point;

        if (x > Show_Right) x = Show_Right;
        if (x < Show_Left) x = Show_Left;
        if (y < Show_Top) y = Show_Top;
        if (y > Show_Bottom) y = Show_Bottom;

        if (i > 0)
        {
            ILI9341_draw_line(last_x, last_y, x, y, color);
        }
        last_x = x;
        last_y = y;
    }
}
/*---------------------
屏幕初始化：网格加参数显示
-----------------------*/
void Show_Data(void)
{
   int y=0;//矩形边
   for(int i=0;i<4;i++)
   {
      y=rectangle_Top+rectangle_Average*i;
    ILI9341_draw_rectangle(rectangle_Left,y,rectangle_Right, y+rectangle_Height,BRRED );
       ILI9341_draw_string(rectangle_Left+2,y+2,Display[i],GRED );
   }

  ILI9341_draw_string(3,2,"V/div:",GRED );
  sprintf(line2, " %.2fV", V_DIV);
   ILI9341_draw_string(50,2,line2,GRED );

     ILI9341_draw_string(130,2,"T/div:",GRED );
  sprintf(line2, " %.1fus", T_DIV);
   ILI9341_draw_string(178,2,line2,GRED );

  sprintf(line2, " %.1fkHz", FREQ);
   ILI9341_draw_string(rectangle_Left+2,44,line2,GRED );

  sprintf(line2, " %.1fV", AMP);
   ILI9341_draw_string(rectangle_Left+2,91,line2,GRED);

     sprintf(line2, " %.2fV", LEVEL);
   ILI9341_draw_string(rectangle_Left+2,140,line2,GRED );

  sprintf(line2, " %.1f", PRASE);
   ILI9341_draw_string(rectangle_Left+2,185,line2,GRED );

   ILI9341_draw_string(270,2,"AC",Mode[0] );
      ILI9341_draw_string(286,2,"/",GRED );
            ILI9341_draw_string(294,2,"DC",Mode[1] );
  
    ILI9341_draw_string(180,220,"Grain:",GRED );
    sprintf(line2, " %.4f", adc_grain);
   ILI9341_draw_string(230,220,line2,GRED );
}
void Draw_Line(void)
{
  for(float i=Show_Top;i<=Show_Bottom;i+=(Show_Height/8))
  {
     ILI9341_draw_line(Show_Left,i,Show_Right, i,LGRAY);
  }
    for(float i=Show_Left;i<=Show_Right;i+=(Show_Width/10))
  {
     ILI9341_draw_line(i,Show_Top-1,i , Show_Bottom+1,LGRAY );
  }
}

/*ADC采集到显示*/
void ADC_Project(void)
{
    Extract_200(adc_buffer, adc_display);
     ADC_Filter(adc_display, adc_Filter, Display_Point);
    ADC_Filter_Adaptive(adc_display, adc_Filter, Display_Point);
    ADC_Measure_amp(adc_Filter);
    if(!first_draw)
    {
        Screen_DrawWave_color(last_wave, last_V_DIV, last_ADC_grain, BLACK);
    }
    Screen_DrawWave_color(adc_Filter, V_DIV, adc_grain, DARKBLUE);
    memcpy(last_wave, adc_Filter, Display_Point * sizeof(uint16_t));
    last_V_DIV = V_DIV;
    last_ADC_grain = adc_grain;
    first_draw = 0;

    sprintf(line2, " %.1fV", AMP);
    ILI9341_draw_string(rectangle_Left+2, 91, line2, BLACK);
    ILI9341_draw_string(rectangle_Left+2, 91, line2, GRED);

    if(Single_flag==0) {
        Single_Trig_flag = 0;
        EXTI_D1->IMR1 |= (1UL << 4);
    }
}




