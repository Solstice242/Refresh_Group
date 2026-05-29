#ifndef __BASIC_H
#define __BASIC_H
#include "main.h"
#include "ili9341_driver.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "arm_math.h"
/*display*/
/*屏幕显示波形区域*/
#define Show_Left 3
#define Show_Right 252
#define Show_Top 25
#define Show_Bottom 210
#define Show_Width (Show_Right-Show_Left)
#define Show_Height (Show_Bottom-Show_Top)
#define Show_Half  (Show_Bottom+Show_Top)/2

#define GRID_X_COUNT 10  // 竖线等分数量
#define GRID_Y_COUNT 8   // 横线等分数量
/*带矩形边框的四种变量：频率、幅度、触发电平、触发方式*/
#define rectangle_Left  253
#define rectangle_Right 317
#define rectangle_Top  20
#define rectangle_Bottom 210
#define rectangle_Height 44
#define rectangle_Average (rectangle_Bottom-rectangle_Top)/4

/*ADC滤波*/
#define sth_area 40//滞回量，噪声
extern uint16_t last_x,last_y;


 extern float sample_rate;
extern float sample_interval;
extern uint16_t last_x,last_y;

extern float tri_step;
extern float tim_div;//时基细调的初始值，根据挡位
extern float TIM_modal[8];
extern char* TIM_V[3];
extern uint16_t  dac_val;
extern float V_div[3];
extern uint16_t Vertical_Sensitivity;
extern float V_Grain[8];//实际增益对应增益数组
extern float adc_grain;
extern float adc_amp_grain;//自校正幅度系数--垂直灵敏度
extern float adc_std_amp;
extern float ffp;
extern float adc_fre_grain;//自校正频率系数
extern float adc_std_fre;

extern uint16_t adc_zero;
extern float shift_vol;//触发电平真实值
extern uint8_t tim_index;//决定屏幕显示到实际改变乘的倍数，1/0.001/0.000001
extern uint8_t tim_modal;//此时对应挡位的下标
extern uint8_t TIM_Change_flag;//0为粗调模式，1为细调模式
extern volatile uint8_t  T_flag;

/*adc*/
extern float sample_interval;

#define Sample_Point 1024
#define Display_Point 200
#define ADC_LSB V_REF/adc_buf_max;//量化步长
extern uint16_t adc_buffer[Sample_Point]; // DMA缓冲
extern uint16_t adc_display[Display_Point];//一屏数组


#define TIM_ADC_FREQ 240000000UL   /* TIM1(ADC采样定时器) 时钟 = 240MHz (APB2 TimerClk) */

extern char line2[50];
extern uint16_t Mode[2];

extern uint8_t  single_triggered;  

extern uint16_t x,y;
 extern uint16_t last_x;
extern float last_V_DIV;
extern float last_V_DIV_2;
extern uint8_t first_draw;
extern uint8_t first_draw_2;
extern volatile uint8_t ADC_flag;//采集
extern volatile uint8_t DAC_flag;//输出触发电平
extern volatile uint8_t AC_flag;//初始为AC状态，0为DC
extern volatile uint8_t AC_DC_flag;
extern volatile uint8_t Correct_flag;//自校正
extern volatile uint8_t V_div_flag;//调垂直灵敏度
extern volatile uint8_t Single_flag;//单次触发
extern volatile uint8_t Freq_flag;
extern volatile uint8_t AUTO_flag;
extern volatile uint8_t tri_flag;
extern volatile uint8_t Single_Trig_flag;
extern volatile uint8_t Fre_flag;
extern float step_1[2];
extern uint8_t a;
extern float V_Grain[8];
extern float V_div[3];
extern uint8_t V_idx;
extern volatile uint8_t Grain_idx;
extern volatile uint8_t Grain_flag;     /* 增益切换标志 */
extern volatile uint8_t FFT_flag;       /* FFT触发标志 */
extern volatile uint8_t system_busy;
extern volatile uint8_t auto_mode;




void V_div_change(uint8_t V_idx);
void AC_DC_Show(void);
void ADC_Start(void);
void TRI_FLAG(void);
 #define FS 500000.0f
 #define Freq_Resolution (FS/Sample_Point)//频率分辨率
 #define Bar_Width 2//柱形宽度
 #define Bar_gap 4//间隔
extern uint16_t  V_Standard;

extern   float32_t FFT_in[Sample_Point];
extern  float32_t hanning_win[Sample_Point];
extern  float32_t fft_mag[Sample_Point/2];
 extern uint32_t psc;   
  extern char Display[4][10] ;
 extern  uint32_t arr;
extern  float32_t base_freq;
void Correct_Process(void);
void FFT_Init(void);
float32_t Find_Base_Freq(void);
void FFT_Analysis(uint16_t *adc_raw,uint16_t len);
void FFT_Display(uint16_t base_idx);

void set_T_div(float t_div);
float Shift_T_div(uint8_t Tim_idx,float tim_display);

void Match_Tim(void);


extern char buf[100];

#define TIME_BASE_MAX 7  // 最大挡位
/*函数声明*/
void Draw_Line(void);
void Show_Data(void);
void Show_AUTO_Mode(void);
void  Screen_DrawWave_color(uint16_t *src, float v_div, float ADC_grain, uint16_t color);
void Zero_Correct(void);
void Correct_Process();
 //  void   Phase_Measure();
void ADC_Project(void);

void ADC_Filter(uint16_t *src, uint16_t *dst, uint16_t len);
void ADC_Filter_EMA(uint16_t *src, uint16_t *dst, uint16_t len, float alpha);
void ADC_Filter_Adaptive(uint16_t *src, uint16_t *dst, uint16_t len);

void TRI_Scan(void);
void TIM_Scan(void);
void AUTO_Scan(void);

void Freq_Capture_Init(void);
void Freq_Capture_Stop(void);
float Freq_Capture_Get(void);

void ADC_Measure_amp(uint16_t *src);


void auto_tri_level(uint16_t* buffer);
void auto_V_div(float FFP);
void auto_T_div(float freq);
void auto_gain(float adc_vpp);   /* 自动硬件增益适配: 根据ADC端Vpp自动选最优增益挡位 */

void Chose_Grain(uint8_t idx);


#endif

