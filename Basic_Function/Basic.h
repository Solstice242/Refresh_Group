#ifndef __BASIC_H
#define __BASIC_H
#include "main.h"
#include "ili9341_driver.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "arm_math.h"

/*==============================================================================
 * 屏幕显示布局
 *============================================================================*/
#define Show_Left      3
#define Show_Right     252
#define Show_Top       25
#define Show_Bottom    210
#define Show_Width     (Show_Right - Show_Left)
#define Show_Height    (Show_Bottom - Show_Top)
#define Show_Half      ((Show_Bottom + Show_Top) / 2)
#define GRID_X_COUNT   10
#define GRID_Y_COUNT   8
#define Bar_Width      2
#define Bar_gap        4

#define rectangle_Left    253
#define rectangle_Right   317
#define rectangle_Top     20
#define rectangle_Bottom  210
#define rectangle_Height  44
#define rectangle_Average ((rectangle_Bottom - rectangle_Top) / 4)

/*==============================================================================
 * ADC / 采样 常量
 *============================================================================*/
#define Sample_Point    1024
#define Display_Point   200
#define ADC_LSB         (V_REF / adc_buf_max)
#define TIM_ADC_FREQ    240000000UL
#define sth_area        40

/*==============================================================================
 * 菜单 常量
 *============================================================================*/
#define ITEM_NUM        8

/*==============================================================================
 * 全局变量 — 采样 & 波形
 *============================================================================*/
extern uint16_t adc_buffer[Sample_Point];
extern uint16_t adc_display[Display_Point];
extern uint16_t adc_Filter[Display_Point];
extern uint16_t last_wave[Display_Point];
extern float    last_ADC_grain;
extern float    last_V_DIV;
extern uint8_t  first_draw;
extern uint16_t last_x, last_y;

extern float    sample_interval;
extern float    sample_rate;
extern uint32_t psc;
extern uint32_t arr;

/*==============================================================================
 * 全局变量 — 幅值 & 增益
 *============================================================================*/
extern float    ffp;
extern float    adc_amp_grain;
extern float    adc_std_amp;
extern float    adc_fre_grain;
extern float    adc_std_fre;
extern uint16_t adc_zero;

extern float    V_Grain[8];
extern uint8_t  Grain_idx;
extern float    adc_grain;

/*==============================================================================
 * 全局变量 — 触发电平
 *============================================================================*/
extern float    tri_step;
extern float    step_1[2];
extern uint8_t  step_dix;
extern uint16_t last_count_1;  /* TRI_Scan / Menu_EncoderB Trig 编码器计数 */
extern uint16_t dac_val;
extern float    LEVEL;

/*==============================================================================
 * 全局变量 — 时基
 *============================================================================*/
extern float    T_DIV;
extern float    TIM_modal[8];
extern char*    TIM_V[3];
extern uint8_t  tim_index;
extern uint8_t  tim_modal;
extern uint8_t  TIM_Change_flag;
extern float    tim_div;

/*==============================================================================
 * 全局变量 — 垂直灵敏度
 *============================================================================*/
extern float    V_div[3];
extern uint8_t  V_idx;
extern float    V_DIV;

/*==============================================================================
 * 全局变量 — 交直流耦合
 *============================================================================*/
extern volatile uint8_t AC_flag;
extern volatile uint8_t AC_DC_flag;
extern uint8_t  AC_idx;

/*==============================================================================
 * 全局变量 — 显示测量值
 *============================================================================*/
extern float    FREQ;
extern float    AMP;
extern float    PRASE;
extern char     line2[50];
extern char     buf[100];
extern uint16_t Mode[2];
extern char     Display[4][10];

/*==============================================================================
 * 全局变量 — 标志位
 *============================================================================*/
extern volatile uint8_t ADC_flag;
extern volatile uint8_t Single_flag;
extern volatile uint8_t Single_Trig_flag;


/*==============================================================================
 * 全局变量 — FFT
 *============================================================================*/
extern float32_t hanning_win[Sample_Point];
extern float32_t FFT_in[Sample_Point];
extern float32_t fft_mag[Sample_Point / 2];
extern arm_rfft_fast_instance_f32 rfft_inst;
extern float32_t base_freq;
extern float    fft_sample_interval;
extern volatile uint8_t FFT_Refresh_flag;  /* FFT显示锁定: 1=保持频谱, 0=恢复波形 */

/*==============================================================================
 * 全局变量 — 编码器 & 菜单 (仅声明, 定义在 Display.c)
 *============================================================================*/
extern uint8_t  menu_idx;
extern uint8_t  now_menu_idx;
extern uint8_t  menu_active;
extern uint8_t  encB_need_sync;    /* 退出模式时置1, 编码器B下次进入重新同步 */
extern uint8_t  modal_active;
extern uint8_t  trigger_correct;       /* 主循环触发自校正 */
extern uint8_t  trigger_fft;           /* 主循环触发FFT */
extern float    captured_sample_interval; /* ADC采集时的采样间隔, FFT用 */

/*==============================================================================
 * 函数声明 — 测频 (GPIO_flag.c)
 *============================================================================*/
void  Freq_Capture_Init(void);
void  Freq_Capture_Stop(void);
float Freq_Capture_Get(void);

/*==============================================================================
 * 函数声明 — FFT (GPIO_flag.c)
 *============================================================================*/
void      FFT_Init(void);
void      FFT_Analysis(uint16_t *adc_raw, uint16_t len);
float32_t Find_Base_Freq(void);
void      FFT_Display(uint16_t base_idx);
void      FFT_Process(void);

/*==============================================================================
 * 函数声明 — 自校正 (GPIO_flag.c)
 *============================================================================*/
void Zero_Correct(void);
void Correct_Process(void);

/*==============================================================================
 * 函数声明 — ADC / 幅值 / 增益 (B_adc.c)
 *============================================================================*/
void  Chose_Grain(uint8_t idx);
void  ADC_Measure_amp(uint16_t *src);
void  ADC_Measure_amp_rt(void);       /* 实时幅值: 1024原始点测峰谷 */
void  auto_gain(float adc_vpp);
void  tri_step_change(void);
void  TRI_Scan(void);
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/*==============================================================================
 * 函数声明 — 滤波 & 波形显示 (B_dac.c)
 *============================================================================*/
void  DMA_Finish(void);
void  Extract_200(uint16_t* dma, uint16_t* dst);
void  ADC_Filter(uint16_t *src, uint16_t *dst, uint16_t len);
void  ADC_Filter_EMA(uint16_t *src, uint16_t *dst, uint16_t len, float alpha);
void  Screen_DrawWave_color(uint16_t *src, float v_div, float ADC_grain, uint16_t color);
void  ADC_Project(void);
void  Show_Data(void);
void  Draw_Line(void);

/*==============================================================================
 * 函数声明 — 显示 & 菜单 (Display.c)
 *============================================================================*/
void  Menu_EncoderA_Scan(void);
void  Menu_EncoderB(void);
void  AC_Draw(void);
void  AC_change(void);
void  AC_output(void);
void  V_Draw(void);
void  V_div_change(void);
void  TIM_Scan(void);
void  EncoderB_Press(void);

/*==============================================================================
 * 函数声明 — 时基 (Display.c)
 *============================================================================*/
void  Match_Tim(void);
float Shift_T_div(uint8_t Tim_idx, float tim_display);
void  set_T_div(float t_div);



#endif
