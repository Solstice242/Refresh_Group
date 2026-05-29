#include "Basic.h"
#include "ili9341_driver.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "arm_math.h"
/*------附加功能（菜单控制------
1.菜单跳转逻辑
2.单次触发（波形存储）
3.测量参数（频率）
4.参数调整（扫描速度、垂直灵敏度）
5.FFT频谱分析
6.交直流耦合
7.自校正
------------------------------*/

 float FREQ=0.0f;
 float  AMP = 0.0f;
 float PRASE=30.0f;
 char line2[50];

//菜单
#define ITEM_NUM 8                            
typedef struct {
    const char *name;                            /* 屏幕显示名 */
    void (*on_enter)(void);                      /* OUT_1按下: 进入模式 */
} Menu_Item;
static const Menu_Item menu[ITEM_NUM] = {
    {"Single",    NULL},
    {"V/DIV",   NULL},                           
    {"T/DIV",    NULL},                           
    {"Trig",    NULL},                       
    {"Measure",    NULL},    
     {"FFT",     NULL},                          
    {"AC/DC",   NULL},                                         
    {"Correct", NULL},                         
};
 uint8_t  menu_idx = 0;               //显示
 uint8_t  now_menu_idx = 0;           //当前模式
uint8_t  menu_active = 0;                 // 0=浏览, 1=进入 (extern声明在Basic.h)
uint8_t  modal_active = 0;                // 编码器B按下 (extern声明在Basic.h)
static int16_t  last_encA = 0;            // TIM3编码器上次值 (仅本文件)
static int16_t  last_encB = 0;            // TIM4编码器上次值 (仅本文件)

/* ── 系统标志位 ── */
volatile uint8_t Single_Trig_flag=0;                 /* 单次触发完成 */
volatile uint8_t Single_flag=0;                      /* 单次触发模式 */
volatile uint8_t FFT_flag=0;                         /* FFT触发 */
volatile uint8_t Correct_flag=0;                     /* 自校正触发 */
volatile uint8_t Grain_flag=0;                       /* 增益切换 */
volatile uint8_t V_div_flag=0;                       /* V/div切换 */
volatile uint8_t T_flag=0;                           /* 时基切换 */
volatile uint8_t AC_flag=0;                          /* 0=DC 1=AC */
volatile uint8_t AC_DC_flag=0;                      /* AC/DC有变更 */
volatile uint8_t AUTO_T_flag=0;                     /* AUTO触发 */
volatile uint8_t system_busy=0;                     /* 系统忙 */

/* ── 垂直灵敏度 ── */
float V_div[3]={1.0,0.1,0.01};//屏幕显示
uint8_t V_idx=1;
 float V_DIV=1.0f;

//时基模式
 float  T_DIV = 20.0f;
float TIM_modal[8]={20,50,100,
                  0.2,1,10,100,
                  0.2};
char* TIM_V[3]={"us","ms","s"};
uint8_t tim_index = 0;
uint8_t tim_modal=0;
uint8_t TIM_Change_flag=0;
float tim_div=20.0f;
float sample_interval= 0.00001f;
float sample_rate;
 uint32_t psc=0;   
    uint32_t arr =0;

//触发电平模式
float LEVEL=0.81f;

//频率测量模式

//交直流耦合模式
uint8_t AC_idx=0;

//FFT频谱分析模式、自校正模式
/*详见GPIO_flag.c*/

/* -----------------------------------------
              模式显示
--------------------------------------------*/
static void Menu_Draw(void)
{
    int x = 5, y = 220;                          
    ILI9341_fill(x, y, x + 200, y + 18, BLACK);  
    uint16_t c = menu_active ? GREEN : GRED;     // 进入=绿, 浏览=红 
    ILI9341_draw_string(x, y, menu[menu_idx].name, c);
}

/* ---------------------------------------
旋转编码器A_TIM3旋转-> 改变菜单索引 
------------------------------------------ */
void Menu_EncoderA_Scan(void)
{
    if(menu_active) return;                      // 进入模式后锁定索引 
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = now - last_encA;
    if(diff >= 4) {
        menu_idx = (menu_idx + 1) % ITEM_NUM;
        last_encA += 4;
        Menu_Draw();
    } else if(diff <= -4) {
        menu_idx = (menu_idx + ITEM_NUM - 1) % ITEM_NUM;
        last_encA -= 4;
        Menu_Draw();
    }
}

/* ----------------------------------
旋转编码器A_OUT_1按下-> 进入/退出当前模式 
详见中断函数
-------------------------------------*/

/* -------------------------------------
旋转编码器B_TIM4旋转 ->根据菜单索引调整参数 
---------------------------------------*/
void Menu_EncoderB(void)
{
switch(menu_idx) {
        case 0: //Single
             break;
            break;
        case 1: V_div_change();
            break;
        case 2: TIM_Scan();
            break;
        case 3: TRI_Scan();
            break;
        case 4: Freq_Capture_Get();
            break;
        case 5: FFT_Process();
            break;
        case 6:  AC_change();
            break;
        case 7: Correct_Process();
            break;
            defalut: break;
    }
}

//交直流耦合模式
void AC_Draw(void)
{
    switch(AC_idx)
    {
        case 0:
        {
ILI9341_draw_string(294,2,"DC",BLACK );
 ILI9341_draw_string(294,2,"DC",Mode[1] );
            break;
        }
        case 1:
        {
  ILI9341_draw_string(294,2,"AC",BLACK );
 ILI9341_draw_string(294,2,"AC",Mode[1] );
            break;
        }
    }
}
void AC_change(void)
{
 int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t diff = now - last_encB;
    if(diff >= 4) {
        AC_idx = (AC_idx + 1) % 2;
         last_encB += 4;
        AC_Draw();
    } else if(diff <= -4) {             
        AC_idx = (AC_idx + 1) % 2;
         last_encB -= 4;
        AC_Draw();
   
    }
}

//垂直灵敏度模式
void V_Draw(void)
{
    V_DIV = V_div[V_idx];
    sprintf(line2, " %.2fV", V_DIV);
    ILI9341_draw_string(50, 2, line2, BLACK);
    ILI9341_draw_string(50, 2, line2, GRED);
}

void V_div_change(void)
{
 int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t diff = now - last_encB;
    if(diff >= 4) {
        V_idx = (V_idx + 1) % 3;
        last_encB += 4;
        V_Draw();
    } else if(diff <= -4) {
        V_idx = (V_idx + 2) % 3;
        last_encB -= 4;
        V_Draw();
    }
}

//时基模式
void TIM_Scan(void)
{
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t diff = now - last_encB;
    float step;
    if(diff != 0)
    {
        switch(TIM_Change_flag)
        {
                 case 0:{
    if(diff >= 4)        
    {
        tim_modal++;
        last_encB += 4;   
    }
    else if(diff <= -4)   
    {
        tim_modal--;
        last_encB -= 4;  
    }
    if(tim_modal == 255)  {tim_modal = 7;}
    if(tim_modal >= 8)  tim_modal = 0;
    switch(tim_modal)
    {
        case 0: case 1: case 2:      tim_index = 0; break;
        case 3: case 4: case 5: case 6: tim_index = 1; break;
        case 7:                     tim_index = 2; break;
    }
      tim_div = TIM_modal[tim_modal];
                   ILI9341_fill(175,2, 204,18, BLACK);
    sprintf(line2, " %.1f", tim_div);
    ILI9341_draw_string(175,2, line2, GRED);
    ILI9341_draw_string(220,2, TIM_V[tim_index], GRED);
    float a= Shift_T_div(tim_index,tim_div);
                set_T_div(a);
                    
    break;
}
            case 1:
            {
                if(diff >=4 || diff <=-4)
                {
                    if(tim_div < 1.0f) step = 0.1f;
                    else step = 1.0f;

                    if(diff >=4) tim_div += step;
                    else tim_div -= step;

                    switch(tim_index)
                    {
                        case 0:
                            if(tim_div > 100.0f)
                            { tim_index = 1; tim_div = 0.1f; }
                            if(tim_div < 20.0f)
                            { tim_div = 20.0f; }
                            break;

                        case 1:
                            if(tim_div > 100.0f)
                            { tim_index = 2; tim_div = 0.1f; }
                            if(tim_div < 0.1f)
                            { tim_index = 0; tim_div = 100.0f; }
                            break;

                        case 2:
                            if(tim_div > 0.2f)
                            { tim_div = 0.2f; }
                            if(tim_div < 0.1f)
                            { tim_index = 1; tim_div = 100.0f; }
                            break;
                    }
                if(diff >=4) last_encB +=4;
                else if(diff <=-4) last_encB -=4;
                     ILI9341_fill(175,2, 204,18, BLACK);
    sprintf(line2, " %.1f", tim_div);
    ILI9341_draw_string(175,2, line2, GRED);
    ILI9341_draw_string(220,2, TIM_V[tim_index], GRED);
                     float a= Shift_T_div(tim_index,tim_div);
                set_T_div(a);
                }

                break;
            }
            default:
                break;
        }

    }
}

/* -------------------------------------
旋转编码器B_OUT_2按下 ->根据菜单索引进入
---------------------------------------*/
void EncoderB_Press(void)
{
    if(!modal_active) return; // 仅在旋钮按下时响应
switch(now_menu_idx) {
        case 0: //Single
       { Single_flag=!Single_flag;
  if(Single_flag==0) {
    Single_Trig_flag=0;
    EXTI_D1->IMR1 |= (1UL << 4);     // EXTI4使能
  }
            break;
        }
        case 1: //V_div
            break;
        case 2: 
    {
        TIM_Change_flag = !TIM_Change_flag;
        if(TIM_Change_flag == 0) {
            Match_Tim();
        } else {
            tim_div = TIM_modal[tim_modal];
        }
    }
            break;
        case 3: tri_step_change();
            break;
        case 4: // Measure 
            break;
        case 5: FFT_Process();
            break;
        case 6:  AC_output();
            break;
        case 7: // Correct
            break;
    }
}

//交直流耦合模式
void AC_output(void)
{
    switch(AC_idx)
    {
        case 0: 
        HAL_GPIO_WritePin(AC_DC_GPIO_Port, AC_DC_Pin, GPIO_PIN_SET);
            break;
        case 1: 
        HAL_GPIO_WritePin(AC_DC_GPIO_Port, AC_DC_Pin, GPIO_PIN_RESET);
            break;
    }
}

void Match_Tim(void)
{
    const float tim_base[8] = {
        20,50,100,200,1000,10000,100000,200000                    
    };
    float current_base;
        switch(tim_index)
    {
        case 0: current_base = tim_div; break;          
        case 1: current_base = tim_div * 1000; break;   
        case 2: current_base = tim_div * 1000000; break;
        default: current_base = 20; break;
    }
    uint8_t best_idx = 0;
    float min_diff = fabs(current_base - tim_base[0]);
    for(uint8_t i=1; i<8; i++)
    {
        float diff = fabs(current_base - tim_base[i]);
        if(diff < min_diff)
        {
            min_diff = diff;
            best_idx = i;
        }
    }
    tim_modal = best_idx;
    tim_div = TIM_modal[best_idx];
    if(tim_modal <= 2)        tim_index = 0;
    else if(tim_modal <= 6)   tim_index = 1;
    else                      tim_index = 2;
     ILI9341_fill(175,2, 204,18, BLACK);
    sprintf(line2, " %.1f", tim_div);
    ILI9341_draw_string(175,2, line2, GRED);
    ILI9341_draw_string(220,2, TIM_V[tim_index], GRED);
      float a= Shift_T_div(tim_index,tim_div);
                set_T_div(a);
}

//时基模式改变-粗调与细调
float Shift_T_div(uint8_t Tim_idx,float tim_display)
{
     switch(tim_index)
    {
        case 0:  // us/div
            T_DIV = tim_display / 1000000.0f;
            break;
        case 1:  // ms/div
            T_DIV = tim_display / 1000.0f;
            break;
        case 2:  // s/div
            T_DIV = tim_display;
            break;
        default:
            T_DIV = 0.2f / 1000.0f;
    }
    return T_DIV;
}

void set_T_div(float t_div)
{
    float total_scan_time = t_div * 10.0f;
     sample_rate = (uint32_t)(Display_Point / total_scan_time);
     arr = (TIM_ADC_FREQ /  sample_rate) - 1;
    while(arr > adc_buf_max && psc < 239)
    {
        psc++;
        arr = (TIM_ADC_FREQ / ((psc + 1) * sample_rate)) - 1;
    }

    if(arr > adc_buf_max) arr = adc_buf_max;
    if(arr < 0) arr = 0;
    __HAL_TIM_SET_PRESCALER(&htim1, psc);     
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    sample_interval=1.0f/sample_rate;

}


