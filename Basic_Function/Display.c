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
#define ITEM_NUM 9
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
    {"Grain",   NULL},
};
 uint8_t  menu_idx = 0;               //显示
 uint8_t  now_menu_idx = 0;           //当前模式
uint8_t  menu_active = 0;                 // 0=浏览, 1=进入 
uint8_t  modal_active = 0;                // 编码器B按下 
static int16_t  last_encA = 0;            // TIM3编码器上次值 (仅本文件)
static int16_t  last_encB = 0;            // TIM4编码器上次值 (仅本文件)



/* ── 垂直灵敏度 ── */
float V_div[3]={1.0,0.1,0.01};//屏幕显示
uint8_t V_idx=1;
 float V_DIV=1.0f;

//时基模式
float TIM_modal[9]={10,50,100,
                  0.5,1,10,100,
                  0.2,2};
char* TIM_V[3]={"us","ms","s"};
uint8_t tim_index = 0;
uint8_t tim_modal=0;
uint8_t TIM_Change_flag=0;
float tim_div=20.0f;//单位前数值
 float  T_DIV = 20.0f;//当前时基（秒）
float sample_interval= 0.00001f;//当前采样间隔
float sample_rate;//当前采样率
 uint32_t psc=0;   
    uint32_t arr =0;

//触发电平模式
float LEVEL=1.65f;

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
详见 B_adc.c 中断函数
-------------------------------------*/
uint8_t encB_need_sync = 1;  /* 退出模式时置1, Menu_EncoderB下次进入时重新同步编码器CNT */

/* -------------------------------------
旋转编码器B_TIM4旋转 -> 进入模式后调整参数
仅 menu_active==1 时有效
---------------------------------------*/
void Menu_EncoderB(void)
{
    if(!menu_active) return;                     /* 未进入模式, 编码器B无效 */

    static int16_t last_encB_menu = 0;           /* 菜单系统专用, 不与旧函数共享 */
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    if(encB_need_sync) {                          /* 刚进入模式: 同步编码器初值, 不触发 */
        last_encB_menu = now;
        encB_need_sync = 0;
        return;
    }
    int16_t diff = now - last_encB_menu;
    int8_t dir = 0;
    if(diff >= 4)      { dir =  1; last_encB_menu += 4; }
    else if(diff <= -4){ dir = -1; last_encB_menu -= 4; }
    else return;

    switch(now_menu_idx) {                       /* 用进入时锁定的索引 */
        case 1: /* V/DIV: 3挡切换 */
            V_idx = (V_idx + dir + 3) % 3;
            V_Draw();
            break;
        case 2: /* T/DIV */
            if(TIM_Change_flag == 0) {           /* 粗调: 循环8挡 */
                tim_modal = (tim_modal + dir + 9) % 9;
                if(tim_modal <= 2)          tim_index = 0;
                else if(tim_modal <= 6)     tim_index = 1;
                else                        tim_index = 2;
                tim_div = TIM_modal[tim_modal];
                Match_Tim();                     /* 粗调: 匹配挡位→硬件 */
            } else {                             /* 细调: 连续值 */
                float step = (tim_div < 1.0f) ? 0.1f : 1.0f;
                tim_div += step * (float)dir;
                float a = Shift_T_div(tim_index, tim_div);
                set_T_div(a);                    /* 细调: 直接用tim_div设硬件, 不走Match_Tim */
            }
            /* 显示时基值 */
            ILI9341_fill(175, 2, 204, 18, BLACK);
            sprintf(line2, " %.1f", tim_div);
            ILI9341_draw_string(175, 2, line2, GRED);
            ILI9341_draw_string(220, 2, TIM_V[tim_index], GRED);
            break;
        case 3: /* Trig: 复用原TRI_Scan逻辑, raw diff比例调节 */
            {   extern uint16_t last_count_1;
                int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
                int16_t diff = now - (int16_t)last_count_1;
                if(diff != 0) {
                    LEVEL += (float)diff * tri_step;
                    if(LEVEL<0.0f) LEVEL=0.0f;        /* 钳位防编码器累积漂移 */
                    if(LEVEL>3.3f) LEVEL=3.3f;
                    //旧: float sv = LEVEL * adc_grain + (float)adc_zero * ADC_LSB;
                    float sv = LEVEL; // LEVEL直接表示比较器端电压(0~3.3V)
                    if(sv < 0.0f) sv = 0.0f; if(sv > 3.3f) sv = 3.3f;
                    dac_val = (uint16_t)((sv / V_REF) * dac_buf_max);
                    if(dac_val > 4095) dac_val = 4095;           /* 12位DAC上限 */
                    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
                    float true_level = LEVEL * adc_grain + (float)adc_zero * ADC_LSB ; // 显示输入端电压水平
                    sprintf(line2, " %.2fV", true_level);
                    ILI9341_draw_string(rectangle_Left+2, 140, line2, BLACK);
                    ILI9341_draw_string(rectangle_Left+2, 140, line2, GRED);
                    last_count_1 = (uint16_t)now;
                }
            }
            break;
        case 6: /* AC/DC: 切换 */
            AC_idx = (AC_idx + 1) % 2;
            AC_Draw();
            break;
        case 8: /* Grain: 手动增益 */
            Grain_idx = (Grain_idx + dir + 8) % 8;
            Chose_Grain(Grain_idx);
            G_Draw();
            break;
        default: break;  /* Single/Measure/FFT/Correct 不用编码器B */
    }
}

//交直流耦合模式
void AC_Draw(void)
{
    /* DC: 位置(294,2), AC_idx=0时绿色高亮, 否则灰色 */
    uint16_t dc_c = (AC_idx == 0) ? GREEN : GRAY;
    ILI9341_draw_string(294, 2, "DC", BLACK);
    ILI9341_draw_string(294, 2, "DC", dc_c);

    /* AC: 位置(270,2), AC_idx=1时绿色高亮, 否则灰色 */
    uint16_t ac_c = (AC_idx == 1) ? GREEN : GRAY;
    ILI9341_draw_string(270, 2, "AC", BLACK);
    ILI9341_draw_string(270, 2, "AC", ac_c);
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
    ILI9341_fill(50, 2, 100, 18, BLACK);
    sprintf(line2, " %.2fV", V_DIV);
    ILI9341_draw_string(50, 2, line2, GRED);
}

//手动增益模式
void G_Draw(void)
{Chose_Grain(Grain_idx);
    sprintf(line2, " %.3f", adc_grain);
    ILI9341_draw_string(rectangle_Left+2,185,line2,BLACK );
     ILI9341_draw_string(rectangle_Left+2,185,line2,GRED );
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
    if(tim_modal == 255)  {tim_modal = 8;}
    if(tim_modal >= 9)  tim_modal = 0;
    switch(tim_modal)
    {
        case 0: case 1: case 2:      tim_index = 0; break;
        case 3: case 4: case 5: case 6: tim_index = 1; break;
        case 7: case 8:               tim_index = 2; break;
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
        case 2: /* T/DIV: 粗调↔细调 */
    {
        TIM_Change_flag = !TIM_Change_flag;
        if(TIM_Change_flag == 0) {
            Match_Tim();                         /* 细调→粗调: 匹配最近挡位 */
        }
        ILI9341_fill(175, 2, 204, 18, BLACK);
        sprintf(line2, " %.1f", tim_div);
        ILI9341_draw_string(175, 2, line2, GRED);
        ILI9341_draw_string(220, 2, TIM_V[tim_index], GRED);
    }
            break;
        case 3: tri_step_change();
            break;
        case 4: // Measure: FFT测频+测幅
            {
                float amp_adc;
                FREQ = Measure_Signal_FFT(&amp_adc) / 4000.0f;
                AMP  = amp_adc / adc_grain;
                sprintf(line2, " %.1fkHz", FREQ);
                ILI9341_draw_string(rectangle_Left+2, 44, line2, BLACK);
                ILI9341_draw_string(rectangle_Left+2, 44, line2, GRED);
                sprintf(line2, " %.2fV", AMP);
                ILI9341_draw_string(rectangle_Left+2, 91, line2, BLACK);
                ILI9341_draw_string(rectangle_Left+2, 91, line2, GRED);
            }
            break;
        case 5: FFT_Process();
            break;
        case 6:  AC_output();
            break;
        case 7: // Correct
         Correct_Process();
            break;
        case 8: // Grain: 无额外动作, 仅旋转调整
            break;
            default:break;
    }
    modal_active = 0;  /* 处理完后清零, 防止下一帧重复执行 */
}

//交直流耦合模式
void AC_output(void)
{
    AC_idx = (AC_idx + 1) % 2;  /* 切换AC/DC */
    AC_Draw();                   /* 刷新屏幕显示 */
    /* main初始化 RESET=DC, 此处保持一致: AC_idx=0→DC→RESET, AC_idx=1→AC→SET */
    switch(AC_idx)
    {
        case 0:
        HAL_GPIO_WritePin(AC_DC_GPIO_Port, AC_DC_Pin, GPIO_PIN_RESET);
            break;
        case 1:
        HAL_GPIO_WritePin(AC_DC_GPIO_Port, AC_DC_Pin, GPIO_PIN_SET);
            break;
    }
}

void Match_Tim(void)
{
    /* tim_base全是μs单位, 用作匹配基准; TIM_modal混用μs/ms/s仅用于显示 */
    const float tim_base[9] = {
        10,50,100,500,1000,10000,100000,200000,2000000
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
     switch(Tim_idx)   /* 使用传入参数, 非全局 tim_index */
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
    float total_scan_time = T_DIV * 10.0f;
     sample_rate = (uint32_t)(Display_Point / total_scan_time);
     if(sample_rate < 100)       sample_rate = 100;
     if(sample_rate > 5000000)   sample_rate = 5000000;
     return sample_rate;
}

void set_T_div(float Sample_rate)
{
    if(Sample_rate <= 0.0f) return;              /* 防除零 */

    psc = 0;                                     /* 全局psc必须重置, 否则累积 */
    arr = (TIM_ADC_FREQ /  Sample_rate) - 1;
    //旧: while(arr > adc_buf_max && psc < 239)  // adc_buf_max=16384, TIM1 ARR上限应为65535
    while(arr > 65535 && psc < 239)
    {
        psc++;
        arr = (TIM_ADC_FREQ / ((psc + 1) * Sample_rate)) - 1;
    }

    if(arr > 65535) arr = 65535;
    //旧: if(arr < 0) arr = 0;  // arr是uint32_t, 永不为负
    HAL_TIM_Base_Stop(&htim1);                     /* 安全: 先停后改 */
    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    htim1.Instance->EGR = TIM_EGR_UG;              /* 强制更新影子寄存器 */
    sample_interval=1.0f/Sample_rate;

}


