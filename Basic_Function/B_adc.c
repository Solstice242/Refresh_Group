#include "Basic.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "gpio.h"
#include "ili9341_driver.h"
#include "usart.h"
/*===================
反思：
1.单独初始化函数，便于更改：采样率、增益
2.自动程控增益-本题只需要针对削波和信号太小情况进行调整
3.自动采样率匹配（上电后）+时基手动可调模式：FFT+AGC+参数测量；T_div模式
===================*/

/*--------前端操作------------
1.（最核心）自动控制增益电路选择AGC（高采样率-测幅值和频率）
2.触发电平可调
3.上升沿触发中断，开启采集
---------------------------*/
//频率变量
float adc_fre_grain=1.0f;
float adc_std_fre=1.0f;

//幅值变量
float ffp=0.0f;
float adc_amp_grain=1.0f;
float adc_std_amp=3.3f;

//增益变量
float V_Grain[8]={0.129,0.24,0.496,0.992,1.984,3.931,7.9,15.74};//放大或衰减倍数
uint8_t Grain_idx=0;        /* 默认增益=1 */
float adc_grain=0.129f;
uint8_t agc_was_clipped = 0; /* AGC: 上次测量是否削波 (仅AGC_Run使用) */
uint8_t auto_mode = 1;       /* 0=MANUAL(T/DIV), 1=AUTO(默认) */
uint8_t adc_owner = 0;       /* ADC占用标志: 0=正常触发, 1=AGC自适配 */
uint8_t agc_done  = 0;       /* AGC流水线完成标志 */

/* ── 自适应引擎状态  ── */
float   measured_freq = 1000.0f;         //hz
static uint8_t  consec_clip  = 0;        /* 连续削波帧数 */
static uint8_t  consec_small = 0;        /* 连续信号过小帧数 */
static uint8_t  consec_low_cycles  = 0;  /* 连续周期数过少帧数 */
static uint8_t  consec_high_cycles = 0;  /* 连续周期数过多帧数 */

/* ── 自适应引擎: 单帧检测结果 ── */
typedef struct {
    uint32_t cross_count;    /* 过零上升沿计数 */
    uint32_t min_code;       /* 帧内最小ADC码值 */
    uint32_t max_code;       /* 帧内最大ADC码值 */
    uint8_t  clipping;       /* 是否检测到削波 (1=削波) */
} AdpDetect_t;

//触发电平: 编码器4cnt×0.0025=0.01V/格, OUT_2切换0.1V/格
float step_1[2]={0.0025f, 0.025f};
 uint8_t step_dix=0;
uint16_t last_count_1=0;
uint16_t adc_zero=8192;//抬升
float tri_step=0.0025f;
 uint16_t  dac_val=0;

/*==================================*
 AGC逻辑函数：
*开机初始化（安全值x1）
*判断特殊情况：削波->降至0.125，再高速短采重测；信号极小->逐级提高增益直至可测
 *====================================*/
/*三个GPIO控制增益通路选择*/
void Chose_Grain(uint8_t idx)
{
    adc_grain = V_Grain[idx];
    Grain_idx = idx;
    switch(idx)
    {
case 0:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_RESET);
         break; }
case 1:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_SET);
         break; }
case 2:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_RESET);
         break; }
case 3:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_SET);
         break; }
case 4:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_RESET);
         break; }
case 5:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_SET);
         break; }
case 6:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_RESET);
         break; }
case 7:{ HAL_GPIO_WritePin(V_OUT_0_GPIO_Port, V_OUT_0_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_1_GPIO_Port, V_OUT_1_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(V_OUT_2_GPIO_Port, V_OUT_2_Pin, GPIO_PIN_SET);
         break; }
default: break;
    }
}

/*
幅值测量：
**前提：采集到至少一个周期
变量：输入数组，改变幅值
找峰谷值、剔除毛刺（中值滤波）--换算电压--校正
*/
float ADC_Measure_amp(uint16_t *src)
{
    uint16_t min_v = 65535, max_v = 0;
    for (int i = 0; i < Sample_Point; i++) {
        if (src[i] < min_v) min_v = src[i];
        if (src[i] > max_v) max_v = src[i];
    }
    ffp = (max_v - min_v) * ADC_LSB;
    float vol = ffp / adc_grain;
    AMP = vol * adc_amp_grain;
     sprintf(line2, " %.2fV", AMP);
                ILI9341_draw_string(rectangle_Left+2, 91, line2, BLACK);
                ILI9341_draw_string(rectangle_Left+2, 91, line2, GRED);
   return AMP;
}

/*──────────────────────────────────────────────────────────
  ApplySampleRate — 配置 TIM1 采样率
──────────────────────────────────────────────────────────*/
void ApplySampleRate(uint32_t fs_hz)
{
    if (fs_hz < 100)       fs_hz = 100;
    if (fs_hz > 5000000)   fs_hz = 5000000;

    uint32_t arr = (TIM_ADC_FREQ / fs_hz) - 1;
    uint32_t psc = 0;
    while (arr > 65535 && psc < 239) {
        psc++;
        arr = (TIM_ADC_FREQ / ((psc + 1) * fs_hz)) - 1;
    }
    if (arr > 65535) arr = 65535;

    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    sample_rate    = TIM_ADC_FREQ / ((psc + 1) * (uint32_t)(arr + 1));
    sample_interval = 1.0f / (float)sample_rate;
}

/*──────────────────────────────────────────────────────────
  Get_SampleRate — 从 TIM1 寄存器读实际采样率
──────────────────────────────────────────────────────────*/
uint32_t Get_SampleRate(void)
{
    uint32_t psc_val = htim1.Instance->PSC + 1;
    uint32_t arr_val = htim1.Instance->ARR + 1;
    if (psc_val == 0 || arr_val == 0) return 0;
    return TIM_ADC_FREQ / psc_val / arr_val;
}

/*
可调触发电平：
菜单选中-旋转编码器调大小-按键调步幅--转换成DAC输出（12位！）--显示
*/
void tri_step_change(void)
{
    tri_step=step_1[step_dix];
	    step_dix++;
	    if(step_dix>1)step_dix=0;
}
/* [未使用] 触发电平调整已迁移到 Menu_EncoderB case 3 */
void TRI_Scan(void)
{
    int16_t now=__HAL_TIM_GET_COUNTER(&htim4);
    int16_t diff=now-last_count_1;
    if(diff!=0)
    {
        LEVEL+=diff*tri_step;
       if(LEVEL<0.0f) LEVEL=0.0f;        /* 钳位防编码器累积漂移 */
       if(LEVEL>3.3f) LEVEL=3.3f;
       //旧: float shift_vol=LEVEL*adc_grain+adc_zero*ADC_LSB; //高增益时溢出DAC范围
       float shift_vol=LEVEL; // LEVEL直接表示比较器端电压(0~3.3V), 不乘增益
       if(shift_vol<0.0) shift_vol=0.0f;
       if(shift_vol>3.3f) shift_vol=3.3f;
     dac_val = (uint16_t)((shift_vol / V_REF) * dac_buf_max);
     if(dac_val > 4095) dac_val = 4095;           /* 12位DAC上限 */
     sprintf(line2, " LEVEL:%.2f\r\nDAC:%d", LEVEL, dac_val);
     HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), 100);
       HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_val);
     sprintf(line2, " %.2fV", LEVEL);
     ILI9341_draw_string(rectangle_Left+2,140,line2,BLACK);
     ILI9341_draw_string(rectangle_Left+2,140,line2,GRED );
       last_count_1=now;
    }

}

/*
中断函数：
1.上升沿触发
失能中断-开启DMA+定时器-单次触发标志位
2.旋转编码器按键控制进入菜单
3.旋转编码器调整参数
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
case Rising_Tri_Pin:{
            if (adc_owner)     return;           /* AGC占用ADC中, 忽略触发 */
            if(Single_Trig_flag) return;
            FFT_Refresh_flag = 0;                /* 新触发→退出FFT显示 */
            EXTI_D1->IMR1 &= ~(1UL << 4);      // 失能EXTI4
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, Sample_Point);
            HAL_TIM_Base_Start(&htim1);
            Single_Trig_flag = 1;
            break;
}
case OUT_1_Pin:
{
    static uint32_t last_t = 0;
    if(HAL_GetTick() - last_t < 200) return;
    last_t = HAL_GetTick();

    if(menu_active) {
        /* 已进入某模式 → 退出, 恢复AUTO */
        menu_active = 0;
        encB_need_sync = 1;   /* 下次进入模式时重新同步编码器B */
        auto_mode = 1;        /* 退出任何调整模式 → 回到自动 */
        agc_done  = 0;        /* 重新触发AGC流水线 */
    } else {
        /* 浏览状态 → 根据菜单项决定行为 */
        now_menu_idx = menu_idx;
        switch(menu_idx) {
            case 0: /* Single: 按一次进单次, 再按一次退连续 */
                Single_flag = !Single_flag;
                if(Single_flag) {
                    ILI9341_draw_string(5, 220, "Single", GREEN);
                } else {
                    ILI9341_draw_string(5, 220, "Single", GRED);
                    Single_Trig_flag = 0;
                    EXTI_D1->IMR1 |= (1UL << 4);     /* 退出时重开连续触发 */
                }
                break;
            case 4: /* Measure: FFT测频+测幅 */
                {
                    float amp_adc;
                    FREQ = Measure_Signal_FFT(&amp_adc) / 1000.0f;
                    AMP  = amp_adc / adc_grain;
                    sprintf(line2, " %.1fkHz", FREQ);
                    ILI9341_draw_string(rectangle_Left+2, 44, line2, BLACK);
                    ILI9341_draw_string(rectangle_Left+2, 44, line2, GRED);
                    sprintf(line2, " %.2fV", AMP);
                    ILI9341_draw_string(rectangle_Left+2, 91, line2, BLACK);
                    ILI9341_draw_string(rectangle_Left+2, 91, line2, GRED);
                }
                break;
            case 5: /* FFT: 设标志位, 主循环执行 */
                trigger_fft = 1;
                break;
            case 7: /* Correct: 设标志位, 主循环执行 */
                trigger_correct = 1;
                break;
            default: /* V/DIV(1),T/DIV(2),Trig(3),AC/DC(6): 进入调整模式 */
                menu_active = 1;
                encB_need_sync = 1;  /* 进入时也复位, 确保每次进入都同步编码器 */
                if (now_menu_idx == 2) auto_mode = 0;  /* T/DIV → 手动模式 */
                break;
        }
    }
    break;
}
case OUT_2_Pin:
{
     static uint32_t last_t = 0;
    if(HAL_GetTick() - last_t < 200) return;     
    last_t = HAL_GetTick();
    modal_active = !modal_active; // 切换旋钮按下状态
    break;
}
default:
    break;
  }
}











