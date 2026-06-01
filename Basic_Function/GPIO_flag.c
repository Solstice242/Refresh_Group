#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "Basic.h"
#include "usart.h"
#include "arm_math.h"
#include "ili9341_driver.h"
#include "stm32h7xx_it.h"
/*
补充：
1.FFT
2.频率测量
3.自校正
*/
// FFT 全局变量
float32_t hanning_win[Sample_Point];              
float32_t FFT_in[Sample_Point];                  //RFFT输入/输出复用 (实数入, 复数出) 
float32_t fft_mag[Sample_Point/2];               // 幅值谱, 仅存前N/2个bin
arm_rfft_fast_instance_f32 rfft_inst;          
float32_t base_freq;                              
float fft_sample_interval = 0.00001f;             
volatile uint8_t FFT_Refresh_flag=0;              /* 从FFT返回波形显示时刷新屏幕 */

/*启动 TIM2 测周(PA5) + TIM8 测频(PC7) */
void Freq_Capture_Init(void)
{
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);        /* TIM2_CH1: PA5, Slave RESET, 测周 */
    HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);        /* TIM8_CH2: PC7, ExtClock1, 边沿计数 */
}

/* 停止测频  */
void Freq_Capture_Stop(void)
{
    HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop(&htim8, TIM_CHANNEL_2);
}

/*==============================================================================
 * Freq_Capture_Get() — 双模测频, 返回 kHz
 * 双模切换: CCR1 ≥ 300 ticks (≤ 3.33 kHz) → 测周法
 *            CCR1 < 300 ticks (> 3.33 kHz) → 测频法
 *============================================================================*/
float Freq_Capture_Get(void)
{
    uint32_t period_us = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);

    /* ── ① 测周法: 低频 (≤ 3.33 kHz), CCR1 直接读出周期(μs) ── */
    if(period_us >= 300) {
        if(period_us == 0) return 0.0f;             /* 无信号 */
        return 1000.0f / (float)period_us;           /* kHz = 1000 / μs */
    }

    /* ── ② 测频法: 高频 (> 3.33 kHz), TIM8 闸门计数 ── */
    static uint32_t last_ms   = 0;
    static uint16_t last_cnt  = 0;
    static float    cached_khz = 0.0f;

    uint32_t now_ms  = HAL_GetTick();
    uint16_t cnt_now = htim8.Instance->CNT;
    uint32_t elapsed = now_ms - last_ms;

    if(elapsed > 500) {                              /* 首次调用 → 初始化基准 */
        last_ms  = now_ms;
        last_cnt = cnt_now;
        return 0.0f;
    }

    if(elapsed >= 100) {                             /* 每100ms刷新 */
        uint16_t edges = cnt_now - last_cnt;
        cached_khz = (float)edges * (1000.0f / (float)elapsed);
        last_ms    = now_ms;
        last_cnt   = cnt_now;
    }
    return cached_khz;
}

/*-----------------------------------------------
  初始化Hanning窗 + RFFT实例
  Hanning窗: w[i] = 0.5*(1 - cos(2pi*i/(N-1)))
-------------------------------------------------*/
void FFT_Init(void)
{
    for(int i = 0; i < Sample_Point; i++) {
        hanning_win[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (Sample_Point - 1)));
    }
    arm_rfft_fast_init_f32(&rfft_inst, Sample_Point);
}

/*-------------------------------------------------------
核心频谱分析
去直流 -> 加Hanning窗 -> 1024点实数FFT -> 计算幅值谱
---------------------------------------------------------*/
void FFT_Analysis(uint16_t *adc_raw, uint16_t len)
{
    uint16_t n = (len < Sample_Point) ? len : Sample_Point;
    float32_t dc_level = (float32_t)adc_zero * (V_REF / adc_buf_max);

    //去直流 + 加Hanning窗 
    for(int i = 0; i < Sample_Point; i++) {
        if(i < n) {
            float32_t v = (float32_t)adc_raw[i] * (V_REF / adc_buf_max) - dc_level;
            FFT_in[i] = v * hanning_win[i];
        } else {
            FFT_in[i] = 0.0f;               
        }
    }
    arm_rfft_fast_f32(&rfft_inst, FFT_in, FFT_in, 0);
    //幅值谱 
    fft_mag[0] = fabsf(FFT_in[0]) / (float32_t)Sample_Point;        
    fft_mag[Sample_Point / 2 - 1] = fabsf(FFT_in[1]) / (float32_t)Sample_Point; 
    for(int i = 1; i < Sample_Point / 2 - 1; i++) {
        float32_t real = FFT_in[2 * i];
        float32_t imag = FFT_in[2 * i + 1];
        fft_mag[i] = sqrtf(real * real + imag * imag) / ((float32_t)Sample_Point / 2.0f);
    }
}

/*-----------------------------------------------
 找基频
 跳过bin 0 (DC分量), 从i=1开始搜索
 频率分辨率 = 1 / (fft_sample_interval * N)
------------------------------------------------*/
float32_t Find_Base_Freq(void)
{
    uint16_t max_idx = 1;
    float32_t max_val = 0.0f;

    for(int i = 1; i < Sample_Point / 2; i++) {
        if(fft_mag[i] > max_val) {
            max_val = fft_mag[i];
            max_idx = i;
        }
    }
    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    return (float32_t)max_idx * freq_res;
}

/*-----------------------------------------------------------
 频谱显示：左侧柱状图(0~N/4 bin) +文字
 柱状图: 每隔一个bin画2px宽绿色柱, 高度=mag*Show_Half/V_REF
 -----------------------------------------------------------*/
void FFT_Display(uint16_t base_idx)
{
    ILI9341_fill(Show_Left, Show_Top, Show_Right, Show_Bottom, BLACK);
    Draw_Line();

    char buf[32];
    for(int i = 0; i < Sample_Point / 4; i += 2) {
        uint16_t x = Show_Left + i * (Bar_Width + Bar_gap);
        if(x + Bar_Width > Show_Left + Show_Width) break;

        uint16_t bar_h = (uint16_t)(fft_mag[i] * (float)Show_Half / V_REF);
        if(bar_h > Show_Height) bar_h = Show_Height;

        ILI9341_fill(x, Show_Bottom - bar_h, x + Bar_Width, Show_Bottom, GREEN);
    }

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    for(int n = 1; n <= 5; n++) {
        uint16_t harm_idx = base_idx * n;
        if(harm_idx < Sample_Point / 2) {
            float32_t harm_freq = (float32_t)harm_idx * freq_res;
            float32_t harm_amp  = fft_mag[harm_idx];
            sprintf(buf, "%dH: %.1fkHz %.2fV", n, harm_freq / 1000.0f, harm_amp);
            ILI9341_draw_string(Show_Left + 10, Show_Top + 20 * (n - 1) + 5, buf, GRED);
        }
    }
}

/*---------------------------------------------------------------
 FFT_Process()FFT总流程
初始化窗+RFFT-同步采样率-去直流加窗FFT-找基频-画频谱
----------------------------------------------------------------*/
void FFT_Process(void)
{
    FFT_Init();                                        
    fft_sample_interval = sample_interval;             //同步当前采样率！
    FFT_Analysis(adc_buffer, Sample_Point);           
    base_freq = Find_Base_Freq();                     

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    uint16_t base_idx = (uint16_t)(base_freq / freq_res);

    ILI9341_fill(Show_Left, Show_Top, Show_Right, Show_Bottom, BLACK);
    FFT_Display(base_idx);
    FFT_Refresh_flag = 1;  /* 标记FFT已显示, 防止ADC_Project覆盖 */
}

/*-------------------------------
 算 DC 偏置 (零点)-50% 方波平均
---------------------------------*/
void Zero_Correct(void)
{
    uint32_t sum = 0;
    for(int i = 0; i < Sample_Point; i++) {
        sum += adc_buffer[i];
    }
    adc_zero = sum / Sample_Point;                    
}

/*--------------------------------------------------------
输出PWM -ADC采集一帧 -零点/幅值/频率三合一 - 算校正系数 -恢复
---------------------------------------------------------*/
void Correct_Process(void)
{
    HAL_TIM_Encoder_Stop(&htim12, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_Delay(10);                                     //等待 PWM 稳定 

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, Sample_Point);
    HAL_TIM_Base_Start(&htim1);

    uint32_t timeout = 1000000;
    while(!ADC_flag && --timeout) {}
    if(timeout == 0) {                                 //超时 
        HAL_TIM_Base_Stop(&htim1);
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
        HAL_TIM_Encoder_Start(&htim12, TIM_CHANNEL_ALL);
        return;
    }
    ADC_flag = 0;

    Zero_Correct();                                   
    ADC_Filter(adc_buffer, adc_Filter, Display_Point); 
    ADC_Measure_amp(adc_Filter);                       
    if(AMP > 0.001f) {
        adc_amp_grain = adc_std_amp / AMP;             
    }
    FREQ = Freq_Capture_Get();                         
    if(FREQ > 0.001f) {
        adc_fre_grain = adc_std_fre / FREQ;            
    }

    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim12, TIM_CHANNEL_ALL);
    Single_Trig_flag = 0;
}



