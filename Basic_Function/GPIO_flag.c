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
float32_t fft_mag[Sample_Point/2 + 1];           // 幅值谱: [0]=DC, [1..510]=bin1..510, [511]=bin511, [512]=Nyquist
arm_rfft_fast_instance_f32 rfft_inst;          
float32_t base_freq;                              
float fft_sample_interval = 0.00001f;             
volatile uint8_t FFT_Refresh_flag=0;              /* 从FFT返回波形显示时刷新屏幕 */
uint8_t trigger_correct = 0;                         /* 主循环触发自校正 */
uint8_t trigger_fft = 0;                             /* 主循环触发FFT */

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
    static float last_valid = 0.0f;                  /* 跳变过滤: 上次有效值 */
    float result;
    uint32_t period_us = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    static uint32_t dbg_t = 0;
    if(HAL_GetTick() - dbg_t > 500) {                /* 每500ms打印一次诊断 */
        dbg_t = HAL_GetTick();
      //  sprintf(line2, "T2=%lu\r\n", period_us);
      //  HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), 100);
    }

    /* ── ① 测周法: 低频 (≤ 3.33 kHz), CCR1 直接读出周期(μs) ── */
    if(period_us >= 300) {
        if(period_us == 0) return last_valid;        /* 无信号, 保留上次值 */
        result = 1000.0f / (float)period_us;         /* kHz = 1000 / μs */
    } else {
        /* ── ② 测频法: 高频 (> 3.33 kHz), TIM8 闸门计数 ── */
        static uint32_t last_ms   = 0;
        static uint16_t last_cnt  = 0;
        static float    cached_hz = 0.0f;    /* 缓存值单位是Hz, 非kHz */

        uint32_t now_ms  = HAL_GetTick();
        uint16_t cnt_now = htim8.Instance->CNT;
        uint32_t elapsed = now_ms - last_ms;

        if(elapsed > 500) {
            last_ms  = now_ms;
            last_cnt = cnt_now;
            return cached_hz / 1000.0f;       /* Hz → kHz */
        }

        if(elapsed >= 100) {
            uint16_t edges = cnt_now - last_cnt;
            cached_hz = (float)edges * (1000.0f / (float)elapsed);  /* Hz */
            last_ms   = now_ms;
            last_cnt  = cnt_now;
        }
        result = cached_hz / 1000.0f;         /* Hz → kHz */
    }

    /* ── 跳变过滤: 拒绝>50%突变, 防噪声毛刺和模式振荡 ── */
    if(last_valid > 0.001f && result > 0.001f) {
        float ratio = result / last_valid;
        if(ratio > 1.5f || ratio < 0.667f) return last_valid;
    }
    if(result > 0.001f) last_valid = result;
    return result;
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
    /* 幅值谱: window_scale=sum(hanning)/N=0.5, 归一化时需 ×(1/window_scale)=×2
       DC/Nyquist:  ×(1.0/(scale*N));  其他bin: ×(2.0/(scale*N))
       fft_mag 布局: [0]=DC, [1..N/2-1]=bin1..511, [N/2]=Nyquist */
    float32_t scale = 0.5f;  /* Hanning窗相干增益 = sum(win)/N */
    fft_mag[0] = fabsf(FFT_in[0]) / (scale * (float32_t)Sample_Point);
    fft_mag[Sample_Point / 2] = fabsf(FFT_in[1]) / (scale * (float32_t)Sample_Point);
    for(int i = 1; i < Sample_Point / 2; i++) {
        float32_t real = FFT_in[2 * i];
        float32_t imag = FFT_in[2 * i + 1];
        fft_mag[i] = 2.0f * sqrtf(real * real + imag * imag) / (scale * (float32_t)Sample_Point);
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

    /* 搜索范围 1..N/2-1, 排除 DC[0] 和 Nyquist[N/2] */
    for(int i = 1; i < Sample_Point / 2; i++) {
        if(fft_mag[i] > max_val) {
            max_val = fft_mag[i];
            max_idx = i;
        }
    }
    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    return (float32_t)max_idx * freq_res;
}

/*---------------------------------------------------------------
 改进B: FFT_FindPeakNear — 在预期bin ±search_range 内搜索真实峰值
 补偿Hanning窗频谱泄漏和栅栏效应, 谐波幅值比直接用整数倍bin更准确
---------------------------------------------------------------*/
static uint16_t FFT_FindPeakNear(uint16_t center_bin, uint16_t search_range)
{
    uint16_t start_bin = (center_bin > search_range) ? (center_bin - search_range) : 1;
    uint16_t end_bin   = center_bin + search_range;
    if (end_bin >= Sample_Point / 2) end_bin = Sample_Point / 2 - 1;  /* 排除 Nyquist */

    uint16_t max_bin = start_bin;
    for (uint16_t i = start_bin + 1; i <= end_bin; i++) {
        if (fft_mag[i] > fft_mag[max_bin]) max_bin = i;
    }
    return max_bin;
}

/*-----------------------------------------------------------
 频谱显示：左侧柱状图(0~N/4 bin) +文字
 改进B: 谐波用FFT_FindPeakNear峰值精修, 补偿频谱泄漏
 改进C: 柱状图高度自适应归一化, 替换固定V_REF缩放
 -----------------------------------------------------------*/
void FFT_Display(uint16_t base_idx)
{
    ILI9341_fill(Show_Left, Show_Top, Show_Right, Show_Bottom, BLACK);
    Draw_Line();

    /* ── 改进C: 找显示范围内最大幅值, 用来自适应缩放频谱高度 ── */
    float max_mag = 0.001f;  /* 防除零 */
    uint16_t disp_bins = Sample_Point / 4;
    for (uint16_t i = 0; i < disp_bins; i++) {
        if (fft_mag[i] > max_mag) max_mag = fft_mag[i];
    }

    char buf[32];
    for(int i = 0; i < Sample_Point / 4; i += 2) {
        uint16_t x = Show_Left + i * (Bar_Width + Bar_gap);
        if(x + Bar_Width > Show_Left + Show_Width) break;

        //旧: 固定V_REF缩放 → 小信号时频谱太矮, 大信号时削顶
        //uint16_t bar_h = (uint16_t)(fft_mag[i] * (float)Show_Half / V_REF);
        uint16_t bar_h = (uint16_t)(fft_mag[i] / max_mag * (float)Show_Height * 0.85f);
        if(bar_h > Show_Height) bar_h = Show_Height;

        ILI9341_fill(x, Show_Bottom - bar_h, x + Bar_Width, Show_Bottom, GREEN);
    }

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    for(int n = 1; n <= 5; n++) {
        //旧: 直接用 base_idx*n, 不补偿频谱泄漏导致谐波幅值偏低
        //uint16_t harm_idx = base_idx * n;
        /* ── 改进B: 在预期谐波位置 ±2 bin 搜索真实峰值 ── */
        uint16_t target_bin = base_idx * n;
        uint16_t harm_idx = FFT_FindPeakNear(target_bin, 2);
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
    /* 并发保护: ADC正在DMA传输时 adc_buffer 数据不完整, 禁止FFT */
    if(Single_Trig_flag) return;

    //FFT_Init();  /* 旧: 每帧重复初始化窗+RFFT, 浪费CPU */
    /* 改进A: FFT_Init() 移到 main.c 启动时一次性调用, Hanning窗是常量无需重算 */
    fft_sample_interval = captured_sample_interval; /* 用采集时的采样间隔, 确保频率轴正确 */
    FFT_Analysis(adc_buffer, Sample_Point);
    base_freq = Find_Base_Freq();

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    uint16_t base_idx = (uint16_t)(base_freq / freq_res);
    if(base_idx == 0) base_idx = 1;                      /* 防DC */
    if(base_idx >= Sample_Point / 2) base_idx = Sample_Point / 2 - 1;  /* 防越界 */

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
    if(Single_Trig_flag) return;          /* ADC采集中, 禁止校正抢占 */
    HAL_TIM_Encoder_Stop(&htim12, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

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
    ADC_Measure_amp(adc_buffer);    /* 1024原始点测幅, 非200滤波点 */

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

    ILI9341_draw_string(5, 220, "Correct", BLACK);
    ILI9341_draw_string(5, 220, "Correct", GREEN);       /* 完成提示 */
}



