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
// FFT 全局变量 (参考Spectrum_*结构: 独立in/out缓冲区)
float32_t hanning_win[Sample_Point];
float32_t FFT_in[Sample_Point];                  // RFFT输入(去直流+加窗后)
float32_t FFT_out[Sample_Point];                 // RFFT输出(独立缓冲区, 不复用输入)
float32_t fft_mag[Sample_Point / 2 + 1];         // 幅值谱
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
核心频谱分析 (参考Spectrum_Process)
 ① 计算实际ADC均值去直流 → ② 加Hanning窗 → ③ RFFT(独立in/out)
 → ④ 幅值谱(Hanning窗校正)
---------------------------------------------------------*/
void FFT_Analysis(uint16_t *adc_raw, uint16_t len)
{
    uint16_t n = (len < Sample_Point) ? len : Sample_Point;
    uint16_t i;
    float32_t scale = 0.5f;  /* Hanning窗相干增益 */
    float32_t Nf    = (float32_t)Sample_Point;

    /* ① 计算实际DC均值 (比固定adc_zero更准) */
    float avg = 0.0f;
    for (i = 0; i < n; i++) {
        avg += (float)adc_raw[i];
    }
    avg = avg / (float)n;

    /* ② 去直流 + 加Hanning窗 → FFT_in */
    for (i = 0; i < Sample_Point; i++) {
        if (i < n) {
            float32_t v = ((float32_t)adc_raw[i] - avg) * (V_REF / adc_buf_max);
            FFT_in[i] = v * hanning_win[i];
        } else {
            FFT_in[i] = 0.0f;
        }
    }

    /* ③ RFFT: 独立输入/输出缓冲区 (不复用, 参考Spectrum_Process) */
    arm_rfft_fast_f32(&rfft_inst, FFT_in, FFT_out, 0);

    /* ④ 幅值谱: DC/Nyquist ÷(scale*N), 其他bin ×2÷(scale*N) */
    fft_mag[0]                    = fabsf(FFT_out[0]) / (scale * Nf);
    fft_mag[Sample_Point / 2]     = fabsf(FFT_out[1]) / (scale * Nf);
    for (i = 1; i < Sample_Point / 2; i++) {
        float32_t real = FFT_out[2 * i];
        float32_t imag = FFT_out[2 * i + 1];
        fft_mag[i] = 2.0f * sqrtf(real * real + imag * imag) / (scale * Nf);
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
 频谱显示 (参考Spectrum_Draw)
  0~200kHz bin压缩映射到Display_Point列 + 谐波文字
 -----------------------------------------------------------*/
void FFT_Display(uint16_t base_idx, uint32_t sample_rate)
{
    uint16_t i, x;
    uint16_t bin_limit;
    float max_mag;
    char buf[32];

    if (sample_rate == 0) return;

    ILI9341_fill(Show_Left, Show_Top, Show_Right, Show_Bottom, BLACK);
    Draw_Line();

    /* ① 限制显示范围 0~200kHz */
    bin_limit = (uint16_t)((200000UL * Sample_Point) / sample_rate);
    if (bin_limit >= Sample_Point / 2) bin_limit = Sample_Point / 2 - 1;
    if (bin_limit < 2) bin_limit = 2;

    /* ② 显示范围内最大幅值, 自适应缩放 */
    max_mag = 0.001f;
    for (i = 1; i < bin_limit; i++) {
        if (fft_mag[i] > max_mag) max_mag = fft_mag[i];
    }

    /* ③ bin→列压缩: 每列取bin范围内max */
    for (x = 0; x < Display_Point; x++) {
        uint16_t bs = (uint16_t)((uint32_t)x * bin_limit / Display_Point);
        uint16_t be = (uint16_t)((uint32_t)(x + 1) * bin_limit / Display_Point);
        float col_max = 0.0f;
        if (be <= bs) be = bs + 1;
        for (i = bs; i < be; i++) {
            if (fft_mag[i] > col_max) col_max = fft_mag[i];
        }
        uint16_t h = (uint16_t)(col_max * (float)Show_Height * 0.85f / max_mag);
        if (h > Show_Height) h = Show_Height;
        uint16_t px = Show_Left + (x * Show_Width) / Display_Point;
        if (h > 0) {
            ILI9341_draw_line(px, Show_Bottom, px, Show_Bottom - h, GREEN);
        }
    }

    /* ④ 谐波文字 */
    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    for (int n = 1; n <= 5; n++) {
        uint16_t target_bin = base_idx * n;
        uint16_t harm_idx = FFT_FindPeakNear(target_bin, 2);
        if (harm_idx < Sample_Point / 2) {
            float32_t harm_freq = (float32_t)harm_idx * freq_res;
            float32_t harm_amp  = fft_mag[harm_idx];
            sprintf(buf, "%dH:%.1fkHz %.2fV", n, harm_freq / 1000.0f, harm_amp);
            ILI9341_draw_string(Show_Left + 10, Show_Top + 20 * (n - 1) + 5, buf, GRED);
        }
    }
}

/*---------------------------------------------------------------
 Measure_Signal_FFT — FFT测频+测幅 (参考Spectrum_GetMainFreq)
  锁定450kHz → 采集 → RFFT → 返回Hz + ADC端峰值V
----------------------------------------------------------------*/
float Measure_Signal_FFT(float *amp_adc)
{
    if (Single_Trig_flag) { if (amp_adc) *amp_adc = 0.0f; return 0.0f; }

    #define MEASURE_FS  450000UL
    uint32_t saved_fs = Get_SampleRate();
    if (saved_fs != MEASURE_FS) ApplySampleRate(MEASURE_FS);

    EXTI_D1->IMR1 &= ~(1UL << 4);
    ADC_flag = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, Sample_Point);
    HAL_TIM_Base_Start(&htim1);

    uint32_t timeout = 10000000;
    while (!ADC_flag && --timeout) {}
    ADC_flag = 0;

    if (timeout == 0) {
        if (saved_fs != MEASURE_FS) ApplySampleRate(saved_fs);
        EXTI_D1->IMR1 |= (1UL << 4);
        if (amp_adc) *amp_adc = 0.0f;
        return 0.0f;
    }

    uint32_t actual_fs = Get_SampleRate();
    fft_sample_interval = 1.0f / (float)actual_fs;
    FFT_Analysis(adc_buffer, Sample_Point);
    base_freq = Find_Base_Freq();

    if (amp_adc) {
        float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
        uint16_t base_idx = (uint16_t)(base_freq / freq_res);
        if (base_idx > 0 && base_idx < Sample_Point / 2)
            *amp_adc = fft_mag[base_idx];
        else
            *amp_adc = 0.0f;
    }

    if (saved_fs != MEASURE_FS) ApplySampleRate(saved_fs);
    EXTI_D1->IMR1 |= (1UL << 4);
    return base_freq;
}

/*---------------------------------------------------------------
 FFT_Process() — FFT频谱显示 (固定450kHz, 参考Spectrum_Draw)
----------------------------------------------------------------*/
void FFT_Process(void)
{
    if (Single_Trig_flag) return;

    #define FFT_SAMPLE_RATE  450000UL
    uint32_t saved_fs = Get_SampleRate();
    if (saved_fs != FFT_SAMPLE_RATE) ApplySampleRate(FFT_SAMPLE_RATE);

    EXTI_D1->IMR1 &= ~(1UL << 4);
    ADC_flag = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, Sample_Point);
    HAL_TIM_Base_Start(&htim1);

    uint32_t timeout = 10000000;
    while (!ADC_flag && --timeout) {}
    ADC_flag = 0;

    if (timeout == 0) {
        if (saved_fs != FFT_SAMPLE_RATE) ApplySampleRate(saved_fs);
        EXTI_D1->IMR1 |= (1UL << 4);
        return;
    }

    uint32_t actual_fs = Get_SampleRate();
    fft_sample_interval = 1.0f / (float)actual_fs;
    FFT_Analysis(adc_buffer, Sample_Point);
    base_freq = Find_Base_Freq();

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    uint16_t base_idx = (uint16_t)(base_freq / freq_res);
    if (base_idx == 0) base_idx = 1;
    if (base_idx >= Sample_Point / 2) base_idx = Sample_Point / 2 - 1;

    sprintf(line2, "FFT:%.1fHz\r\n", (double)base_freq);
    HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), 100);

    FFT_Display(base_idx, actual_fs);
    FFT_Refresh_flag = 1;

    if (saved_fs != FFT_SAMPLE_RATE) ApplySampleRate(saved_fs);
    EXTI_D1->IMR1 |= (1UL << 4);
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



