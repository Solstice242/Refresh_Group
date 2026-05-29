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



/* ── FFT 全局变量 ── */
uint8_t FFT_flag=0;                              /* FFT触发标志 (由按键/菜单置1) */
float32_t hanning_win[Sample_Point];              /* Hanning窗系数, 减少频谱泄漏 */
float32_t FFT_in[Sample_Point];                  /* RFFT输入/输出复用 (实数入, 复数出) */
float32_t fft_mag[Sample_Point/2];               /* 幅值谱, 仅存前N/2个bin */
arm_rfft_fast_instance_f32 rfft_inst;             /* CMSIS-DSP RFFT实例 */
float32_t base_freq;                              /* 基频(Hz), Find_Base_Freq输出 */
float fft_sample_interval = 0.00001f;             /* FFT采样间隔(s), 使用前与sample_interval同步 */
volatile uint8_t FFT_Refresh_flag=0;              /* 从FFT返回波形显示时刷新屏幕 */


float phase_diff=0.0f;
 uint16_t  V_Standard =(uint16_t)adc_buf_max/V_REF;

volatile uint8_t system_busy = 0;

/*DSP 运算必须固定 32 位浮�?*/
/*==============================================================================
 * FFT_Init() — 初始化Hanning窗 + RFFT实例
 * Hanning窗: w[i] = 0.5*(1 - cos(2pi*i/(N-1)))
 * 作用: 减少非整周期采样导致的频谱泄漏 (旁瓣从-13dB降到-32dB)
 * 只在进入FFT模式时执行一次, 窗系数和RFFT实例不变
 *============================================================================*/
void FFT_Init(void)
{
    for(int i = 0; i < Sample_Point; i++) {
        hanning_win[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (Sample_Point - 1)));
    }
    arm_rfft_fast_init_f32(&rfft_inst, Sample_Point);
}

/*==============================================================================
 * FFT_Analysis() — 核心频谱分析
 * 流程: 去直流 -> 加Hanning窗 -> 1024点实数FFT -> 计算幅值谱
 *
 * CMSIS-DSP 实数FFT输出布局 (arm_rfft_fast_f32, N=1024):
 *   FFT_in[0]   = DC分量 (实数)
 *   FFT_in[1]   = Nyquist分量 (实数)
 *   FFT_in[2i]  = 第i个正频率bin的实部 (i=1..N/2-1)
 *   FFT_in[2i+1]= 第i个正频率bin的虚部
 *
 * 幅值谱 (单边谱, 与输入电压同量纲):
 *   mag[0]       = |DC| / N
 *   mag[N/2-1]   = |Nyquist| / N
 *   mag[i]       = 2*sqrt(Ri^2+Ii^2) / N   (i=1..N/2-2)
 *============================================================================*/
void FFT_Analysis(uint16_t *adc_raw, uint16_t len)
{
    uint16_t n = (len < Sample_Point) ? len : Sample_Point;
    float32_t dc_level = (float32_t)adc_zero * (V_REF / adc_buf_max);

    /* 1.去直流 + 加Hanning窗 */
    for(int i = 0; i < Sample_Point; i++) {
        if(i < n) {
            float32_t v = (float32_t)adc_raw[i] * (V_REF / adc_buf_max) - dc_level;
            FFT_in[i] = v * hanning_win[i];
        } else {
            FFT_in[i] = 0.0f;               /* 数据不足1024点时补零 */
        }
    }

    /* 2.实数FFT (原地: FFT_in既是输入也是输出) */
    arm_rfft_fast_f32(&rfft_inst, FFT_in, FFT_in, 0);

    /* 3.幅值谱 */
    fft_mag[0] = fabsf(FFT_in[0]) / (float32_t)Sample_Point;         /* DC */

    fft_mag[Sample_Point / 2 - 1] = fabsf(FFT_in[1]) / (float32_t)Sample_Point; /* Nyquist */

    for(int i = 1; i < Sample_Point / 2 - 1; i++) {
        float32_t real = FFT_in[2 * i];
        float32_t imag = FFT_in[2 * i + 1];
        fft_mag[i] = sqrtf(real * real + imag * imag) / ((float32_t)Sample_Point / 2.0f);
    }
}

/*==============================================================================
 * Find_Base_Freq() — 找幅值谱中最大bin对应的频率(基频)
 * 跳过bin 0 (DC分量), 从i=1开始搜索
 * 频率分辨率 = 1 / (fft_sample_interval * N), 例: 100kHz采样时 = 97.7Hz/bin
 * 无bin间插值: 误差上限约半个分辨率(49Hz), 精确测频应使用TIM2硬件捕获
 *============================================================================*/
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

/*==============================================================================
 * FFT_Display() — 两区布局: 左侧柱状图(0~N/4 bin) + 左侧文字(前5次谐波)
 * 柱状图: 每隔一个bin画2px宽绿色柱, 高度=mag*Show_Half/V_REF
 * 文字:   "1H: XXkHz X.XXV" ~ "5H: ..."
 *============================================================================*/
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

/*==============================================================================
 * FFT_Process() — FFT总调度 (由TRI_FLAG检测FFT_flag==1触发)
 * 流程: 初始化窗+RFFT -> 同步采样率 -> 去直流加窗FFT -> 找基频 -> 画频谱
 *============================================================================*/
void FFT_Process(void)
{
    FFT_Init();                                        /* 初始化 (首次耗时~2ms) */
    fft_sample_interval = sample_interval;             /* 同步当前采样率 */
    FFT_Analysis(adc_buffer, Sample_Point);            /* 核心FFT计算 */
    base_freq = Find_Base_Freq();                      /* 找基频 */

    float32_t freq_res = 1.0f / (fft_sample_interval * (float32_t)Sample_Point);
    uint16_t base_idx = (uint16_t)(base_freq / freq_res);

    ILI9341_fill(Show_Left, Show_Top, Show_Right, Show_Bottom, BLACK);
    FFT_Display(base_idx);                             /* 画柱状图+谐波 */
}
void TRI_FLAG(void)
{
  
    // 校正
    if(Correct_flag == 1) {
        system_busy = 1;
        Correct_Process();
        system_busy = 0;
        AUTO_flag = 0;        
        return;                
    }

  
 
    // FFT频谱 
    if(FFT_flag == 1) {
        FFT_flag = 0;
        system_busy = 1;
        FFT_Process();
        system_busy = 0;
    }
    if(FFT_Refresh_flag == 1) {
        FFT_Refresh_flag = 0;
        ILI9341_fill(0,0,358,239,BLACK);
        Show_Data();
        AUTO_flag = 0;
    }
    if(AUTO_T_flag) {
        AUTO_T_flag = 0;
        sprintf(buf, "AUTO mode=%d\r\n", auto_mode);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        switch(auto_mode) {
            case 0:  auto_V_div(ffp); break;                          // 自动V/div (内部先调auto_gain硬件增益, 再算屏幕挡位)
            case 1:  FREQ = Freq_Capture_Get();                         // 自动时基 (TIM2硬件捕获测频, 直接读CCR1)
                     auto_T_div(FREQ); break;
            case 2:  auto_tri_level(adc_display); break;              // 自动触发电平
            case 3:  auto_V_div(ffp);                                 // 全部自动
                     auto_tri_level(adc_display);
                     FREQ = Freq_Capture_Get();                       // TIM2硬件捕获测频
                     auto_T_div(FREQ); break;
            default:  break;
        }
    }
    if(Fre_flag == 1) {
        Fre_flag = 0;
        FREQ = Freq_Capture_Get();                    /* TIM2硬件捕获测频, 替代原软件过零法 */
         
    }

    if(Correct_flag){
        Correct_flag=0;
        Zero_Correct();
        Correct_Process();
    }
    
}


void Zero_Correct(void){
      uint32_t sum=0,cnt=0;
      for(int i=0;i<Sample_Point;i++)
      {
            if(adc_buffer[i]<200)
            {
            sum+=adc_buffer[i];cnt++;}
      }
      adc_zero=sum/cnt;
}
void Correct_Process()
{
      HAL_TIM_Encoder_Stop(&htim12, TIM_CHANNEL_ALL);  // 停编码器
      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
      HAL_Delay(10);

Zero_Correct();

      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, Sample_Point);
      HAL_TIM_Base_Start(&htim2);
      uint32_t timeout = 1000000;  // ~0.5s超时
      while(!ADC_flag && --timeout) {}
      if(timeout == 0) {
          HAL_TIM_Base_Stop(&htim2);
          HAL_ADC_Stop_DMA(&hadc1);
          HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
          HAL_TIM_Encoder_Start(&htim12, TIM_CHANNEL_ALL);
          Correct_flag = 0;
          return;
      }
      ADC_flag = 0;

      ADC_Measure_amp(adc_display);
      if(AMP > 0.001f) {
          adc_amp_grain = adc_std_amp / AMP;
      }
      FREQ = Freq_Capture_Get();                       /* TIM2硬件捕获测频, 替代原软件过零法 */
      if(FREQ > 0.001f) {
          adc_fre_grain = adc_std_fre / FREQ;
      }
      Single_Trig_flag = 0;
     // Single_Trig_flag_2 = 0;
      Correct_flag = 0;
      HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
      HAL_TIM_Encoder_Start(&htim12, TIM_CHANNEL_ALL);  // 恢复编码器
  
				 
}

