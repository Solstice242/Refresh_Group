#include "Basic.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "gpio.h"
#include "ili9341_driver.h"
#include "usart.h"
/*--------前端操作------------
1.自动控制增益电路选择（根据幅度-幅值实时测量）
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
float V_Grain[8]={0.129,0.26,0.496,0.992,1.984,3.931,7.9,15.74};//放大或衰减倍数
uint8_t Grain_idx=2;
float adc_grain=0.496f;

//触发电平
float step_1[2]={0.0025,0.025};
 uint8_t step_dix=1;
uint16_t last_count_1=0;
uint16_t adc_zero=8192;//抬升
float tri_step=0.0025f;
 uint16_t  dac_val=0;

/*三个GPIO控制增益通路选择*/
void Chose_Grain(uint8_t idx)
{
    adc_grain = V_Grain[idx];
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
变量：输入数组，改变幅值
找峰谷值、剔除毛刺（中值滤波）--换算电压--校正
*/
void ADC_Measure_amp(uint16_t *src)
{
    uint16_t min_v = 65535, max_v = 0;
    for (int i = 0; i < Display_Point; i++) {
        if (src[i] < min_v) min_v = src[i];
        if (src[i] > max_v) max_v = src[i];
    }
    ffp = (max_v - min_v) * ADC_LSB;
    float vol = ffp / adc_grain;
    AMP = vol * adc_amp_grain;
    ILI9341_draw_string(rectangle_Left+2,91,"10.1",BLACK);
     sprintf(line2, " %.1fV", AMP);
     
   ILI9341_draw_string(rectangle_Left+2,91,line2,GRED);
}

/*
自动切换增益挡位：
变量：输入幅值  改变增益索引、增益、GPIO状态 
过滤无效信号-计算理想增益-用差值找到合适挡位-滞回防抖-执行切换
*/
void auto_gain(float adc_vpp)
{
    if(adc_vpp < 0.005f) return; 

    float actual_ffp = adc_vpp / adc_grain; 
    if(actual_ffp < 0.0005f) return;
    float target_adc = 1.5f;                 /* 目标Vpp: 约50%满幅, 上下各留一半余量 */
    float ideal_gain = target_adc / actual_ffp;

    //限幅增益范围 
    if(ideal_gain < V_Grain[0])  ideal_gain = V_Grain[0];
    if(ideal_gain > V_Grain[7])  ideal_gain = V_Grain[7];

    uint8_t best_idx= 0;
    float min_diff = fabs(ideal_gain - V_Grain[0]);
    for(uint8_t i = 1; i < 8; i++) {
        float diff = fabs(ideal_gain - V_Grain[i]);
        if(diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }

    if(best_idx == Grain_idx) return;            
//滞回防抖，设置死区率0.35，防反复跳变
    int8_t gap = (int8_t)best_idx - (int8_t)Grain_idx;
    if(gap == 1 || gap == -1) {
        float rate = ideal_gain / V_Grain[Grain_idx];
        if(rate > 0.65f && rate < 1.35f) return;
    }

    Grain_idx = best_idx;
    Chose_Grain(best_idx);
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
void TRI_Scan(void)
{
    int16_t now=__HAL_TIM_GET_COUNTER(&htim4);
    int16_t diff=now-last_count_1;
    if(diff!=0)
    {
        LEVEL+=diff*tri_step;
       float shift_vol=LEVEL*adc_grain+adc_zero*ADC_LSB ;//从输入端转换到比较器端
       if(shift_vol<0.0) shift_vol=0.0f;
       if(shift_vol>3.3f) shift_vol=3.3f;
     dac_val = (uint16_t)((shift_vol / V_REF) * dac_buf_max);
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
            //if(system_busy || AUTO_flag || Single_Trig_flag) return;
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
  menu_active = !menu_active;   // 切换菜单活跃状态
  if(menu_active) {
      now_menu_idx = menu_idx; // 进入模式，记录当前索引
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











