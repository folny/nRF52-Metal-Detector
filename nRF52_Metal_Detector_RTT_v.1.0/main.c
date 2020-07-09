
#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "math.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_gpiote.h"
#include "SEGGER_RTT.h" 

//J-Link RTT Viewer Commands: 1 = Ennable, 0 = Dissable Power / r = nRF Reset / c = Calibrate / + = Tx Phase + / - = Tx Phase - / F = Tx Frequency + / f = Tx Frequency - (default 11.36kHz) / G = Groundbalanc + / g = Groundbalanc - / p =  Static/Dynamic Mode 

int Power = 0; 

int Frequency = 350; //Set Frequency 11.36kHz
int Phase_Set = 163; //Set TX Phase (0 - Frequency/2 Max!)
		
double GB_Set = 89.90;

int PP = 0; 

#define Tx_Coil           NRF_GPIO_PIN_MAP(0,3)
#define Power_Controll    NRF_GPIO_PIN_MAP(0,28)
#define Status_LED        NRF_GPIO_PIN_MAP(1,11)

#define GPIOTE_TASK(ch, pin, initstate) do { NRF_GPIOTE->CONFIG[ch] = ((3UL << 0) | ((pin) << 8) | (3UL << 16) | ((initstate) << 20)); } while (0)
	
#define SAMPLES 4
	
#define SAMPLES_IN_BUFFER 1

static nrf_saadc_value_t       m_buffer_pool[1][SAMPLES_IN_BUFFER];

int Calibrate = 0;
int CalibCtr  = 0;
int16_t VDI;
double X = 0.00; 
double Y = 0.00;
double X_Tmp = 0.00;
double Y_Tmp = 0.00;
double X_Filtered = 0.00;
double Y_Filtered = 0.00;	
double GEB_Ch = 0.00;	
float ADC_Data = 0.00;
int16_t ctr = -1;  
int16_t bins[SAMPLES]; 
int16_t calib[SAMPLES];
const double radiansToDegrees = 180.0/3.14;
volatile int16_t averages[SAMPLES];
volatile bool sampleReady = false;

double bin0 = 0.00;
double bin1 = 0.00;
double bin2 = 0.00;
double bin3 = 0.00;

float BandPass_a0 =  1.000000000000000000;
float BandPass_a1 = -0.000000000000000222;
float BandPass_a2 =  0.999613800419481557;
float BandPass_b0 =  0.000216661528192711;
float BandPass_b1 =  0.000000000000000000;
float BandPass_b2 = -0.000216661528192711;

float BandPass_X1, BandPass_X2, BandPass_Y1, BandPass_Y2; 

float IIR_BandPass_Filter(float x)
{
 float y, CenterTap;

 CenterTap = x * BandPass_b0 + BandPass_b1 * BandPass_X1 + BandPass_b2 * BandPass_X2;
 y = BandPass_a0 * CenterTap - BandPass_a1 * BandPass_Y1 - BandPass_a2 * BandPass_Y2;

 BandPass_X2 = BandPass_X1;
 BandPass_X1 = x;
 BandPass_Y2 = BandPass_Y1;
 BandPass_Y1 = y;

 return (float)y;
}

float XY_HPF_a0 =  1.000000000000000000;
float XY_HPF_a1 = -0.939062505817492399;
float XY_HPF_a2 =  0.000000000000000000;
float XY_HPF_b0 =  2.435352399105962910;
float XY_HPF_b1 = -2.435352399105962910;
float XY_HPF_b2 =  0.000000000000000000;

float X_HPF_X1, X_HPF_X2, X_HPF_Y1, X_HPF_Y2; 
float Y_HPF_X1, Y_HPF_X2, Y_HPF_Y1, Y_HPF_Y2; 

double IIR_X_HPF_Filter(double x){
	
 double y, CenterTap;

 CenterTap = x * XY_HPF_b0 + XY_HPF_b1 * X_HPF_X1 + XY_HPF_b2 * X_HPF_X2;
 y = XY_HPF_a0 * CenterTap - XY_HPF_a1 * X_HPF_Y1 - XY_HPF_a2 * X_HPF_Y2;

 X_HPF_X2 = X_HPF_X1;
 X_HPF_X1 = x;
 X_HPF_Y2 = X_HPF_Y1;
 X_HPF_Y1 = y;

 return (double)y;
}

double IIR_Y_HPF_Filter(double x)
{
 double y, CenterTap;

 CenterTap = x * XY_HPF_b0 + XY_HPF_b1 * Y_HPF_X1 + XY_HPF_b2 * Y_HPF_X2;
 y = XY_HPF_a0 * CenterTap - XY_HPF_a1 * Y_HPF_Y1 - XY_HPF_a2 * Y_HPF_Y2;

 Y_HPF_X2 = Y_HPF_X1;
 Y_HPF_X1 = x;
 Y_HPF_Y2 = Y_HPF_Y1;
 Y_HPF_Y1 = y;

 return (double)y;
}

float XY_LPF_a0 =  1.000000000000000000;
float XY_LPF_a1 = -0.369527377351241304;
float XY_LPF_a2 =  0.195815712655833113;
float XY_LPF_b0 =  0.206572083826147945;
float XY_LPF_b1 =  0.413144167652295891;
float XY_LPF_b2 =  0.206572083826147945;

float X_LPF_X1, X_LPF_X2, X_LPF_Y1, X_LPF_Y2; 
float Y_LPF_X1, Y_LPF_X2, Y_LPF_Y1, Y_LPF_Y2; 

double IIR_X_LPF_Filter(double x){
	
 double y, CenterTap;

 CenterTap = x * XY_LPF_b0 + XY_LPF_b1 * X_LPF_X1 + XY_LPF_b2 * X_LPF_X2;
 y = XY_LPF_a0 * CenterTap - XY_LPF_a1 * X_LPF_Y1 - XY_LPF_a2 * X_LPF_Y2;

 X_LPF_X2 = X_LPF_X1;
 X_LPF_X1 = x;
 X_LPF_Y2 = X_LPF_Y1;
 X_LPF_Y1 = y;

 return (double)y;
}

double IIR_Y_LPF_Filter(double x){
	
 double y, CenterTap;

 CenterTap = x * XY_LPF_b0 + XY_LPF_b1 * Y_LPF_X1 + XY_LPF_b2 * Y_LPF_X2;
 y = XY_LPF_a0 * CenterTap - XY_LPF_a1 * Y_LPF_Y1 - XY_LPF_a2 * Y_LPF_Y2;

 Y_LPF_X2 = Y_LPF_X1;
 Y_LPF_X1 = x;
 Y_LPF_Y2 = Y_LPF_Y1;
 Y_LPF_Y1 = y;

 return (double)y;
}

void timers_init(int frequency, int tx_phase)
{		
	  NRF_TIMER3->BITMODE                 = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER               = 2;
    NRF_TIMER3->SHORTS                  = TIMER_SHORTS_COMPARE0_CLEAR_Msk << 0;
    NRF_TIMER3->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	  NRF_TIMER3->CC[0] = frequency;
	  NRF_TIMER3->CC[2] = tx_phase + 1;
    NRF_TIMER3->CC[1] = ((tx_phase + 1) + frequency/2) % frequency;   
	
    NRF_TIMER4->BITMODE                 = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER4->PRESCALER               = 0;
    NRF_TIMER4->SHORTS                  = TIMER_SHORTS_COMPARE0_CLEAR_Msk << 0;
    NRF_TIMER4->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	  NRF_TIMER4->CC[0] = frequency;
 	
    NRF_TIMER4->TASKS_START = 1;		
    NRF_TIMER3->TASKS_START = 1;              
}
    
void hf_clock_initialization( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

void lf_clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if(p_event->type == NRF_DRV_SAADC_EVT_DONE){
		
    ctr++;
			
    nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER); 
		
	  ADC_Data = p_event->data.done.p_buffer[0];
    bins[ctr] = (double)IIR_BandPass_Filter(ADC_Data);
	
 if(ctr == SAMPLES -1){

 if(Calibrate == 1){	
		calib[0] = bins[0];
		calib[1] = bins[1];
		calib[2] = bins[2];
		calib[3] = bins[3];
	  Calibrate = 0;
		}else{
	  if (!sampleReady){
    bin0 = ((bins[0]-calib[0]) + ((bins[1]-calib[1]) - (bins[3]-calib[3])));
    bin1 = ((bins[1]-calib[1]) + ((bins[0]-calib[0]) + (bins[2]-calib[2])));
    bin2 = ((bins[2]-calib[2]) + ((bins[1]-calib[1]) + (bins[3]-calib[3])));
    bin3 = ((bins[3]-calib[3]) + ((bins[2]-calib[2]) - (bins[0]-calib[0]))); 
	  sampleReady = true;
    }
    }
		ctr = -1;
   } 			
  }
}

void rx_saadc_init(void) {
 				
	  nrf_drv_saadc_config_t rx_saadc_config;
    nrf_saadc_channel_config_t rx_channel_config;
	
    //Rx Configure SAADC
    rx_saadc_config.low_power_mode = false;                                                    
    rx_saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                
	  rx_saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED; 
    rx_saadc_config.interrupt_priority = APP_IRQ_PRIORITY_HIGH;                              
	
    nrf_drv_saadc_init(&rx_saadc_config,saadc_callback);                       
		
    rx_channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              
    rx_channel_config.gain = NRF_SAADC_GAIN4;  
    rx_channel_config.burst = NRF_SAADC_BURST_DISABLED;		
    rx_channel_config.acq_time = NRF_SAADC_ACQTIME_3US;                                     
    rx_channel_config.mode = NRF_SAADC_MODE_DIFFERENTIAL;                                    
    rx_channel_config.pin_p = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN7;                                         
    rx_channel_config.pin_n = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN6;                                      
    rx_channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                               
    rx_channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
		
		nrf_drv_saadc_channel_init(0, &rx_channel_config);				
	  nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
}

void gpiote_init(void){
	
    GPIOTE_TASK(1, Tx_Coil, 0);
	
	  NRF_PPI->CH[1].EEP =(uint32_t)& NRF_TIMER3->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[1];
    NRF_PPI->CHENSET = (1UL << 1);
    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[2];
    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[1];
    NRF_PPI->CHENSET = (1UL << 2);
	
	  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;
    NRF_PPI->CHENSET = (1UL << 0);
}

int main(void)
{
	  NRF_POWER->DCDCEN = true;
	
	  lf_clock_initialization();
    hf_clock_initialization();
	
		NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	  nrf_gpio_cfg_output(Tx_Coil);
	  nrf_gpio_cfg_output(Power_Controll);
	  nrf_gpio_pin_write(Power_Controll,0);
	  nrf_gpio_cfg_output(Status_LED);
	  nrf_gpio_pin_write(Status_LED,0);
								
	  timers_init(Frequency,Phase_Set);
		
	  gpiote_init();	
	  rx_saadc_init();
	
	  app_timer_init();			
	  bsp_init(BSP_INIT_LEDS, NULL);
	
	  bsp_indication_set(BSP_INDICATE_ADVERTISING);  
		
	  NRF_TIMER4->TASKS_START = 0;		
    NRF_TIMER3->TASKS_START = 0; 
	  SEGGER_RTT_WriteString(0,"Config Ready... Power Disabled\n");
		nrf_delay_ms(200);

while(1){  	
		
    char RTT_Data = 0;

	  RTT_Data = SEGGER_RTT_GetKey(); 

    if(RTT_Data == '1'){
	  Power = 1;
	  bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
    nrf_gpio_pin_write(Power_Controll,1);	
	  NRF_TIMER4->TASKS_START = 1;		
    NRF_TIMER3->TASKS_START = 1;
    CalibCtr  = 0;	
    SEGGER_RTT_WriteString(0,"Config Ready... Power Enabled\n");			
	  nrf_delay_ms(200);
	  }

    if(RTT_Data == '0'){
	  Power = 0;
	  bsp_indication_set(BSP_INDICATE_ADVERTISING);
	  nrf_gpio_pin_write(Power_Controll,0);			
	  NRF_TIMER4->TASKS_START = 0;		
    NRF_TIMER3->TASKS_START = 0; 
	  SEGGER_RTT_WriteString(0,"Config Ready... Power Disabled\n");
    nrf_delay_ms(200);;
	  }	
			
	  if(RTT_Data == 'r'){
	  SEGGER_RTT_WriteString(0,"nRF Restart\n");
    NVIC_SystemReset();
	  }
			
	  if(RTT_Data == 'c'){
	  SEGGER_RTT_WriteString(0,"Calibrate\n");
	  Calibrate = 1;
	  }
	
    if(RTT_Data == '+'){
   	SEGGER_RTT_printf(0,"TX Phase:%d\n",(int)Phase_Set);
	  Phase_Set++;
    NRF_TIMER3->CC[2] = Phase_Set + 1;
    NRF_TIMER3->CC[1] = ((Phase_Set + 1) + Frequency/2) % Frequency;
	  CalibCtr  = 0;
	  }
	
    if(RTT_Data == '-'){
	  SEGGER_RTT_printf(0,"TX Phase:%d\n",(int)Phase_Set);
	  Phase_Set--;
	  NRF_TIMER3->CC[2] = Phase_Set + 1;
    NRF_TIMER3->CC[1] = ((Phase_Set + 1) + Frequency/2) % Frequency;
	  CalibCtr  = 0;
	  }
	
	  if(RTT_Data == 'F'){
	  SEGGER_RTT_printf(0,"Frequency:%d\n",(int)Frequency);
	  Frequency++;
    NRF_TIMER3->CC[0] = Frequency;
    NRF_TIMER3->CC[2] = Phase_Set + 1;
    NRF_TIMER3->CC[1] = ((Phase_Set + 1) + Frequency/2) % Frequency;
  	NRF_TIMER4->CC[0] = Frequency;
  	CalibCtr  = 0;
	  }
	
    if(RTT_Data == 'f'){
	  SEGGER_RTT_printf(0,"Frequency:%d\n",(int)Frequency);
	  Frequency--;
    NRF_TIMER3->CC[0] = Frequency;
    NRF_TIMER3->CC[2] = Phase_Set + 1;
    NRF_TIMER3->CC[1] = ((Phase_Set + 1) + Frequency/2) % Frequency;
   	NRF_TIMER4->CC[0] = Frequency;
  	CalibCtr  = 0;
  	}
	
    if(RTT_Data == 'G'){
   	GB_Set += 0.1;
    SEGGER_RTT_printf(0,"GB:%d\n",(int)GB_Set);
	  }
	
    if(RTT_Data == 'g'){
	  GB_Set -= 0.1;
    SEGGER_RTT_printf(0,"GB:%d\n",(int)GB_Set);
	  }
	
	  if(RTT_Data == 'p'){
    if(PP == 1){
	  PP = 0;
    SEGGER_RTT_WriteString(0,"Dynamic Mode\n");
	  }else{
   	PP = 1;
    SEGGER_RTT_WriteString(0,"Static Mode\n");
    Calibrate = 1;
	}
}
	  if(Power == 1){
			
	  if(CalibCtr++ == 20){
		Calibrate = 1;	
		}
				
	  if(PP == 1){
	  X = (bin0 + bin2);
	  Y = (bin1 + bin3);
		}else{
	  X = IIR_X_HPF_Filter(bin0 + bin2);
	  Y = IIR_Y_HPF_Filter(bin1 + bin3);	
		}

	  X_Filtered = IIR_X_LPF_Filter(X);
    Y_Filtered = IIR_Y_LPF_Filter(Y);
	  sampleReady = false;	
				
	  GEB_Ch = (X_Filtered - (0.9-((double)GB_Set/100)) * Y_Filtered);	
		
    if(GEB_Ch >= 5){
	  VDI = (atan2(Y_Filtered - Y_Tmp,X_Filtered - X_Tmp) * radiansToDegrees);	 
	  if(GEB_Ch >= 999){
	  SEGGER_RTT_printf(0,"Amp:%d VDI:%d\n",1000,(int)VDI);
		}else{
	  SEGGER_RTT_printf(0,"Amp:%d VDI:%d\n",(int)GEB_Ch,(int)VDI);
		}
		if(GEB_Ch >= 999){
	  SEGGER_RTT_printf(0,"Amp:%d VDI:%d\n",1000,(int)VDI);
		}else{
	  SEGGER_RTT_printf(0,"Amp:%d VDI:%d\n",(int)GEB_Ch,(int)VDI);	
		}
		}else{
    X_Tmp = X_Filtered;
    Y_Tmp = Y_Filtered;
		if(GEB_Ch <= -1){
    SEGGER_RTT_printf(0,"Amp:%d \n",0);
		}else{
	  SEGGER_RTT_printf(0,"Amp:%d \n",(int)GEB_Ch);	
		}
	 }
	}
  nrf_delay_ms(16);
 }
}
