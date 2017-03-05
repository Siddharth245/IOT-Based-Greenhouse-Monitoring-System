#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.c"
#include <stdio.h>
#include <math.h>
#include "adc.c"
#define wet 1300.0f
#define dry 3450.0f
#define temp_max 32.0f
#define temp_min 21.0f
#define temp_diff 11.0f
#define tread_max 1300.0f
#define tread_min 1020.0f
#define tread_diff 180.0f
#define temp_thresh 26

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t mmsTicks; // counts 1 ms timeTicks
// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void initialise_monitor_handles();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
uint32_t motor = 100;

void SysTick_Handler(void){
  mmsTicks++;
}

// initialize the system tick
void init_systick(void)
{
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 100000)) { /* SysTick 0.01 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n)
{
  uint32_t mmsTicks2 = mmsTicks + 100*n;
  while(mmsTicks < mmsTicks2) ;
}

void delay_mms(uint32_t n)
{
  uint32_t mmsTicks2 = mmsTicks + n; // multiply by 10 since mmsTicks is counting 0.1 ms timeTicks
  while(mmsTicks < mmsTicks2) ;
}


struct TimedTask
{
	void (*func_ptr)(void);
	float rep_time_interval;
	uint32_t last_call;
	int count;
}
timed_tasks[10];

uint32_t array_ind=0;

void add_timed_task(void (*func_ptr)(void), float rep_time_interval)
{
	timed_tasks[array_ind].func_ptr = func_ptr;
	timed_tasks[array_ind].rep_time_interval = rep_time_interval;
	timed_tasks[array_ind].last_call = msTicks;
	timed_tasks[array_ind].count = 0;
	array_ind++;
}





void init_pin()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA
  GPIOA->MODER &= ~(0x3 << (2*0));
  GPIOA->MODER &= ~(0x3 << (2*7)); // clear the 2 bits corresponding to pin i
  GPIOA->MODER |= (1 << (2*7));  
}

void init_Servo_pin()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD
    GPIOD->MODER &= ~(0x3 << (2*1)); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*1));    // set pin i to be general purpose output
}

void servoRotate(uint32_t num) // Angle of rotation
{
  unsigned int i;
  for(i=0;i<20;i++)
  {
    GPIOD->BSRRL = 1 << (1) ;
    delay_mms(num);
    GPIOD->BSRRH = 1 << (1) ;
    delay_mms(2000-num);
  }
}

int power(int base, int exp)
{
    int result = 1;
    while(exp) { result *= base; exp--; }
    return result;
}


void SystemRotate()
{
		
		servoRotate(motor);
		if ((ADC3ConvertedValue[1] - ADC3ConvertedValue[0]) > 100)
		{
			printf("Sun is in the East. Adjusting System...  (%d) \n", ADC3ConvertedValue[1]);
			if(motor>=50)
			{
  				motor -= 15;   			
				servoRotate(motor);
   				//delay_mms(1000);
			}
		}
		else if((ADC3ConvertedValue[0] - ADC3ConvertedValue[1]) > 100)
		{
			printf("Sun is in the West. Adjusting System...  (%d) \n", ADC3ConvertedValue[0]);
			if(motor<=200)			
			{
				motor += 15;
   				servoRotate(motor) ;
   				//delay_mms(1000);
			}
		}
		else
		{
			printf("The System is aligned with Sun\n");
		}
}

void MoistPerc()
{
	float mope = ((dry - ADC3ConvertedValue[3])/(dry-wet))*100;
	printf("Moisture Content : %.2f%% \n ", mope, ADC3ConvertedValue[3]);	
}

void Temp_Sense()
{
	float temp = (((ADC3ConvertedValue[2]-tread_min)/(tread_diff))*temp_diff) + temp_min;
	if (temp > temp_thresh)
		{	
			printf("Temperature is %.2fC. The Fan is On\n", temp);	
			GPIO_SetBits(GPIOD, GPIO_Pin_5);// dc motor 	
		}
		else
		{
			printf("Temperature is %.2fC. The Fan is Off\n", temp);
			GPIO_ResetBits(GPIOD, GPIO_Pin_5);
		}
	
}


int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_Servo_pin();
  init_pin();
  init_Port_C();
  printf("Input Starts Here: \n");
	servoRotate(100);//initialise the position of the structure
	while(1)
	{	
		MoistPerc();
		SystemRotate();	
		Temp_Sense();
		//delay_ms(2000);
	}

	return 1;
}



