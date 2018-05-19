// 0.Documentation Section 
// CECS347FINAL.c
// Runs on LM4F120 or TM4C123
// Authors: Michael Diep, Steven Sallack
// Date: May 14, 2018
// 
// 1.Program Description
// This program is for a wall-following robot. This program requires external components such as  
// the an H-Bridge, voltage regulator, a power supply, and IR sensors. The program will constantly
// poll each sensor to determine if the robot needs to adjust its course. This robot is made to
// stay in the middle of its course. The IR sensors can handle 10-70cm each side meaning that it could
// try to stay in the middle which each side would be roughly a maximum of 60cm. This robot is also capable 
// of making 90 degree turns towards the left and right. When the robot is out of its course and all the sensors
// are out of range, the robot will stop. 

// 2.LaunchPad built-in hardware
// PB6 will control PWM of motor 1
// PB7 will control PWM of motor 2
// PC4,5 will control the movement direction bit of motor 1
// PC6,7 will control the movement direction bit of motor 2
// PE1 connected to forward facing IR distance sensor
// PE0 connected to forward right IR distance sensor
// PE5 connected to forward left IR distance sensor
// 
// Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

#include <stdint.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "ADCSWTrigger.h"

#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  					// Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  					// Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  					// Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  					// Set the output signal to 0


#define SYSCTL_RCC_USEPWMDIV    0x00100000  					// Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  					// PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  					// /2

#define FORWARD  0xA0																	// Wheels move forward	
#define BACKWARD 0x50																	// Wheels move backward

#define PWMMINL 4200																	// Minimum PWM L value when turning left 
#define PWMMAXL 5800																	// Maximum PWM L value when turning right 

#define PWMMINR 3600																	// Minimum PWM R value when turning right 
#define PWMMAXR 5600																	// Maxumum PWM R value when turning left 

#define STEADYL 5800																	// Steady, normal pace for L PWM
#define STEADYR 5300 																	// Steady, normal pace for R PWM

#define JUMPL   7500																	// Jump start L PWM after done making hard turn
#define JUMPR   5900																	// Jump start R PWM after done making hard turn

#define STOP 	  0000																	// Zero PWM

#define HARDR 	6700																	// PWM R when making a 90 degree left turn
#define HARDL	  7200																	// PWM L when making a 90 degree left turn

// Function Prototypes
void stop(void);
void wait10ms(int n);
void adjustLeft(void);
void adjustRight(void);
void hardLeft(void);
void hardRight(void);
void Motor_Init(void);
void EnableInterrupts(void);
void DisableInterrupts(void);  
void WaitForInterrupt(void);
void Nokia5110_Init(void);
void ADC_Init298(void);
void ADC_In298(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8);
double Find_Distance(unsigned long adc_value);
double Find_DistanceL(unsigned long adc_value);

	
long  duty_cycle = 0; 																// variable to control left wheel duty cycle
long R_duty_cycle = 0; 																// variable to control right wheel duty cycle
unsigned long PWM_Percent_R = 0;											// PWM percentage to display on LCD
unsigned long PWM_Percent_L = 0;											// PWM percentage to display on LCD
long error;
double frwddist, leftdist, rightdist;

// SYSTICK INIT START
void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EnableInterrupts();
}

// SYSTICK INIT END

// PWM INIT START 
void PWM0A_Init(uint16_t period, uint16_t duty){ volatile unsigned long delay;
	volatile unsigned long delay2;
	SYSCTL_RCGC2_R |= 0x00000002;     									// B clock
  delay = SYSCTL_RCGC2_R;	// delay   
  SYSCTL_RCGCPWM_R |= 0x01;             							// 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            							// 2) activate port B
  delay2 = SYSCTL_RCGCGPIO_R;
  GPIO_PORTB_AFSEL_R |= 0x40;           							// enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     							// configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          							// disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x40;             							// enable digital I/O on PB6
  SYSCTL_RCC_R = 0x00100000 |           							// 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   							//    configure for /2 divider
  PWM0_0_CTL_R = 0;                     							// 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 							// low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = period - 1;           							// 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty - 1;             							// 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           							// 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          							// enable PB6/M0PWM0
}
void PWM0A_Duty(uint16_t duty){
  PWM0_0_CMPA_R = duty - 1;             							// 6) count value when output rises
}
void PWM0B_Init(uint16_t period, uint16_t duty){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x01;             							// 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            							// 2) activate port B
  delay = SYSCTL_RCGCGPIO_R;            							// allow time to finish activating
  GPIO_PORTB_AFSEL_R |= 0x80;           							// enable alt funct on PB7
  GPIO_PORTB_PCTL_R &= ~0xF0000000;     							// configure PB7 as M0PWM1
  GPIO_PORTB_PCTL_R |= 0x40000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;          							// disable analog functionality on PB7
  GPIO_PORTB_DEN_R |= 0x80;             							// enable digital I/O on PB7
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; 							// 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; 							//    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  							//    configure for /2 divider
  PWM0_0_CTL_R = 0;                     							// 4) re-loading down-counting mode
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = period - 1;           							// 5) cycles needed to count down to 0
  PWM0_0_CMPB_R = duty - 1;             							// 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           							// 7) start PWM0
  PWM0_ENABLE_R |= 0x00000002;          							// enable PB7/M0PWM1
}

void PWM0B_Duty(uint16_t duty){
  PWM0_0_CMPB_R = duty - 1;             							// 6) count value when output rises
}
// PWM INIT END 

// MOTOR INIT START
void Motor_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000004; 											// activate clock for port C
	delay = SYSCTL_RCGC2_R;
  GPIO_PORTC_AMSEL_R &= ~0xF0;    	  								// disable analog functionality on PC4,PC5,PC6,PC7 
  GPIO_PORTC_PCTL_R &= ~0xFFFF0000; 									// configure PC4,PC5,PC6,PC7 as GPIO
  GPIO_PORTC_DIR_R |= 0xF0;     											// make PC4,PC5,PC6,PC7 out
  GPIO_PORTC_DR8R_R |= 0xF0;    											// enable 8 mA drive on PC4,PC5,PC6,PC7 
  GPIO_PORTC_AFSEL_R &= ~0xF0;  											// disable alt funct on PC4,PC5,PC6,PC7
  GPIO_PORTC_DEN_R |= 0xF0;     											// enable digital I/O on PC4,PC5,PC6,PC7 
  GPIO_PORTC_DATA_R = FORWARD;   											// make PC4,PC5,PC6,PC7 go forward on start-up (bit 7654 3210)
																											//																			backward	    1010 0000
																											//																			forward   		0101 0000
	duty_cycle = STEADYL; 								  					  
  R_duty_cycle = STEADYR; 														
}
// MOTOR INIT END


// ADC FILTER INIT
unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3){
unsigned long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}
void ReadADCMedianFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){
  //                   x(n-2)        x(n-1)
  static unsigned long ain2oldest=0, ain2middle=0;
  static unsigned long ain9oldest=0, ain9middle=0;
  static unsigned long ain8oldest=0, ain8middle=0;

  // save some memory; these do not need to be 'static'
  //            x(n)
  unsigned long ain2newest;
  unsigned long ain9newest;
  unsigned long ain8newest;

  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);

  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest; 
}
// ADC FILTER END

double Find_Distance(unsigned long adc_value){
	double distance_value;
	double base = adc_value;
	double a, b;
	a = 20628;
	b = 1/(pow(base, 1.078));
	distance_value = a * b;
	
	if ((distance_value < 0) ||  (distance_value > 70))
	{
		distance_value = 70;
	}

	return distance_value;
}

// Since our left IR sensor was attained differently,
// it receives different values as inputs, thus we need 
// a different converter.
double Find_DistanceL(unsigned long adc_value){
	double distance_value;
	double base = adc_value;
	double a, b;
	a = 222052;
	b = 1/(pow(base, 1.276));
	distance_value = a * b;
	
	if ((distance_value < 0) ||  (distance_value > 70))
	{
		distance_value = 70;
	}

	return distance_value;
}



// Delay function
int i;
void wait10ms(int n) {unsigned long volatile time;
	
  for (i = 0; i < n; i++){
	time = 727240*200/910;  // 0.01sec
  while(time){
		time--;}}
};


// COURSE ADJUSTMENT FUNCTIONS START	
void adjustLeft(void){
	//turn left 
	R_duty_cycle = PWMMAXR;
	duty_cycle = PWMMINL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait .5s
	if (rightdist < 8)
	wait10ms(6); else
	wait10ms(5);
	//turn right
	R_duty_cycle = PWMMINR;
	duty_cycle = PWMMAXL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait .5s
	wait10ms(4);
	//go straight
	R_duty_cycle = STEADYR;
	duty_cycle = STEADYL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
}
void adjustRight(void){
	//turn right
	R_duty_cycle = PWMMINR;
	duty_cycle = PWMMAXL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait .5s
	if (leftdist < 8)
	wait10ms(7); else
	wait10ms(6);
	//turn left
	R_duty_cycle = PWMMAXR;
	duty_cycle = PWMMINL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait .5s
	wait10ms(4);
	//go straight
	R_duty_cycle = STEADYR;
	duty_cycle = STEADYL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
}
void hardLeft(void){
	//stop (for debugging)
	R_duty_cycle = STOP;
	duty_cycle = STOP;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	wait10ms(5);
	//90 degree left turn
	GPIO_PORTC_DATA_R = BACKWARD; 
	R_duty_cycle = STOP;
	duty_cycle = HARDL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	
	//wait for turn to complete
	wait10ms(8);
	//continue straight jump start
	GPIO_PORTC_DATA_R = FORWARD; 
	R_duty_cycle = JUMPR;
	duty_cycle = JUMPL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	wait10ms(5);
	//continue straight 
	R_duty_cycle = STEADYR;
	duty_cycle = STEADYL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait unsure amount
	//wait10ms(5);
}
void hardRight(void){
	//stop (for debugging)
	R_duty_cycle = STOP;
	duty_cycle = STOP;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	wait10ms(4);
	//90 degree right turn
	GPIO_PORTC_DATA_R = BACKWARD; 
	R_duty_cycle = HARDR;
	duty_cycle = STOP;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	
	//wait for turn to complete
	wait10ms(7);
	R_duty_cycle = STOP;
	duty_cycle = STOP;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	wait10ms(4);
	//continue straight jump start
  GPIO_PORTC_DATA_R = FORWARD; 	
	R_duty_cycle = JUMPR + 1500;
	duty_cycle = JUMPL - 800;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	wait10ms(5);
	//continue
	R_duty_cycle = STEADYR;
	duty_cycle = STEADYL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	//wait unsure amount
	//wait10ms(5);
}

void stop(void){
	R_duty_cycle = STOP;
	duty_cycle = STOP;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	PWM_Percent_R = (R_duty_cycle/200);
	PWM_Percent_L = (duty_cycle/200); 
	wait10ms(500);
}
// COURSE ADJUSTMENT FUNCTIONS END



// Interrupt service routine
// Executed every 62.5ns*(period)
	unsigned long frwdleft, frwdright, ahead, pot;
void SysTick_Handler(void){
	
	  ReadADCMedianFilter( &ahead, &frwdright, &frwdleft);
	  leftdist = Find_DistanceL(frwdleft);
	  rightdist = Find_Distance(frwdright);
	  frwddist = Find_Distance(ahead);
	  
		error = leftdist - rightdist;
	
}


int main(void){
	DisableInterrupts();  															// Disable interrupts while initializing	
	SysTick_Init(80000);																// Initialize SysTick interrupt w/ 80Hz
  ADC_Init298(); 														  				// Initialize ADC 
	PWM0A_Init(20000, 0);         											// Initialize PWM0A, 2.5ms period, 0% duty
  PWM0B_Init(20000, 0); 															// Initialize PWM0B, 2.5ms period, 0% duty
	Motor_Init();																				// Initialize DC Motor
	Nokia5110_Init();																		// Initialize Nokia5110 
  Nokia5110_Clear();												  				// Clear LCD Screen
	EnableInterrupts();  																// Enable SysTick
	

  while(1){	
		
	// If all sensors are out of range, stop the robot	
  if ((frwddist == 70) && (leftdist == 70) && (rightdist==70)) {
		wait10ms(10);
		if ((frwddist == 70) && (leftdist == 70) && (rightdist==70))
		stop();
	}
	// if front object is close and there is an object to the left
	else if ((frwddist < 11) && (error < 20))
		hardRight();
	// if front object is close and there is an object to the right
	else if ((frwddist < 11) && (error > 20))
		hardLeft();
	
  else if (error > 45) 
		  adjustLeft(); 
	else if (error < -45) 
		  adjustRight(); 
	
	// else stay on the course
	else {
	R_duty_cycle = STEADYR;
	duty_cycle = STEADYL;
	PWM0A_Duty(R_duty_cycle);							
	PWM0B_Duty(duty_cycle);
	}
	
	PWM_Percent_R = (R_duty_cycle/200); 								// convert R duty cycle to a percentage to display
	PWM_Percent_L = (duty_cycle/200); 									// convert L duty cycle to a percentage to display		
		
		Nokia5110_SetCursor(0, 0);          							// zero leading spaces, first row
		Nokia5110_OutString("ADC_L:");
		//Nokia5110_OutUDec(frwdleft);
		Nokia5110_OutUDec(leftdist);
		Nokia5110_SetCursor(0, 1);          							// zero leading spaces, second row
		Nokia5110_OutString("ADC_F:");		
		Nokia5110_OutUDec(frwddist);
    Nokia5110_SetCursor(0, 2);          							// zero leading spaces, third row
		Nokia5110_OutString("ADC_R:");		
		Nokia5110_OutUDec(rightdist);
		//Nokia5110_OutUDec(frwdright);
    Nokia5110_SetCursor(0, 3);          							// zero leading spaces, fourth row
		Nokia5110_OutString("PWM_R:");				
		Nokia5110_OutUDec(PWM_Percent_R);
		Nokia5110_SetCursor(0,4);													// zero leading spaces, fifth row
		Nokia5110_OutString("PWM_L:");
		Nokia5110_OutUDec(PWM_Percent_L);
		Nokia5110_SetCursor(0,5);													// zero leading spaces, six row
		Nokia5110_OutString("error");
		Nokia5110_OutUDec(error);



  }
}

