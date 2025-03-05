/*  Time of Flight for 2DX4 -- Studio W8-0 -- 2DX3 Final Project
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE
						
						Adapted April 1st, 2024
						by Salini Sivalingam
						- adapted for final project live demonstration use, taking three measurements of an apparatus
						- utilizes bus speed of 24 MHz
						- utilizes PN0 as measurement status and PN1 as notifcation of untangling
						- utilizes PJ1 interrupts as trigger to beginning measurements

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
int motor_status = 0;
int motor_s2 = 0;
int steps = 0;

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
uint16_t Distance;
uint8_t RangeStatus;
uint8_t dataReady;
uint32_t delay = 2 ;
float degree = 0.0;
uint32_t myArr[48] = {0};

int input = 0;
	char TxChar;
int InterruptCount = 3;

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

                                                                            // 6) configure PB2,3 as I2C

  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)

        
}


//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

void PortN_Init(void){
	//Use PortN onboard LEDs
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// Activate clock for Port N
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x07;        								// Make PN0 and PN1 output (Built-in LEDs: D1 (PN1) and D2 (PN0))
  GPIO_PORTN_AFSEL_R &= ~0x07;     								// Disable alt funct on PN0 and PN1
  GPIO_PORTN_DEN_R |= 0x07;        								// Enable digital I/O on PN0 and PN1
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x07;     								// Disable analog functionality on PN0 and PN1
	FlashLED1(1);																		// Flash LED D1 (Hello World)
	return;
}


void blinkLEDN(){																			
	
	GPIO_PORTN_DATA_R = 0b00000001;
	SysTick_Wait10ms(4);
	GPIO_PORTN_DATA_R = 0b00000000;
}

void blinkLEDN1(){																		
	
	GPIO_PORTN_DATA_R ^= 0b00000010;
	
}
void SysTick_Wait10us(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(240);  // change this to wait 10us (assumes 24 MHz clock)
  }
}

//used to show bus speed was accomplished
void DutyCycle_Percent(uint8_t duty){
		float percent;	
		percent = ((float)duty*1000)/(255);
		int percent_int;
		percent_int = (int)percent;
		GPIO_PORTN_DATA_R ^= 0b00000101;
		SysTick_Wait10us(percent);  
		GPIO_PORTN_DATA_R ^= 0b00000101;
		SysTick_Wait10us(1000 - percent);
}



// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}



// Give clock to Port J and initalize PJ1 as Digital Input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x02;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	Enable weak pull up resistor on PJ1
}


// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	

		GPIO_PORTJ_IS_R = 0;   						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;  						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;  						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x02; 					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;  					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000; 					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000;				// (Step 4) Set interrupt priority to 5

		EnableInt();											// (Step 3) Enable Global Interrupt. lets go!
}


//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOJ_IRQHandler(void){

	int j = 1;
	
	for(int k = 0; k < InterruptCount; k++)
	{
		
		degree = 0;
		steps = 0;
		
		status = VL53L1X_StartRanging(dev);   //This function was called to enable the ranging

			
				// wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			
		
			//rotate motor 360 degrees
			for(int i = 0; i < 512; i++)
			{
					GPIO_PORTM_DATA_R = 0b00000011;
					SysTick_Wait10ms(delay);											
					GPIO_PORTM_DATA_R = 0b00000110;
					SysTick_Wait10ms(delay);
					GPIO_PORTM_DATA_R = 0b00001100;
					SysTick_Wait10ms(delay);
					GPIO_PORTM_DATA_R = 0b00001001;
					SysTick_Wait10ms(delay);
				
					//when step is a multiple of 32 or at the last step, take a measurement
					if((i % 32 == 0 || i == 511)&& i > 0)
					{
						//read the data values from ToF sensor
						status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
						status = VL53L1X_GetDistance(dev, &Distance);
						degree = 22.5*j;    //increment degree value by 22.5 everytime a new measurement is taken
						myArr[j -1] = Distance;    //add distance value to the array holding all the measurements
						if(j % 16 == 0)                       //special condition for the last value added to the list
						{
							myArr[16*(k+1)-1] = Distance;
						}
						j = j + 1;
						
						blinkLEDN();                     //toggle measurement status LED
						status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
						
						sprintf(printf_buffer,"%u, %f, %u\r\n", RangeStatus, degree, Distance);
						UART_printf(printf_buffer);
					}
				
			}
			
			blinkLEDN1();       //turn on additional status LED when untangling
			//loop to untangle wire, gives time to move on the x- axis as well before next measurement
			for(int k = 0; k < 512; k++)
			{
					
					
					GPIO_PORTM_DATA_R = 0b00001001;
					SysTick_Wait10ms(delay);											
					GPIO_PORTM_DATA_R = 0b00001100;
					SysTick_Wait10ms(delay);
					GPIO_PORTM_DATA_R = 0b00000110;
					SysTick_Wait10ms(delay);
					GPIO_PORTM_DATA_R = 0b00000011;
					SysTick_Wait10ms(delay);
				
					
					
			}
			blinkLEDN1();
			
		
		
		VL53L1X_StopRanging(dev);

	}
	
		// wait for the transmission code from pc. if 's' recieved then send data	
		while(1){		
			//wait for the right transmition initiation code
			while(1){
				input = UART_InChar();
				if (input == 's')
					break;
			}
			
			// simulating the transmission of 3*16 = 48 measurements
			for(int i = 0; i < 48; i++) {
				
				//write the data into string format
				sprintf(printf_buffer,"%u\r\n",myArr[i]);
				//send string to uart
				UART_printf(printf_buffer);
				FlashLED3(1);
				SysTick_Wait10ms(50);
			}
			
		
		}
	
	GPIO_PORTJ_ICR_R = 0x02; 					// Acknowledge flag by setting proper bit in ICR register
}

void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                   // configure PG0 as GPIO
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************


int main(void) {
  uint8_t byteData,byteData2, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortN_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
	
  // Initialize the sensor with the default setting 
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

		

		// always wait for the transmission code from pc. if 's' recieved then send data	
	while(1){		
		//wait for PJ1 to be pressed
		
		WaitForInt();
		
	}
}

