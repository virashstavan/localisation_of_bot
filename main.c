#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<avr/interrupt.h>
#include<avr/sfr_defs.h>
#include<math.h>
#include "USART_128.h"

#define BAUDrate 9600
#define F_CPU 8000000UL                                 // define baud
#define ubrr 51


float error;
float preError=0;										//previous error
float kp,kd,x,y;										//x and y are coordinate
float tError=0;											//pid value
float totError(float,float);



#define PI 3.14
#define lcofangle										//DEGREE
#define lcofdist 14.86								//MM
#define l 211											//axial dist between wheels
int countl=0,countr=0;		
float distx=0.0, disty=0.0, theta=0.0;
int R=0.0;
volatile int  a=0,b=0;											//temporary used for savin value of countr and countl
volatile int flag = 1;
char snum[10];
int overFlow=0;

float rVelocity=0, lVelocity=0;							//left wheel velocity and right wheel velocity
float reqAngle, presentAngle;							// required angle is angle between x axis and required point
int turn;
#define max 200;										//max velocity of bot
#define baseVelocity 150;								//define base velocity for bot

float totError(float reqAngle,float presentAngle)		//function used for finding total error or you can say pid value
{
	float e;
	error=reqAngle-presentAngle;
	e=(kp*error)+(kd*(preError- error));
	preError=error;
	if(e>50)
	{
		e=50;
	}
	else if (e<-50)
	{
		e=-50;
	}
	return e;
	
}


void clearstring(char *string)								//To clear string
{
	memset(string,0,strlen(string));
}
 

void main()
{	 
	
	
	USART_Init(51,0);										//Initiating USART 
	
	DDRD &= ~(1<<PD5);										//INPUT B		PIN D5		for right wheel
	DDRD &= ~(1<<PD3);										// INPUT A		PIN D1		for right wheel
	
	DDRD &= ~(1<<PD0);										//INPUT B		PIN D6		for left wheel
	DDRD &= ~(1<<PD1);										// INPUT A		PIN D3		for left wheel

	EIMSK |= (1<<INT3);										//ENABLING INTERRUPT INPUT A	PIN D3 for left encoder
	EICRA |= (1<<ISC31) | (1<<ISC30) ;						//calling interrupt at RISING EDGE
	
	EIMSK |= (1<<INT1);										//ENABLING INTERRUPT INPUT A	PIN D1 for right encoder
	EICRA |= (1<<ISC11) | (1<<ISC10) ;						//calling interrupt at RISING edge
	
	TCCR0 |= (1<<CS02) | (1<<CS01);							//pre scaling of 1024
	TIMSK |= (1<<TOIE0);									//Timer overflow interrupt enable
	
	//DDRE |=	(1<<PE5);										//pwm pin
	//DDRB |=	(1<<PB6);										//pwm pin
	
	//TCCR1A |= (1<<COM1B1) | (1<<WGM11);						//TIMER 1 fast pwm NON inverting	ICR1
	//TCCR1B |= (1<<WGM13) | (1<<WGM12);
	
	//TCCR3A |= (1<<COM3C1) | (1<<WGM31);						//TIMER 3 fast pwm non inverting	ICR3
	//TCCR3B |= (1<<WGM33) | (1<<WGM32);
	
	//ICR1 = 10000;
	//ICR3 = 10000;
	/* Replace with your application code */
	sei();													//CALLING INTERRUPT VECTOR
	distx=0;
	disty=0;
	theta=0;
	
	while(1)
	{	
		//USART_TransmitString(" hello ",0);
		/*reqAngle = atan(y/x);								//calculating angle 
		reqAngle = (reqAngle * 180) / PI;					//converting that into radian
		tError=totError(reqAngle,presentAngle);				//calling total error funtion 
		lVelocity= baseVelocity+tError;						//cofigrating left
		rVelocity=baseVelocity-tError;						//and right value
		*/
		//OCR1B = (lVelocity*10000)/200;						//coverting into pwm value
		//OCR3C = (rVelocity*10000)/200;
		
		/*itoa(a, snum, 10);
		USART_TransmitString("A= ",0);
		USART_TransmitString(snum,0);
		USART_TransmitString(" ",0);
		clearstring(snum);
		
		itoa(b, snum, 10);
		USART_TransmitString("B= ",0);
		USART_TransmitString(snum,0);
		USART_TransmitString(" ",0);
		clearstring(snum);*/
		if (flag)
		{
				R = (l/2.0)*(a + b)/(a - b);
				
				USART_TransmitString("a= ",0);
				USART_TransmitNumber(a,0);
				USART_TransmitString("  ",0);
				USART_TransmitString("b= ",0);
				USART_TransmitNumber(b,0);
				USART_TransmitString("  ",0);
					
				distx+=2.0*R*(cos(theta + ( (a -b) / (4.0*l) )*lcofdist)*sin( (a-b) / ( 4.0*l) * lcofdist));
				dtostrf(distx,5,2,snum);
				USART_TransmitString("distx= ",0);
				USART_TransmitString(snum,0);
				USART_TransmitString("  ",0);
				clearstring(snum);
		
				disty+=2.0*R*(sin((theta + ( (a -b) / (4.0*l) )*lcofdist))*sin( (a-b) / (4.0*l) * lcofdist ));
				dtostrf(disty,6,2,snum);
				USART_TransmitString("disty= ",0);
				USART_TransmitString(snum,0);
				USART_TransmitString("  ",0);
				clearstring(snum);
		
				theta+=(a-b) / (2.0*l) * lcofdist;                                            // calculating d_theta
				dtostrf(theta,6,2,snum);
				USART_TransmitString("theta= ",0);
				USART_TransmitString(snum,0);
				USART_TransmitString("  ",0);
				clearstring(snum);
				flag=0;
	}
}
}

ISR(INT3_vect)
{
	if(bit_is_clear(PIND,5))		//clockwise
		countl--;
	else 
		countl++;
				
	/*USART_TransmitString("LEFT= ",0);
	itoa(countl, snum, 10);
	USART_TransmitString(snum,0);
	USART_TransmitString("             ",0);
	clearstring(snum);
	*/	
			
}
ISR(INT1_vect)
{	
	if(bit_is_clear(PIND,0))		//clockwise
		countr++;
	else
		countr--;
		
	
	/*itoa(countr, snum, 10);
	USART_TransmitString("RIGHT= ",0);
	USART_TransmitString(snum,0);
	USART_TransmitString("             ",0);
	clearstring(snum);
	*/
}

ISR(TIMER0_OVF_vect)
{
	overFlow++;
	if (overFlow==62)
	{
		a= countl;
		/*USART_TransmitString("a= ",0);
		USART_TransmitNumber(a,0);
		USART_TransmitString("   ",0);*/
		b= countr;
		/*USART_TransmitString("b= ",0);
		USART_TransmitNumber(b,0);
		USART_TransmitString("   ",0);*/
		countr=0;
		countl=0;
			
		flag=1;
		overFlow=0;
	}
	
}