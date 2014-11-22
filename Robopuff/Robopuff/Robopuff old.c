/*
 * Robopuff.c
 *
 * Created: 11/18/2014 12:32:28 PM
 *  Author: adriacat
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include "math_util.h"
#include "m_general.h"
#include "point.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_wii.h"

#include "instructions.h"

#define SYSTEM_CLK 16000000
#define N_DIV 3
#define PRESCALER 64

#define MY_ADDRESS 0xC// OTHERS ARE D, E
#define CHANNEL 1
#define PACKET_LENGTH 8

#define NUM_COORDINATES_HIST 2//0 current point, 1 historical point

//constellation points and other constants --> might move to a header file
double x_ab = 10.563;
double x_bd =11.655;
double y_cb = 14.5*2;
double y_ca = 14.5 + 2.583;
double y_cd = 14.5 + 8.741;
double stars_origin[2] = {10.563, 14.5};

///////////////////////LOCALIZATION RELATED VARS AND FUNCS///////////////////////////////////////////////////
//This is our desired time frequency for LOCALIZATION

void setup_mWii(void);
void read_mwii_data(int *);
void localize_robot();
void save_old_location(double robo_coordinates[][2], float robo_thetas[]);

volatile float timer_freq = .4;//FOR OUR TESTING SHOULD MAKE FASTER LATER
//buffer for mwii
unsigned int blobs [12]= {0,0,0,0,0,0,0,0,0,0,0,0};
double robo_coordinates [NUM_COORDINATES_HIST][2]={{0,0},{0,0}};
float robo_thetas [NUM_COORDINATES_HIST]={0,0};
volatile int localize_me = 0;

//////////////////////////WIRELESS RELATED VARS AND FUNCS////////////////////////////////////////////////
// buffer for wireless
char buffer[PACKET_LENGTH]={0,0,0,0,0,0,0,0};
volatile int received_a_new_command = FALSE;

void init(void);
void timer1_init(void);
void timer3_init(void);
void setPrescaler(int);
void setupMRF(void);

int calculateOCR3A(float);
float calculateHighPassFilter(float, float, float, float);
float calculateLowPassFilter(float, float, float);


int main(void)
{
	init();
	m_green(TOGGLE);
	
	while(1){
		
		if(localize_me){
			m_red(TOGGLE);	
			if (m_usb_rx_available())
				{m_usb_tx_string("\nLocalizing! ...");
				}
		
		localize_robot();
		localize_me=0;
		}
	}
return 0;
}


/************************************************************************/
/* Save the old robot coordinates into the second row of the robo coordinates
/* and thetas arrays in case we need for filtering or sth like that        */
/************************************************************************/
void save_old_location(double robo_coordinates[][2], float robo_thetas[] ){
	robo_coordinates[1][0]=robo_coordinates[0][0];
	robo_coordinates[1][1]=robo_coordinates[0][1];
	robo_thetas[1]=robo_thetas[0];
}


void localize_robot(){
	Point *camera_origin = Point_create(512,384);
	int rows_ignored = 0;

	//save_old_location(robo_coordinates, robo_thetas);

	int sampled_points[8]={0,0,0,0,0,0,0,0};

	//get the mwii localization 
	read_mwii_data(sampled_points);
	
	//process data
	//read the sampled points x0,y0,x1,y1,x2,y2,x3,y3
	Point *sp0 = Point_create(sampled_points[0], sampled_points[1]);
	Point *sp1 = Point_create(sampled_points[2], sampled_points[3]);
	Point *sp2 = Point_create(sampled_points[4], sampled_points[5]);
	Point *sp3 = Point_create(sampled_points[6], sampled_points[7]);

	
	int there_are_bad_stars = 0;// use if we do the 3 star scenario
	//ignore if we have a bad star
		
	if(Util_is_bad_star(sp0)||Util_is_bad_star(sp1)||Util_is_bad_star(sp2)||Util_is_bad_star(sp3)){
		rows_ignored++;
		there_are_bad_stars=1;
	
		if (m_usb_rx_available()){m_usb_tx_string("\nIgnoring because there are bad stars ");
		}
	}
		
	if(!there_are_bad_stars){
		
	if (m_usb_rx_available()){
		m_usb_tx_string("\nSampled stars ");
		m_usb_tx_string("\tP0: (");
		m_usb_tx_int(Point_get_x(sp0));
		m_usb_tx_string(",");
		m_usb_tx_int(Point_get_y(sp0));
		
		m_usb_tx_string(")\t P1: (");
		m_usb_tx_int(Point_get_x(sp1));
		m_usb_tx_string(",");
		m_usb_tx_int(Point_get_y(sp1));
		
		m_usb_tx_string(")\t P2: (");
		m_usb_tx_int(Point_get_x(sp2));
		m_usb_tx_string(",");
		m_usb_tx_int(Point_get_y(sp2));
				
		m_usb_tx_string(")\t P3: (");
		m_usb_tx_int(Point_get_x(sp3));
		m_usb_tx_string(",");
		m_usb_tx_int(Point_get_y(sp3));
		m_usb_tx_string(")");
	}
		
	Point sample_points[4]={*sp0,*sp1,*sp2,*sp3};
	Matrix *distance_matrix = Util_get_distance_matrix(sample_points);
	
	if(m_usb_rx_available()){
		
		  int i,j;
		  m_usb_tx_string("\n---Distance matrix ---\n");
		  for (i=0;i<4;i++){
			  for (j=0;j<4;j++){
				 m_usb_tx_int(Matrix_get(distance_matrix, i,j));
				 m_usb_tx_string(", ");
			
			  }
			  m_usb_tx_string("\n");
		  }
	}
	
	
	// find indexes of min and max r-row, c-col
	int r_c_min[2]={0,0};
	int r_c_max[2]={0,0};
	Matrix_find_min(r_c_min, distance_matrix);
	
	if (m_usb_rx_available())
		{m_usb_tx_string("\nFound min: ");
		m_usb_tx_string("\ti,j: ");
		m_usb_tx_int(r_c_min[0]);
		m_usb_tx_string(", ");	
		m_usb_tx_int(r_c_min[1]);
		m_usb_tx_string("\n");	
		m_usb_tx_int(Matrix_get(distance_matrix,r_c_min[0],r_c_min[1]));
		}
	
	Matrix_find_max(r_c_max, distance_matrix);
	
	if (m_usb_rx_available())
		{m_usb_tx_string("\nFound max: ");
		m_usb_tx_int(Matrix_get(distance_matrix,r_c_max[0],r_c_max[1]));
		}
		
		//find which star is pointing in the positive y direction
		
		int north_star = Util_find_intersection(r_c_min,r_c_max);
		if (isnan(north_star)){
			rows_ignored++;
			
			if (m_usb_rx_available())
			{m_usb_tx_string("\n Couldn't find intersection ");
			}
		}
		
		int south_star = Util_find_other_star(north_star, r_c_max);
		Point * sample_point_origin = Point_get_midpoint(&sample_points[north_star], &sample_points[south_star]);//check!!!
		Point * origin_offset = Point_sub(sample_point_origin, camera_origin);
		Point * vect_south_star_to_origin = Point_sub(&sample_points[south_star], sample_point_origin);//slope of axis
		float theta = atan2f(-Point_get_x(vect_south_star_to_origin), Point_get_y(vect_south_star_to_origin));
		double p0x = Point_get_x(origin_offset);
		double p0y = Point_get_y(origin_offset);
		
		//H_stars_robot
		Matrix * H_stars_robot = Matrix_create(3, 3);
		float sin_theta = sinf(theta);
		float cos_theta = cosf(theta);
		Matrix_set(H_stars_robot, 0, 0, cos_theta);  Matrix_set(H_stars_robot, 0, 1, -sin_theta); Matrix_set(H_stars_robot, 0, 2, p0x);
		Matrix_set(H_stars_robot, 1, 0, sin_theta);  Matrix_set(H_stars_robot, 1, 1, cos_theta);  Matrix_set(H_stars_robot, 1, 2, p0y);
		Matrix_set(H_stars_robot, 2, 0, 0);          Matrix_set(H_stars_robot, 2, 1, 0);          Matrix_set(H_stars_robot, 2, 2, 1);
		
		//    printf("\n -------------- H_stars_robot -------------- \n");
		//Matrix_dump(H_stars_robot);
		Matrix * H_robot_stars = Matrix_inverse(H_stars_robot);
		//    printf("\n -------------- H_robot_stars -------------- \n");
		//Matrix_dump(H_robot_stars);

		float robo_theta = acosf(Matrix_get(H_robot_stars, 0, 0));
		double robo_x = Matrix_get(H_robot_stars, 0, 2);
		double robo_y = Matrix_get(H_robot_stars, 1, 2);
		
		robo_coordinates[0][0]=robo_x;
		robo_coordinates[0][1]=robo_y;
		robo_thetas[0]=robo_theta;
			
		if(m_usb_rx_available()){
			m_usb_tx_string("\n100 x Robo x: ");
			m_usb_tx_long(robo_x*100);
			
			m_usb_tx_string("\t100 x Robo y: ");
			m_usb_tx_long(robo_y*100);
			
			m_usb_tx_string("\t100 x Theta: ");
			m_usb_tx_long(robo_theta*100);
			}
			
			
			free(camera_origin);
			free(sp0);
			free(sp1);
			free(sp2);
			free(sp3);
			free(distance_matrix);
			free(sample_point_origin);
			free(origin_offset);
			free(vect_south_star_to_origin);
			free(H_robot_stars);
			free(H_stars_robot);		
		}
		

		
	}	

void init(){
	m_clockdivide(N_DIV);
	setup_mWii();
	//setupMRF();
	m_usb_init();
	//init peripherals
	m_bus_init();
	timer3_init();
	//enable global interrupts for  our timers! and other stuff
	sei();
	
}

/*
void timer1_init(void)
{
	// Timer mode (15): Single-slope UP to OCR1A
	set(TCCR1B, WGM13);
	set(TCCR1B, WGM12);
	set(TCCR1A, WGM11);
	set(TCCR1A, WGM10);
	// Pin B6 selected as output compare
	set(DDRB, 6);
	// Clock prescaler set at /64
	clear(TCCR1B, CS12);
	set(TCCR1B, CS11);
	set(TCCR1B, CS10);
}
*/

void timer3_init(void)
{
	// Timer mode 4, count up to OCR3A
	clear(TCCR3B, WGM33);
	set(TCCR3B,WGM32);
	clear(TCCR3A, WGM31);
	clear(TCCR3A,WGM30);
	
	setPrescaler(PRESCALER);
	

	set(DDRC,6);
	//TCCR3A:	COM3A1	TCCR3A:	COM3A0
	clear(TCCR3A,COM3A1);
	set(TCCR3A,COM3A0);

	//Enable interrupt when TCNT3 matches OCR3A
	set(TIMSK3, OCIE3A);

	//set OCR3A so that timer 3 counts ~1kHz
	OCR3A = calculateOCR3A(timer_freq);
}


void setPrescaler(int prescaleFactor){

	switch(prescaleFactor){
		case 1:
		clear(TCCR3B, CS32);
		clear(TCCR3B, CS31);
		set(TCCR3B, CS30);
		break;

		case 8:
		clear(TCCR3B, CS32);
		set(TCCR3B, CS31);
		clear(TCCR3B, CS30);
		break;

		case 64:
		clear(TCCR3B, CS32);
		set(TCCR3B, CS31);
		set(TCCR3B, CS30);
		break;

		case 256:
		set(TCCR3B, CS32);
		clear(TCCR3B, CS31);
		clear(TCCR3B, CS30);
		break;

		case 1024:
		// Clock prescaler /1024
		set(TCCR3B, CS32);
		clear(TCCR3B, CS31);
		set(TCCR3B, CS30);
		break;
		
	}
}

int calculateOCR3A(float timer_freq){
	return (int) ((float)SYSTEM_CLK/((pow(2,N_DIV+1))*PRESCALER*timer_freq))-1;
}

ISR(TIMER3_COMPA_vect){
	localize_me = TRUE;
}

/************************************************************************/
/* Turn on the wireless module                                          */
/************************************************************************/
void setupMRF(){
	m_rf_open(CHANNEL, MY_ADDRESS, PACKET_LENGTH);
}

/************************************************************************/
/* Turn on the wii camera                                               */
/************************************************************************/
void setup_mWii(){
	char wiiOpen= m_wii_open();
	if (wiiOpen){
		m_green(ON);
	}
}


void read_mwii_data(int sample_points[]){
	m_wii_read( blobs);
	sample_points[0]=blobs[0];
	sample_points[1]=blobs[1];
	sample_points[2]=blobs[3];
	sample_points[3]=blobs[4];
	sample_points[4]=blobs[6];
	sample_points[5]=blobs[7];
	sample_points[6]=blobs[9];
	sample_points[7]=blobs[10];
}

/*
ISR(INT2_vect){
	m_rf_read(buffer,PACKET_LENGTH);
	received_a_new_command=1;
}*/