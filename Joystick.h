// This Class manages the joystick
// it initalizes pins, reads position,sets maximum and minimum drive levels
// also deals with sensor calibration if needed
// Also prints and receives simple serial text commands to talk to Unity 3D
#ifndef Joystick_h
#define Joystick_h

#include "Arduino.h"

class Joystick
{
  public:
    Joystick(	int pwm1_pin,		int dir1_pin, 	int pwm2_pin, 	int dir2_pin,
				int x_axis_min, 	int x_axis_max, int y_axis_min,	int y_axis_max, float theta_range, int h_center,float sensor_wavenumber,
				int x_sensor_pin,	int y_sensor_pin);

	int Ny=-1;
	int Nx=-1;

	int x_min=-1;
	int y_min=-1;
	int x_max=-1;
	int y_max=-1;

	float channel1_resistance=0.4;
	float channel2_resistance=0.4;

	float supply_voltage=12;

	float maximum_drive_amps=8; // 5 amps is maxiumum
	float channel_voltage=12;

	float max_drive_float=((maximum_drive_amps*channel1_resistance)/supply_voltage*255);
	int max_drive_int=80;  

	int spring_y_on=0;
	int spring_y_gain=1;

	int spring_x_on=0;
	int spring_x_gain=1;
		  
	int x_axis_sign=1;
	int y_axis_sign=1;		  

	int 	x_raw=-1;
	int 	y_raw=-1;
	
	float x_degrees_per_ADC=1;
	float y_degrees_per_ADC=1;
	   
	float theta_x=-1;
	float theta_y=-1;

	float arg_check=-1;

	void 	drive_x(int drive_level);
	void 	drive_y(int drive_level);

// REPORTING FUCTIONS
int 	get_x();
int 	get_y();

float 	get_x_angle();
float 	get_y_angle();

void 	report_values();
void find_sensor_limits(int drive_level,int period_ms);



// variables used for unity functions
float get_x_pos_unity();
float x_pos_unity=0;
float x_range_unity=0;
float get_y_pos_unity();
float y_pos_unity=0;
float y_range_unity=0;

private:

int 	_dir1_pin=-1;
int 	_dir2_pin=-1;
int 	_pwm1_pin=-1;
int 	_pwm2_pin=-1;

float _h_wavenumber=-1;

float h_amp_x=-1;
float h_amp_y=-1;
float h_center=-1;

int  	x_pin;
int 	y_pin;


};


#endif
