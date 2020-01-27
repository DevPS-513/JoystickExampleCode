
#include "Joystick.h"

// Define Pins,minimum and maximum sensor readings, and angle of rotation range
// "h" referes to hall effect sensor
 Joystick::Joystick(int pwm1,int dir1,int pwm2,         int dir2, int x_axis_min, 
					          int x_axis_max,   int y_axis_min,   int y_axis_max, 
					          float theta_range,int center_point, float sensor_wavenumber, 
					          int x_sensor_pin, int y_sensor_pin)                     
{ 	

// Assign pin values to private variables.
	_pwm1_pin=pwm1;
	_pwm2_pin=pwm2;

	_dir1_pin=dir1;
	_dir2_pin=dir2;

  _x_pin=x_sensor_pin;
  _y_pin=y_sensor_pin;

// Assign Calibration values  
	x_min=x_axis_min;
	y_min=y_axis_min;
	x_max=x_axis_max;
	y_max=y_axis_max;
 
// Amplitude of sensor values, assume to vary from min to max
// hall effect has an amplitude and a center point that represents
// Joystick deflection range
  h_amp_x= (float) (x_axis_max-x_axis_min)/2;
  h_amp_y= (float) (y_axis_max-y_axis_min)/2;
  h_center= (float) center_point;	
	_h_wavenumber=sensor_wavenumber; // Converts sensor values to sinewave


	drive_x(0);
	drive_y(0);
	
}
 

void Joystick::drive_x(int pwm_value)
{  
	pwm_value=x_axis_sign*pwm_value; // Reverse sign if needed
	if (pwm_value>=0)
	{
		analogWrite(_dir1_pin,0);
		if (pwm_value>max_drive_int)
		{analogWrite(_pwm1_pin,max_drive_int);}
		else{analogWrite(_pwm1_pin,pwm_value);}
	}			
	
	if (pwm_value<0)
	{
		analogWrite(_dir1_pin,255);
		if (-1*pwm_value>max_drive_int)
		{analogWrite(_pwm1_pin,max_drive_int);}
		else{analogWrite(_pwm1_pin,-1*pwm_value);}
	}

}

void Joystick::drive_y(int pwm_value)
{  
	pwm_value=y_axis_sign*pwm_value; // Reverse sign if needed	

	if (pwm_value>=0)
	{
		analogWrite(_dir2_pin,255);
		if (pwm_value>max_drive_int)
		{analogWrite(_pwm2_pin,max_drive_int);}
		else{analogWrite(_pwm2_pin,pwm_value);}
	}			
	
	if (pwm_value<0)
	{
		analogWrite(_dir2_pin,0);
		if (-1*pwm_value>max_drive_int)
		{analogWrite(_pwm2_pin,max_drive_int);}
		else{analogWrite(_pwm2_pin,-1*pwm_value);}
	}

}


void Joystick::report_values()
{
	Serial.print("x");Serial.print("\t");Serial.print(get_x()); Serial.print("\t");
	Serial.print("y");Serial.print("\t");Serial.print(get_y()); Serial.print("\t");
	Serial.print("x(deg)");Serial.print("\t");Serial.print(get_x_angle()); Serial.print("\t");
	Serial.print("y(deg)");Serial.print("\t");Serial.print(get_y_angle()); Serial.print("\t");	 
	Serial.println();
}

	 
int Joystick::get_x()
{
	 x_raw=analogRead(_x_pin); 
	 return x_raw;
}

float Joystick::get_x_pos_unity()
{
   x_pos_unity= (float) (analogRead(_x_pin)-x_min);
x_range_unity=(float) (x_max-x_min);

x_pos_unity=x_pos_unity/x_range_unity;

if (x_pos_unity<0)
x_pos_unity=0;
if(x_pos_unity>1)
x_pos_unity=1;

x_pos_unity= (x_pos_unity-0.5)/0.5;
 
   return x_pos_unity;
}

float Joystick::get_y_pos_unity()
{
   y_pos_unity= (float) (analogRead(_y_pin)-y_min);
y_range_unity=(float) (y_max-y_min);

y_pos_unity=y_pos_unity/y_range_unity;

if (y_pos_unity<0)
y_pos_unity=0;
if(y_pos_unity>1)
y_pos_unity=1;
y_pos_unity= (y_pos_unity-0.5)/0.5; 
   return y_pos_unity;
}
		 
int Joystick::get_y()
{
	y_raw=analogRead(_y_pin);  
	return y_raw;
}

float Joystick::get_x_angle()
{
arg_check=(( (float) get_x()-h_center)/h_amp_x);

if (arg_check>1)
arg_check=1;
if(arg_check<-1)
arg_check=-1;

  theta_x=asin(arg_check)/_h_wavenumber*2;


	
	 return theta_x;
}
		 
float Joystick::get_y_angle()
{
arg_check=(( (float) get_y()-h_center)/h_amp_y);
  if (arg_check>1)
arg_check=1;
if(arg_check<-1)
arg_check=-1;
theta_y=asin(arg_check)/_h_wavenumber*2;		
	 return theta_y;
}
