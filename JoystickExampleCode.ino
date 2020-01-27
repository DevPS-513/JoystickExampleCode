
#include "Arduino.h"
#include "Joystick.h"
#include "Filter.h"

// Declare a filter for the X and Y axis
ExponentialFilter<long> ADCFilter_x(5, 0);
ExponentialFilter<long> ADCFilter_y(5, 0);
    

// Intialize Pins
int hall1_pin=22; // x axis sensor
int hall2_pin=23; // y axis sensor

// Driver 1
int d1_pwm_pin=23;
int d1_dir_pin=20;

// Driver 2
int d2_pwm_pin=21;
int d2_dir_pin=15;

// Assign the 4 LED PINS
int led1_pin=6;
int led2_pin=5;
int led3_pin=4;
int led4_pin=3;

// flags
int stream_flag;
int PD_flag=0;
int P_flag=0;

// Calibration Parameters for position sensors
int x_min=248;
int x_max=823;

int y_min=278;
int y_max=812;

// Assign center value of the sensor reading
float h_center=548;

// Initalize rotation angle for each axis
float theta_x=0;
float theta_y=0;
float theta_range=27.3;  		//degrees
float h_k=6.2832/theta_range; 	// wave vector assume y=dc+amp*sin(kx)
int raw_x=0;
int raw_y=0;

float pos_x=0;
float pos_y=0;

// PD Control parameters, set gain constants
int max_drive_level=70;
float Kp=22;
float Kd=26;

float Kp_default=Kp;
float Kd_default=Kd;

int voltage_command_x;
float pos_error_x=0;
float vel_error_x=0;
int   voltage_command_y;
float pos_error_y=0;
float vel_error_y=0;
float spring_x_center=h_center;
float spring_y_center=h_center;

float x_center_100=(h_center-x_min)/(x_max-x_min);
float y_center_100=(h_center-y_min)/(y_max-y_min);

// Flag to turn spring effect on or off
int spring_x_flag=1;
int spring_y_flag=1;

// parameters for talking to Unity3D
// These are used in place of pressing the arrow keys
float x_pos_percentage=0; // a value from -1 to 1 for unity to use
float y_pos_percentage=0; // a value from -1 to 1 for unity to use

// 85 is maximum that should be written.
void circle_blink();
String command;

Joystick Joystick1(d1_pwm_pin,d1_dir_pin,d2_pwm_pin,d2_dir_pin,
                  x_min,x_max,
                  y_min,y_max,theta_range,h_center,h_k,hall1_pin,hall2_pin);

int verbosity=0; 

elapsedMillis velocity_timer1;
elapsedMillis print_timer;


float dt=4;       // time to make velocity measurement in ms

float velocity_x=0;
float velocity_y=0;

float pos_prev_x=0;
float pos_prev_y=0;

void setup()
{  
    pinMode(hall1_pin,INPUT);
    pinMode(hall2_pin,INPUT);
    pinMode(led1_pin,OUTPUT);
    pinMode(led2_pin,OUTPUT);
    
    pinMode(led3_pin,OUTPUT);
    pinMode(led4_pin,OUTPUT);
    Serial.begin(9600);
// Human hearing is 20Hz to 20KHz. This will make the phases inaudible when driving
    analogWriteFrequency(d1_pwm_pin, 21000); // Teensy 3.0 pin 3 also changes to 375 kHz

Serial.println(" Type help for a list of commands");

//Serial.println("help 1")
circle_blink(4);

}

void loop()
{


raw_x= (int) Joystick1.get_x(); // Get the Raw Values
raw_y= (int) Joystick1.get_y();
ADCFilter_x.Filter(raw_x);      // Filter the Raw Values
ADCFilter_y.Filter(raw_y);

pos_x=ADCFilter_x.Current();
pos_y=ADCFilter_y.Current();

if (velocity_timer1>dt)
{velocity_timer1=0;

velocity_x=(pos_x-pos_prev_x);
velocity_y=(pos_y-pos_prev_y);
pos_prev_x=pos_x;
pos_prev_y=pos_y;

}


if (stream_flag==1)
// Only print every dt, same as when vlocity is captured.
{ if (print_timer>dt)
{ print_timer=0;
Serial.print("x: ");Serial.print(pos_x);Serial.print("\ty: ");Serial.print(pos_y);
Serial.print("\traw_x: ");Serial.print( raw_x);Serial.print("\traw_y: ");Serial.print("\t");Serial.print(raw_y); 
Serial.println();}
}


pos_error_x=(pos_x-(x_min+x_center_100*(x_max-x_min)));
voltage_command_x=-(Kp/100)*pos_error_x-(Kd/100)*velocity_x;

pos_error_y=(pos_y-(y_min+y_center_100*(y_max-y_min)));
voltage_command_y=-(Kp/100)*pos_error_y-(Kd/100)*velocity_y;



if (spring_x_flag==1)
{

  analogWrite(led1_pin,abs(voltage_command_x)*2);
  analogWrite(led2_pin,abs(voltage_command_y)*2);
  analogWrite(led3_pin,abs(voltage_command_x)*2);
  analogWrite(led4_pin,abs(voltage_command_y)*2);

Joystick1.drive_x(voltage_command_x);
}

if (spring_y_flag==1)
{

  analogWrite(led1_pin,abs(voltage_command_x)*2);
  analogWrite(led2_pin,abs(voltage_command_y)*2);
  analogWrite(led3_pin,abs(voltage_command_x)*2);
  analogWrite(led4_pin,abs(voltage_command_y)*2);

Joystick1.drive_y(voltage_command_y);
}



    // Check if something hass been entered into the serial port
    if (Serial.available())
    {
      char c = Serial.read(); 	// Read each charecter as it becomes available
      if (c == '\n') {         	// keep reading serial until a newline is entered
        parseCommand(command);   // Parse the command
        command = "";
      }           				// reset to zero
      else {                   // else keep adding to the current command unti newline is detected
        command += c;
      }
    }



}



  void parseCommand(String command)
  {

    // Commands are
    // off, drive everything to 0
    // drive_at_phase_angle
    // forward-drive it forward and give it position
    // backward-drive it backward and give it position


    String command_name;               // Assumes command can take form of "drive 255", part 1 being drive and part 2 being 255
    String command_value = "0";

    int space_index = command.indexOf(" ");
    int command_integer = 0;      // Default is zero
    // If there is no space, assume no number was given
    if (space_index == -1)
    {

      command_name = command.substring(0, command.length() - 1);    // Looks at everything up until the space, but not includeing

      if (verbosity == 1) {
        Serial.print("space index: "); Serial.print(space_index); Serial.println();
        Serial.print("parseing command name:["); Serial.print(command_name); Serial.print("]"); Serial.print("length:\t"); Serial.print(command_name.length()); Serial.println();
        Serial.print("parseing command value: "); Serial.print(command_integer); Serial.println();
      }

    }
    else
    {
      command_name = command.substring(0, space_index);    // Looks at everything up until the space, but not includeing
      command_value = (command.substring(command.indexOf(" ") + 1));    // everything after the space is converted to an integer, I guess not starting index is specefied
      command_integer = command_value.toInt(); // Convert to integer

         if (verbosity==1) {
      Serial.print("space index: "); Serial.print(space_index); Serial.println();
      Serial.print("parseing command name:["); Serial.print(command_name); Serial.print("]"); Serial.println();
      Serial.print("parseing command value: "); Serial.print(command_integer); Serial.println();
      }

    }

   
    // Drive motor 1 only

//Command 1
String command_1="fx ___\t\t\t: [0->80] drives x axis (Currently Capped at 80~30W/axis)";	
if (strcmp(command_name.c_str(), "fx") == 0)
{Joystick1.drive_x(command_integer);
spring_x_flag=0;spring_y_flag=0;}

//Command 2
String command_2="fy ___\t\t\t: [0->80] drives y axis (Currently Capped at 80~30W/axis)"; 
if (strcmp(command_name.c_str(), "fy") == 0)
{Joystick1.drive_y(command_integer);
spring_x_flag=0;spring_y_flag=0;}

//Command 3
String command_4="increase_cap_200 ___\t\t: will allow to set the drive cap up to 200/255 but will set a 10 second timer to reset the cap" ;
if (strcmp(command_name.c_str(), "increase_cap_200") == 0)
{}

//Command 6
String command_6="stream: 1\t\t: stream position values on/off=1/0";
if (strcmp(command_name.c_str(), "stream") == 0)
{   
  stream_flag=command_integer;

}

//Command 7
String command_7="x_center_100_perc: ___\t: [0->100] control x position from 0 to 100%";
if (strcmp(command_name.c_str(), "x_center_100_perc") == 0)
{     
x_center_100= (float) command_integer;
x_center_100=x_center_100/100;
}

//command 8
String command_8="y_center_100_perc: ___\t: [0->100] control y position from 0 to 100%";
if (strcmp(command_name.c_str(), "y_center_100_perc") == 0)
{  

y_center_100= (float) command_integer;
y_center_100=y_center_100/100;}
    


//command 12
String command_12="damping 1\t\t:demo only damping, use 'off 1' to reset device";
if (strcmp(command_name.c_str(), "damping") == 0)
{Kd=80;Kp=0;spring_x_flag=1;spring_y_flag=1;}

//command 13
String command_13="on 1\t\t\t: turn on spring with 1, off with 0";
if (strcmp(command_name.c_str(), "on") == 0)
{spring_y_flag=command_integer;spring_x_flag=command_integer;
Kp=Kp_default;Kd=Kd_default;}



//command 14
String command_14="off 1\t\t\t: turn everything off";
if (strcmp(command_name.c_str(), "off") == 0)
{spring_y_flag=0;
spring_x_flag=0;
Joystick1.drive_x(0);
Joystick1.drive_y(0);
Kp=Kp_default;Kd=Kd_default;
stream_flag=0;
  analogWrite(led1_pin,0);
  analogWrite(led2_pin,0);
  analogWrite(led3_pin,0);
  analogWrite(led4_pin,0);
}
        

//command 15
String command_15="blink  ___\t\t: [1 to 30] blink all the lights n times";
if (strcmp(command_name.c_str(), "blink") == 0)
{if ((command_integer>1)&&(command_integer<30))
            {circle_blink(command_integer);                    }}


String command_20="full_power 1 \t\t: !warning! drive x axis at 100 watts for 5 seconds, will time out for 10s \n \t\t\t\tplease hold the handle to reduce end stop damage";
if (strcmp(command_name.c_str(), "full_power") == 0)
 {if(command_integer>0)
 {Joystick1.drive_x(180);delay(8000);Joystick1.drive_x(0);delay(16000);}  }



if (strcmp(command_name.c_str(), "help") == 0)
{

  
Serial.println("*********************************************************");
Serial.println("Device:");
Serial.println(" Welcome, I am a force-feedback joystick");
Serial.println(" I respond to text commands over serial.");
Serial.println(" Each command expects one word then one integer,");
Serial.println(" for example 'off 1' should turn everything off and reset,");
Serial.println(" and 'on 1' will engage a spring effect,");
Serial.println(" and 'fx 80' will drive the x axis at 80/255.");
Serial.println(" 80/255 correponds to ~30 watts per axis ~8A/driver,");
Serial.println(" 'damping 1' will demo only damping");
Serial.println("*********************************************************");

Serial.println(command_1 ); // fx command
Serial.println(command_2 ); // fy command
//Serial.println(command_3 );
//Serial.println(command_4 );
//Serial.println(command_5 );
Serial.println(command_6 ); 
Serial.println(command_7 );
Serial.println(command_8 );
//Serial.println(command_9 );
//Serial.println(command_10);
//Serial.println(command_11);
Serial.println(command_12);
Serial.println(command_13);   // on command
Serial.println(command_14);   // off command
Serial.println(command_15);   // Blink the lights n times

Serial.println(command_20);

  }

  }


void circle_blink(int num_times)
{

 for (int i = 1; i <= num_times; ++i)
{
    delay(50);
  analogWrite(led1_pin,0);

  analogWrite(led2_pin,0);

  analogWrite(led3_pin,0);

  analogWrite(led4_pin,0);


  delay(50);
  analogWrite(led1_pin,60);
 
  analogWrite(led2_pin,60);

  analogWrite(led3_pin,60);
 
  analogWrite(led4_pin,60);


  delay(50);
  analogWrite(led1_pin,120);

  analogWrite(led2_pin,120);

  analogWrite(led3_pin,120);

  analogWrite(led4_pin,120);


  delay(50);
  analogWrite(led1_pin,180);

  analogWrite(led2_pin,180);
 
  analogWrite(led3_pin,180);

  analogWrite(led4_pin,180);


  
  delay(50);
  analogWrite(led1_pin,0);

  analogWrite(led2_pin,0);
 
  analogWrite(led3_pin,0);

  analogWrite(led4_pin,0);


}
}
   
