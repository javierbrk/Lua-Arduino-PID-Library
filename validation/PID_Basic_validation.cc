/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <iostream>
#include <string>
#include <stdexcept>


#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, input_temp, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=6, Kd=2;


bool tryParse(std::string& input, double& output) {
    try{
        output = std::strtod(input.c_str(),NULL);
    } catch (std::invalid_argument) {
        return false;
    }
    return true;
}
int main(int argc, char* argv[]) {
    	if (argc != 4) 
    		return -1;
	Kp = atof(argv[1]);
	Ki = atof(argv[2]);
	Kd = atof(argv[3]);
	std::string input;
	int x;
	Setpoint = 38;
	double temp_inicial = 15;
	input_temp = temp_inicial;	
	PID myPID(&input_temp, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
//	while (true)
	for (int i=0; i<3200; i++)
	{
		//getline(std::cin, input);
		/*if (!tryParse(input, input_temp))
		{
			std::cout << "Bad entry. Enter a NUMBER: ";
			break;
		}*/
		input_temp=input_temp+(Setpoint-temp_inicial)/2000*(Output/255);
		myPID.Compute();
		std::cout << input_temp << ";" << Output << std::endl;
	}
return 0;
}


