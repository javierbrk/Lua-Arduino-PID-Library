local myPID=require('PID_v1')
local Setpoint = 38
local input_temp = 15
local temp_inicial = input_temp
local Output = 0
myPID.initPID(Setpoint,2,150,150,1,1)
for i=0, 3200, 1
do
	input_temp=input_temp+(Setpoint-temp_inicial)/2000*(Output/255);
	Output=myPID.Compute(input_temp)
	print (input_temp,";" ,Output)
end



