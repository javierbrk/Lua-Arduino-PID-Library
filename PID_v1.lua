-----------------------------------------------------------------------------
-- Copyright (c) 2023  Javier Jorge <jjorge@inti.gob.ar>
-- Copyright (c) 2023  Instituto Nacional de Tecnología Industrial
-- Copyright (C) 2023  Asociación Civil Altermundi <info@altermundi.net>
--
--  SPDX-License-Identifier: AGPL-3.0-only
-----------------------------------------------------------------------------

local M={
	name=...,       -- module name, upvalue from require('module-name')
	model=nil,      
	verbose=false,    -- verbose output
	debug=nil,      -- additional ckecks
	time=0,
	inAuto=false,
	outMax=255,
	outMin=0,
	mySetpoint=38,
	SampleTime=200,
	myOutput=0,
	outputSum=0,
	lastInput=0,
	lastTime=0,
	P_ON_M= 0,
	P_ON_E= 1,
	pOn=0,
	pOnE=false,
	REVERSE=1
}
_G[M.name]=M



local function millis()
	M.time = M.time + 200;
	return M.time;
end

--[[Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 **************************************************************************--]]
function M.initPID(Setpoint,Kp,Ki,Kd,POn,ControllerDirection)
	M.mySetpoint = Setpoint;
	M.inAuto = true;

	M.SetOutputLimits(0, 255);				--default output limit corresponds to arduino
	M.SampleTime = 100;							--default Controller Sample Time is 0.1 seconds

	--M.SetControlerDirection(ControllerDirection);
	M.SetTunings(Kp, Ki, Kd, POn);

	M.lastTime = millis()-M.SampleTime;
end
--[[ Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 *********************************************************************************--]]
function M.Compute(input)
	if not M.inAuto then
		return -1
	end
	local now = millis();
	local timeChange = (now - M.lastTime);
	if timeChange>=M.SampleTime then

		--[[Compute all the working error variables--]]
		error = M.mySetpoint - input;
		dInput = (input - M.lastInput);
		M.outputSum = M.outputSum + (M.ki * error);

		--[[Add Proportional on Measurement, if P_ON_M is specified--]]
		if not M.pOnE then 
			M.outputSum =M.outputSum - M.kp * dInput;
		end

		if M.outputSum > M.outMax then
			M.outputSum= M.outMax
		elseif M.outputSum < M.outMin then
			M.outputSum= M.outMin;
		end
		--[[Add Proportional on Error, if P_ON_E is specified--]]

		if M.pOnE then
			M.myOutput = M.kp * error;
		else 
			M.myOutput = 0;
		end

		--[[Compute Rest of PID Output--]]
		M.myOutput =M.myOutput + M.outputSum - M.kd * dInput;

		if M.myOutput > M.outMax then
			M.myOutput = M.outMax
		elseif M.myOutput < M.outMin then
			M.myOutput = M.outMin
		end
		--[[Remember some variables for next time--]]
		M.lastInput = input;
		M.lastTime = now;
		return M.myOutput;

	else
		return -1;
	end
end

--[[ SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 *****************************************************************************--]]
function M.SetTunings(Kp,Ki, Kd, POn)
	if Kp<0 or Ki<0 or Kd<0 then
		return
	end

	M.pOn =POn;
	if POn == M.P_ON_E then
		M.pOnE=true
	end

--   dispKp = Kp; dispKi = Ki; dispKd = Kd;

	SampleTimeInSec = M.SampleTime/1000;
	M.kp = Kp;
	M.ki = Ki * SampleTimeInSec;
	M.kd = Kd / SampleTimeInSec;

	--[[if controllerDirection == M.REVERSE then
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	 end--]]
end

--[[ SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 *****************************************************************************
void PID::SetTunings(double Kp, double Ki, double Kd){
	SetTunings(Kp, Ki, Kd, pOn); 
}
--]]
--[[ SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 *****************************************************************************--]]
function M.SetSampleTime(NewSampleTime)
	if (NewSampleTime > 0) then
		local ratio  = NewSampleTime/M.SampleTime;
		M.ki = M.ki * ratio;
		M.kd = M.kd / ratio;
		M.SampleTime = NewSampleTime;
	end
end


--[[ SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 *************************************************************************--]]
function M.SetOutputLimits(Min,Max)
	if Min >= Max then
		return
	end
	M.outMin = Min;
	M.outMax = Max;

	if M.inAuto then
		if M.myOutput > M.outMax then
			M.myOutput = M.outMax
		elseif M.myOutput < M.outMin then
			M.myOutput = M.outMin
		end

		if M.outputSum > M.outMax then
			M.outputSum= M.outMax
		elseif M.outputSum < M.outMin then
			M.outputSum= M.outMin;
		end
	end
end

--[[ SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 *****************************************************************************
void PID::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto && !inAuto)
	{  --we just went from manual to auto
		PID::Initialize();
	}
	inAuto = newAuto;
}
--]]
--[[ Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 *****************************************************************************
void PID::Initialize()
{
	 outputSum = *myOutput;
	 lastInput = *myInput;
	 if(outputSum > outMax) outputSum = outMax;
	 else if(outputSum < outMin) outputSum = outMin;
}
--]]
--[[ SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 *****************************************************************************
void PID::SetControllerDirection(int Direction)
{
	 if(inAuto && Direction !=controllerDirection)
	 {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	 }
	 controllerDirection = Direction;
}--]]

--[[ Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 *****************************************************************************
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

--]]
return M
