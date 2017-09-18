#ifndef pid_hpp
#define pid_hpp
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

static long long int count;

long long int millis(void)
{
	return count;
}

void createMillisTimer(void)
{
	struct sigaction sa;
	struct itimerval timer;

	static const auto increment = [&count] (int signum) {++count;};

	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = increment;
	sigaction (SIGVTALRM, &sa, NULL);

	/* Configure the timer to expire after 1 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = 1000;
	/* ... and every 1 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = 1000;
	/* Start a virtual timer. It counts down whenever this process is
	  executing. */
	setitimer (ITIMER_VIRTUAL, &timer, NULL);
}

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;

#define MANUAL 0
#define AUTOMATIC 1

#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);

      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;

      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}

void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;

  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;

   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;

   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}

void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}

void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

#endif
