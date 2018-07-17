#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  rightDrive,     sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  leftDrive,      sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           clawHandL,     tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           leftFront,     tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           leftBack,      tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port4,           rightFront,    tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           rightBack,     tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port6,           leftMid,       tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           rightMid,      tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          clawHandR,     tmotorVex393HighSpeed_HBridge, openLoop)

#define MOTOR_NUM									kNumbOfTotalMotors
#define MOTOR_MAX_VALUE   				127
#define MOTOR_MIN_VALUE						(-127)
#define MOTOR_DEFAULT_SLEW_RATE 	10
#define MOTOR_FAST_SLEW_RATE			256
#define MOTOR_TASK_DELAY					15
#define MOTOR_DEADBAND						10
#define JOY_THRESHOLD							15

int motorReq[ MOTOR_NUM ];
int motorSlew[ MOTOR_NUM ];

int diameter = 4;
float circumference = 4*PI
int encoder_ticks = 392;

task SlewRate()
{
	unsigned int motorIndex;
	int motorTmp;
	for(motorIndex=0;motorIndex<MOTOR_NUM;motorIndex++)
	{
		motorReq[motorIndex] = 0;
		motorSlew[motorIndex] = MOTOR_DEFAULT_SLEW_RATE;
	}
	while( true )
	{

		for( motorIndex=0; motorIndex<MOTOR_NUM; motorIndex++)
		{
			motorTmp = motor[ motorIndex ];

			if( motorTmp != motorReq[motorIndex] )
			{
				if( motorReq[motorIndex] > motorTmp )
				{
					motorTmp += motorSlew[motorIndex];

					if( motorTmp > motorReq[motorIndex] )
						motorTmp = motorReq[motorIndex];
				}
				if( motorReq[motorIndex] < motorTmp )
				{
					motorTmp -= motorSlew[motorIndex];
					if( motorTmp < motorReq[motorIndex] )
						motorTmp = motorReq[motorIndex];
				}
				motor[motorIndex] = motorTmp;
			}
		}
		wait1Msec( MOTOR_TASK_DELAY );
	}
}

void leftDriveWheel(int speed)
{
	motorReq[leftBack]=speed;
	motorReq[leftFront]=speed;
	motorReq[leftMid]=speed;
}
void rightDriveWheel(int speed)
{
	motorReq[rightBack]=speed;
	motorReq[rightFront]=speed;
	motorReq[rightMid]=speed;
}

void drive_straight(float distance)
{
	SensorValue[leftDrive] = 0;
	SensorValue[rightDrive] = 0;
	float ticks = distance/(circumference)*encoder_ticks;
	while (abs(ticks) >  abs(SensorValue[leftDrive]) || (abs(ticks) > abs(SensorValue[rightDrive]))
	{
		// finds difference between left and right motors and calculates an increase in speed for one motor
		int diff = (abs(SensorValue[leftDrive]) - abs(SensorValue[rightDrive]));
		float mod = sgn(diff)*80*atan(ticks-abs(SensorValue[leftDrive]))*0.1;

		leftDriveWheel(80*atan(ticks-abs(SensorValue[leftDrive])*0.1));
		rightDriveWheel(80*atan(ticks-abs(SensorValue[rightDrive]*0.1))+ mod);
	}
	rightDriveWheel(0);
	leftDriveWheel(0);
	wait1Msec(500);
}

void drive_turn(float angle)
{
	SensorValue[leftDrive] = 0;
	SensorValue[rightDrive] = 0;
	int speed = sgn(angle)*80; //speed is -80 when robot is turning left and 80 when robot is turning right

	//int ticks = 7*angle;
	int desiredDriveTicks = abs((((angle * ((8 * PI)/360))/(circumference)) * encoder_ticks));//abs(angle/360 * (10 * PI)/(4*PI) * 392);
	while ((abs(SensorValue[leftDrive]) < desiredDriveTicks) && (abs(SensorValue[rightDrive]) < desiredDriveTicks))
	{
		int diff = (abs(SensorValue[leftDrive]) - abs(SensorValue[rightDrive]));
		int mod = sgn(diff)*speed*atan(desiredDriveTicks-abs(SensorValue[leftDrive]))*0.1;

		leftDriveWheel(speed* atan((desiredDriveTicks - abs(SensorValue[leftDrive]))*0.1));
		rightDriveWheel(-speed* atan((desiredDriveTicks - abs(SensorValue[rightDrive]))*0.1)-mod);
	}
	leftDriveWheel(0);
	rightDriveWheel(0);
	wait1Msec(500);
}

void coneLift(int time, int speed)
{
	motorReq[clawHandR]=speed;
	motorReq[clawHandL]=speed;
	wait1Msec(time);
	motorReq[clawHandR]=0;
	motorReq[clawHandL]=0;
	wait1Msec(100);
}



#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"


void pre_auton()
{
  bStopTasksBetweenModes = true;
}

task autonomous()
{

	int ninety = 60;
	startTask(SlewRate);
	//startTask(arcadeDrive);
	coneLift(2000,127);
	drive_straight(36);
	coneLift(2500,-127);
	drive_turn(-190);
	drive_straight(32);
	drive_turn(ninety);
	drive_straight(1);
	drive_turn(-58);
	drive_straight(70);
	coneLift(2000,127);
	wait1Msec(500);
	drive_straight(-6);
	coneLift(2000,-127);

	//-----30 POINT AUTON-----

	drive_straight(-9);
	wait1Msec(500);
	drive_turn(60);
	drive_straight(5);
	drive_turn(62);
	coneLift(2000,127);
	drive_straight(30);
	coneLift(3000,-127);
	drive_turn(-185);
	drive_straight(45);
	coneLift(2000,127);
	drive_straight(-6);
	coneLift(2000,-127);
	drive_straight(-120);
	drive_straight(20);
}

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
  while(true){
		motor[leftBack]=vexRT[Ch3];
		motor[rightFront]=vexRT[Ch2];
		motor[leftMid]=vexRT[Ch3];
		motor[rightBack]=vexRT[Ch2];
		motor[leftFront]=vexRT[Ch3];
		motor[rightMid]=vexRT[Ch2];

		if(vexRT[Btn6D]==true)
		{
			motor[clawHandL]=127;
			motor[clawHandR]=127;
		}
		else if(vexRT[Btn5D]==true)
		{
			motor[clawHandL]=-127;
			motor[clawHandR]=-127;
		}
		else{
			motor[clawHandR]=0;
			motor[clawHandL]=0;
			motor[port2]=0;
			motor[port3]=0;
			motor[port4]=0;
			motor[port5]=0;
			motor[port6]=0;
			motor[port7]=0;
		}}
  }
}
