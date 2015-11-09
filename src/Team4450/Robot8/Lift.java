
package Team4450.Robot8;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CANTalon;

class Lift
{
	private final Talon         liftMotor;
	private final CANTalon		liftMotorCan;
	private final DigitalInput  liftTopSwitch;
	private final DigitalInput  liftBottomSwitch;
	private final FestoSA		kickerSwitch;
	private final FestoDA		binSwitch;
	private final Robot			robot;
	private final boolean		competitionRobot;
	
	Lift(Robot robot)
	{
	  	this.robot = robot;
		
		Util.consoleLog();
	  	
		liftMotor = new Talon(4);
		liftTopSwitch = new DigitalInput(0);
		liftBottomSwitch = new DigitalInput(1);
		kickerSwitch = new FestoSA(0);
		binSwitch = new FestoDA(1);
		BinClose();	// set gearbox to high to start for testing.
		
		liftMotorCan = new CANTalon(1);
		liftMotorCan.clearStickyFaults();
		liftMotorCan.enableControl();
		liftMotorCan.changeControlMode(ControlMode.PercentVbus);
		
		// testing talon onboard limit switch via breakout board.
		liftMotorCan.enableLimitSwitch(false, true);
		liftMotorCan.ConfigRevLimitSwitchNormallyOpen(false);
	
		if (robot.robotProperties.getProperty("RobotId").equals("comp")) 
			competitionRobot = true;
		else
			competitionRobot = false;
    
		liftMotor.set(0);
		liftMotorCan.set(0);
	}

	void dispose()
	{
		Util.consoleLog();
		
		liftMotor.free();
		liftTopSwitch.free();
		liftBottomSwitch.free();
		kickerSwitch.dispose();
		binSwitch.dispose();
		liftMotorCan.delete();
	}

	void KickerOut()
	{
		Util.consoleLog();
		
		kickerSwitch.Open();
	}

	void KickerIn()
	{
		Util.consoleLog();
		
		kickerSwitch.Close();
	}

	void BinClose()
	{
		Util.consoleLog();
		
		binSwitch.Close();
	}

	void BinOpen()
	{
		Util.consoleLog();
		
		binSwitch.Open();
	}
	
	void LiftUpDown(double speed)
	{
		double tempSpeed = 0;
    
		//Util.consoleLog("angle motor input speed = %f", speed );
    
		if (speed > 0.20) // this .15 creates a dead zone around controller zero.
		{
			//Util.consoleLog("switch = %b", liftTopSwitch.get() );
    
//			if (competitionRobot && liftTopSwitch.get())
//				tempSpeed = 0; 		// limit switch closed, stop motor. sb zero.
//			else
				tempSpeed = speed; 	// up speed of motor.
		}
		else if (speed < -0.20)
		{
			//Util.consoleLog("switch = %b", liftBottomSwitch.get() );
    
//			if (competitionRobot && liftBottomSwitch.get())
//				tempSpeed = 0; 		// limit switch closed, stop motor. sb zero.
//			else
				tempSpeed = speed; 	// down speed of motor.
		}
    
		//Util.consoleLog("tempSpeed = %f", tempSpeed);
		LCD.printLine(8, "tempspeed = %f", tempSpeed);

		//liftMotor.set(tempSpeed);
		liftMotorCan.set(tempSpeed);
	}
}