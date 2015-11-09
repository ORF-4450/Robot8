
package Team4450.Robot8;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.camera.AxisCamera;
//import edu.wpi.first.wpilibj.camera.AxisCamera.ResolutionT;

public class Autonomous
{
	private final Robot		robot;
	private final int		program = (int) SmartDashboard.getNumber("AutoProgramSelect");
	private Lift			lift;
	
	// encoder is plugged into dio port 2 - orange=+5v blue=signal, dio port 3 black=gnd yellow=signal. 
	private Encoder		encoder = new Encoder(2, 3, true, EncodingType.k4X);

	Autonomous(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		lift = 	new Lift(robot);
	}

	public void dispose()
	{
		Util.consoleLog();
		
		lift.dispose();
		encoder.free();
	}

	public void execute()
	{
		Util.consoleLog("Alliance=%s, Location=%d, Program=%d, FMS=%b", robot.alliance.name(), robot.location, program, robot.ds.isFMSAttached());
		LCD.printLine(2, "Alliance=%s, Location=%d, FMS=%b, Program=%d", robot.alliance.name(), robot.location, robot.ds.isFMSAttached(), program);

		robot.robotDrive.setSafetyEnabled(false);
    
		switch (program)
		{
//			case 1:		// left field simple drive fwd to auto zone.
//				// drive fwd into auto zone.
//				robot.robotDrive.tankDrive(-.50, -.50);
//				Timer.delay(1.75);
//				robot.robotDrive.tankDrive(0, 0);
//				break;
				
			case 1:		// Move to bin and grasp.
				// Open grabber, drive fwd, stop, close grabber.
				lift.BinOpen();
				Timer.delay(1);
				robot.robotDrive.tankDrive(-.50, -.50);
				Timer.delay(1.1);
				robot.robotDrive.tankDrive(0, 0);
				lift.BinClose();
				break;
				
			case 2:		// Move to bin, grasp, lift, back into auto zone with encoder.
				// Open grabber, drive fwd, stop, close grabber.
				lift.BinOpen();
				Timer.delay(1);
				robot.robotDrive.tankDrive(-.50, -.50);
				Timer.delay(1.3);
				robot.robotDrive.tankDrive(0, 0);
				lift.BinClose();

				// Lift bin.
				lift.LiftUpDown(.50);
				Timer.delay(2.0);
				lift.LiftUpDown(.25);

				// drive backward for set distance then stop.
				encoder.reset();
				robot.robotDrive.tankDrive(.60, .60);
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 1300) 
				{
					LCD.printLine(7, "encoder=%d", encoder.get());
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(0, 0);				
				break;
				
//			case 2:		// right field simple drive fwd to auto zone over ramp.
//				// raise lift a bit.
//				lift.LiftUpDown(.50);
//				Timer.delay(.5);
//				lift.LiftUpDown(0);
//				// drive fwd into auto zone.
//				robot.robotDrive.tankDrive(-.50, -.50);
//				Timer.delay(2.5);
//				robot.robotDrive.tankDrive(0, 0);
//				break;
				
			case 3:		// left field pick up tote, reverse into auto zone.
				// move fwd to tote.
				robot.robotDrive.tankDrive(-.50, -.50);
				Timer.delay(.5);
				robot.robotDrive.tankDrive(0, 0);
				// pick up tote.
				lift.LiftUpDown(.50);
				Timer.delay(1);
				lift.LiftUpDown(0);
				lift.KickerOut();
				// drive backward into auto zone.
				robot.robotDrive.tankDrive(.50, .50);
				Timer.delay(3.5);
				robot.robotDrive.tankDrive(0, 0);
				break;
				
			case 4:		// right field pick up tote, reverse into auto zone over ramp.
				// move fwd to tote.
				robot.robotDrive.tankDrive(-.50, -.50);
				Timer.delay(.5);
				robot.robotDrive.tankDrive(0, 0);
				// pick up tote.
				lift.LiftUpDown(.50);
				Timer.delay(1);
				lift.LiftUpDown(0);
				lift.KickerOut();
				// drive backward into auto zone.
				robot.robotDrive.tankDrive(.50, .50);
				Timer.delay(5.0);
				robot.robotDrive.tankDrive(0, 0);
				break;
				
		}
		
		Util.consoleLog("end");
	}

	public void executeWithCamera()
	{
		Vision2014	vision = new Vision2014(robot);
		//ResolutionT		resolution ;
		//int					brightness;
		boolean		HotGoal = false;
    
		Util.consoleLog();
    
		robot.robotDrive.setSafetyEnabled(false);
    
		//brightness = robot.camera.getBrightness();
    
		//robot.camera.writeBrightness(0);
    
		//resolution = robot.camera.getResolution();
    
		//robot.camera.writeResolution(AxisCamera.ResolutionT.k640x480);
    
		for (int i = 0; i < 3; i++)
		{
			Timer.delay(0.1);
    
			if (vision.locateTarget() == Vision2014.HotTargetFound)
			{
				Util.consoleLog("Hot Goal Found");
				HotGoal = true;
				break;
			}
		}
    
		if (!HotGoal)
		{
			Timer.delay(2.0);
			Util.consoleLog("autonomous: Wait");
		}
		
		Util.consoleLog("autonomous: execute");
    
		execute();
    
		//robot.camera.writeBrightness(brightness);
    
		//robot.camera.writeResolution(resolution);
    
		/*
		while (robot->IsEnabled())
		{
			result = vision.locateTarget();
	
			LCD::ConsoleLog("autonomous: target=%d, X=%d, Y=%d, dist=%f", result, vision.centerX, vision.centerY, vision.distance);
			
			Wait(1.0);
		}
		*/
    
		vision.dispose();
		
		Util.consoleLog("end");
	}
}