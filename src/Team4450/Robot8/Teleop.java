
package Team4450.Robot8;

import java.lang.Math;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.GamePad;
import Team4450.Lib.JoyStick;
import Team4450.Lib.LCD;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.Util;
import Team4450.Lib.GamePad.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;

class Teleop
{
	private final 		Robot robot;
	private 			double	powerFactor = 1.0;
	private JoyStick	rightStick, leftStick, utilityStick;
	private LaunchPad	launchPad;
	private GamePad		gamePad;
	private Lift		lift;
	
	private BuiltInAccelerometer	accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	
	// encoder is plugged into dio 2 - orange=+5v blue=signal, dio 3 black=gnd yellow=signal. 
	private Encoder		encoder = new Encoder(2, 3, true, EncodingType.k4X);

	//private DigitalInput	proxSwitch = new DigitalInput(4);
	private AnalogInput		proxSwitchA = new AnalogInput(2);
	
	// Constructor.
	
	Teleop(Robot robot)
	{
		Util.consoleLog();

		this.robot = robot;
		
		lift = 	new Lift(robot);
	}

	// Free all objects that need it.
	
	void dispose()
	{
		Util.consoleLog();
		
		lift.dispose();
		if (leftStick != null) leftStick.dispose();
		if (rightStick != null) rightStick.dispose();
		if (utilityStick != null) utilityStick.dispose();
		if (launchPad != null) launchPad.dispose();
		if (gamePad != null) gamePad.dispose();
		encoder.free();
	}

	void OperatorControl()
	{
		double			rightY, leftY;
        
        // Motor safety turned off during initialization.
        robot.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();
		
		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, robot.ds.isFMSAttached());

		SmartDashboard.putNumber("Power Factor", powerFactor * 100);
		
		// Configure LaunchPad and Joystick event handlers.
		
		launchPad = new LaunchPad(robot.launchPad, LaunchPadControlIDs.BUTTON_SIX, this);
		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.BUTTON_FOUR);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_ONE);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_TWO);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_EIGHT);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_ELEVEN);
        launchPad.addLaunchPadEventListener(new LaunchPadListener());
        launchPad.Start();

		leftStick = new JoyStick(robot.leftStick, "LeftStick", JoyStickButtonIDs.TOP_LEFT, this);
		leftStick.AddButton(JoyStickButtonIDs.TOP_RIGHT);
		leftStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		leftStick.AddButton(JoyStickButtonIDs.TOP_BACK);
        leftStick.addJoyStickEventListener(new LeftStickListener());
        leftStick.Start();
        
		rightStick = new JoyStick(robot.rightStick, "RightStick", JoyStickButtonIDs.TOP_LEFT, this);
        rightStick.addJoyStickEventListener(new RightStickListener());
        rightStick.Start();
        
		utilityStick = new JoyStick(robot.utilityStick, "UtilityStick", JoyStickButtonIDs.TOP_LEFT, this);
		utilityStick.AddButton(JoyStickButtonIDs.TRIGGER);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		utilityStick.AddButton(JoyStickButtonIDs.TRIGGER);
        utilityStick.addJoyStickEventListener(new UtilityStickListener());
        utilityStick.Start();
        
//        gamePad = new GamePad(robot.gamePad, "GamePad 1", this);
//        gamePad.addGamePadEventListener(new GamePadListener());
//        GamePadButton povButton = gamePad.FindButton(GamePadButtonIDs.POV);
//        gamePad.Start();
        
        // Motor safety turned on.
        robot.robotDrive.setSafetyEnabled(true);

        robot.gyro.reset();
        
		// Driving loop runs until teleop is over.

		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object.

			// using direct calls to joystick objects.
//			rightY = robot.rightStick.getY();		// fwd/back right
//			leftY = robot.leftStick.getY();		// fwd/back left
			
			// using calls to our JoyStick class.
			rightY = rightStick.GetY();		// fwd/back right
			leftY = leftStick.GetY();		// fwd/back left

			//LCD.printLine(4, "leftY=%.4f  rightY=%.4f  pov=%d pov2=%d", leftY, rightY, povButton.lastPOVAngle, gamePad.GetLastPOVangle());
			LCD.printLine(4, "leftY=%.4f  rightY=%.4f", leftY, rightY);

			// GamePad testing.
			//rightY = gamePad.GetRightY();
			//leftY = gamePad.GetLeftY();
			
			//LCD.printLine(4, "lx=%.4f  ly=%.4f  rx=%.4f  ry=%.4f", gamePad.GetLeftX(), gamePad.GetLeftY(), gamePad.GetRightX(), gamePad.GetRightY());
			//LCD.printLine(5, "lt=%.4f  rt%.4f  pov=%d pov2=%d", gamePad.GetLeftTrigger(), gamePad.GetRightTrigger(), povButton.lastPOVAngle, gamePad.GetLastPOVangle());

			// dead zone. Not used here as our JoyStick class has dead zone built in.
			// Need this if you read the JS directly.
			//if (Math.abs(rightY) < .1) rightY = 0;
			//if (Math.abs(leftY) < .1) leftY = 0;
			
			// This corrects stick alignment error when trying to drive straight. 
			if (Math.abs(rightY - leftY) < 0.2) rightY = leftY;
			
			//LCD.printLine(5, "leftY=%.4f  rightY=%.4f, power=%f", leftY, rightY, powerFactor);
			
			// Set motors.

			robot.robotDrive.tankDrive(leftY * powerFactor, rightY * powerFactor);

			// Set lift direction/speed.
			
			LCD.printLine(6, "liftY=%.4f", robot.utilityStick.getY());

			lift.LiftUpDown(robot.utilityStick.getY());
			
			// launchpad button direct access example. Access button state without event handler.
			
			if (robot.leftStick.getRawButton(JoyStickButtonIDs.TRIGGER.value))
			{
				Util.consoleLog("button 4  cs=%b  ls=%b", launchPad.GetCurrentState(LaunchPadControlIDs.BUTTON_FOUR), launchPad.GetLatchedState(LaunchPadControlIDs.BUTTON_FOUR));
				Util.consoleLog("topmiddle cs=%b  ls=%b", rightStick.GetCurrentState(JoyStickButtonIDs.TOP_MIDDLE), rightStick.GetLatchedState(JoyStickButtonIDs.TOP_MIDDLE));
			}

			LCD.printLine(7, "encoder=%d", encoder.get());

			LCD.printLine(9, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());

			//LCD.printLine(10, "aX=%d, aY=%d, aZ=%d", (int) (accel.getX() * 10), (int) (accel.getY() * 10), (int) (accel.getZ() * 10));

			// Test printline with column.
			//LCD.printLine(10, "1234567890123456789012345678901234567890");
			//LCD.print(10, 5, "ZZZ");
			//LCD.print(10, 20, "ZZZ");
			
			// Test accelerometer.
			//Util.consoleLog("aX=%d, aY=%d, aZ=%d", (int) (accel.getX() * 10), (int) (accel.getY() * 10), (int) (accel.getZ() * 10));
			
			//LCD.printLine(5, "proximity switch=%b", proxSwitch.get());
			LCD.printLine(5, "proximity switch=%d", proxSwitchA.getValue());
					
			// End of driving loop.
			
			Timer.delay(.020);	// wait 20ms for update from driver station.
		}
		
		proxSwitchA.free();
		
		// End of teleop mode.
		
		Util.consoleLog("end");
	}

	// Handle LaunchPad control events.
	
	public class LaunchPadListener implements LaunchPadEventListener 
	{
	    public void ButtonDown(LaunchPadEvent launchPadEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.id.name(),  launchPadEvent.control.latchedState);
			
			// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (launchPadEvent.control.id.equals(LaunchPad.LaunchPadControlIDs.BUTTON_SIX))
				if (launchPadEvent.control.latchedState)
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam2);
				else
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam1);
			
			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_ONE)
			{
				((Teleop) launchPadEvent.getSource()).powerFactor = 1.0;
				SmartDashboard.putNumber("Power Factor", ((Teleop) launchPadEvent.getSource()).powerFactor * 100);
			}
			
			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_TWO)
				((Teleop) launchPadEvent.getSource()).encoder.reset();
			
			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_EIGHT)
			{
				((Teleop) launchPadEvent.getSource()).powerFactor = 0.5;
				SmartDashboard.putNumber("Power Factor", ((Teleop) launchPadEvent.getSource()).powerFactor * 100);
			}

			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_ELEVEN)
    			if (launchPadEvent.control.latchedState)
//    				((Teleop) launchPadEvent.getSource()).lift.KickerOut();
//    			else
//    				((Teleop) launchPadEvent.getSource()).lift.KickerIn();
    				((Teleop) launchPadEvent.getSource()).lift.BinOpen();
    			else
    				((Teleop) launchPadEvent.getSource()).lift.BinClose();
	    }
	    
	    public void ButtonUp(LaunchPadEvent launchPadEvent) 
	    {
	    	//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
	    }

	    public void SwitchChange(LaunchPadEvent launchPadEvent) 
	    {
	    	Util.consoleLog("%s", launchPadEvent.control.id.name());

	    	// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (launchPadEvent.control.id.equals(LaunchPadControlIDs.BUTTON_FOUR))
				if (launchPadEvent.control.latchedState)
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam2);
				else
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam1);
	    }
	}

	// Handle Right JoyStick Button events.
	
	private class RightStickListener implements JoyStickEventListener 
	{
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			// Switch dashboard camera selection control between USB and IP camera.
			
//			if (joyStickEvent.button.equals(JoyStickButtons.TOP_MIDDLE))
//				if (joyStickEvent.button.latchedState)
//					SmartDashboard.putNumber("CameraSelect", robot.IP_CAMERA);
//				else
//					SmartDashboard.putNumber("CameraSelect", robot.USB_CAMERA);
			
			// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT))
				if (joyStickEvent.button.latchedState)
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam2);
				else
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam1);			
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.name());
	    }
	}

	// Handle Left JoyStick Button events.
	
	private class LeftStickListener implements JoyStickEventListener 
	{
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			// Change the power factor setting.

			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT)) ((Teleop) joyStickEvent.getSource()).powerFactor = 1.0;
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_RIGHT)) ((Teleop) joyStickEvent.getSource()).powerFactor = 0.5;
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
			{
				if (((Teleop) joyStickEvent.getSource()).powerFactor == 1.0)
					((Teleop) joyStickEvent.getSource()).powerFactor = .75;
				else if (((Teleop) joyStickEvent.getSource()).powerFactor == .75)
					((Teleop) joyStickEvent.getSource()).powerFactor = .50;
				else if (((Teleop) joyStickEvent.getSource()).powerFactor == .50)
					((Teleop) joyStickEvent.getSource()).powerFactor = .25;
			}
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_BACK))
			{
				if (((Teleop) joyStickEvent.getSource()).powerFactor == .25)
					((Teleop) joyStickEvent.getSource()).powerFactor = .50;
				else if (((Teleop) joyStickEvent.getSource()).powerFactor == .50)
					((Teleop) joyStickEvent.getSource()).powerFactor = .75;
				else if (((Teleop) joyStickEvent.getSource()).powerFactor == .75)
					((Teleop) joyStickEvent.getSource()).powerFactor = 1.0;
			}

			SmartDashboard.putNumber("Power Factor", ((Teleop) joyStickEvent.getSource()).powerFactor * 100);
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.name());
	    }
	}

	// Handle Utility JoyStick Button events.
	
	private class UtilityStickListener implements JoyStickEventListener 
	{
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			// Switch dashboard camera selection control between USB and IP camera.
			
//			if (joyStickEvent.button.equals(JoyStickButtons.TOP_MIDDLE))
//				if (joyStickEvent.button.latchedState)
//					SmartDashboard.putNumber("CameraSelect", robot.IP_CAMERA);
//				else
//					SmartDashboard.putNumber("CameraSelect", robot.USB_CAMERA);
			
			// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT))
				if (joyStickEvent.button.latchedState)
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam2);
				else
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam1);
			
//			if (joyStickEvent.button.equals(JoyStickButtons.TRIGGER))
//				if (joyStickEvent.button.latchedState)
//					((MyTeleop) joyStickEvent.getSource()).lift.KickerOut();
//				else
//					((MyTeleop) joyStickEvent.getSource()).lift.KickerIn();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TRIGGER))
				if (joyStickEvent.button.latchedState)
//					((Teleop) joyStickEvent.getSource()).lift.BinOpen();
//				else
//					((Teleop) joyStickEvent.getSource()).lift.BinClose();
					((Teleop) joyStickEvent.getSource()).lift.KickerOut();
				else
					((Teleop) joyStickEvent.getSource()).lift.KickerIn();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
				if (joyStickEvent.button.latchedState)
					((Teleop) joyStickEvent.getSource()).lift.BinOpen();
				else
					((Teleop) joyStickEvent.getSource()).lift.BinClose();
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.id.name());
	    }
	}

	// Handle GamePad Button events.
	
	private class GamePadListener implements GamePadEventListener 
	{
	    public void ButtonDown(GamePadEvent gamePadEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b povA=%d", gamePadEvent.button.id.name(),  gamePadEvent.button.latchedState, gamePadEvent.button.povAngle);
	    }
	    
	    public void ButtonUp(GamePadEvent gamePadEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.id.name());
	    }
	}
}
