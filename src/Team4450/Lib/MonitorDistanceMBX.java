
package Team4450.Lib;

import Team4450.Robot8.Robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Task to monitor ultrasonic sensor and report distance to driver station.
 * Runs as a thread separate from Robot class. Runs until our
 * program is terminated from the RoboRio.
 * Displays distance value on DS.
 * Uses MaxBotix ultrasonic sensor.
 */

public class MonitorDistanceMBX extends Thread
{
    Robot robot;
 
    /**
     * @param robot Robot class instance.
     */
	public MonitorDistanceMBX(Robot robot)
	{
		Util.consoleLog();
        this.robot = robot;
        this.setName("MonitorDistance");
    }
    
    public void run()
    {
        AnalogInput 	ultra = new AnalogInput(1);

		double rangeInches;
		double rangeFeet;

		try
		{
			Util.consoleLog();

			while (true)
			{
				if (robot.isEnabled())
				{
					rangeInches = ultra.getVoltage() / .0098;
					rangeFeet = rangeInches / 12;
                    
					//Util.consoleLog("range=" + Util.format(rangeFeet));

					if (rangeFeet > 20) rangeFeet = 0.0;
					
					//SmartDashboard.putString("Range", String.format("%f", rangeFeet));
					LCD.printLine(3, "range=%f, voltage=%f", rangeFeet, ultra.getVoltage());
				}
				else
				{
					//SmartDashboard.putString("Range", "0.0");
					LCD.printLine(3, "range=%f", 0.0);
				}

				Timer.delay(1.0);
			}
		}
		catch (Throwable e)  {e.printStackTrace(Util.logPrintStream);}
	}
}
