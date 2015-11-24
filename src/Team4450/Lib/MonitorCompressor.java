
package Team4450.Lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;

/**
 * Compressor monitoring task.
 * Runs as a separate thread from our MyRobot class. Runs until our
 * program is terminated from the RoboRio. Displays compressor on/off
 * LED on DS.
 */

public class MonitorCompressor extends Thread
{
  private final Compressor	compressor = new Compressor(0);
    
  public MonitorCompressor()
  {
	  Util.consoleLog();
	  this.setName("MonitorCompressor");
  }
    
  /**
   * Start monitoring. Called by Thread.start().
   */
  public void run()
  {      
	boolean	saveState = false, compressorState;
	
	try
	{
		Util.consoleLog();

		while (true)
		{
			compressorState = compressor.enabled();
			
			if (compressorState != saveState)
			{
				saveState = compressorState;
				SmartDashboard.putBoolean("Compressor", saveState);
				Util.consoleLog("compressor on=%b", saveState);
			}
			
			Timer.delay(2.0);
		}
	}
	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }
}
