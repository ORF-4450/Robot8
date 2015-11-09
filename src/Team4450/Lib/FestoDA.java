
package Team4450.Lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Interface class for Festo Double Action pneumatic valve.
 * Double action is a sliding valve. Calling open applies power to the open side momentarily causing
 * the valve to move to the open position. Power is then turned off and the valve
 * stays where it is (open). Calling close applies power to the close side momentarily causing the
 * valve to move to the closed position. Power is then turned off and the valve
 * stays where it is (closed).
 */

public class FestoDA
{
	private final Solenoid	valveOpenSide, valveCloseSide;

	public double           solenoidSlideTime;

	/**
	 * @param port DIO port wired to open side of valve. Close side is wired to DIO next port.
	 */

	public FestoDA(int port)
	{
	  	Util.consoleLog("port=%d", port);

		valveOpenSide = new Solenoid(port);
		valveCloseSide = new Solenoid(port + 1);
    
		solenoidSlideTime = .05;
    
		//Close();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		valveOpenSide.free();
		valveCloseSide.free();
	}

	/**
	 * Open the valve.
	 */
	public void Open()
	{
		Util.consoleLog();
    
		valveCloseSide.set(false);
    
		valveOpenSide.set(true);
		Timer.delay(solenoidSlideTime);
		valveOpenSide.set(false);
	}

	/**
	 * Close the valve.
	 */
	public void Close()
	{
		Util.consoleLog();
    
		valveOpenSide.set(false);
    
		valveCloseSide.set(true);
		Timer.delay(solenoidSlideTime);
		valveCloseSide.set(false);
	}
}