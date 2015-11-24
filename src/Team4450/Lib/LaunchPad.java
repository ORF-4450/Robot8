
package Team4450.Lib;

import java.util.EventObject;
import java.util.EventListener;
import java.util.HashSet;
import java.util.Set;








//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

// This class handles the interface to the TI Launch Pad device. It monitors
// the state of the LP controls and raises events when control state changes.
// It currently only supports buttons.

public class LaunchPad
{
	private Object			caller;
	private	Joystick		joyStick;
	private 				Set<LaunchPadEventListener> listeners;
	private					Set<LaunchPadControl> controls;
	private Thread			monitorLaunchPadThread;

	public LaunchPad(Joystick	joystick, Object caller)
	{
		LaunchPadControl	control;
		
		Util.consoleLog();

		try
		{
    		this.joyStick = joystick;
    		this.caller = caller;
    		
    		// Build full set of launch pad controls and register them
    		// for monitoring.
    		
    		listeners = new HashSet<LaunchPadEventListener>();
    		controls = new HashSet<LaunchPadControl>();
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_ONE);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_TWO);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_THREE);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_FOUR);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_FIVE);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_SIX);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_SEVEN);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_EIGHT);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_NINE);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_TEN);
    		controls.add(control);
    		
    		control = new LaunchPadControl(LaunchPadControlIDs.BUTTON_ELEVEN);
    		controls.add(control);

    		Start();
		}
		catch (Exception  e) {e.printStackTrace(Util.logPrintStream);}
	}
	
	public LaunchPad(Joystick joystick, LaunchPadControlIDs controlID, Object caller)
	{
		Util.consoleLog(controlID.name());

		try
		{
    		this.joyStick = joystick;
    		this.caller = caller;
    		
    		listeners = new HashSet<LaunchPadEventListener>();
    		controls = new HashSet<LaunchPadControl>();
    		
    		AddControl(controlID);
    	}
    	catch (Exception  e) {e.printStackTrace(Util.logPrintStream);}
	}
	
	// Add additional control to be monitored.
	
	public LaunchPadControl AddControl(LaunchPadControlIDs controlID)
	{
		Util.consoleLog(controlID.name());

//		try
//		{
//			if (!controls.contains(control)) controls.add(control);
//		}
//		catch (Exception  e) {e.printStackTrace(Util.logPrintStream);}

		LaunchPadControl control = FindButton(controlID);
		
		if (control == null)
		{
			control = new LaunchPadControl(controlID);
			controls.add(control);
		}

		return control;
	}
	
	public LaunchPadControl FindButton(LaunchPadControlIDs controlID)
	{
		Util.consoleLog(controlID.name());

        for (LaunchPadControl control: controls) 
        	if (control.id.value == controlID.value) return control;

        return null;
	}

	// Call to start LP monitoring once all controls are added.
	
	public void Start()
	{
		Util.consoleLog();
		
		monitorLaunchPadThread = new MonitorLaunchPad();
		monitorLaunchPadThread.start();
	}
	
	public void Stop()
	{
		Util.consoleLog();
		
		monitorLaunchPadThread.interrupt();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		monitorLaunchPadThread.interrupt();
	}

	// Launch Pad Monitor thread.
	
	private class MonitorLaunchPad extends Thread
	{
    	boolean	previousState;
    	
		MonitorLaunchPad()
		{
			Util.consoleLog();
			this.setName("MonitorLaunchPad");
	    }
	    
	    public void run()
	    {
	    	Util.consoleLog();
	    	
	    	try
	    	{
    	    	while (!isInterrupted())
    	    	{
    	    		// Loop through the set of Launch Pad controls and read the value of each
    	    		// saving the control state and raising events for change in control state.
    	    		
    	            for (LaunchPadControl control: controls) 
    	            {
    	            	if (control.controlType.equals(LaunchPad.LaunchPadControlTypes.BUTTON))
    	            	{
    	            		// Checking not because the buttons on DS are wired backwards.
        	            	if (!joyStick.getRawButton(control.id.value)) //(control.joyStickButton.value))
            				{
            					previousState = control.currentState;
            					control.currentState = true;
            					
            					if (!previousState)
            					{
            						control.latchedState = !control.latchedState;
            						
            						notifyButtonDown(control);
            					}
            				}
            				else
            				{
            					previousState = control.currentState;
            					control.currentState = false;
            					
            					if (previousState) notifyButtonUp(control);
            				}
    	            	}
    	            	
    	            	if (control.controlType.equals(LaunchPad.LaunchPadControlTypes.SWITCH))
    	            	{
        					previousState = control.currentState;
        					
        					control.currentState = joyStick.getRawButton(control.id.value); //(control.joyStickButton.value);
        					
        					control.latchedState = control.currentState;
        							
        					if (control.currentState != previousState) notifySwitchChange(control);
    	            	}
    	            }
    	            
    	            sleep(50);
    	    	}
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
	    }
	}
	
	public boolean GetCurrentState(LaunchPadControlIDs requestedControl)
	{
        for (LaunchPadControl control: controls) 
        	if (control.equals(requestedControl)) return control.currentState;
        
        return false;
	}
	
	public boolean GetLatchedState(LaunchPadControlIDs requestedControl)
	{
        for (LaunchPadControl control: controls) 
        	if (control.id.equals(requestedControl)) return control.latchedState;
        
        return false;
	}
	
	// Event Handling classes.
	
    public class LaunchPadEvent extends EventObject 
    {
		private static final long serialVersionUID = 1L;

		public LaunchPadControl	control;
		
		public LaunchPadEvent(Object source, LaunchPadControl control) 
		{
            super(source);
            this.control = control;
        }
    }
    
    public interface LaunchPadEventListener extends EventListener 
    {
        public void ButtonDown(LaunchPadEvent launchPadEvent);
        
        public void ButtonUp(LaunchPadEvent launchPadEvent);
        
        public void SwitchChange(LaunchPadEvent launchPadEvent);
    }
    
    public void addLaunchPadEventListener(LaunchPadEventListener listener) 
    {
        this.listeners.add(listener);
    }
     
    public void removeLaunchPadEventListener(LaunchPadEventListener listener) 
    {
        this.listeners.remove(listener);
    }  
    
    private void notifyButtonUp(LaunchPadControl control) 
    {
        for (LaunchPadEventListener launchPadEventListener: listeners) 
        {
            launchPadEventListener.ButtonUp(new LaunchPadEvent(this, control));
        }
    }
    
    private void notifyButtonDown(LaunchPadControl control) 
    {
        for (LaunchPadEventListener launchPadEventListener: listeners) 
        {
            launchPadEventListener.ButtonDown(new LaunchPadEvent(caller, control));
        }
    }
    
    private void notifySwitchChange(LaunchPadControl control) 
    {
        for (LaunchPadEventListener launchPadEventListener: listeners) 
        {
            launchPadEventListener.SwitchChange(new LaunchPadEvent(caller, control));
        }
    }
    
//    private void notifyButtonUp(int control) 
//    {
//        for (LaunchPadEventListener launchPadEventListener: listeners) 
//        {
//            launchPadEventListener.ButtonUp(new LaunchPadEvent(this, control));
//        }
//    
    public enum LaunchPadControlTypes
    {
    	BUTTON,
    	SWITCH
    };
    
    public class LaunchPadControl
    {
    	public  LaunchPadControlIDs		id;
    	public 	LaunchPadControlTypes	controlType = LaunchPadControlTypes.BUTTON;
        public	boolean					currentState, latchedState;
    	
        public LaunchPadControl(LaunchPadControlIDs controlID)
        {
      	  id = controlID;
        }
    }
    
    public enum LaunchPadControlIDs
    {
        BUTTON_ONE (1),
        BUTTON_TWO (2),
        BUTTON_THREE (3),
        BUTTON_FOUR (4),
        BUTTON_FIVE (5),
        BUTTON_SIX (6),
        BUTTON_SEVEN (7),
        BUTTON_EIGHT (8),
        BUTTON_NINE (9),
        BUTTON_TEN (10),
        BUTTON_ELEVEN (11);
        
        private int value;

        private LaunchPadControlIDs(int value) 
        {
      	  this.value = value;
        }
    };
}