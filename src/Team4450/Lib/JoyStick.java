
package Team4450.Lib;

import java.util.EventObject;
import java.util.EventListener;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Joystick;

// This class handles the interface to the Joystick buttons. It monitors
// the state of the JS buttons and raises events when button state changes.

public class JoyStick
{
	private final Object	caller;
	private final Joystick	joyStick;
	private 				Set<JoyStickEventListener> listeners = new HashSet<JoyStickEventListener>();
	private					Set<JoyStickButton> buttons = new HashSet<JoyStickButton>();
	private Thread			monitorJoyStickThread;
	private String			joyStickName = "";
	
	public  double			deadZone = 0.1;
	
	// Constructor which adds all buttons.
	
	public JoyStick(Joystick joystick, String name, Object caller)
	{
		JoyStickButton	button;
		
		Util.consoleLog(name);

		joyStick = joystick;
		joyStickName = name;
		this.caller = caller;
		
		// Build set of all the joystick buttons which will be monitored.
		
		button = new JoyStickButton(JoyStickButtonIDs.TRIGGER);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.BACK_LEFT);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.BACK_RIGHT);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.LEFT_FRONT);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.LEFT_REAR);
		buttons.add(button);

		button = new JoyStickButton(JoyStickButtonIDs.RIGHT_FRONT);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.RIGHT_REAR);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.TOP_BACK);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.TOP_LEFT);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.TOP_MIDDLE);
		buttons.add(button);
		
		button = new JoyStickButton(JoyStickButtonIDs.TOP_RIGHT);
		buttons.add(button);

		Start();
	}
	
	// Constructor which adds single button to be monitored.
	
	public JoyStick(Joystick joystick, String name, JoyStickButtonIDs button, Object caller)
	{
		Util.consoleLog(name);

		this.joyStick = joystick;
		joyStickName = name;
		this.caller = caller;
		
		AddButton(button);
	}
	
	// Add additonal button to be monitored.
	
	public JoyStickButton AddButton(JoyStickButtonIDs button)
	{
		Util.consoleLog("%s (%s)", joyStickName, button.name());
	
		JoyStickButton jsButton = FindButton(button);
		
		if (jsButton == null)
		{
			jsButton = new JoyStickButton(button);
			buttons.add(jsButton);
		}

		return jsButton;
	}
	
	public JoyStickButton FindButton(JoyStickButtonIDs button)
	{
		Util.consoleLog("%s (%s)", joyStickName, button.name());

        for (JoyStickButton jsButton: buttons) 
        	if (jsButton.id.value == button.value) return jsButton;

        return null;
	}

	// Call to start Joystick monitoring once all buttons are added.
	
	public void Start()
	{
		Util.consoleLog(joyStickName);

		monitorJoyStickThread = new MonitorJoyStick();
		monitorJoyStickThread.start();
	}

	public void Stop()
	{
		Util.consoleLog(joyStickName);

		monitorJoyStickThread.interrupt();
	}
	
	public void dispose()
	{
		Util.consoleLog(joyStickName);
		
		monitorJoyStickThread.interrupt();
	}

	public double GetX()
	{
		double x;
		
		x = joyStick.getX();
		if (Math.abs(x) < deadZone) x = 0.0;
		return x;
	}
	
	public double GetY()
	{
		double y;
		
		y = joyStick.getY();
		if (Math.abs(y) < deadZone) y = 0.0;
		return y;
	}
	
	// JoyStick Button Monitor thread.
	
	private class MonitorJoyStick extends Thread
	{
		boolean previousState;
  	
		MonitorJoyStick()
		{
			Util.consoleLog(joyStickName);
			
			this.setName("Monitor " + joyStickName);
	    }
		
//		public void interrupt()
//		{
//	    	Util.consoleLog("Joystick.%s.MonitorJoyStick.interrupt", joyStickName);
//	    	super.interrupt();
//		}
		
	    public void run()
	    {
	    	Util.consoleLog(joyStickName);
	    	
	    	try
	    	{
    	    	while (!isInterrupted())
    	    	{
    	    		// Loop through the set of joystick buttons and read the value of each
    	    		// saving the button state and raising events for change in button state.
    	    		
    	            for (JoyStickButton button: buttons) 
    	            {
          	    		if (joyStick.getRawButton(button.id.value))
          				{
          					previousState = button.currentState;
          					button.currentState = true;
          						
          					if (!previousState)
          					{
          						button.latchedState = !button.latchedState;
          						
          						notifyButtonDown(button);
          					}
          				}
          				else
          				{
          					previousState = button.currentState;
          					button.currentState = false;
          					
          					if (previousState) notifyButtonUp(button);
          				}
    	            }
    	            
    	            // We sleep since JS updates come from DS every 20ms or so. We wait 50ms so this thread
    	            // does not run at the same time as the teleop thread.
    	            sleep(50);
    	    	}
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
	    }
	}	// end of MonitorJoystick thread class.
	
	public boolean GetCurrentState(JoyStickButtonIDs requestedbutton)
	{
      for (JoyStickButton button: buttons) 
      	if (button.id.equals(requestedbutton)) return button.currentState;
      
      return false;
	}
	
	public boolean GetLatchedState(JoyStickButtonIDs requestedbutton)
	{
      for (JoyStickButton button: buttons) 
      	if (button.id.equals(requestedbutton)) return button.latchedState;
      
      return false;
	}
	
  // Event Handling classes.
	
  // Event description class returned to event handlers.
	
  public class JoyStickEvent extends EventObject 
  {
	  private static final long serialVersionUID = 1L;

	  public JoyStickButton	button;
	
	  public JoyStickEvent(Object source, JoyStickButton button) 
	  {
		  super(source);
		  this.button = button;
	  }
  }
  
  // Interface defintion for event listener. Actual listener implements
  // the actions associated with button up and down events.
  
  public interface JoyStickEventListener extends EventListener 
  {
      public void ButtonDown(JoyStickEvent JoyStickEvent);
      
      public void ButtonUp(JoyStickEvent JoyStickEvent);
  }
  
  // Add (register) an event listener.
  
  public void addJoyStickEventListener(JoyStickEventListener listener) 
  {
      this.listeners.add(listener);
  }
   
  // Remove an event listener registration.
  
  public void removeJoyStickEventListener(JoyStickEventListener listener) 
  {
      this.listeners.remove(listener);
  }  
  
  // Notify all registered handlers of button up event.
  
  private void notifyButtonUp(JoyStickButton button) 
  {
      for (JoyStickEventListener JoyStickEventListener: listeners) 
      {
          JoyStickEventListener.ButtonUp(new JoyStickEvent(caller, button));
      }
  }
  
  // Notify all registered handlers of button down event.
  
  private void notifyButtonDown(JoyStickButton button) 
  {
      for (JoyStickEventListener JoyStickEventListener: listeners) 
      {
          JoyStickEventListener.ButtonDown(new JoyStickEvent(caller, button));
      }
  }

  // Button object which contains button id and current and latched state of the button
  // when contained in an event and if you directly request button state.
  
  public class JoyStickButton
  {
	  public JoyStickButtonIDs	id;
      public boolean			currentState, latchedState;
  
      public JoyStickButton(JoyStickButtonIDs buttonID)
      {
    	  id = buttonID;
      }
  }
  
  // Joystick button id enumeration. 
  
  public enum JoyStickButtonIDs
  {
      TOP_MIDDLE (3),
      TOP_LEFT (4),
      TOP_RIGHT (5),
      TRIGGER (1),
      TOP_BACK (2),
      LEFT_FRONT (6),
      LEFT_REAR (7),
      RIGHT_FRONT (11),
      RIGHT_REAR (10),
      BACK_LEFT (8),
      BACK_RIGHT (9);
      
      public int value;
      
      private JoyStickButtonIDs(int value) 
      {
    	  this.value = value;
      }
  };
}
