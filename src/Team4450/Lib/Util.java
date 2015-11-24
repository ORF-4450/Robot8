

package Team4450.Lib;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.Properties;
import java.util.TimeZone;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
//import java.util.logging.SimpleFormatter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Formatter;
import java.util.logging.LogRecord;

public class Util
{
	// PrintStream that writes to our logging system.
	public static final PrintStream	logPrintStream = new PrintStream(new LoggingOutputStream());

	// Logging class for use by other classes to log through our custom logging scheme. All
	// logging should be done by calls to methods on this class instance.
	public final static Logger logger = Logger.getGlobal();

	// Read our properties file from RoboRio memory.
	
	public static Properties readProperties() throws IOException
	{
		consoleLog();
		
		Properties props = new Properties();
		
		FileInputStream is = new FileInputStream("/home/lvuser/Robot.properties");
		
		props.load(is);
		
		is.close();
		
		//props.list(new PrintStream(System.out));
		props.list(logPrintStream);

		return props;
	}
	
	// Configures and holds (static) classes for our custom logging.
	
	public static class CustomLogger 
	{
        static private FileHandler 		fileTxt;
        //static private SimpleFormatter	formatterTxt;
        static private LogFormatter		logFormatter;
        
        // Initialize our logging system.
        static public void setup() throws IOException 
        {
            // get the global logger to configure it and add a file handler.
            Logger logger = Logger.getGlobal();
                   
            logger.setLevel(Level.ALL);

            // If we decide to redirect system.out to our log handler, then following
            // code will delete the default log handler for the console to prevent
            // a recursive loop. We would only redirect system.out if we only want to
            // log to the file. If we delete the console hanlder we can skip setting
            // the formatter...otherwise we set our formatter on the console logger.
            
            Logger rootLogger = Logger.getLogger("");

            Handler[] handlers = rootLogger.getHandlers();
            
//            if (handlers[0] instanceof ConsoleHandler) 
//            {
//                rootLogger.removeHandler(handlers[0]);
//                return;
//            }

            logFormatter = new LogFormatter();

            // Set our formatter on the console log handler.
            if (handlers[0] instanceof ConsoleHandler) handlers[0].setFormatter(logFormatter);

            // Now create a handler to log to a file on roboRio "disk".
            
            //if (true) throw new IOException("Test Exception");
            
            fileTxt = new FileHandler("/home/lvuser/Logging.txt");

            fileTxt.setFormatter(logFormatter);

            logger.addHandler(fileTxt);
        }
	}
	
	// Our custom formatter for logging output.
	
	private static class LogFormatter extends Formatter 
	{
        public String format(LogRecord rec) 
        {
            StringBuffer buf = new StringBuffer(1024);
            
            buf.append(String.format("<%d>", rec.getThreadID())); //Thread.currentThread().getId()));
            buf.append(formatDate(rec.getMillis()));
            buf.append(" ");
            buf.append(formatMessage(rec));
            buf.append("\n");
        
            return buf.toString();
        }
        
        private String formatDate(long millisecs) 
        {
            SimpleDateFormat dateFormat = new SimpleDateFormat("hh:mm:ss:S");
            dateFormat.setTimeZone(TimeZone.getTimeZone("America/Los_Angeles"));
            Date resultDate = new Date(millisecs);
            return dateFormat.format(resultDate);
        }
	}
	
	// An output stream that writes to our logging system. Writes data with flush on
	// flush call or on a newline character in the stream.
	
	private static class LoggingOutputStream extends OutputStream 
	{
	    private static final int	DEFAULT_BUFFER_LENGTH = 2048;
	    private boolean 			hasBeenClosed = false;
	    private byte[] 				buf;
	    private int 				count, curBufLength;

	    public LoggingOutputStream()
	    {
	        curBufLength = DEFAULT_BUFFER_LENGTH;
	        buf = new byte[curBufLength];
	        count = 0;
	    }

	    public void write(final int b) throws IOException 
	    {
	        if (hasBeenClosed) {throw new IOException("The stream has been closed.");}
	        
	        // don't log nulls
	        if (b == 0) return;
	        
	        // force flush on newline character, dropping the newline.
	        if ((byte) b == '\n') 
	        {
	        	flush();
	        	return;
	        }
	        
	        // would this be writing past the buffer?
	        if (count == curBufLength) 
	        {
	            // grow the buffer
	            final int newBufLength = curBufLength + DEFAULT_BUFFER_LENGTH;
	            final byte[] newBuf = new byte[newBufLength];
	            System.arraycopy(buf, 0, newBuf, 0, curBufLength);
	            buf = newBuf;
	            curBufLength = newBufLength;
	        }

	        buf[count] = (byte) b;
	        
	        count++;
	    }

	    @SuppressWarnings("deprecation")
		public void flush() 
	    {
	        if (count == 0) return;
	        
	        final byte[] bytes = new byte[count];

	        System.arraycopy(buf, 0, bytes, 0, count);
	        
	        String str = new String(bytes);
	        
	        LCD.consoleLogNoFormat(str);
	        
	        count = 0;
	    }

	    public void close() 
	    {
	        flush();
	        hasBeenClosed = true;
	    }
	}

	/**
     * Returns program location where call to this method is located.
     */
    public static String currentMethod()
    {
        return currentMethod(2);
    }

    private static String currentMethod(Integer level)
    {
        StackTraceElement stackTrace[];
    
        stackTrace = new Throwable().getStackTrace();
    
        try
        {
        	return stackTrace[level].toString().split("Robot8.")[1];
        }
        catch (Throwable e)
        {
        	return stackTrace[level].toString().split("Lib.")[1];
        }
	}

	// Works the same as LCD.consoleLog but automatically includes the program location from which
	// trace was called.
    
    /**
     * Write message to console log with optional formatting and program location.
     * @param message message with optional format specifiers for listed parameters
     * @param parms parameter list matching format specifiers
     */
	public static void consoleLog(String message, Object... parms)
	{
		// logs to the console as well as our log file on RR disk.
		logger.log(Level.INFO, String.format("robot: %s: %s", currentMethod(2), String.format(message, parms)));
	}
    
	/**
	 * Write blank line with program location to the console log.
	 */
	public static void consoleLog()
	{
		// logs to the console as well as our log file on RR disk.
		logger.log(Level.INFO, String.format("robot: %s", currentMethod(2)));
	}
}
