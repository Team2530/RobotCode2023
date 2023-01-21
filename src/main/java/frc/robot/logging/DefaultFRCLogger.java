package frc.robot.logging;

import java.io.*;
import java.util.Date;

/**
 * 
 * Default FRC Logger Implementation
 * 
 * @author Michael M.
 */
public class DefaultFRCLogger implements FRCLogger {

    /**
     * The logger prefix. It is appended to the start of all messages.
     */
    private final String prefix;

    /**
     * Whether the logger should log to a file.
     */
    private final boolean logToFile;

    /**
     * Whether the logger should log to console.
     */
    private final boolean logToConsole;

    private File loggingDir;
    private File logFile;

    /**
     * Constructs a new DefaultFRCLogger
     * 
     * @param prefix       The prefix, this will be printed before each message.
     * @param logToFile    Whether the logger should log to a file.
     * @param logToConsole Whether the logger should log to console.
     */
    public DefaultFRCLogger(String prefix, boolean logToFile, boolean logToConsole) {
        this.prefix = prefix;
        this.logToFile = logToFile;
        this.logToConsole = logToConsole;

        this.loggingDir = new File("logs");
    }

    /**
     * Sets the default directory to put log files in.
     * 
     * @param loggingDir The new directory to put log files in
     * @throws IllegalArgumentException if logging to files is disabled
     */
    public void setLogDirectory(File loggingDir) {
        if (!this.logToFile)
            throw new IllegalArgumentException("Cannot set log directory if not logging to a file!");

        this.loggingDir = loggingDir;
    }

    /**
     * Sets up the logger
     * 
     * @throws IllegalStateException if no output
     */
    @Override
    public void setup() {
        if (logToFile) {
            if (!this.loggingDir.exists()) {
                this.loggingDir.mkdir();
            }

            this.logFile = new File(this.loggingDir, String.format("log-%s.txt", new Date().toString()));
            System.out.println(this.logFile.getAbsolutePath());
            try {
                this.logFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if(!logToConsole && !logToFile) {
            throw new IllegalStateException("I don't print to the console. I don't write to files. Why do I exist?");
        }
    }

    @Override
    public void info(String s, Object... objects) {
        String formattedString = prefix + String.format(s, objects);

        if (logToConsole) {
            System.out.println(formattedString);
        }
        if (logToFile) {
            writelnToFile(formattedString);
        }
    }

    @Override
    public void infoIf(boolean condition, String s, Object... objects) {
        if(condition) {
            this.info(s, objects);
        }
    }

    @Override
    public void warn(String s, Object... objects) {
        String formattedString = prefix + String.format(s, objects);

        if (logToConsole) {
            System.out.println(formattedString);
        }
        if (logToFile) {
            writelnToFile(formattedString);
        }
    }

    @Override
    public void warnIf(boolean condition, String s, Object... objects) {
        if(condition) {
            this.warn(s, objects);
        }
    }

    /**
     * @implNote This method prints to standard error, not standard out.
     */
    @Override
    public void error(String s, Object... objects) {
        String formattedString = prefix + String.format(s, objects);

        if (logToConsole) {
            System.err.println(formattedString);
        }
        if (logToFile) {
            writelnToFile(formattedString);
        }
    }

    @Override
    public void error(Throwable error) {
        this.error(error.getMessage());
    }

    @Override
    public void errorIf(boolean condition, String s, Object... objects) {
        if(condition) {
            this.error(s, objects);
        }
    }

    /**
     * Writes a line to a file.
     * 
     * @param line The line to write
     */
    private void writelnToFile(String line) {
        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(this.logFile, true));
            bw.write(line);
            bw.newLine();
            bw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
