package frc.robot.libraries;

<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartShuffle{
    public static void put(String key, int n) {
        SmartDashboard.putNumber(key, n);
    }

    public static void put(String key, boolean b) {
        SmartDashboard.putBoolean(key, b);
    }

    public static void put(String key, String myString) {
        SmartDashboard.putString(key, myString);
=======
import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartShuffle {
    private String key;
    private Object value;
    /** Network Tables Entry */
    private GenericEntry entry;
    /**Widget type */
    static BuiltInWidgets widget;

    /** Current Table to make values on */
    private static String currentTable = "Test Values";

    /** All created SmartShuffle items */
    private static HashMap<String, SmartShuffle> map = new HashMap<String, SmartShuffle>();

    /** Creates a SmartShuffle object */
    private SmartShuffle(String key, Object value) {
        this.key = key;
        this.value = value;
        entry = Shuffleboard.getTab(currentTable).add(key, value).getEntry();
    }

    /**
     * Adds a SmartShuffle object to the map
     * 
     * @param key   name of SmartShuffle
     * @param value value in the SmartShuffle
     */
    private static void add(String key, Object value) {
        map.put(key, new SmartShuffle(key, value));
    }

    /**
     * Gets the SmartShuffle specified
     * 
     * @param key          the name of the SmartShuffle you would like to access
     * if it doesn't already exist, it creates one with the specified name
     * @param defaultValue the default value for the type of
     *                     SmartShuffle object
     * @return the SmartShuffle Object
     */
    public static SmartShuffle get(String key, Object defaultValue) {
        if (!map.containsKey(key)) {
            SmartShuffle.add(key, defaultValue);
        }
        return map.get(key);
    }
    /**
     * Set the value of the SmartShuffle
     * @param value value you would like to set it to
     */
    public void setValue(Object value) {
        this.value = value;
        if (value instanceof String) {
            entry.setString((String) value);
        } else if (value instanceof Integer) {
            entry.setInteger((Integer) value);
        } else if(value instanceof Boolean) {
            entry.setBoolean((Boolean) value);
        } else if(value instanceof Double) {
            entry.setDouble((Double) value);
        }
    }

    public static void setCurrentTable(String currentTable) {
        SmartShuffle.currentTable = currentTable;
    }

    public static void setWidget(BuiltInWidgets widget) {
        SmartShuffle.widget = widget;
    }

    public void setPosition(int x, int y) {
>>>>>>> Stashed changes
    }
}