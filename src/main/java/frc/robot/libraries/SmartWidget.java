package frc.robot.libraries;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SmartWidget {
    private String key;
    private Object value;
    /** Network Tables Entry */
    private SimpleWidget widget;

    private int ticks = 0;

    /** Current Table to make values on */
    private static String currentTable = "Test Values";

    /** All created SmartShuffle items */
    private static HashMap<String, SmartWidget> map = new HashMap<String, SmartWidget>();

    /** Creates a SmartShuffle object */
    private SmartWidget(String key, Object value) {
        this.key = key;
        this.value = value;
        widget = Shuffleboard.getTab(currentTable).add(key, value);
    }

    /**
     * Adds a SmartShuffle object to the map
     * 
     * @param key   name of SmartShuffle
     * @param value value in the SmartShuffle
     */
    private static void add(String key, Object value) {
        map.put(key, new SmartWidget(key, value));
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
    public static SmartWidget get(String key, Object defaultValue) {
        if (!map.containsKey(key)) {
            SmartWidget.add(key, defaultValue);
        }
        return map.get(key);
    }

    public static void setCurrentTable(String currentTable) {
        SmartWidget.currentTable = currentTable;
    }

    public void flashColor(String firstColor, String secondColor) {
        if(value instanceof Boolean) {
            if(ticks > 50) {
                ticks = 0;
            }
            if(ticks < 25) {
                widget.withProperties(Map.of("colorWhenTrue", firstColor));
            } else {
                widget.withProperties(Map.of("colorWhenTrue", secondColor));
            }
        } else {
            throw new Error("Flash can only be used with boolean");
        }
    }
}
