package frc.robot.libraries;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SmartShuffle {
    
    private GenericEntry entry;
    private SimpleWidget simpleWidget;

    // position of the placement of a SmartShuffle object
    private static int posx = 0, posy = 0;

    private static int width = 1;
    private static int height = 1;

    private int ticks;

    /** Use to change between different Shuffleboard tables */
    public static String tableName = "Driver Dashboard";

    private static BuiltInWidgets widget = BuiltInWidgets.kNumberBar;

    private static HashMap<String, SmartShuffle> shuffleObjects = new HashMap<String, SmartShuffle>();

    private SmartShuffle(String title, Object defaultValue) {
        simpleWidget = Shuffleboard.getTab(tableName)
                .add(title, defaultValue)
                .withWidget(widget) // with specified widget
                .withProperties(Map.of("min", -100, "max", 100)) // can be updated with different types
                .withPosition(posx, posy) // specify the widget here
                .withSize(width, height);

        entry = simpleWidget.getEntry();

        posx += width;
        // if we go off the screen
        if (posx >= 9) {
            posy++;
            posx = 6;
        }
    }

    /**
     * Switches between tables
     * 
     * @param name Name of table to switch to
     */
    public static void changeTable(String name) {
        tableName = name;
    }

    /**
     * Adds a SmartShuffle object to the list
     * 
     * @param title        name of shuffle object
     * @param defaultValue default value of the object
     *                     (0, "Hello", 1.5, etc.)
     */
    public static void add(String title, Object defaultValue) {
        shuffleObjects.put(title, new SmartShuffle(title, defaultValue));
    }

    /**
     * Updates the Entry on ShuffleBoard
     * 
     * @param o the value you want updated
     */
    public void update(Object o) {
        entry.setValue(o);
    }

    /**
     * Gets the SmartShuffle with the given name
     * 
     * @param name Name of SmartShuffle object
     * @return The SmartShuffle object with the given name
     */
    public static SmartShuffle get(String name) {
        return shuffleObjects.get(name);
    }

    public static void setHeight(int height) {
        SmartShuffle.height = height;
    }

    public static void setWidth(int width) {
        SmartShuffle.width = width;
    }

    public static void setPosx(int posx) {
        SmartShuffle.posx = posx;
    }

    public static void setPosy(int posy) {
        SmartShuffle.posy = posy;
    }

    public static void setWidget(BuiltInWidgets widget) {
        SmartShuffle.widget = widget;
    }

    /**
     * 
     * @param color Color you would like to display on boolean box
     */
    public void changeColor(String color) {
        simpleWidget.withProperties(Map.of("colorWhenTrue", color));
    }

    public void flashColor(String color1, String color2, double ticks) {
        if (this.ticks < ticks) {
            changeColor(color1);
        } else {
            changeColor(color2);
        }

        this.ticks++;

        if (this.ticks > ticks * 2) {
            this.ticks = 0;
        }
    }
}