package frc.robot.libraries;

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
    }
}