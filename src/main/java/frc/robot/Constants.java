package frc.robot;

import java.sql.Driver;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constans provides a place for all robot
 * constants to exist and be easily changed
 * throughout the entire program
 */
public class Constants {
    // ---------- Motor Constants ----------\\
    public static final int MOTOR_FL_PORT = 1;
    public static final int MOTOR_FR_PORT = 2;
    public static final int MOTOR_BL_PORT = 3;
    public static final int MOTOR_BR_PORT = 4;

    // ---------- Other Motor Ports ----------\\
    public static final int MOTOR_SHOOTER_PORT = 5;

    // ---------- Sensor Ports ----------\\

    // ---------- Sensor Constants ----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;

    public static final double LIMELIGHT_HEIGHT = 1.5;
    // Todo: Find Camera Port for robot USB camera

    // ---------- PID Constants ----------\\

    // ---------- Field Constants ----------\\
    public static final double GRAVITY = 9.8;
    // in meters
    public static final double GOAL_HEIGHT = 5;


    // in meters (standard field is roughly this size)
    public static final double FIELD_WIDTH = 8.1;
    public static final double FIELD_LENGTH = 16.48;

    // ---------- Driving Constants ----------\\
    public static final double WHEEL_RADIUS = 3;
    public static final double DRIVETRAIN_GEAR_RATIO = 16.2;
    public static final double TRACK_WIDTH_METERS = .72;
    public static final double ROBOT_MASS = Units.lbsToKilograms(125);
    public static final double INERTIA = 5;
    public static final double TURN_TOLERANCE = 0.2;


    // ---------- Joystick Constants ----------\\
    public static int JOYSTICK_PORT = 0;
    public static int XBOX_PORT = 0;

    public static final double DEADZONE = 0.05;
    // Todo: Add all joystick buttons as Constants
    // Joystick Buttons (Use J_ before the button name to indicate use with Joystick)
    public static final int J_DRIVETRAIN_TOGGLE = 1;
    public static final int J_SIMULATION_RESET = 2;
    public static final int J_VECTOR_DRIVE = 3;

    // Xbox Buttons (Use X_ before the button name to indicate use with Xbox controller)
    public static final int X_AIM_TOWARDS_TARGET = 1;

    // Sets controler ports
    static {
        if(RobotBase.isReal()) {
            JOYSTICK_PORT = getJoystickPort();
            XBOX_PORT = getXboxPort();

            System.out.println(JOYSTICK_PORT);
            System.out.println(XBOX_PORT);
        } else {
           JOYSTICK_PORT = 0;
           XBOX_PORT = 1;
        }
        
    }

    /**
     * Gets the first joystick port that exists. If the robot is simulated, 
     * the Joystick port is defaulted to 0
     * @return Joystick port
     * @throws Error if the joystick isn't connected to the computer
     */
    private static int getJoystickPort() {
        for(int i = 0; i < 6; i++) {
            if(DriverStation.getJoystickName(i).contains("Logitech Extreme 3D")
            && DriverStation.isJoystickConnected(i)) {
                return i;
            }
        }
        throw new Error("Joystick seems not to be connected! " + 
            " Make sure the Joystick is connected to the computer");
    }
    /**
     * Gets the first Xbox port that exists. If the robot is simulated, 
     * the Xbox port is defaulted to 1
     * @return Xbox port
     * @throws Error if the Xbox isn't connected to the computer
     */
    private static int getXboxPort() {
        for(int i = 0; i < 6; i++) {
            if(DriverStation.getJoystickName(i).contains("Xbox") &&
            DriverStation.isJoystickConnected(i)) {
                return i;
            }
        }

        throw new Error("Xbox seems not to be connected! Make sure the Xbox is connected to the computer");
    }

}