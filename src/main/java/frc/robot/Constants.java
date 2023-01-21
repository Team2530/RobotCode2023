package frc.robot;

import java.sql.Driver;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constans provides a place for all robot
 * constants to exist and be easily changed
 * throughout the entire program
 */
public class Constants {

    // Sets controler ports
    static {
        Controller.JOYSTICK_PORT = 0; // getJoystickPort();
        Controller.XBOX_PORT = 1; // getXboxPort();

        System.out.println("Joystick port: " + Controller.JOYSTICK_PORT);
        System.out.println("Xbox port: " + Controller.XBOX_PORT);
    }
    /**
     * Ports for all sorts of motors and motor controllers
     */
    public static class Ports {
        public static final int MOTOR_FL_PORT = 10;
        public static final int MOTOR_FR_PORT = 20;
        public static final int MOTOR_BL_PORT = 30;
        public static final int MOTOR_BR_PORT = 40;
    }
    /**
     * Sensor Constants
     */
    public static class Sensor {
        public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
        public static final double LIMELIGHT_HEIGHT = 1.5;
    }
    /**
     * PID Constants
     */
    public static class PID {
        //! Values for FredBOt were: 0.05, 0.0, 0.005
        /**PID for Robot Rotation */
        public static final PIDController rotPID = new PIDController(0.0, 0.0, 0.0);
    }
    /**
     * All Field Constants
     */
    public static class Field {
        public static final double GRAVITY = 9.8;
        public static final double TARGET_HEIGHT = 5;

        // in meters (standard field is roughly this size)
        public static final double FIELD_WIDTH = 8.1;
        public static final double FIELD_LENGTH = 16.48;
    }
    /**
     * Driving / DriveTrain Constants
     */
    public static class DriveTrain {
        public static final double WHEEL_RADIUS = 3;
        public static final double DRIVETRAIN_GEAR_RATIO = 16.2;
        public static final double TRACK_WIDTH_METERS = .72;
        public static final double ROBOT_MASS = Units.lbsToKilograms(125);
        public static final double INERTIA = 5;
        public static final double TURN_TOLERANCE = 0.2;

        public static final double CUT_OFF_MOTOR_SPEED = 0.1;
        public static final double MAX_DRIVE_SPEED = 1;
    }
    /**
     * Controller Constants
     */
    public static class Controller {
        public static int JOYSTICK_PORT = 0;
        public static int XBOX_PORT = 0;

        public static final double DEADZONE = 0.05;
        // Todo: Add all joystick buttons as Constants
        // Joystick Buttons (Use J_ before the button name to indicate use with
        // Joystick)
        public static final int J_DRIVETRAIN_TOGGLE = 1;
        public static final int J_SIMULATION_RESET = 2;
        public static final int J_VECTOR_DRIVE = 3;

        // Xbox Buttons (Use X_ before the button name to indicate use with Xbox
        // controller)
        public static final int X_AIM_TOWARDS_TARGET = 1;
    }

    /**
     * Gets the first joystick port that exists. If the robot is simulated,
     * the Joystick port is defaulted to 0
     * 
     * @return Joystick port
     * @throws Error if the joystick isn't connected to the computer
     */
    private static int getJoystickPort() {
        if (RobotBase.isReal()) {
            for (int i = 0; i < 6; i++) {
                if (DriverStation.getJoystickName(i).contains("Logitech Extreme 3D")) {
                    return i;
                }
            }
            throw new Error(
                    "Joystick seems not to be connected! " + " Make sure the Joystick is connected to the computer");
        } else {
            return 0;
        }
    }

    /**
     * Gets the first Xbox port that exists. If the robot is simulated,
     * the Xbox port is defaulted to 1
     * 
     * @return Xbox port
     * @throws Error if the Xbox isn't connected to the computer
     */
    private static int getXboxPort() {
        if (RobotBase.isReal()) {
            for (int i = 0; i < 6; i++) {
                if (DriverStation.getJoystickName(i).contains("Xbox")) {
                    return i;
                }
            }
            throw new Error("Xbox seems not to be connected! Make sure the Xbox is connected to the computer");
        } else {
            return 1;
        }
    }

}