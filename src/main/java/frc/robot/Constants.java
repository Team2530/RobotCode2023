package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/**
 * Constans provides a place for all robot
 * constants to exist and be easily changed
 * throughout the entire program
 */
public class Constants {

    // Sets controller ports
    static {
        ControllerConstants.JOYSTICK_PORT = 0; // frc.robot.subsystems.Controller.getJoystickPort();
        ControllerConstants.XBOX_PORT = 1; // frc.robot.subsystems.Controller.getXboxPort();

        System.out.println("Joystick port: " + ControllerConstants.JOYSTICK_PORT);
        System.out.println("Xbox port: " + ControllerConstants.XBOX_PORT);
    }

    /**
     * Ports for all sorts of motors and motor controllers
     */
    public static class PortsConstants {
        public static final int MOTOR_FL_PORT = 10;
        public static final int MOTOR_FR_PORT = 20;
        public static final int MOTOR_BL_PORT = 30;
        public static final int MOTOR_BR_PORT = 40;
    }

    /**
     * Sensor Constants
     */
    public static class SensorConstants {
        public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
        public static final double LIMELIGHT_HEIGHT = 1.5;
    }

    /**
     * PID Constants
     */
    public static class PIDConstants {
        // ! Values for FredBOt were: 0.05, 0.0, 0.005
        /** PID for Robot Rotation */
        // P: 0.025, I: 0.0 D: 0.006
        public static final PIDController rotPID = new PIDController(0.0, 0.0, 0.0);
    }

    /**
     * All Field Constants
     */
    public static class FieldConstants {
        public static final double GRAVITY = 9.8;
        public static final double TARGET_HEIGHT = 5;

        // in meters (standard field is roughly this size)
        public static final double FIELD_WIDTH = 8.1;
        public static final double FIELD_LENGTH = 16.48;
    }

    /**
     * Driving / DriveTrain Constants
     */
    public static class DriveTrainConstants {
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
    public static class ControllerConstants {
        public static int JOYSTICK_PORT = 0;
        public static int XBOX_PORT = 1;

        public static final double DEADZONE = 0.05;
        // Todo: Add all joystick buttons as Constants
        // Joystick Buttons (Use J_ before the button name to indicate use with
        // Joystick)
        public static final int J_DRIVETRAIN_TOGGLE = 1;
        public static final int J_SIMULATION_RESET = 4;
        public static final int J_VECTOR_DRIVE = 3;
        public static final int J_DRIVE_STRAIGHT = 2;

        // Xbox Buttons (Use X_ before the button name to indicate use with Xbox
        // controller)
        public static final int X_AIM_TOWARDS_TARGET = 1;
    }
}