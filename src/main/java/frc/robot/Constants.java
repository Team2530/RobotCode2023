package frc.robot;

import java.sql.Driver;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.subsystems.Controller;

/**
 * Constans provides a place for all robot
 * constants to exist and be easily changed
 * throughout the entire program
 */
public class Constants {

    // Sets controller ports
    static {
        Controller.JOYSTICK_PORT = 0; // frc.robot.subsystems.Controller.getJoystickPort();
        Controller.XBOX_PORT = 1; // frc.robot.subsystems.Controller.getXboxPort();

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
     * LimeLight Constants
     */
    public static class LimeLightConstants {
        // how many degrees back is your limelight rotated from perfectly vertical?
        public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0;

        // distance from the center of the Limelight lens to the floor
        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 20.0;

        // distance from the target to the floor
        public static final double GOAL_HEIGHT_INCHES= 60.0;
    }
    /**
     * PID Constants
     */
    public static class PID {
        // ! Values for FredBOt were: 0.05, 0.0, 0.005
        /** PID for Robot Rotation */
        // P: 0.025, I: 0.0 D: 0.006
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
        public static final int J_DRIVE_STRAIGHT = 2;

        // Xbox Buttons (Use X_ before the button name to indicate use with Xbox
        // controller)
        public static final int X_AIM_TOWARDS_TARGET = 1;
    }

    /**
     * Controller Constants
     */
    public static class DriveConstants {
        public static final double KS_VOLTS = 0.01;
        public static final double KV_VOLT_SECONDS_PER_METER = .25;
        public static final double KA_VOLT_SECONDS_SQURED_PER_METER = 0.2;
        public static final double KP_DRIVE_VEL = 5;
        public static final double K_TRACK_WIDTH_METERS = 0.31 * 2;;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
    }

    public static class AutoConstants {
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;
    }
}