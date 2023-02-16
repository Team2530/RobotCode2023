package frc.robot;

import java.sql.Driver;
import java.util.function.Supplier;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Controller;
import frc.robot.subsystems.LimelightConfig;

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

        /** Arm Encoder port (UP down tilt axis) */
        public static final int ARM_ENCODER_PORT = 5;

        /* Servo Port */
        public static final int GRABBER_PORT = 0;

        // Arm motor ports
        public static final int LINEAR_ACTUATOR_PORT = 14;
        public static final int EXTENTION_PORT = 2;
    }

    /**
     * Sensor Constants
     */
    public static class SensorConstants {
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
        public static final double GOAL_HEIGHT_INCHES = 60.0;
    }

    /**
     * PID Constants
     */
    public static class PIDConstants {
        // ! Values for FredBOt were: 0.05, 0.0, 0.005
        /** PID for Robot Rotation (when we tuned) */
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
        public static final int J_FULL_SPEED = 1;
        public static final int J_TURTLE_TOGGLE = 2;
        public static final int J_SIMULATION_RESET = 4;
        public static final int J_DRIVE_STRAIGHT = 12;

        // Xbox Buttons (Use X_ before the button name to indicate use with Xbox
        // controller)
        public static final int X_AIM_TOWARDS_TARGET = 1;
    }

    /**
     * Controller Constants
     */
    public static class DriveConstants {
        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLT_SECONDS_PER_METER = 1.98;
        public static final double KA_VOLT_SECONDS_SQURED_PER_METER = 0.2;
        public static final double KP_DRIVE_VEL = 8.5;
        public static final double K_TRACK_WIDTH_METERS = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                K_TRACK_WIDTH_METERS);
    }

    public static class AutoConstants {
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;
    }

    public static class VisionConstants {

        /**
         * Physical location of the apriltag camera on the robot, relative to the center
         * of the robot.
         */
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
        public static final Transform3d APRILTAG_ROBOT_TO_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();

        /**
         * Physical location of the shooter camera on the robot, relative to the center
         * of the robot.
         */
        public static final Transform3d SHOOTER_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(-0.128, 0.0075, -1.002), new Rotation3d(0.0, degreesToRadians(7.06), 0.0));
        public static final Transform3d SHOOTER_ROBOT_TO_CAMERA = SHOOTER_CAMERA_TO_ROBOT.inverse();

        public static final LimelightConfig SHOOTER_LIMELIGHT_CONFIG = new LimelightConfig("limelight",
                SHOOTER_CAMERA_TO_ROBOT);
    }

    public static class ArmConstants {
        // height from ground to encoder
        public static final double ENCODER_HEIGHT = 13.22;
        public static final double POSITION_TOLERANCE = 0.05;
        /** How far the arm moves relative to an encoder tick */
        public static final double EXTENSION_PER_TICK = 0.001;
        /** How much the arm rotates in degrees per encoder tick */
        public static final double DELTA_ANGLE_PER_PULSE = 0.1;
        /** How far the endmost point of the arm is relative to the end of the robot */
        public static final double ENDPOINT_TO_ROBOT = 4.826;
    }
}