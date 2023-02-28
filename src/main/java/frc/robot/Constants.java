package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Constans provides a place for all robot
 * constants to exist and be easily changed
 * throughout the entire program
 */
public class Constants {

    
    static {
        // Sets controller ports
        ControllerConstants.JOYSTICK_PORT = 0; // frc.robot.subsystems.Controller.getJoystickPort();
        ControllerConstants.XBOX_PORT = 1; // frc.robot.subsystems.Controller.getXboxPort();

        System.out.println("Joystick port: " + ControllerConstants.JOYSTICK_PORT);
        System.out.println("Xbox port: " + ControllerConstants.XBOX_PORT);
    }

    /**
     * Ports for all sorts of motors and motor controllers
     */
    public static class PortsConstants {
        /**
         * Front Left Motor Port
         */
        public static final int MOTOR_FL_PORT = 10;

        /**
         * Front Right Motor port
         */
        public static final int MOTOR_FR_PORT = 20;

        /**
         * Back left motor port
         */
        public static final int MOTOR_BL_PORT = 30;

        /**
         * Back right motor port
         */
        public static final int MOTOR_BR_PORT = 40;

        /** Arm Encoder port (UP down tilt axis) */
        public static final int ARM_ENCODER_PORT = 0;

        /* Servo Port */
        public static final int GRABBER_PORT = 0;

        // Arm motor ports
        public static final int LINEAR_ACTUATOR_PORT = 14;
        public static final int EXTENTION_PORT = 15;
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

        /**
         * When changing controller constants, make sure out driver knows about it--and put it on the list if there is one.
         */
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
    public static class AutoConstants {
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 2;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
        public static final HashMap<String, Command> BlueAllianceChargeStation = new HashMap<>();
        public static final HashMap<String, Command> BlueAllianceForwardBackward = new HashMap<>();
        public static final HashMap<String, Command> RedAllianceChargeStation = new HashMap<>();
        public static final HashMap<String, Command> RedAllianceForwardBackward = new HashMap<>();
        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;
    }

    public static class DriveConstants {
        public static final double KS_VOLTS = 0.14666;
        public static final double KV_VOLT_SECONDS_PER_METER = 0.00051567;
        public static final double KA_VOLT_SECONDS_SQURED_PER_METER = 4.9522E-05;
        //public static final double KA_VOLT_SECONDS_SQURED_PER_METER = 8.6662;
        public static final double KP_DRIVE_VEL = 3;
        public static final double K_TRACK_WIDTH_METERS = .72;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
        public static final SimpleMotorFeedforward K_FEED_FORWARD = new SimpleMotorFeedforward(
            DriveConstants.KS_VOLTS,
            DriveConstants.KV_VOLT_SECONDS_PER_METER,
            DriveConstants.KA_VOLT_SECONDS_SQURED_PER_METER);
        // Voltage constraints
        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            K_FEED_FORWARD,
            kDriveKinematics,
            10);
    }

}