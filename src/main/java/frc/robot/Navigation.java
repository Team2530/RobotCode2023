package frc.robot;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain;

public class Navigation {

    public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.DriveTrain.TRACK_WIDTH_METERS);

    public static DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();
    public static Pose2d pose = new Pose2d(0, 0, new Rotation2d());

    public static double leftVoltage, rightVoltage;

    public static BiConsumer<Double, Double> voltage = (a, b) -> {
        leftVoltage = a;
        rightVoltage = b;
    };

    public static Supplier<DifferentialDriveWheelSpeeds> wsSupplier = () -> {
        return wheelSpeeds;
    };

    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(
            "Example Path",
            new PathConstraints(4, 3));

    // This trajectory can then be passed to a path follower such as a
    // PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following

    // Sample the state of the path at 1.2 seconds
    PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

    /**
     * 
     * @param traj
     * @param isFirstPath
     * @return
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        RobotContainer.getDriveTrain().resetOdometry(traj.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        Navigation::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(0, 0, 0),
                        kinematics, // DifferentialDriveKinematics
                        wsSupplier, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                        voltage, // Voltage biconsumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                        RobotContainer.getDriveTrain() // Requires this drive subsystem
                ));
    }

    public static void updateWheelSpeed(int left, int right) {
        wheelSpeeds.leftMetersPerSecond = left;
        wheelSpeeds.rightMetersPerSecond = right;
    }

    public static void updatePose(int x, int y, Rotation2d rot) {
        pose = new Pose2d(x, y, rot);
    }

    private static Pose2d getPose() {
        return pose;
    }

}
