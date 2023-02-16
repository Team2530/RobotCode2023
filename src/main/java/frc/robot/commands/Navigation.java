// package frc.robot.commands;

// import java.util.function.BiConsumer;
// import java.util.function.Supplier;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.auto.BaseAutoBuilder;
// import com.pathplanner.lib.commands.PPRamseteCommand;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Constants.DriveTrain;
// import frc.robot.*;

// public class Navigation extends BaseAutoBuilder {

// public static DifferentialDriveKinematics kinematics = new
// DifferentialDriveKinematics(
// Constants.DriveTrain.TRACK_WIDTH_METERS);

// public static DifferentialDriveWheelSpeeds wheelSpeeds = new
// DifferentialDriveWheelSpeeds();
// public static Pose2d pose = new Pose2d(0, 0, new Rotation2d());

// public static double leftVoltage, rightVoltage;

// public static BiConsumer<Double, Double> voltage = (a, b) -> {
// leftVoltage = a;
// rightVoltage = b;
// };

// public static Supplier<DifferentialDriveWheelSpeeds> wsSupplier = () -> {
// return wheelSpeeds;
// };

// private PathPlannerTrajectory trajectory;
// private Supplier<Pose2d> poseSupplier;
// private RamseteController controller;
// private SimpleMotorFeedforward feedforward;
// private Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
// private PIDController leftController;
// private PIDController rightController;
// private BiConsumer<Double, Double> output;
// private boolean useAllianceColor;
// private boolean isFirstPath = true;

// // public Navigation(PathPlannerTrajectory path, boolean isFirstPath) {
// // trajectory = path;
// // this.isFirstPath = isFirstPath;

// // }

// /**
// *
// * @param traj
// * @param isFirstPath
// * @return
// */
// public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
// isFirstPath) {
// return new SequentialCommandGroup(
// new InstantCommand(() -> {
// // Reset odometry for the first path you run during auto
// if (isFirstPath) {
// RobotContainer.getDriveTrain().resetOdometry(traj.getInitialPose());
// }
// }),
// new InstantCommand(() -> {
// PathPlannerState e = (PathPlannerState) traj.sample(1);
// System.out.println(e.timeSeconds + " " + e.poseMeters.toString());
// }),
// new PPRamseteCommand(
// traj,
// Navigation::getPose, // Pose supplier
// new RamseteController(),
// new SimpleMotorFeedforward(0, 0, 0),
// kinematics, // DifferentialDriveKinematics
// wsSupplier, // DifferentialDriveWheelSpeeds supplier
// new PIDController(0, 0, 0), // Left controller. Tune these values for your
// robot. Leaving them 0
// // will only use feedforwards.
// new PIDController(0, 0, 0), // Right controller (usually the same values as
// left controller)
// voltage, // Voltage biconsumer
// true, // Should the path be automatically mirrored depending on alliance
// color.
// RobotContainer.getDriveTrain() // Requires this drive subsystem
// ));
// }

// public static void updateWheelSpeed(int left, int right) {
// wheelSpeeds.leftMetersPerSecond = left;
// wheelSpeeds.rightMetersPerSecond = right;
// }

// public static void updatePose(int x, int y, Rotation2d rot) {
// pose = new Pose2d(x, y, rot);
// }

// private static Pose2d getPose() {
// return pose;
// }

// @Override
// public CommandBase followPath(PathPlannerTrajectory trajectory) {
// // TODO Auto-generated method stub
// return null;
// }

// }
