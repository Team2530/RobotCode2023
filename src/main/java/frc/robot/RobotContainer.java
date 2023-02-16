package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.libraries.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;
import static edu.wpi.first.wpilibj2.command.Commands.print;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    final Joystick stick = new Joystick(Constants.ControllerConstants.JOYSTICK_PORT);
    final XboxController xbox = new XboxController(Constants.ControllerConstants.XBOX_PORT);

    private static final AHRS m_ahrs = new AHRS();

    // ---------- Subsystems ----------\\
    private final static DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick, xbox);
    private final AutonomousTrajectory autonomousTrajectory = new AutonomousTrajectory(m_driveTrain);
    private final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick, xbox);
    
    static {
        if(RobotBase.isReal()) {
            final USBCamera driveCamera = new USBCamera();
        }
    }
    private final Arm m_arm = new Arm(m_driveTrain, stick, xbox);


    // ---------- Autonomous Commands ----------\\
    PathPlannerTrajectory m_auto = PathPlanner.loadPath("TestPath", new PathConstraints(4, 1));

    // ---------- Commands ----------\\
    InstantCommand example = new InstantCommand(() -> {

    });

    HashMap<String, Command> eventMap = new HashMap<>();

    // ---------- Global Toggles ----------\\

    private final PhotonCamera photonCamera = new PhotonCamera();
    public RobotContainer() {
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // new JoystickButton(stick, Constants.J_DRIVETRAIN_TOGGLE).onTrue(
        // new InstantCommand(() -> {
        // m_driveTrain.toggleDriveMode();

        //? Button 2 is used for Turtle Mode
        new JoystickButton(stick, Constants.ControllerConstants.J_TURTLE_TOGGLE).onTrue(
                new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.5);
                })).onFalse(new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.75);
                }));

        //? Button 1 (trigger is used for fullspeed)
        new JoystickButton(stick, Constants.ControllerConstants.J_FULL_SPEED).onTrue(
                new InstantCommand(() -> {
                     m_driveTrain.toggleTurtleMode(1.0);
                })).onFalse(new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.75);
                }));

        // if(RobotBase.isSimulation()) {
        // new JoystickButton(stick, Constants.J_SIMULATION_RESET).onTrue(
        // new InstantCommand(() -> {
        // m_driveTrain.simulationReset(new Pose2d(1, 1, new Rotation2d()));
        // }));

        // if(RobotBase.isSimulation()) {
        // new JoystickButton(stick, Constants.J_SIMULATION_RESET).onTrue(
        // new InstantCommand(() -> {
        // m_driveTrain.simulationReset(new Pose2d(1, 1, new Rotation2d()));
        // }));

    }

    public static DriveTrain getDriveTrain() {
        return m_driveTrain;
    }

    /**
     * Use this to pass the enable command to the main {@link Robot} class.
     * This command is run immediately when the robot is enabled (not simply turned
     * on), regardless of whether the robot is in teleop or autonomous.
     *
     * @return the command to run when the robot is enabled
     */
    public Command getEnableCommand() {
        // return new InstantCommand(() -> m_driveTrain.reset());
        return null;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    
        // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            1,
            1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these no interior waypoints
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.fromDegrees(90)),
            config);

            

    return print("Starting auto")
        .andThen(runOnce(
            () -> m_driveTrain.getPose(), m_driveTrain))
        .andThen(m_driveTrain.createCommandForTrajectory(autonomousTrajectory.goForwardThreeMetersAndComeBack(), m_driveTrain::getPose))
        //.andThen(m_driveTrain.createCommandForTrajectory(turn90Tracjectory, m_driveTrain::getPose))
        .andThen(runOnce(m_driveTrain::stop, m_driveTrain))
        .andThen(print("Done with auto"));
                // return new PPRamseteCommand(m_auto,
                // m_driveTrain::getPose,
                // new RamseteController(),
                // m_driveTrain.m_kinematics,
                // m_driveTrain.wheelspeed_setter,
                // true,
                // m_driveTrain);
    }

    /**
     * Command to run in Telop mode
     * 
     * @return the command to run in Telop
     */
    public Command getTelopCommand() {
        return new SingleJoystickDrive(m_driveTrain, stick, xbox);
    }

    /**
     * Command to run in Test Mode
     * 
     * @return the command to run in Test
     */
    public Command getTestCommand() {
        return null;
    }

    public static AHRS getAhrs() {
        return m_ahrs;
    }
}