package frc.robot;

import frc.robot.commands.*;
import frc.robot.libraries.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    final static Joystick stick = new Joystick(Constants.Controller.JOYSTICK_PORT);
    final static XboxController xbox = new XboxController(Constants.Controller.XBOX_PORT);

    private static final AHRS m_ahrs = new AHRS();

    // ---------- Subsystems ----------\\
    private final static DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick, xbox);

    // ---------- Autonomous Commands ----------\\
    PathPlannerTrajectory m_auto = PathPlanner.loadPath("New Path", new PathConstraints(4, 1));

    // ---------- Commands ----------\\
    InstantCommand example = new InstantCommand(() -> {

    });

    HashMap<String, Command> eventMap = new HashMap<>();

    // ---------- Global Toggles ----------\\

    public RobotContainer() {
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // new JoystickButton(stick, Constants.J_DRIVETRAIN_TOGGLE).onTrue(
        // new InstantCommand(() -> {
        // m_driveTrain.toggleDriveMode();
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
        return new PPRamseteCommand(m_auto,
                m_driveTrain::getPose,
                new RamseteController(),
                m_driveTrain.m_kinematics,
                m_driveTrain.wheelspeed_setter,
                true,
                m_driveTrain);
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