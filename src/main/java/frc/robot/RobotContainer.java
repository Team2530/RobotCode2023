package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final static Joystick stick = new Joystick(Constants.ControllerConstants.JOYSTICK_PORT);
    public final static XboxController xbox = new XboxController(Constants.ControllerConstants.XBOX_PORT);
    public static final AHRS m_ahrs = new AHRS();

    // ---------- Subsystems ----------\\
    public static final DriveTrain driveTrain = new DriveTrain(m_ahrs, stick, xbox);
    public static final Arm arm = new Arm(driveTrain, stick, xbox);

    // ---------- Commands ----------\\

    public static final Command driveCommand = new SingleJoystickDrive(stick, xbox);
    public static final Command spinCommand = new SpinCommand();
    // ---------- Global Toggles ----------\\

    public RobotContainer() {
        driveTrain.setDefaultCommand(driveCommand);
        configureButtonBindings();
    }

    public void configureButtonBindings() {

        // ? Button 2 is used for Turtle Mode
        new JoystickButton(stick, Constants.ControllerConstants.J_TURTLE_TOGGLE).onTrue(
                new InstantCommand(() -> {
                    driveTrain.setDriveSpeedMultiplier(0.5);
                })).onFalse(new InstantCommand(() -> {
                    driveTrain.setDriveSpeedMultiplier(0.75);
                }));

        // ? Button 1 (trigger is used for fullspeed)
        new JoystickButton(stick, Constants.ControllerConstants.J_FULL_SPEED).onTrue(
                new InstantCommand(() -> {
                    driveTrain.setDriveSpeedMultiplier(1.0);
                })).onFalse(new InstantCommand(() -> {
                    driveTrain.setDriveSpeedMultiplier(0.75);
                }));

        // ? Button 6 (trigger used for slow speed for turning)
        new JoystickButton(stick, Constants.ControllerConstants.J_SLOW_SPEED_TURRNNG).onTrue(
                new InstantCommand(() -> {
                    driveTrain.setTurnSpeed(0.5);
                })).onFalse(new InstantCommand(() -> {
                    driveTrain.setTurnSpeed(1.0);
                }));

        // NOTE: This is just a test for overriding the default command of DriveTrain.
        // A similar button running a command would be used for auto scoring in the
        // future.
        new JoystickButton(stick, 4).onTrue(spinCommand);

        // ? For testing leveling only
        // new JoystickButton(stick, 12).toggleOnTrue(
        // new WaitUntilCommand(m_driveTrain::level));
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
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
        return null; // SingleJoystickDrive is the default command for DriveTrain now (driveCommand)
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new Autonomous();
    }

    /**
     * Command to run in Telop mode
     * 
     * @return the command to run in Telop
     */
    public Command getTelopCommand() {
        return null;// new SingleJoystickDrive(stick, xbox);
    }

    /**
     * Command to run in Test Mode
     * 
     * @return the command to run in Test
     */
    public Command getTestCommand() {
        return new WaitUntilCommand(arm::zeroArm);
    }

    public static AHRS getAhrs() {
        return m_ahrs;
    }
}