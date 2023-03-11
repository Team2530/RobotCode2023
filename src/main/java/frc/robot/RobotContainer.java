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

    final Joystick stick = new Joystick(Constants.ControllerConstants.JOYSTICK_PORT);
    final XboxController xbox = new XboxController(Constants.ControllerConstants.XBOX_PORT);

    private static final AHRS m_ahrs = new AHRS();

    // ---------- Subsystems ----------\\
    private final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick, xbox);

    static {
        // if(RobotBase.isReal()) {
        final USBCamera driveCamera = new USBCamera();
    }
    // }
    private final Arm m_arm = new Arm(m_driveTrain, stick, xbox);
    private final LimeLight m_limelight = new LimeLight(m_driveTrain);

    // ---------- Autonomous Commands ----------\\

    // ---------- Commands ----------\\
    InstantCommand example = new InstantCommand(() -> {
        /**
         * code goes here
         */
    });

    // ---------- Global Toggles ----------\\

    public RobotContainer() {
        configureButtonBindings();
    }

    public void configureButtonBindings() {

        // ? Button 2 is used for Turtle Mode
        new JoystickButton(stick, Constants.ControllerConstants.J_TURTLE_TOGGLE).onTrue(
                new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.5);
                })).onFalse(new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.75);
                }));

        // ? Button 1 (trigger is used for fullspeed)
        new JoystickButton(stick, Constants.ControllerConstants.J_FULL_SPEED).onTrue(
                new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(1.0);
                })).onFalse(new InstantCommand(() -> {
                    m_driveTrain.toggleTurtleMode(0.75);
                }));

        // ? Button 6 (trigger used for slow speed for turning)
        new JoystickButton(stick, Constants.ControllerConstants.J_SLOW_SPEED_TURRNNG).onTrue(
                new InstantCommand(() -> {
                    m_driveTrain.toggleSlowTurning(0.5);
                })).onFalse(new InstantCommand(() -> {
                    m_driveTrain.toggleSlowTurning(1.0);
                }));

        new JoystickButton(xbox, 7).whileTrue(new RepeatCommand(new InstantCommand(() -> {
            m_limelight.aimAtTarget();
        })));

        // ? For testing leveling only
        // new JoystickButton(stick, 12).toggleOnTrue(
        // new WaitUntilCommand(m_driveTrain::level));

    }

    public DriveTrain getDriveTrain() {
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
        return new Autonomous(m_driveTrain, m_ahrs, m_arm);
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
        return new WaitUntilCommand(m_arm::zeroArm);
    }

    public static AHRS getAhrs() {
        return m_ahrs;
    }

    public Arm getArm() {
        return m_arm;
    }
}