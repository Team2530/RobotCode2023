package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autonomous;
import frc.robot.commands.SingleJoystickDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.USBCamera;

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
    public SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final AHRS m_ahrs = new AHRS();

    // ---------- Subsystems ----------\\
    private final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick, xbox);

    static {
        // if(RobotBase.isReal()) {
        final USBCamera driveCamera = new USBCamera();
    }
    // }
    private final Arm m_arm = new Arm(m_driveTrain, stick, xbox);

    // ---------- Autonomous Commands ----------\\

    // ---------- Commands ----------\\
    InstantCommand example = new InstantCommand(() -> {
        /** 
         * code goes here
         */
    });

    // ---------- Global Toggles ----------\\

    public RobotContainer() {
        m_driveTrain.setDefaultCommand(new SingleJoystickDrive(m_driveTrain, stick, xbox));

        configureButtonBindings();
        //Shuffleboard.getTab("Autonomous").add(m_chooser);
        //m_chooser.setDefaultOption("Nothing", new InstantCommand());
        //m_chooser.addOption("Blue Alliance Charge Station", ramAutoBuilder("BlueAllianceChargeStation", Constants.AutoConstants.BlueAllianceChargeStation));
        //m_chooser.addOption("Blue Alliance Forward Backward", ramAutoBuilder("BlueAllianceForwardBackward", Constants.AutoConstants.BlueAllianceForwardBackward));
        //m_chooser.addOption("Red Alliance Charge Station", ramAutoBuilder("RedAllianceChargeStation", Constants.AutoConstants.RedAllianceChargeStation));
        //m_chooser.addOption("Red Alliance Forward Backward", ramAutoBuilder("RedAllianceForwardBackward", Constants.AutoConstants.RedAllianceForwardBackward));
    }

    public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
        // PathPlanner AutoBuilder, builds a full autonomous command
        RamseteAutoBuilder testRouteBuilder = new RamseteAutoBuilder(
          m_driveTrain::getPose,
          m_driveTrain::resetOdometry,
          new RamseteController(AutoConstants.K_RAMSETE_B, AutoConstants.K_RAMSETE_ZETA),
          DriveConstants.kDriveKinematics,
          new SimpleMotorFeedforward(
            DriveConstants.KS_VOLTS,
            DriveConstants.KA_VOLT_SECONDS_SQURED_PER_METER,
            DriveConstants.KV_VOLT_SECONDS_PER_METER
          ),
          m_driveTrain::getWheelSpeeds,
          new com.pathplanner.lib.auto.PIDConstants(DriveConstants.KP_DRIVE_VEL, 0, 0),
          m_driveTrain::tankDriveVolts,
          eventMap,
          false,
          m_driveTrain
        );
        List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName, PathPlanner.getConstraintsFromPath(pathName));
        final Command auto = testRouteBuilder.fullAuto(pathToFollow);
        return auto;
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

        // if(RobotBase.isSimulation()) {
        // new JoystickButton(stick, Constants.J_SIMULATION_RESET).onTrue(
        // new InstantCommand(() -> {
        // m_driveTrain.simulationReset(new Pose2d(1, 1, new Rotation2d()));
        // }));

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
        return new Autonomous(m_driveTrain, m_ahrs).getAutonomousCommand();
        //return ramAutoBuilder("BlueAllianceChargeStation", Constants.AutoConstants.BlueAllianceChargeStation);
        //return ramAutoBuilder("BlueAllianceForwardBackward", Constants.AutoConstants.BlueAllianceForwardBackward);
        //return ramAutoBuilder("RedAllianceChargeStation", Constants.AutoConstants.RedAllianceChargeStation);
        //return ramAutoBuilder("RedAllianceForwardBackward", Constants.AutoConstants.RedAllianceForwardBackward);
        //return m_chooser.getSelected();
    }

    /**
     * Command to run in Test Mode
     * 
     * @return the command to run in Test
     */
    public Command getTestCommand() {
        return null;
    }
}