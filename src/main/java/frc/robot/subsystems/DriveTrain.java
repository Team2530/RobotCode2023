// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.libraries.Deadzone;
import frc.robot.libraries.SmartShuffle;
import frc.robot.logging.*;

public class DriveTrain extends SubsystemBase {
  private Joystick stick;
  private XboxController xbox;

  RobotDriveBase driveBase;

  public static enum Modes {
    Stop,
    Move,
    Brake,
    Full,
    Crawl,
    Half
  }

  private Modes currentDriveMode = Modes.Move;

  // 3 meters per second.
  public static final double kMaxSpeed = 3.0;
  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = 1;

  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0762;
  private static final int kEncoderResolution = -4096;

  // ---------- Motors ----------\\
  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(30);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(40);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(10);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(20);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  // ---------- Encoders ----------\\
  private final Encoder m_leftEncoder = new Encoder(2, 3);
  private final Encoder m_rightEncoder = new Encoder(4, 5);

  // ---------- PID Controllers ----------\\
  private final PIDController m_leftPIDController = new PIDController(0, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(0, 0, 0);
  private AHRS ahrs = RobotContainer.getAhrs();
  // Double for Rot PID
  private double yawCtl = 0.0;
  private double yawTarget = 0.0;

  // Double set by toggleTurtleMode. Sets adjustable maximum motor speed
  private double driveModeSpeed = 1.0;

  private double deltaTime = 0.0;
  private double startTime = 0.0;

  // What is driveModeSpeed set to in Turtle Mode
  private final double TURTLE_MODE_MULTIPLYER = 0.5;

  // ---------- Kinematics & Odometry ----------\\
  public Pose2d pose = new Pose2d();
  public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  public static double leftVoltage = 0;
  public static double rightVoltage = 0;
  public static BiConsumer<Double, Double> voltage = (a, b) -> {
    leftVoltage = a;
    rightVoltage = b;
  };

  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // ---------- Simulation Classes ----------\\
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getFalcon500(2), 8, kTrackWidth, kWheelRadius, null);

  /** Subsystem constructor. */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;

    // Todo: Create DriveTrain type and reverse motors if needed
    m_rightLeader.setInverted(false);
    m_rightFollower.setInverted(false);
    m_leftLeader.setInverted(true);
    m_leftFollower.setInverted(true);

    // ? Construct a Tank Drive
    this.tankDrive();
    createValues();

    ahrs.reset();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);

    // who needs Safelt?
    m_leftFollower.setSafetyEnabled(false);
    m_leftLeader.setSafetyEnabled(false);
    m_rightLeader.setSafetyEnabled(false);
    m_rightFollower.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    updatePeriodic();
    updateShuffleBoardValues();
    pose = getPose();
    voltage.accept(m_leftLeader.getMotorOutputVoltage(), m_rightLeader.getMotorOutputVoltage());

    SmartDashboard.putNumber("Roll", ahrs.getRoll());
  }

  public void setMode(Modes m) {
    currentDriveMode = m;
  }

  public void singleJoystickDrive(double StickY, double StickZ) {

    if (RobotBase.isReal()) {
      ((DifferentialDrive) driveBase).arcadeDrive(
          Deadzone.deadZone(StickY * driveModeSpeed, Constants.ControllerConstants.DEADZONE),
          -Deadzone.deadZone(StickZ * driveModeSpeed, Constants.ControllerConstants.DEADZONE));
    } else {
      ((DifferentialDrive) driveBase).arcadeDrive(
          Deadzone.deadZone(StickY * driveModeSpeed, Constants.ControllerConstants.DEADZONE),
          Deadzone.deadZone(StickZ * driveModeSpeed, Constants.ControllerConstants.DEADZONE));
    }

  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void reset(Pose2d pose) {
    ahrs.reset();
    m_odometry.resetPosition(ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  // Todo: implement a reset method
  /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  /**
   * Use to create a MecanumDrive
   */
  private void mecanumDrive() {
    // driveBase = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    // driveBase.setSafetyEnabled(false);
  }

  /**
   * Use to create a Tank Drive (Differential Drive)
   */
  private void tankDrive() {
    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);
    driveBase = new DifferentialDrive(m_leftLeader, m_rightLeader);
  }

  private void setAll(double speed) {
    m_leftGroup.set(speed);
    m_rightGroup.set(speed);
  }

  /** Update all ShuffleBoard values */
  private void updateShuffleBoardValues() {

  }

  private void createValues() {

  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.

    if (DriverStation.isTeleop()) {
      m_drivetrainSimulator.setInputs(
          -m_leftGroup.get() * RobotController.getInputVoltage(),
          m_rightGroup.get() * RobotController.getInputVoltage());
      m_drivetrainSimulator.update(0.02);

      m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
      m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
      m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
      m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
          "Yaw"));
      angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
    }
  }

  /** Update odometry - this should be run every robot loop. */
  public void updatePeriodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void toggleTurtleMode(double maxSpeed) {
    driveModeSpeed = maxSpeed;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
    driveBase.feed();
  }
  
  /**
   * Sets the desired speeds to zero
   */
  public void stop() {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds()));
  }
  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD, THETA_CONSTRAINTS);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
          trajectory,
          this::getPose,
          new RamseteController(AutoConstants.K_RAMSETE_B, AutoConstants.K_RAMSETE_ZETA),
          new SimpleMotorFeedforward(
              DriveConstants.KS_VOLTS,
              DriveConstants.KV_VOLT_SECONDS_PER_METER,
              DriveConstants.KA_VOLT_SECONDS_SQURED_PER_METER),
          DriveConstants.kDriveKinematics,
          this::getWheelSpeeds,
          new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
          new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
          // RamseteCommand passes volts to the callback
          this::tankDriveVolts,
          this);
            
      return ramseteCommand;
  }
}
