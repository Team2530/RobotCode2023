// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;

public class DriveTrain extends SubsystemBase {

  DifferentialDrive driveBase;

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

  MotorControllerGroup left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  MotorControllerGroup right = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  // ---------- Encoders ----------\\
  private final Encoder m_leftEncoder = new Encoder(2, 3);
  private final Encoder m_rightEncoder = new Encoder(4, 5);

  private final PIDController levelPID = new PIDController(0.028, 0, 0);
  private AHRS ahrs = RobotContainer.getAhrs();

  // Double set by toggleTurtleMode. Sets adjustable maximum motor speed
  private double driveModeSpeed = 0.75;
  private double zTurningSpeed = 1.0;

  // ---------- Kinematics & Odometry ----------\\
  public Pose2d pose = new Pose2d();
  public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  public static double leftVoltage = 0;
  public static double rightVoltage = 0;

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

    m_rightLeader.setInverted(false);
    m_rightFollower.setInverted(false);
    m_leftLeader.setInverted(false);
    m_leftFollower.setInverted(false);

    left.setInverted(true);
    right.setInverted(false);

    // ? Construct a Tank Drive
    this.tankDrive();
    // createValues();

    ahrs.reset();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);

    // who needs Safelt?
    m_leftFollower.setSafetyEnabled(false);
    m_leftLeader.setSafetyEnabled(false);
    m_rightLeader.setSafetyEnabled(false);
    m_rightFollower.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L motor", m_leftLeader.get());

    updatePeriodic();
    updateShuffleBoardValues();
    pose = getPose();
  }

  public void singleJoystickDrive(double StickY, double StickZ) {

    if (RobotBase.isReal()) {
      driveBase.arcadeDrive(
          Deadzone.deadZone(StickY * driveModeSpeed, Constants.ControllerConstants.DEADZONE),
          -Deadzone.deadZone(StickZ * driveModeSpeed * zTurningSpeed, Constants.ControllerConstants.DEADZONE));
    } else {
      driveBase.arcadeDrive(
          Deadzone.deadZone(StickY * driveModeSpeed, Constants.ControllerConstants.DEADZONE),
          Deadzone.deadZone(StickZ * driveModeSpeed * zTurningSpeed, Constants.ControllerConstants.DEADZONE));
    }

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
   * Use to create a Tank Drive (Differential Drive)
   */
  private void tankDrive() {
    driveBase = new DifferentialDrive(left, right);
  }

  private void setAll(double speed) {
    left.set(speed);
    right.set(speed);
  }

  /** Update all ShuffleBoard values */
  private void updateShuffleBoardValues() {

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
    // if (DriverStation.isTeleop()) {
    m_drivetrainSimulator.setInputs(
        -m_leftLeader.get() * RobotController.getInputVoltage(),
        m_rightLeader.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
        "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
    // }
  }

  /** Update odometry - this should be run every robot loop. */
  public void updatePeriodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void setDriveSpeedMultiplier(double maxSpeed) {
    driveModeSpeed = maxSpeed;
  }

  public void setTurnSpeed(double slowTurning) {
    zTurningSpeed = slowTurning;
  }

  public void arcadeDrive(double y, double z) {
    driveBase.arcadeDrive(y, z);
  }

  public boolean level() {
    double currentPitch = ahrs.getRoll();
    // PID output for achieving "levelness"
    double speed = levelPID.calculate(currentPitch, 0.0);
    // Keep speed maxed at 100%
    speed = Math.abs(speed) > 1 ? 1 * Math.signum(speed) * 0.4 : speed * 0.4;
    this.setAll(-speed);

    // ! for simulation purposes only
    currentPitch += speed;

    // See values
    System.out.println(speed + " " + currentPitch);

    // If the charge station is level, we can stop moving
    return Math.abs(currentPitch) < 0.7;
  }
}
