// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

  // For PID negation
  private static double deltaTime = 0.0;
  private static double startTime = 0.0;

  // ---------- Motors ----------\\
  private final WPI_VictorSPX m_leftLeader = new WPI_VictorSPX(30);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(40);
  private final WPI_VictorSPX m_rightLeader = new WPI_VictorSPX(10);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(20);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);
  DifferentialDrive differentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);
  
  // ---------- Encoders ----------\\
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  // ---------- PID Controllers ----------\\
  private final PIDController m_leftPIDController = new PIDController(0, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(0, 0, 0);
  private AHRS ahrs = RobotContainer.getAhrs();
  // Double for Rot PID
  private double yawCtl = 0.0;
  private double yawTarget = 0.0;

  // ---------- Kinematics & Odometry ----------\\
  public Pose2d pose = new Pose2d();
  public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  public static double leftOutput = 0;
  public static double rightOutput = 0;
  public BiConsumer<Double, Double> wheelspeed_setter = (a, b) -> {
    leftOutput = a;
    rightOutput = b;
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

    // Todo: Declare using provided method based on DriveTrain type Ex:
    // tankDrive();s
    // motorFL.setInverted(true);
    // motorBL.setInverted(true);
    // motorBR.setInverted(true);
    this.tankDrive();
    createValues();

    ahrs.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);

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
    setSpeeds(new DifferentialDriveWheelSpeeds(leftOutput, rightOutput));
    // motorOutputs.accept(m_leftLeader.get(), m_rightLeader.get());
  }

  public void setMode(Modes m) {
    currentDriveMode = m;
  }

  /**
   * Single Joystick Drive
   * <p>
   * <h1>
   * PID will activate after 0.5 seconds
   * of "rest" from the Joystick Z (twist) axis.
   * 
   * @param x
   * @param y
   * @param z
   */
  public void singleJoystickDrive(double x, double y, double z) {
    // if(currentDriveMode != Modes.Stop) {
    // // TODO: Implement DriveTrain driving method Ex: ((DifferentialDrive)
    // driveBase).arcadeDrive(x, z);
    // ((DifferentialDrive) driveBase).arcadeDrive(Deadzone.deadZone(stick.getY(),
    // Constants.DEADZONE),
    // Deadzone.deadZone(stick.getZ(), Constants.DEADZONE));
    // // System.out.println(motor10.get());
    // }

    // if we aren't turning the stick
    if (!(Deadzone.deadZone(stick.getZ(), Constants.Controller.DEADZONE) > 0.05)) {
      deltaTime = Timer.getFPGATimestamp() - startTime;
    } else {
      startTime = Timer.getFPGATimestamp();
    }

    // If we are actualy turning the stick
    if (Math.abs(stick.getZ()) <= 0.1 || stick.getRawButton(Constants.Controller.J_DRIVE_STRAIGHT)) {
      yawCtl = Constants.PID.rotPID.calculate(ahrs.getAngle(), yawTarget);

    } else {
      // we are currently turning
      yawTarget = ahrs.getAngle();
      yawCtl = stick.getZ();

    }

    // System.out.println(yawTarget);

    // Enforce Limits

    double driveZ;

    if (deltaTime < .5) {
      driveZ = stick.getZ();
      yawTarget = ahrs.getAngle();
      yawCtl = stick.getZ();
    } else {
      driveZ = Deadzone.cutOff(yawCtl, Constants.DriveTrain.CUT_OFF_MOTOR_SPEED)
          * Constants.DriveTrain.MAX_DRIVE_SPEED;
    }

    ((DifferentialDrive) driveBase).arcadeDrive(Deadzone.deadZone(stick.getY(), Constants.Controller.DEADZONE),
        Deadzone.deadZone(driveZ, Constants.Controller.DEADZONE));
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

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot    the rotation
   */
  public void drive(double xSpeed, double rot) {
    // ! ONLY FOR SIMULATION DRIVING
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  public void reset(Pose2d pose) {
    ahrs.reset();
    m_odometry.resetPosition(ahrs.getRotation2d(), m_leftEncoder.get(), m_rightEncoder.get(), pose);
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
    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);
    driveBase = new DifferentialDrive(m_leftLeader, m_rightFollower);
  }

  private void setAll(double speed) {
    m_leftGroup.set(speed);
    m_rightGroup.set(speed);
  }

  /** Update all ShuffleBoard values */
  private void updateShuffleBoardValues() {
    SmartShuffle.get("Stick Y").update(stick.getY() * -100);
    SmartShuffle.get("Stick Z").update(stick.getZ() * 100);
  }

  private void createValues() {
    SmartShuffle.setPosx(0);
    SmartShuffle.setPosy(2);
    SmartShuffle.add("Stick Y", 0);
    SmartShuffle.setWidget(BuiltInWidgets.kDial);
    SmartShuffle.add("Stick Z", 0);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    differentialDrive.feed();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftGroup.get() * RobotController.getInputVoltage(),
        m_rightGroup.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /** Update odometry - this should be run every robot loop. */
  public void updatePeriodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }
}
