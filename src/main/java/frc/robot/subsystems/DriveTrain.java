// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.vmx.AHRSJNI;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  //---------- Drive Motors ----------\\
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.MOTOR_FL_PORT);
  WPI_TalonFX motorBL = new WPI_TalonFX(Constants.MOTOR_BL_PORT);
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.MOTOR_FR_PORT);
  WPI_TalonFX motorBR = new WPI_TalonFX(Constants.MOTOR_BR_PORT);

  private DifferentialDriveOdometry m_odometry;
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  // ---------- Simulation Clases ---------- \\
  private AHRS ahrsSim;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private final Field2d m_fieldSim = new Field2d();
  private final DifferentialDrivetrainSim drivetrainSim = 
  new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), Constants.DRIVETRAIN_GEAR_RATIO
  , Constants.INERTIA, Constants.ROBOT_MASS, 
  Constants.WHEEL_RADIUS, Constants.TRACK_WIDTH_METERS
  , null
  );


  private AHRS ahrs;
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

  /** Creates a new DriveTrain. */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;

    if(RobotBase.isSimulation()) {
      m_odometry = new DifferentialDriveOdometry(
          ahrs.getRotation2d(), m_leftEncoder.getDistance(), 
          m_rightEncoder.getDistance());
      ahrsSim = ahrs;
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    } else {
      m_odometry = null;
      m_leftEncoderSim = null;
      m_rightEncoderSim = null;
    }

    // Todo: Create DriveTrain type and reverse motors if needed

    // Todo: Declare using provided method based on DriveTrain type Ex: tankDrive();s
    tankDrive();

    SmartDashboard.putData("Field" , m_fieldSim);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
        motorFL.get() * RobotController.getInputVoltage(),
        motorFR.get() * RobotController.getInputVoltage());
    drivetrainSim.update(0.020);

    m_leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
    // Update navX stuff
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-drivetrainSim.getHeading().getDegrees());
  }

  public void singleJoystickDrive(double x, double y, double z) {
    if(currentDriveMode != Modes.Stop) {
      // TODO: Implement DriveTrain driving method Ex: ((DifferentialDrive) driveBase).arcadeDrive(x, z);
      ((DifferentialDrive) driveBase).arcadeDrive(x, z);
    }
    
  }

  public void toggleDriveMode() {
    if(currentDriveMode == Modes.Move) {
      currentDriveMode = Modes.Stop;
    } else if(currentDriveMode == Modes.Stop) {
      currentDriveMode = Modes.Move;
    }
  }

  public void reset() {
    // Todo: implement a reset method
  }

  public void stop() {
    // Todo: implement a stop method
  }

  /**
   * Use to create a MecanumDrive
   */
  private void mecanumDrive() {
    driveBase = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    driveBase.setSafetyEnabled(false);
  }
  /**
   * Use to create a Tank Drive (Differential Drive)
   */
  private void tankDrive() {
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    driveBase = new DifferentialDrive(motorFR, motorFL);
  }

  private void setAll(double speed) {
    motorFL.set(speed);
    motorBL.set(speed);
    motorFR.set(speed);
    motorBR.set(speed);
  }

  private void updateOdometry() {
    m_odometry.update(ahrsSim.getRotation2d(), 
        m_leftEncoder.getDistance(), 
        m_rightEncoder.getDistance());
  }
}