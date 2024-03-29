// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is meant to serve as a base class for all Limelight operations. It
 * contains the base code for operating the robot
 * from vision using the Limelight.
 * <p>
 * Note that values from year to year will change and constants as well as PID
 * values will need to be adjusted accordingly.
 */
public class LimeLight extends SubsystemBase {
  DriveTrain driveTrain;
  Joystick stick;

  static double xoff;
  static double yoff;
  static double area;
  /** Any targets? */
  static double tv;

  static double turnRate;

  /** Gain for turning for the LimeLight */
  static double limekP = 0.012;
  /** If the turn value is really low, we add to it so it still moves */
  static double minCommand = 0.07;
  /** Amount we are willing to compromise for in our distance */
  static double disttolerance = 0.9;

  /* PID FOr Turning 0.055, .004, 0.001 */
  private PIDController LimelightPID = new PIDController(0.055, 0.075, 0.006);

  // not sure if this is right or not
  static int lightMode = 3;

  int cameraMode = 0;

  /** Creates a new LimeLight. */
  public LimeLight(DriveTrain driveTrain, Joystick stick) {
    this.driveTrain = driveTrain;
    this.stick = stick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();
    showValues();
    double distanceFromTarget = distanceToTarget();
    // System.out.println(distanceFromTarget);
    // Shuffleboard.getTab("limelight").addNumber("Distance", distanceFromTarget);
    SmartDashboard.putNumber("Lime Distance", distanceFromTarget);
  }

  /**
   * Updates the LimeLight's values
   */
  public static void updateValues() {
    xoff = getLimeValues("ty");
    yoff = getLimeValues("tx");
    // tv = table.getEntry("tv").getDouble(0.0);
    area = getLimeValues("ta");
  }

  public void showValues() {
    SmartDashboard.putNumber("LimelightX", xoff);
    SmartDashboard.putNumber("LimelightY", yoff);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public double toRadians(double input) {
    return input * (Math.PI / 180.0);
  }

  public double distanceToTarget() {
    double r = toRadians(Constants.SensorConstants.LIMELIGHT_HEIGHT + yoff);
    return (Constants.FieldConstants.TARGET_HEIGHT - Constants.SensorConstants.LIMELIGHT_HEIGHT) / Math.tan(r);
  }

  public void changeMode() {
    if (lightMode == 3) {
      lightMode = 1;
    } else {
      lightMode = 3;
    }
  }

  /**
   * Assume that there is a valid target, we will turn to aim at it
   */
  public void aimAtTarget() {
    double error = -getLimeValues("ty");
    SmartDashboard.putNumber("TY but TX", error);

    turnRate = LimelightPID.calculate(error);
    SmartDashboard.putNumber("TurnRate", turnRate);
    driveTrain.singleJoystickDrive(stick.getY() * 0.75, Math.signum(turnRate) * Math.min(Math.abs(turnRate), 0.5));

  }

  /**
   * Backs up to a given distance based on maths
   * <p>
   * Meant to be used called multiple times whilst preparing to shoot
   * 
   * @param dist Distance away from goal
   */
  public void backToDistance(double dist) {
    double currentdist = distanceToTarget();
  }

  /**
   * Gets the different values from NetworkTables limelight
   * 
   * @param tvar String of the t value you want (ta , tx , ty , etc.) Use a string
   *             when trying to get value
   *             or you won't get anything returned
   */

  public static double getLimeValues(String tvar) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(tvar).getDouble(0.0);
  }
}