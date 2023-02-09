// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.libraries.Deadzone;


/**
 * This is meant to serve as a base class for all Limelight operations. It contains the base code for operating the robot
 * from vision using the Limelight.
 * <p>Note that values from year to year will change and constants as well as PID values will need to be adjusted accordingly.
 */
public class LimeLight extends SubsystemBase {
  static DriveTrain driveTrain;

  static double xoff;
  static double yoff;
  static double area;
  static double istargetvalid;
  static double limelightlatency;
  static double[] botpose;
  static double[] robottargpose;
  static double[] camerapose;
  static double[] targetbotpose;
  static double[] camtargpose;
  /** Any targets? */
  static double tv;

  static double turnRate;

  /** Gain for turning for the LimeLight */
  static double limekP = 0.012;
  /** If the turn value is really low, we add to it so it still moves */
  static double minCommand = 0.07;
  /** Amount we are willing to compromise for in our distance */
  static double disttolerance = 0.9;

  // not sure if this is right or not
  static int lightMode = 3;

  int cameraMode = 0;

  private final static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);// TODO: change
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);// TODO: change

  /** Creates a new LimeLight. */
  public LimeLight(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();
    showValues();
    double distanceFromTarget = distanceToTarget();// TODO: change
    // System.out.println(distanceFromTarget);
    // Shuffleboard.getTab("limelight").addNumber("Distance", distanceFromTarget);
    SmartDashboard.putNumber("Lime Distance", distanceFromTarget);
  }

  /**
   * Updates the LimeLight's values
   */
  public static void updateValues() {
    xoff = getlimevalue("tx");// TODO: change
    yoff = getlimevalue("ty");// TODO: change
    istargetvalid = getlimevalue("tv");// TODO: change
    limelightlatency = getlimevalue("tl") + 11;// TODO: change
  
    // arrays
    camerapose = getlimevalues("camerapose_targetspace");// TODO: change?
    camtargpose = getlimevalues("targetpose_cameraspace");// TODO: change?
    robottargpose = getlimevalues("targetpose_robotspace");// TODO: change?
    targetbotpose = getlimevalues("botpose_targetspace");// TODO: change?
  
    if (true/*Team alliance none*/) {
      botpose = getlimevalues("botpose");// TODO: change?
    }
    //*else if (true/*Team alliance blue*/) {
    //  botpose = getlimevalues("botpose_wpiblue");
    //}
    //else if (true/*Team alliance red*/) {
    //  botpose = getlimevalues("botpose_wpired");
    //}
  
  }

  public void showValues() {// TODO: make sure these work
    SmartDashboard.putNumber("LimelightX", xoff);
    SmartDashboard.putNumber("LimelightY", yoff);
    SmartDashboard.putNumber("Tags?", istargetvalid);
    SmartDashboard.putNumber("LATENCYYYYYYYYYYY", limelightlatency);
    SmartDashboard.putNumberArray("Camera Pose Target Space", camerapose);
    SmartDashboard.putNumberArray("TargetPose Camera Space", camtargpose);
    SmartDashboard.putNumberArray("Target Pose Robot Space", robottargpose);
    SmartDashboard.putNumberArray("Bot Pose Target Space", targetbotpose);
    SmartDashboard.putNumberArray("Bot Space", botpose);
    

  }

  public double toRadians(double input) {
    return input * (Math.PI / 180.0);// TODO: change..........? proably not but make sure
  }

  public double distanceToTarget() {// TODO: Definetly change. Chnage^2
    double r = toRadians(Constants.Sensor.LIMELIGHT_HEIGHT + yoff);
    return (Constants.Field.TARGET_HEIGHT - Constants.Sensor.LIMELIGHT_HEIGHT) / Math.tan(r);
  }

  /*public double distanceToTargetInInches() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    
    double angleToGoalDegrees = LimeLightConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    return (LimeLightConstants.GOAL_HEIGHT_INCHES - LimeLightConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(angleToGoalRadians);
  }*/

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
  public void aimAtTarget() {// TODO: change
    double error = -xoff;
    System.out.println("Error: " + error);

    if (xoff < 1) {
      turnRate = limekP * error + minCommand;
    } else {
      turnRate = limekP * error - minCommand;
    }
    System.out.println(turnRate);
    // Use this method to turn to robot at the speeds

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
   * @param tvar String of the t value you want (ta , tx , ty , etc.) Use a string when trying to get value
   * or you won't get anything returned
   */

  public static double getlimevalue(String tvar) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(tvar).getDouble(0.0);
  }

  public static double[] getlimevalues(String vals) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(vals).getDoubleArray(new double[6]);
  }

  public static boolean setlimevalue(String pals, double value) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(pals).setNumber(value);
  }

  public static void driveBasedOnLimeLight()
  {
    //if yoff greater than 0.1 turn left
    if(xoff > 0.1){
      driveTrain.singleJoystickDrive(0, 0, xoff);
    }
    //if yoff less than 0.1 turn left
    else if (xoff < -0.1){
      driveTrain.singleJoystickDrive(0, 0, xoff); 
    }
    //if yoff equal to 0.1 go straight
    else {
      driveTrain.singleJoystickDrive(0.1, 0, 0);
    }

  }

  //This method for future to get path planner to hookup with lime light reading
  public static Command getRamseteCommand(DriveTrain m_driveTrain){
    RamseteCommand ramseteCommand=null;
    Boolean isValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    isValidTarget = true;
    if(isValidTarget){
      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              DriveConstants.KS_VOLTS,
              DriveConstants.KV_VOLT_SECONDS_PER_METER,
              DriveConstants.KA_VOLT_SECONDS_SQURED_PER_METER),
          DriveConstants.kDriveKinematics,
          10);
      // Create config for trajectory
      TrajectoryConfig config =
      new TrajectoryConfig(
              AutoConstants.K_MAX_SPEED_METERS_PER_SECOND,
              AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      m_driveTrain.getPose(),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      //new Pose2d(getlimevalue("tx"), getlimevalue("ty"), new Rotation2d(0)),
      new Pose2d(3,5, new Rotation2d(0)),
      // Pass config
      config);

      ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_driveTrain::getPose,
            new RamseteController(AutoConstants.K_RAMSETE_B, AutoConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                DriveConstants.KS_VOLTS,
                DriveConstants.KV_VOLT_SECONDS_PER_METER,
                DriveConstants.KA_VOLT_SECONDS_SQURED_PER_METER),
            DriveConstants.kDriveKinematics,
            m_driveTrain::getWheelSpeeds,
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            m_driveTrain::tankDriveVolts,
            m_driveTrain);
            return ramseteCommand;
    }
    return new SequentialCommandGroup(new InstantCommand(() -> m_driveTrain.resetOdometry(m_driveTrain.getPose())));
  }

}