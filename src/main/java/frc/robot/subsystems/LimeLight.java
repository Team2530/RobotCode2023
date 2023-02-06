// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private final static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /** Creates a new LimeLight. */
  public LimeLight(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

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
    xoff = getlimevalue("tx");
    yoff = getlimevalue("ty");
    istargetvalid = getlimevalue("tv");
    limelightlatency = getlimevalue("tl") + 11;
  
    // arrays
    camerapose = getlimevalues("camerapose_targetspace");
    camtargpose = getlimevalues("targetpose_cameraspace");
    robottargpose = getlimevalues("targetpose_robotspace");
    targetbotpose = getlimevalues("botpose_targetspace");
  
    if (true/*Team alliance none*/) {
      botpose = getlimevalues("botpose");
    }
    //*else if (true/*Team alliance blue*/) {
    //  botpose = getlimevalues("botpose_wpiblue");
    //}
    //else if (true/*Team alliance red*/) {
    //  botpose = getlimevalues("botpose_wpired");
    //}
  
  }

  public void showValues() {
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
    return input * (Math.PI / 180.0);
  }

  public double distanceToTarget() {
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
  public void aimAtTarget() {
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
    if(yoff > 0.1){
      driveTrain.drive(0, -yoff);
    }
    //if yoff less than 0.1 turn left
    else if (yoff < -0.1){
      driveTrain.drive(0, yoff);
    }
    //if yoff equal to 0.1 go straight
    else {
      driveTrain.drive(1,0);
    }

  }

  public static JSONArray readJSONTemplateAndUpdateLimeLightValue(){
    JSONParser parser = new JSONParser();
    JSONArray jsonArray = null;
    try {
      JSONObject josnObject= (JSONObject) parser.parse(new FileReader("C:\\Users\\psent\\Documents\\2530\\RobotCode2023\\src\\main\\deploy\\LimeLightPathTemplate.json"));
      JSONArray jArrayWaypoints = (JSONArray) josnObject.get("waypoints");
      JSONObject jArrayWaypoint = (JSONObject) jArrayWaypoints.get(0);
      JSONObject jAnchorObject = (JSONObject) jArrayWaypoint.get("anchorPoint");
      System.out.println(jAnchorObject);
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return jsonArray;
  }

}