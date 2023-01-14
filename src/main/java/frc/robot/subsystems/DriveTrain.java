// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Date;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.libraries.*;

public class DriveTrain extends SubsystemBase {
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

  //---------- Drive Motors ----------\\
  WPI_VictorSPX motorFL = new WPI_VictorSPX(Constants.MOTOR_FL_PORT);
  WPI_VictorSPX motorBL = new WPI_VictorSPX(Constants.MOTOR_BL_PORT);
  WPI_VictorSPX motorFR = new WPI_VictorSPX(Constants.MOTOR_FR_PORT);
  WPI_VictorSPX motorBR = new WPI_VictorSPX(Constants.MOTOR_BR_PORT);


  /** Creates a new DriveTrain. */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;

    // Todo: Create DriveTrain type and reverse motors if needed
    tankDrive();
    // Todo: Declare using provided method based on DriveTrain type Ex: tankDrive();s

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void singleJoystickDrive(double x, double y, double z) {
    if(currentDriveMode != Modes.Stop) {
      // TODO: Implement DriveTrain driving method Ex: ((DifferentialDrive) driveBase).arcadeDrive(x, z);
      ((DifferentialDrive) driveBase).arcadeDrive(Deadzone.deadZone(stick.getY(), Constants.DEADZONE) * 0.1,
        Deadzone.deadZone(stick.getZ(), Constants.DEADZONE) * 0.1);
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
}