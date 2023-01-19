// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Date;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SmartShuffle;

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
  WPI_TalonFX motor10 = new WPI_TalonFX(10);
  WPI_TalonFX motor20 = new WPI_TalonFX(20);
  WPI_TalonFX motor30 = new WPI_TalonFX(30);
  WPI_TalonFX motor40 = new WPI_TalonFX(40);


  /** Creates a new DriveTrain. */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;

    // Todo: Create DriveTrain type and reverse motors if needed
    motor10.setInverted(false);
    motor20.setInverted(false);
    motor30.setInverted(true);
    motor40.setInverted(true);

    // Todo: Declare using provided method based on DriveTrain type Ex: tankDrive();s
    // motorFL.setInverted(true);
    // motorBL.setInverted(true);
    // motorBR.setInverted(true);
    this.tankDrive();
    createValues();
  }

  @Override
  public void periodic() {
    updateShuffleBoardValues();
  }

  public void setMode(Modes m) {
    currentDriveMode = m;
  }

  public void singleJoystickDrive(double x, double y, double z) {
    if(currentDriveMode != Modes.Stop) {
      // TODO: Implement DriveTrain driving method Ex: ((DifferentialDrive) driveBase).arcadeDrive(x, z);
      ((DifferentialDrive) driveBase).arcadeDrive(Deadzone.deadZone(stick.getY(), Constants.DEADZONE),
        Deadzone.deadZone(stick.getZ(), Constants.DEADZONE));
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
    // driveBase = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    // driveBase.setSafetyEnabled(false);
  }
  /**
   * Use to create a Tank Drive (Differential Drive)
   */
  private void tankDrive() {
    motor40.follow(motor30);
    motor10.follow(motor20);
    driveBase = new DifferentialDrive(motor30, motor20);
  }

  private void setAll(double speed) {
    // motorFL.set(speed);
    // motorBL.set(speed);
    // motorFR.set(speed);
    // motorBR.set(speed);
  }
  /**Update all ShuffleBoard values */
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
}