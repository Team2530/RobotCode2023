// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.Modes;
import frc.robot.libraries.*;
import frc.robot.*;


import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleJoystickDrive extends CommandBase {
  /**
   * Creates a new SingleJoystickDrive.
   */
  DriveTrain driveTrain;
  Joystick stick;
  XboxController xbox;

  public SingleJoystickDrive(DriveTrain driveTrain, Joystick stick, XboxController xbox) {
    this.driveTrain = driveTrain;
    this.stick = stick;
    this.xbox = xbox;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setMode(Modes.Crawl);
    driveTrain.singleJoystickDrive(Deadzone.deadZone(stick.getRawAxis(1), Constants.DEADZONE),
        Deadzone.deadZone(stick.getRawAxis(0), Constants.DEADZONE),
        Deadzone.deadZone(stick.getRawAxis(2), Constants.DEADZONE));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}