// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.libraries.*;
import frc.robot.*;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleJoystickDrive extends CommandBase {
  Joystick stick;
  XboxController xbox;

  public SingleJoystickDrive(Joystick stick, XboxController xbox) {
    addRequirements(RobotContainer.driveTrain);
    this.stick = stick;
    this.xbox = xbox;
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    System.out.println("sjd running");
    RobotContainer.driveTrain.singleJoystickDrive(
        Deadzone.deadZone(stick.getY(), Constants.ControllerConstants.DEADZONE),
        Deadzone.deadZone(stick.getZ(), Constants.ControllerConstants.DEADZONE));
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}