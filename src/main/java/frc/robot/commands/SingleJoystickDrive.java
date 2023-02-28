// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.libraries.*;
import frc.robot.*;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleJoystickDrive extends CommandBase {
  DriveTrain driveTrain;
  Joystick stick;
  XboxController xbox;

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();

  public SingleJoystickDrive(DriveTrain driveTrain, Joystick stick, XboxController xbox) {
    this.driveTrain = driveTrain;
    this.stick = stick;
    this.xbox = xbox;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    driveTrain.singleJoystickDrive(
        Deadzone.deadZone(stick.getY(), Constants.ControllerConstants.DEADZONE),
        Deadzone.deadZone(stick.getZ(), Constants.ControllerConstants.DEADZONE));
  }

  /** 
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) { /* driveTrain.stop(); */ }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}