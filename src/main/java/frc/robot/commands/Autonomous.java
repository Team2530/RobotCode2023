// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
/**
 * This class uses multiple commands run in sequence to run Autonomous (Or PathWeaver if we can figure it out)
 */
public class Autonomous extends CommandBase {
  DriveTrain driveTrain;
  AHRS ahrs;


  /** Creates a new Autonomous. */
  public Autonomous(DriveTrain driveTrain, AHRS ahrs) {
   this.driveTrain = driveTrain;
   this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup(
    // Todo: Create commands to run during autonomous
    );

    autonomousCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}