// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class Autonomous extends CommandBase {
  private AHRS ahrs;
  private DriveTrain driveTrain;
  private Arm arm;

  /** Creates a new Autonomous. */
  public Autonomous(DriveTrain driveTrain, AHRS ahrs, Arm arm) {
    this.ahrs = ahrs;
    this.driveTrain = driveTrain;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double time = Timer.getFPGATimestamp();
    // sets desired angle to 0
    System.out.println("Starting auto");
    SequentialCommandGroup auto = new SequentialCommandGroup(
        // new Command(() -> {
        // driveTrain.singleJoystickDrive(-0.3, 0);
        // }),
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmAngle(33.5);
          }
        }),

        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(0.4, 0);
            return (Timer.getFPGATimestamp() - time) >= 3.0;
          }
        }),

        // new WaitCommand(5),

        new InstantCommand(() -> {
          driveTrain.drive(0, 0);
        }));

    auto.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
