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
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  private double startTime = 0.0;

  /** Creates a new Autonomous. */
  public Autonomous(DriveTrain driveTrain, AHRS ahrs, Arm arm) {
    this.ahrs = ahrs;
    this.driveTrain = driveTrain;
    this.arm = arm;
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    // sets desired angle to 0
    System.out.println("Starting auto");
    SequentialCommandGroup auto = new SequentialCommandGroup(
        // Close the grabber at the start
        new InstantCommand(() -> {
          driveTrain.toggleTurtleMode(1);
        }),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(-0.2, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 1;
          }
        }),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.closeGrabber(startTime);
          }
        }),
        new PrintCommand("1"),
        // Set arm angle to desired angle
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmAngle(33.5);
          }
        }),
        new PrintCommand("2"),
        // set arm extension to granted extension
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmExtension(39);
          }
        }),
        new PrintCommand("3"),
        // update timer for grabber as it relies on time
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        // release the kraken (gamepiece)
        new InstantCommand(() -> {
          arm.openGrabber(startTime);
        }),
        // Wait for a second for things to slow down and settle
        new WaitCommand(1),
        new WaitUntilCommand(arm::zeroArm),
        // update timer
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(0.5, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 4.5;
          }
        }),
        new InstantCommand(() -> {
          driveTrain.drive(0, 0);
        }));

    auto.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
