// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.swing.SortingFocusTraversalPolicy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup normalAuto = new SequentialCommandGroup(
        // Close the grabber at the start
        new InstantCommand(() -> {
          driveTrain.toggleTurtleMode(1);
        }),
        new PrintCommand("1"),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("4"),

        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.closeGrabber(startTime);
          }
        }),
        new PrintCommand("5"),

        // Set arm angle to desired angle
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmAngle(33.5);
          }
        }),
        new PrintCommand("6"),

        // set arm extension to granted extension
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmExtension(39);
          }
        }),
        new PrintCommand("7"),

        // update timer for grabber as it relies on time
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("8"),

        // release the kraken (gamepiece)
        new InstantCommand(() -> {
          arm.openGrabber(startTime);
        }),
        new PrintCommand("9"),

        // Wait for a second for things to slow down and settle
        new WaitCommand(1),
        new PrintCommand("10"),

        new WaitUntilCommand(arm::zeroArm),
        new PrintCommand("11"),

        // update timer
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("START OF BACKING UP!"),

        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(0.5, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 4.5;
          }
        }),
        new PrintCommand("13"),

        new InstantCommand(() -> {
          driveTrain.drive(0, 0);
        }),
        new PrintCommand("14"),

        new InstantCommand(() -> {
          driveTrain.toggleTurtleMode(0.75);
        }));

    /**
     * Auto with balance
     */
    SequentialCommandGroup autoBalance = new SequentialCommandGroup(

        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("4"),

        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.closeGrabber(startTime);
          }
        }),
        new PrintCommand("5"),

        // Set arm angle to desired angle
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmAngle(33.5);
          }
        }),
        new PrintCommand("6"),

        // set arm extension to granted extension
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmExtension(39);
          }
        }),
        new PrintCommand("7"),

        // update timer for grabber as it relies on time
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("8"),

        // release the kraken (gamepiece)
        new InstantCommand(() -> {
          arm.openGrabber(startTime);
        }),
        new PrintCommand("9"),

        // Wait for a second for things to slow down and settle
        //TODO -- could be shorter/gone
        new WaitCommand(1),
        new PrintCommand("10"),

        //TODO -- could do it during moving to autoleveling
        new WaitUntilCommand(arm::zeroArm),
        new PrintCommand("11"),

        // Close the grabber at the start
        new InstantCommand(() -> {
          driveTrain.toggleTurtleMode(1);
        }),
        new PrintCommand("1"),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("2"),
        //get onto the charge station the first time (No balance)
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(0.65, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 1.5;
          }
        }),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        //slow down to get down the charge station
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(0.3, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 2;
          }
        }),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        // Get back up the charge station
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(-0.6, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 1;
          }
        }),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        // level the robot
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return driveTrain.level(startTime);
          }
        }),

        new PrintCommand("Robot is Level!")

    );

    SequentialCommandGroup autoBackBalance = new SequentialCommandGroup(

        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("Close Grabber and Start"),
        // close grabber
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.closeGrabber(startTime);
          }
        }),
        new PrintCommand("Put arm to angle"),

        // Set arm angle to desired angle
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmAngle(33.5);
          }
        }),
        new PrintCommand("Extend arm"),

        // set arm extension to granted extension
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return arm.waitForArmExtension(39);
          }
        }),

        // update timer for grabber as it relies on time
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("Open Grabber"),

        // release the kraken (gamepiece)
        new InstantCommand(() -> {
          arm.openGrabber(startTime);
        }),
        new PrintCommand("Wait For Cone to Drop"),

        // Wait for a second for things to slow down and settle
        new WaitCommand(0.3d),
        new PrintCommand("Zero Arm"),

        // Zero arm and Start Backing Up
        new PrintCommand("Zero Arm and Start Backing Up"),
        new ParallelCommandGroup(
            new WaitUntilCommand(arm::zeroArm),
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                  driveTrain.toggleTurtleMode(1);
                }),
                new PrintCommand("Go Backward for time"),
                new InstantCommand(() -> {
                  startTime = Timer.getFPGATimestamp();
                }),
                // Go over Charge Station
                new WaitUntilCommand(new BooleanSupplier() {
                  public boolean getAsBoolean() {
                    driveTrain.singleJoystickDrive(0.6, 0);
                    return (Timer.getFPGATimestamp() - startTime) >= 3.25;
                  }
                }))),

        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new PrintCommand("Go back over"),
        // Get back on
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            driveTrain.singleJoystickDrive(-0.6, 0);
            return (Timer.getFPGATimestamp() - startTime) >= 1.7d - 0.25d;
          }
        }),

        // Level

        new PrintCommand("Level the Robot"),
        new InstantCommand(() -> {
          startTime = Timer.getFPGATimestamp();
        }),
        new WaitUntilCommand(new BooleanSupplier() {
          public boolean getAsBoolean() {
            return driveTrain.level(startTime);
          }
        }),

        new PrintCommand("Robot is Level!")

    );
    // ! AUTO CONFIGURATION+

    switch (Robot.autoChooser.getSelected()) {
      case "Normal Auto":
        System.out.println("Normal Auto Starting...");
        normalAuto.schedule();
        break;
      case "Fancy Auto":
        System.out.println("Fancy Auto Starting...");
        autoBalance.schedule();
        break;

      case "Over Auto":
        System.out.println("Over Auto Staring...");
        autoBackBalance.schedule();
        break;
    }
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
