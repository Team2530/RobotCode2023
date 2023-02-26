// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Autonomous extends CommandBase {

  private AHRS ahrs;
  private DriveTrain driveTrain;

  /** Creates a new Autonomous. */
  public Autonomous(DriveTrain driveTrain, AHRS ahrs) {
    this.ahrs = ahrs;
    this.driveTrain = driveTrain;
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {}

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  public Trajectory goForwardAndComeBack(){
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0));

    var trajectoryOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
            DriveConstants.kTrajectoryConfig);

    var trajectoryTwo = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            DriveConstants.kTrajectoryConfig.setReversed(true));

    trajectoryOne = trajectoryOne.concatenate(trajectoryTwo);

    this.driveTrain.resetOdometry(trajectoryOne.getInitialPose());
    return trajectoryOne;
}

  public Command getAutonomousCommand() {
    var trajectory = goForwardAndComeBack();
    return print("Starting auto")
    .andThen(runOnce(
        () -> driveTrain.resetOdometry(trajectory.getInitialPose()), driveTrain))
        .andThen(driveTrain.createCommandForTrajectory(trajectory, driveTrain::getPose))
        .andThen(runOnce(driveTrain::stop, driveTrain))
        .andThen(print("Done with auto"));
  }
}
