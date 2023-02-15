package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;

public class AutonomousTrajectory extends SubsystemBase {
    private DriveTrain driveTrain;

    public AutonomousTrajectory(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public Trajectory goForwardThreeMetersAndComeBack(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(1.0));
        var trajectoryOne =
            TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
            config);

            var trajectoryTwo =
            TrajectoryGenerator.generateTrajectory(
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            config.setReversed(true)
           );
           trajectoryOne = trajectoryOne.concatenate(trajectoryTwo);

            this.driveTrain.resetOdometry(trajectoryOne.getInitialPose());
            return trajectoryOne;
    }

    public Trajectory getAnglTrajectory(){
        var trajectoryOne =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(1.0)));

        var trajectoryTwo =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(3, 3)),
        new Pose2d(0, 3, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(1.0)));

        trajectoryOne = trajectoryOne.concatenate(trajectoryTwo);

        this.driveTrain.resetOdometry(trajectoryOne.getInitialPose());
        return trajectoryOne;
    }

    @Override
    public void periodic() {
        driveTrain.updateOdometry();
    }

}
