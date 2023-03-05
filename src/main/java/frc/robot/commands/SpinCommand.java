package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SpinCommand extends CommandBase {
    private long start;
    private long duration;

    public SpinCommand() {
        this.duration = (long) (1.5d * 1000000.d); // 1.5 seconds, of course
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {
        this.start = RobotController.getFPGATime();
        super.initialize();
        System.out.println("Spin started");
    }

    @Override
    public void execute() {
        RobotContainer.driveTrain.singleJoystickDrive(0.0, 1.0);
    }

    @Override
    public boolean isFinished() {
        boolean f = (RobotController.getFPGATime() - this.start) > this.duration;
        if (f) {
            System.out.println("Spin finished");
        }
        return f;
    }
}
