package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunForCommand extends CommandBase {
    private long start;
    private long duration;
    private Runnable action;
    private Runnable init;
    private Runnable finish;

    public RunForCommand(double seconds, Runnable actionable) {
        this.duration = (long) (seconds * 1000000.d);
        this.action = actionable;
    }

    public RunForCommand(double seconds, Runnable actionable, Runnable initialize, Runnable finishtask) {
        this.init = initialize;
        this.finish = finishtask;
        new RunForCommand(seconds, actionable);
    }

    @Override
    public void initialize() {
        this.start = RobotController.getFPGATime();
        if (this.init != null) {
            this.init.run();
        }
        super.initialize();
    }

    @Override
    public void execute() {
        if (this.action != null) {
            this.action.run();
        }
    }

    @Override
    public boolean isFinished() {
        boolean f = (RobotController.getFPGATime() - this.start) > this.duration;
        if (f && this.finish != null) {
            this.finish.run();
        }
        return f;
    }
}
