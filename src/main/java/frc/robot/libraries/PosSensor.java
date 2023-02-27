package frc.robot.libraries;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class PosSensor {
    DigitalInput io;
    DutyCycle ds;

    private static final double T_MAX = 3978000.d;

    public PosSensor(int dio) {
        this.io = new DigitalInput(dio);
        this.ds = new DutyCycle(this.io);
    }

    public PosSensor(DigitalInput d) {
        this.io = d;
        this.ds = new DutyCycle(this.io);
    }

    public double getAngleDeg() {
        return ((double) this.ds.getHighTimeNanoseconds() / T_MAX) * 360.d;
    }
}
