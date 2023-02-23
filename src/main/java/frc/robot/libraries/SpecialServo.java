package frc.robot.libraries;

import edu.wpi.first.wpilibj.Servo;

public class SpecialServo {

    private final Servo servo;

    private final double kMinValue = 0.0, kMaxValue = 0.6;

    public SpecialServo(int channel) {
        servo = new Servo(channel);
    }

    public void setRelativeAngle(double value) {
        value = Math.min(value, kMaxValue);
        value = Math.max(value, kMinValue);
        servo.set(value);
    }

}
