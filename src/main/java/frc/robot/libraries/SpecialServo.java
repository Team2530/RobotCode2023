package frc.robot.libraries;

import edu.wpi.first.wpilibj.Servo;

public class SpecialServo {

    Servo servo;

    double kMinValue = 0.0;
    double kMaxValue = 0.6;

    public SpecialServo(int channel) {
        servo = new Servo(channel);
    }

    public void setRelativeAngle(double value) {
        value = Math.min(value, kMaxValue);
        value = Math.max(value, kMinValue);
        servo.set(value);
    }

}
