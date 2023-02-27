package frc.robot.libraries;

import edu.wpi.first.wpilibj.Servo;

public class DoubleServo {
    Servo leftServo;
    Servo rightServo;

    public DoubleServo(int PWMPort) {
        leftServo = new Servo(PWMPort);
        rightServo = new Servo(PWMPort + 1);
    }

    /**
     * Sets the Servos to the <em>angle</em> provided
     * 
     * @param degrees not actuaslly degrees?
     */
    public void setRelativeAngle(double setpoint) {
        leftServo.setAngle(setpoint * 180);
        rightServo.setAngle(180 - (setpoint * 180));
    }

    public void set(int power) {
        leftServo.set(-power);
        rightServo.set(power);
    }
}
