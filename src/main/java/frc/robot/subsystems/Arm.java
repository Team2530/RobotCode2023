package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.libraries.*;

public class Arm extends SubsystemBase {

    // ---------- Motors ----------\\
    private WPI_TalonSRX linearActuator = new WPI_TalonSRX(Constants.PortsConstants.LINEAR_ACTUATOR_PORT);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(Constants.PortsConstants.EXTENTION_PORT);
    private DoubleServo grabberServo = new DoubleServo(Constants.PortsConstants.GRABBER_PORT);

    private PosSensor angleEncoder = new PosSensor(new DigitalInput(0));

    // For simulation purposes

    // ---------- Subsystems ----------\\
    private DriveTrain driveTrain;

    // ---------- Controllers ----------\\
    private XboxController xbox;
    private Joystick stick;

    double calculatedLength = 0.0;

    // --------- Values ----------\\
    private double currentAngle = 0.0;
    private double currentExtension = 0.0;

    /** How far off our calculated and reset when we hit our encoder */
    private double eOff = 0.0;

    private boolean canGoUp = true;
    private boolean canExtend = true;

    // Distance from edge of robot to the rotation axis of the arm
    private final double kDistanceInsideRobot = -18.443;
    // Distance above ground for pivot point
    private final double kArmPivotHeight = 16.681;
    // Length of the arm fully retracted
    private final double kArmFullRetractedLength = 35.257;

    // Min && Max Angles
    private final double kMinAngle = -25;
    private final double kMaxAngle = 64.0;

    // Min && Max Extension Values for interpolating
    private final double minExtension = 0.0;
    private final double maxExtension = 47.5;

    // ---------- Sensor Reading Constants ---------- \\
    // Min && max Encoder Readings for interpolation
    private final double minExtensionEncoderReading = 0.0;
    private final double maxExtensionEncoderReading = 1.0;

    private final double angleChangePerPulse = 22.83;
    private final double extensionChangePerPulse = 6406.177;

    private double armOutLimitSpeed = 0.25;
    private double armInLimitSpeed = 0.25;

    // If the arm is at full extension and past the limit, we need to retract
    private boolean needsToGoIn = false;

    /**
     * Constructs a new Arm
     * 
     * @param driveTrain our DriveTrain
     */
    public Arm(DriveTrain driveTrain, Joystick stick, XboxController xbox) {
        this.driveTrain = driveTrain;
        this.xbox = xbox;
        this.stick = stick;

        // Initial Arm Conditions

        // ---------- Shuffle Things ----------\\
        initialiseShuffleBoardValues();
        extensionMotor.overrideLimitSwitchesEnable(true);
        linearActuator.setSensorPhase(false);
    }

    @Override
    public void periodic() {
        // ? Update to our current data
        // 360 is for reversing the angle and 46.3 gets us to our zero point
        currentAngle = 360 - angleEncoder.getAngleDeg() - 46.3;
        currentExtension = extensionMotor.getSelectedSensorPosition() / extensionChangePerPulse + eOff;

        // reset our encoder reading if our reverse limit switch is closed
        if (extensionMotor.isRevLimitSwitchClosed() == 1) {
            eOff = -extensionMotor.getSelectedSensorPosition() / extensionChangePerPulse;
        }

        // check current data
        ensureBounds();
        // Does all arm updating
        if (DriverStation.isTeleopEnabled()) {
            updateArm();
        }
        // Sets the grabber to the xbox left trigger
        if (DriverStation.isTeleopEnabled()) {
            grabberServo.setRelativeAngle(xbox.getRawAxis(2));
        }
        // Does all the linear actuator updating
        if (DriverStation.isTeleopEnabled()) {
            updateLinearActuator();
        }

        // Updates shuffleboard values
        updateShuffleBoardValues();
    }

    /**
     * Use to update the values on Shuffleboard
     * ! Make sure method is called in periodic or values won't update
     */
    public void updateShuffleBoardValues() {
        if (canGoUp) {
            SmartShuffle.get("Pos").changeColor("green");
        } else {
            SmartShuffle.get("Pos").flashColor("red", "white", 15);
        }

        if (canExtend) {
            SmartShuffle.get("Ext").changeColor("green");
        } else {
            SmartShuffle.get("Ext").flashColor("red", "white", 15);
        }

        SmartShuffle.get("Extension Encoder").update(currentExtension);
        SmartShuffle.get("Angle Encoder").update(currentAngle);

    }

    private void initialiseShuffleBoardValues() {
        SmartShuffle.setHeight(1);
        SmartShuffle.setWidth(1);

        // Flashers for Extension and Position
        SmartShuffle.setWidget(BuiltInWidgets.kBooleanBox);
        SmartShuffle.add("Ext", true);
        SmartShuffle.add("Pos", true);
        SmartShuffle.add("Fwd Limit", 0 != extensionMotor.isFwdLimitSwitchClosed());
        SmartShuffle.add("Rev Limit", 0 != extensionMotor.isRevLimitSwitchClosed());
        SmartShuffle.setWidget(BuiltInWidgets.kTextView);
        SmartShuffle.add("Angle Encoder", currentAngle);
        SmartShuffle.add("Extension Encoder", extensionMotor.getSelectedSensorPosition());
    }

    private void ensureBounds() {
        // calcualte based on our current extension value and angle value
        double calculatedLength = Math.cos(Math.toRadians(currentAngle))
                * (kDistanceInsideRobot + kArmFullRetractedLength + currentExtension);
        double calculatedHeight = Math.sin(Math.toRadians(currentAngle))
                * kArmFullRetractedLength + kArmPivotHeight;
        // If we are close to being our of bounds, we shall rumble
        if (calculatedHeight > 77) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canGoUp = false;
        } else {
            canGoUp = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        if (calculatedLength > (Math.abs(currentAngle) > 2 ? 45 : 48)) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canExtend = false;
        } else {
            canExtend = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        if (!canExtend && calculatedLength > (Math.abs(currentAngle) > 2 ? 45 : 48)) {
            needsToGoIn = true;
        } else {
            needsToGoIn = false;
        }

        // ? Slow down when we are reaching ends
        if (currentExtension > 33) {
            armOutLimitSpeed = 0.25;
        } else {
            armOutLimitSpeed = 0.75;
        }

        if (currentExtension < 6) {
            armInLimitSpeed = 0.25;
        } else {
            armInLimitSpeed = 0.75;
        }
    }

    public boolean zeroArm() {
        if (currentExtension > 0.1) {
            extensionMotor.set(-armInLimitSpeed);
        } else {
            extensionMotor.set(0.0);
        }

        // grabberServo.setRelativeAngle(1);

        if (currentAngle < kMaxAngle) {
            linearActuator.set(1);
        } else {
            linearActuator.set(0.0);
        }

        return Math.abs(currentAngle - kMaxAngle) < 2 && currentExtension < 0.1;
    }

    /**
     * Will set the arm angle to the desired angle
     * 
     * @param angle the wanted angle
     * @return current state of the arm
     */

    public boolean waitForArmAngle(double desiredArmAngle) {
        if (currentAngle < (desiredArmAngle - 2)) {
            linearActuator.set(1);
        } else if (currentAngle > (desiredArmAngle + 2)) {
            linearActuator.set(-1);
        } else {
            linearActuator.set(0.0);
            return true;
        }

        System.out.println(desiredArmAngle - currentAngle);

        return false;
    }

    /**
     * Will set the arm extension to the wanted extension
     * 
     * @param extensionInches the wanted extension
     * @return current state of extension
     */
    public boolean waitForArmExtension(double extensionInches) {
        if (currentExtension < (extensionInches - 3)) {
            extensionMotor.set(armOutLimitSpeed);
        } else if (currentExtension > (extensionInches + 3)) {
            extensionMotor.set(-armInLimitSpeed);
        } else {
            extensionMotor.set(0.0);
            return true;
        }

        return false;
    }

    /**
     * Updates the arm based on input and limits
     */
    private void updateArm() {
        // Get Xbox POV for Extending
        switch (xbox.getPOV()) {
            // Run arm out
            case 0:
                // Arm can extend
                if (canExtend) {
                    extensionMotor.set(armOutLimitSpeed);
                } else {
                    // Set to 0 if nothing needs to happen
                    extensionMotor.set(0.0);
                }
                break;
            // Run arm in
            case 180:
                extensionMotor.set(-armInLimitSpeed);
                break;
            // Run arm in slowly
            case 90:
                extensionMotor.set(armInLimitSpeed * .5);
                break;
            // Run arm out slowly
            case 270:
                extensionMotor.set(-armInLimitSpeed * .5);
                break;
            // If no button is pressed
            case -1:
                extensionMotor.set(0.0);
                break;
        }
    }

    public boolean presetDown() {
        if (currentExtension > 9.5) {
            extensionMotor.set(-armInLimitSpeed);
        } else if (currentExtension < 8.5) {
            extensionMotor.set(-armOutLimitSpeed);
        } else {
            extensionMotor.set(0.0);

        }

        // grabberServo.setRelativeAngle(1);

        if (currentAngle < -23.2) {
            linearActuator.set(1);
        } else if (currentAngle > -23.7) {
            linearActuator.set(-1);
        } else {
            linearActuator.set(0.0);
        }

        return Math.abs(currentAngle - kMaxAngle) < 0.1 && Math.abs(currentExtension - 9) < .5;
    }

    /**
     * Updated the linear actuator based on inputs and make sure our limits are <em>
     * fairly good</em>
     */
    private void updateLinearActuator() {
        // If Y button is pressed, we will make the arm go up
        if (xbox.getYButton()) {
            linearActuator.set(1);
            // If arm goes up and it will make us be outside our limit, we go in else, leave
            // it alone
            if (needsToGoIn) {
                extensionMotor.set(-0.5);
            } else {
                extensionMotor.set(0.0);
            }
            // If A button is pressed, make the arm go down
        } else if (xbox.getAButton()) {
            linearActuator.set(-1);
            // If arm goes down and it will make us be outside our limit, we go in else,
            // leave
            // it alone
            if (needsToGoIn) {
                extensionMotor.set(-0.5);
            } else {
                extensionMotor.set(0.0);
            }
        } else {
            linearActuator.set(0.0);
        }
    }

    public double getArmAngle() {
        return currentAngle;
    }

    public boolean closeGrabber(double startSeconds) {
        grabberServo.setRelativeAngle(1);
        return Timer.getFPGATimestamp() - startSeconds > 1.5;
    }

    public boolean openGrabber(double startSeconds) {
        grabberServo.setRelativeAngle(0.5d);
        return Timer.getFPGATimestamp() - startSeconds > 1.5;
    }

}