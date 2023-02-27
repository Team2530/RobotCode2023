package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final double kMaxAngle = 45.0;

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
        // Update to our current data
        currentAngle = angleEncoder.getAngleDeg();

        currentExtension = extensionMotor.getSelectedSensorPosition() / extensionChangePerPulse + 2.21;

        // check current data
        ensureBounds();

        grabberServo.setRelativeAngle(xbox.getRawAxis(2));

        // Get Xbox POV for Extending
        switch (xbox.getPOV()) {
            case 0:
                if (canExtend) {
                    extensionMotor.set(armOutLimitSpeed);
                } else {
                    extensionMotor.set(0.0);
                }
                break;
            case 180:
                extensionMotor.set(-armInLimitSpeed);
                break;

            case 90:
                extensionMotor.set(armInLimitSpeed * .25);
                break;

            case 270:
                extensionMotor.set(-armInLimitSpeed * .25);
                break;
            case -1:
                extensionMotor.set(0.0);
                break;
        }

        if (xbox.getYButton()) {
            linearActuator.set(1);
        } else if (xbox.getAButton()) {
            linearActuator.set(-1);
        } else {
            linearActuator.set(0.0);
        }

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
        // double calculatedLength = Math.cos(Math.toRadians(currentAngle)) *
        // (kDistanceInsideRobot + kArmFullRetractedLength + currentExtension);
        double calculatedHeight = Math.sin(Math.toRadians(currentAngle)) * kArmFullRetractedLength + kArmPivotHeight;
        // If we are close to being our of bounds, we shall rumble
        if (calculatedHeight > 77) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canGoUp = false;
        } else {
            canGoUp = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        if (calculatedLength > 47) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canExtend = false;
        } else {
            canExtend = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

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

        grabberServo.setRelativeAngle(0.0);

        // if (currentAngle < kMaxAngle) {
        // linearActuator.set(0.5);
        // } else {
        // linearActuator.set(0.0);
        // }

        return /* Math.abs(currentAngle - kMaxAngle) < 0.1 && **/ currentExtension < 0.1;
    }
}