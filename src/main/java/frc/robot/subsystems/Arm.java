package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private PIDController extensionPID = new PIDController(0.1, 0.0, 0.0);
    private PIDController anglePID = new PIDController(0.1, 0.0, 0.0);

    // ---------- Controllers ----------\\
    private XboxController xbox;
    private Joystick stick;

    // --------- Values ----------\\
    private double currentAngle = 0.0;
    private double currentExtension = 0.0;
    private double currentWantedExtension = 0.0;
    private double currentWantedAngle = 0.0;

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
    private final double kMaxAngle = 68.0;

    // Min && Max Extension Values for interpolating
    private final double minExtension = 0.0;
    private final double maxExtension = 47.5;

    // ---------- Sensor Reading Constants ---------- \\
    // Min && max Encoder Readings for interpolation
    private final double extensionChangePerPulse = 6406.177;

    private double armOutLimitSpeed = 0.25;
    private double armInLimitSpeed = 0.25;

    private double autoGrabbberPosition = 0;

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
        currentAngle = 360 - angleEncoder.getAngleDeg() - 36.3;
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
            grabberServo.setRelativeAngle(xbox.getRawAxis(2) + ((xbox.getRawAxis(2) > 0.5)
                    ? 0.05 * Math.sin((((float) RobotController.getFPGATime() / 1000000.f) * Math.PI) * 8.f)
                    : 0.0));
        }

        if (DriverStation.isAutonomousEnabled()) {
            grabberServo.setRelativeAngle(autoGrabbberPosition +
                    0.05 * Math.sin((((float) RobotController.getFPGATime() / 1000000.f) * Math.PI) * 8.f));
        }
        // Does all the linear actuator updating
        if (DriverStation.isTeleopEnabled()) {
            updateLinearActuator();
        }

        // Updates shuffleboard values
        updateShuffleBoardValues();

        // do all presetThings
        presets();
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
        if (currentExtension > 0.3) {
            extensionMotor.set(-armInLimitSpeed);
        } else {
            extensionMotor.set(0.0);
        }

        // grabberServo.setRelativeAngle(1);

        if (currentAngle < kMaxAngle) {
            linearActuator.set(0.5);
        } else {
            linearActuator.set(0.0);
        }

        currentWantedAngle = currentAngle;
        currentWantedExtension = currentExtension;
        return Math.abs(currentAngle - kMaxAngle) < 5 && currentExtension < 3;
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
     * Updates the arm based on input and limits (All extension logic)
     */
    private void updateArm() {
        // Get Xbox POV for Extending
        currentWantedExtension -= Deadzone.deadZone(xbox.getRawAxis(1), 0.05);
        // Make max to whatever value at angle is
        // If angle is within 20 degrees, we make the max arm wanted extension to less
        // than max
        currentWantedExtension = Math.max(0, Math.min(currentWantedExtension, Math.abs(currentAngle) < 20 ? 26.5 : 34));
        SmartDashboard.putNumber("Wanted Extemsoin", currentWantedExtension);
        // Set arm speed to calculated from PID
        extensionMotor.set(extensionPID.calculate(currentExtension, currentWantedExtension));

    }

    /**
     * Updated the linear actuator based on inputs and make sure our limits are <em>
     * fairly good</em>
     */
    private void updateLinearActuator() {
        currentWantedAngle -= Deadzone.deadZone(xbox.getRawAxis(5), 0.05);
        currentWantedAngle = Math.max(-22, Math.min(currentWantedAngle, 68));
        SmartDashboard.putNumber("Wanted Angle", currentWantedAngle);

        linearActuator.set(extensionPID.calculate(currentAngle, currentWantedAngle));
    }

    public double getArmAngle() {
        return currentAngle;
    }

    public boolean closeGrabber(double startSeconds) {
        grabberServo.setRelativeAngle(1);
        autoGrabbberPosition = 1d;
        return Timer.getFPGATimestamp() - startSeconds > 0.25d;
    }

    public boolean openGrabber(double startSeconds) {
        grabberServo.setRelativeAngle(0.5d);
        return Timer.getFPGATimestamp() - startSeconds > .25;
    }

    public void presets() {

        if (xbox.getAButton()) {
            setFloorGrabPreset();
        }

        if (xbox.getBButton()) {
            // just to floor angle
        }

        if (xbox.getYButton()) {
            setMediumPreset();
        }

        if (xbox.getRawButton(5)) {
            setHighScorePreset();
        }

        if (xbox.getRawButton(6)) {
            setDrivingPreset();
        }

        if (xbox.getXButtonPressed()) {
            setHumanPickupPreset();
        }
    }

    public void setFloorGrabPreset() {
        currentWantedAngle = -25;
        currentWantedExtension = 20.62;
    }

    public void setHighScorePreset() {
        currentWantedAngle = 33;
        currentWantedExtension = 37;
    }

    public void setDrivingPreset() {
        currentWantedExtension = 0;
        currentWantedAngle = 45;

    }

    public void setMediumPreset() {
        currentWantedAngle = 27;
        currentWantedExtension = 16.5;
    }

    public void setHumanPickupPreset() {
        currentWantedAngle = 24;
        currentWantedExtension = 20;
    }

    // Sets when we enable to whatever the arm is at
    public void setWantedToCurrent() {
        currentWantedAngle = currentAngle;
        currentWantedExtension = currentExtension;
    }

}