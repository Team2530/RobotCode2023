package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.libraries.*;

public class Arm extends SubsystemBase {

    // ---------- Motors ----------\\
    private WPI_TalonSRX positionMotor = new WPI_TalonSRX(Constants.PortsConstants.LINEAR_ACTUATOR_PORT);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(Constants.PortsConstants.EXTENTION_PORT);
    private SpecialServo grabberServo = new SpecialServo(Constants.PortsConstants.GRABBER_PORT);

    private Encoder positionEncoder = new Encoder(Constants.PortsConstants.ARM_ENCODER_PORT,
            Constants.PortsConstants.ARM_ENCODER_PORT + 1);
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


    // ---------- Arm Constants ---------- \\ (All Values are in inches)
    // Distance from edge of robot to the rotation axis of the arm
    private final double kDistanceInsideRobot = -18.443;
    // Distance above ground for pivot point
    private final double kArmPivotHeight = 16.681;
    // Length of the arm fully retracted
    private final double kArmFullRetractedLength = 35.257;

    // Min && Max Angles
    private final double kMinAngle = -17.2;
    private final double kMaxAngle = 60.0;

    //Min && Max Extension Values for interpolating
    private final  double minExtension = 0.0;
    private final double maxExtension = 47.5;

    // ---------- Sensor Reading Constants ---------- \\

    //Min && Max Encoder Readings so we can interpolate between them
    private final double minAngleEncoderReading = 0.0;
    private final double maxAngleEncoderReading = 1.0;

    // Min && max Encoder Readings for interpolation
    private final double minExtensionEncoderReading = 0.0;
    private final double maxExtensionEncoderReading = 1.0;

    private final double AngleChangePerPulse = .1;
    private final double ExtensionChangePerPulse = .1;

    private double armLimitSpeed = 1.0;

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
        positionEncoder.setDistancePerPulse(1f / 4096f);
        extensionMotor.overrideLimitSwitchesEnable(true);
    }

    @Override
    public void periodic() {
        //Update to our current data
        currentAngle = positionEncoder.getDistance() * AngleChangePerPulse;
        currentExtension = extensionMotor.getSelectedSensorPosition() * ExtensionChangePerPulse;
        // check current data
        ensureBounds();

        grabberServo.setRelativeAngle(1 - xbox.getRawAxis(2));

        SmartDashboard.putNumber("Position Encoder", positionEncoder.getDistance());
        // Get Xbox POV for Extending
        switch (xbox.getPOV()) {
            case 0:
                if(canExtend) {
                    extensionMotor.set(0.75 * armLimitSpeed);
                } else {
                    extensionMotor.set(0.0);
                }
                break;
            case 180:
                extensionMotor.set(-0.75 * armLimitSpeed);
                break;
            case -1:
                extensionMotor.set(0.0);
                break;
        }
        // Buttons Y and A for angle adjustment
        if (xbox.getRawButton(4)) {
            if(canGoUp) {
                positionMotor.set(1);
            } else {
                positionMotor.set(0.0);
            }
        } else if (xbox.getRawButton(1)) {
            positionMotor.set(-1);
        } else {
            positionMotor.set(0.0);
        }

        updateShuffleBoardValues();
    }

    /**
     * Use to update the values on Shuffleboard
     * ! Make sure method is called in periodic or values won't update
     */
    public void updateShuffleBoardValues() {
        SmartDashboard.putBoolean("Limit 1", 0 != extensionMotor.isFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("Limit 2", 0 != extensionMotor.isRevLimitSwitchClosed());
        if(canGoUp) {
            SmartShuffle.get("Pos").changeColor("green");
        } else {
            SmartShuffle.get("Pos").flashColor("red", "white", 15);
        }

        if(canExtend) {
            SmartShuffle.get("Ext").changeColor("green");
        } else {
            SmartShuffle.get("Ext").flashColor("red", "white", 15);
        }
        
    }

    private void initialiseShuffleBoardValues() {
        SmartShuffle.setHeight(1);
        SmartShuffle.setWidth(1);

        // Flashers for Extension and Position
        SmartShuffle.setWidget(BuiltInWidgets.kBooleanBox);
        SmartShuffle.add("Ext", true);
        SmartShuffle.add("Pos", true);
    }

    private void ensureBounds() {
        // calcualte based on our current extension value and angle value
        // double calculatedLength = Math.cos(Math.toRadians(currentAngle)) * (kDistanceInsideRobot + kArmFullRetractedLength + currentExtension);
        double calculatedHeight = Math.sin(Math.toRadians(currentAngle)) * kArmFullRetractedLength + kArmPivotHeight;
        calculatedLength += stick.getX();
        // If we are close to being our of bounds, we shall rumble
        if(calculatedHeight > 77) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canGoUp = false;
        } else {
            canGoUp = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        if(calculatedLength > 47) {
            xbox.setRumble(RumbleType.kBothRumble, 1);
            canExtend = false;
        } else {
            canExtend = true;
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        if(currentExtension > 45 || currentExtension < 2) {
            armLimitSpeed = 0.25;
        } else {
            armLimitSpeed = 1.0;
        }
    }

    public boolean zeroArm() {
        if(currentExtension > 0) {
            extensionMotor.set(-0.5);
        } else {
            extensionMotor.set(0.0);
        }

        if(currentAngle < kMaxAngle) {
            positionMotor.set(0.5);
        } else {
            positionMotor.set(0.0);
        }

        return Math.abs(currentAngle - kMaxAngle) < 0.1 && Math.abs(currentExtension) < 0.1;
    }
}