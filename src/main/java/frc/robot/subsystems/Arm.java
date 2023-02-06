package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.*;

public class Arm extends SubsystemBase {

    //---------- Motors ----------\\
    private WPI_TalonSRX positionMotor = new WPI_TalonSRX(Constants.PortsConstants.LINEAR_ACTUATOR_PORT);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(Constants.PortsConstants.EXTENTION_PORT);
    private Servo grabberServo = new Servo(Constants.PortsConstants.GRABBER_PORT);

    private Encoder positionEncoder = new Encoder(Constants.PortsConstants.ARM_ENCODER_PORT, Constants.PortsConstants.ARM_ENCODER_PORT + 1);
    private CANCoder extensionEncoder = new CANCoder(Constants.PortsConstants.EXTENTION_PORT);
    // For simulation purposes
    private EncoderSim simPositionEncoder = new EncoderSim(positionEncoder);

    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 1, 2, 1, Math.PI / 6, Math.PI / 3, 2, false);

    //---------- Subsystems ----------\\
    private DriveTrain driveTrain;

    //--------- Values ----------\\
    /**Max Angle between the bottom position and the top position */
    private final double kMaxAngleAboveHorizontal = 62.953;
    /**Max Extension (how far we can extend from the base of the arm) */
    private final double kMaxExtension = 42.357; // refrenced from the back of the arm

    /**Our wanted position (double) */
    private double positionValue;
    /**An Enum representation of our wanted position*/
    private Position position;
    /**Our wanted extension (double) */
    private double extensionValue;
    /**An Enum representation of our wanted extetnion*/
    private Extension extension;

    /**Current encoder reading of extension (how much out/in the arm is)*/
    private double currentExtension;

    /**Current encoder reading of position (how much angle the arm has)
    * <p><em>Zero should be the equilvalent to zero degrees relative to the ground</em>
    */
    private double currentPosition;

    private static final double kPositionTolerance = 0.05;
    /**How far the arm moves relative to an encoder tick */
    private static final double kExtensionPerTick = 0.001;
    /**How much the arm rotates in degrees per encoder tick */
    private static final double kDeltaAnglePerPulse = 0.1;
    /**How far the endmost point of the arm is relative to the end of the robot */
    private static final double kEndpointToRobot = 4.826;



    //---------- Preset Positions ----------\\
    public enum Extension {
        FULL(1),
        TWOTHIRDS(.66),
        HALF(.5),
        ONEQUARTER(.25),
        RETRACTED(0),
        CUSTOM(-1);

        /**The double value represented by the extension enum */
        private double length;

        private static final Extension[] vals = values();

        private Extension(double length){
            this.length = length;
        }

        public Extension extend() {
            if(this != FULL) {
                return vals[(this.ordinal() - 1)];
            } else {
                return FULL;
            }   
        }

        public Extension retract() {
            if(this != RETRACTED) {
                return vals[(this.ordinal() + 1)];
            } else {
                return RETRACTED;
            }   
        }
    }

    public enum Position {
        HIGH(1),
        MEDIUM(.66),
        HALF(0.5),
        LOW(.2),
        FLOOR(0),
        Custom(-1);

        /**The double value represented by the position enum */
        private double angle;

        private static final Position[] vals = values();

        private Position(double position){
            this.angle = position;
        }

        public Position raise() {
            if(this != HIGH) {
                return vals[(this.ordinal() - 1)];
            } else {
                return HIGH;
            }   
        }

        public Position lower() {
            if(this != FLOOR) {
                return vals[(this.ordinal() + 1)];
            } else {
                return FLOOR;
            }   
        }
    }

    /**
     * Constructs a new Arm
     * @param driveTrain our DriveTrain
     */
    public Arm(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        // Initial Arm Conditions
        this.position = Position.HIGH;
        this.extension = Extension.RETRACTED;
        // More initial Conditions
        this.positionValue = 1.0;
        this.extensionValue = 0.0;

        //---------- Shuffle Things ----------\\
        SmartShuffle.add("Arm Position", "");
        SmartShuffle.add("Arm Extension", "");
        positionEncoder.setDistancePerPulse(kDeltaAnglePerPulse);

        //TODO: Reverse Motors if needed!
    }

    @Override
    public void periodic() {
        currentPosition = positionEncoder.getDistance();

        if(Math.abs(currentPosition - positionValue) > kPositionTolerance) {
            positionMotor.set(Math.signum(positionValue - currentPosition));
        } else {
            positionMotor.set(0);
        }

        currentExtension = extensionEncoder.getPosition();

        if(Math.abs(currentExtension - extensionValue) > kPositionTolerance) {
            // This motor will be a lot more zoomy
            extensionMotor.set(Math.signum(extensionValue - currentExtension) * 0.2);
        } else {
            extensionMotor.set(0);
        }

        updateShuffleBoardValues();
    }

    /**
     * Sets the Arm Position to a preset position
     * @param position the Position value in {@code Position}
     */
    public void setArmPosition(Position position) {
        this.positionValue = position.angle;
        this.position = position;
    }

    /**
     * Sets the arm to the passed in position. This will set the position enum to 
     * custom so we may move more freely as we choose a position
     * @param position the double value of position to move the arm to
     */
    public void setArmPosition(double position) {
        this.positionValue = position;
        this.position = Position.Custom;
    }

    /**
     * Sets the arm to the preset extension
     * @param extension the extension value in {@code extension}
     */
    public void setArmExtension(Extension extension) {
        this.positionValue = extension.length;
        this.extension = extension;
    }

    /**
     * Sets the arm extension to the double extension
     * @param extension the double value of extension to move the arm to
     */
    public void setArmExtension(double extension) {
        this.extensionValue = extension;
        this.extension = Extension.CUSTOM;
    }

    /**
     * Raises the arm by a setpoint
     */

    public void raiseArm() {
        position = position.raise();
        positionValue = position.angle;
    }

    /**
     * Lowers the arm by a setpoint
     */
    public void lowerArm() {
        position = position.lower();
        positionValue = position.angle;
    }

    /**
     * Extends the arm by a setpoint
     */
    public void extendArm() {
        extension = extension.extend();
        extensionValue = extension.length;
    }

    /**
     * Retracts the arm by a setpoint
     */
    public void retractArm() {
        extension = extension.retract();
        extensionValue = extension.length;
    }

    public void grab(){
        grabberServo.set(1);
    }

    public void release(){
        grabberServo.set(-1);
    }

    /**Makes sure the Arm doesn't become <em>illegal</em>*/
    private void ensureLength() {
        // Normalized Values
        double cExtension = currentExtension * kExtensionPerTick;
        double cPosition = currentPosition * kDeltaAnglePerPulse;

        double extensionFromRobot = cExtension * Math.cos(cPosition) + kEndpointToRobot;
        // If we are greater than our maximum
        if(extensionFromRobot >= 47) {
            extensionValue = 47;
            extension = Extension.FULL;
            extensionMotor.set(0.0);
        }

    }

    /**
     * Use to update the values on Shuffleboard
     * ! Make sure method is called in periodic or values won't update
     */
    public void updateShuffleBoardValues() {
        SmartShuffle.get("Arm Position").update(position + " " + positionValue);
        SmartShuffle.get("Arm Extension").update(extension + " " + extensionValue);
    }
}