package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.*;

public class Arm extends SubsystemBase {

    //---------- Motors ----------\\
    private WPI_TalonSRX positionMotor = new WPI_TalonSRX(Constants.PortsConstants.LINEAR_ACTUATOR_PORT);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(Constants.PortsConstants.EXTENTION_PORT);
    private Servo grabberServo = new Servo(Constants.PortsConstants.GRABBER_PORT);

    private Encoder positionEncoder = new Encoder(Constants.PortsConstants.ARM_ENCODER_PORT, Constants.PortsConstants.ARM_ENCODER_PORT + 1);
    // For simulation purposes
    private EncoderSim simPositionEncoder = new EncoderSim(positionEncoder);

    //---------- Subsystems ----------\\
    private DriveTrain driveTrain;


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
    /**Max Angle between the bottom position and the top position */
    private double kmaxAngle = 57.01;

    private double kmaxExtension = 42.357;

    /**Our wanted position (double) */
    private double positionValue;
    /**An Enum representation of our wanted position*/
    private Position position;
    /**Our wanted extension (double) */
    private double extensionValue;
    /**An Enum representation of our wanted extetnion*/
    private Extension extension;

    private double currentExtension;
    private double currentPosition;

    private static final double kPositionTolerance = 0.05;

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
        positionEncoder.reset();

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

        currentExtension = extensionMotor.getSelectedSensorPosition();

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
    public void setArmextension(Extension extension) {
        this.positionValue = extension.length;
        this.extension = extension;
    }

    /**
     * Sets the arm extension to the double extension
     * @param extension the double value of extension to move the arm to
     */
    public void setArmextension(double extension) {
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

    /**
     * Use to update the values on Shuffleboard
     * ! Make sure method is called in periodic or values won't update
     */
    public void updateShuffleBoardValues() {
        SmartShuffle.get("Arm Position").update(position + " " + positionValue);
        SmartShuffle.get("Arm Extension").update(extension + " " + extensionValue);
    }


}
