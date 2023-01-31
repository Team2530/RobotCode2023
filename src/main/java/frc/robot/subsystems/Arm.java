package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    //---------- Motors ----------\\
    private TalonSRX positionMotor = new TalonSRX(Constants.PortsConstants.LINEAR_ACTUATOR_PORT);
    private TalonFX extentionMotor = new TalonFX(Constants.PortsConstants.EXTENTION_PORT);

    private Encoder positionEncoder = new Encoder(Constants.PortsConstants.ARM_ENCODER_PORT, Constants.PortsConstants.ARM_ENCODER_PORT + 1);
    // For simulation purposes
    private EncoderSim simPositionEncoder = new EncoderSim(positionEncoder);

    //---------- Subsystems ----------\\
    private DriveTrain driveTrain;


    //---------- Preset Positions ----------\\
    public enum Extention {
        FULL(1),
        TWOTHIRDS(.66),
        HALF(.5),
        ONEQUARTER(.25),
        RETRACTED(0);

        private double length;

        private Extention(double length){
            this.length = length;
        }
    }

    public enum Position {
        HIGH(1),
        MEDIUM(.66),
        HALF(0.5),
        LOW(.2),
        FLOOR(0),
        Custom(-1);

        private double position;

        private static final Position[] vals = values();

        private Position(double position){
            this.position = position;
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

    /**Our wanted position */
    private double position;
    /**An Enum representation of our wanted position*/
    private Position positionValue;
    /**Our wanted extention */
    private double extention;
    /**An Enum representation of our wanted extetnion*/
    private Extention extentionValue;

    private double currentExtention;
    private double currentPosition;

    private static final double kPositionTolerance = 0.05;

    /**
     * Constructs a new Arm
     * @param driveTrain our DriveTrain
     */
    public Arm(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        // Initial Arm Conditions
        this.positionValue = Position.HIGH;
        this.extentionValue = Extention.RETRACTED;
        // More initial Conditions
        this.position = 1.0;
        this.extention = 0.0;
    }

    @Override
    public void periodic() {

        currentPosition = positionEncoder.getDistance();

        if(Math.abs(currentPosition - position) > kPositionTolerance) {
            positionMotor.set(TalonSRXControlMode.PercentOutput, Math.signum(position - currentExtention));
        }

        // System.out.println(currentPosition + " " + position + " " + positionValue.toString());
    }

    /**
     * Sets the Arm Position to a preset position
     * @param position the Position value in {@code Position}
     */
    public void setArmPosition(Position position) {
        this.position = position.position;
        this.positionValue = position;
    }

    /**
     * Sets the arm to the passed in position
     * @param position the double value of position to move the arm to
     */
    public void setArmPosition(double position) {
        this.position = position;
        positionValue = Position.Custom;
    }

    /**
     * Sets the arm to the preset extention
     * @param extention the Extention value in {@code Extention}
     */
    public void setArmExtention(Extention extention) {
        this.extention = extention.length;
    }

    /**
     * Sets the arm extention to the double extention
     * @param extention the double value of extention to move the arm to
     */
    public void setArmExtention(double extention) {
        this.extention = extention;
    }

    /**
     * Raises the arm by a Setpoint
     */

    public void raiseArm() {
        positionValue = positionValue.raise();
        position = positionValue.position;
    }

    /**
     * Lowers the arm by a Setpoint
     */
    public void lowerArm() {
        positionValue = positionValue.lower();
        position = positionValue.position;
    }


}
