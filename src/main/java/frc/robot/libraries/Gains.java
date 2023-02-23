/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.libraries;

import edu.wpi.first.math.controller.PIDController;

public class Gains {
	/**
	 * proportional
	 */
	public final double kP;

	/**
	 * integral
	 */
	public final double kI;

	/**
	 * derivative
	 */
	public final double kD;

	/**
	 * according to colton, this is the kFinn constant. 
	 */
	public final double kF;

	public Gains(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}

	public Gains(double kP, double kI, double kD) {
		this(kP, kI, kD, 0.0d);
	}

	/**
	 * Returns a new PID Controller
	 */
	public PIDController getPID() {
		return new PIDController(kP, kI, kD);
	}
}
