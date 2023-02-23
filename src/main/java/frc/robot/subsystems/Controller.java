// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Unifies all input sources into a singular input interface. */
public class Controller extends SubsystemBase {
  /**
   * Gets the first joystick port that exists. If the robot is simulated,
   * the Joystick port is defaulted to 0
   * 
   * @return Joystick port
   * @throws Error if the joystick isn't connected to the computer
   */
  public static int getJoystickPort() {
      for (int i = 0; i < 6; i++) {
        if (DriverStation.getJoystickName(i).contains("Logitech Extreme 3D")) {
          return i;
        }
      }
      throw new Error(
          "Joystick seems not to be connected! " + " Make sure the Joystick is connected to the computer");
  }

  /**
   * Gets the first Xbox port that exists. If the robot is simulated,
   * the Xbox port is defaulted to 1
   * 
   * @return Xbox port
   * @throws Error if the Xbox isn't connected to the computer
   */
  public static int getXboxPort() {
      for (int i = 0; i < 6; i++) {
        if (DriverStation.getJoystickName(i).contains("Xbox")) {
          return i;
        }
      }
      throw new Error("Xbox seems not to be connected! Make sure the Xbox is connected to the computer");
  }
}
