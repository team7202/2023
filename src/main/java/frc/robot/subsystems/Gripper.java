// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {

  private final Compressor compressor;
  private final DoubleSolenoid solenoid;

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  private Value gripperMode = Value.kReverse;

  /** Creates a new Gripper. */
  public Gripper(Joystick leftJoystick, Joystick rightJoystick) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.compressor = new Compressor(Constants.Gripper.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);
    this.compressor.enableDigital();
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    this.solenoid.set(gripperMode);
  }

  @Override
  public void periodic() {
    // if(leftJoystick.getTriggerPressed()) {
    //   gripperMode = Value.kReverse;
    // } else if(rightJoystick.getTriggerPressed()) {
    //   gripperMode = Value.kForward;      
    // }

    if(rightJoystick.getTriggerPressed()) {
      if(gripperMode == Value.kReverse) {
        gripperMode = Value.kForward;
      } else {
        gripperMode = Value.kReverse;
      }
    }

    solenoid.set(gripperMode);
  }

  public void setGripperMode(Value value) {
    this.gripperMode = value;
  }
}

