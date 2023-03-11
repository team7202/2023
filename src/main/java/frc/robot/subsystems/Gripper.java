// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Gripper extends SubsystemBase {

  public final DoubleSolenoid solenoid;
  public final Value value;

  /** Creates a new Gripper. */
  public Gripper() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.SOLENOID1[0],
        Constants.PCM.SOLENOID1[1]);
    this.value = Value.kReverse;
    this.solenoid.set(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Gripper Mode", this.solenoid.get() == Value.kForward ? "Reverse" : "Forward");

  }

  public void toggleValue() {
    Value val = value == Value.kForward ? Value.kReverse : Value.kForward;
    this.solenoid.set(val);
  }
}
