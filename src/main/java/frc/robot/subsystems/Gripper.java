// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Gripper extends SubsystemBase {

  private final Compressor compressor;
  public final DoubleSolenoid solenoid;
  
  /** Creates a new Gripper. */
  public Gripper() {
    this.compressor = new Compressor(Constants.PCM.COMPRESSOR, PneumaticsModuleType.CTREPCM);
    this.compressor.enableDigital();
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.SOLENOID1[0], Constants.PCM.SOLENOID1[1]);
    this.solenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {}

  public void setValue(Value value) {
    this.solenoid.set(value);
  }
}
