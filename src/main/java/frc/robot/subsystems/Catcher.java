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

public class Catcher extends SubsystemBase {
  
  private final Compressor compressor;
  public final DoubleSolenoid solenoid;
  private Value value;
  
  /** Creates a new Catcher. */
  public Catcher(Compressor compressor) {
    this.compressor = compressor;
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.SOLENOID2[0], Constants.PCM.SOLENOID2[1]);
    this.value = Value.kReverse;
  }

  @Override
  public void periodic() {
    this.solenoid.set(value);
    // This method will be called once per scheduler run
  }

  public void setValue(Value value) {
    this.value = value;
  }
}
