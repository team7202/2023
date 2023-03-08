// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arms extends SubsystemBase {

  public final CANSparkMax leftArm;
  public final CANSparkMax rightArm;

  public final RelativeEncoder leftEncoder;
  public final RelativeEncoder rightEncoder;

  public final PIDController armPID = new PIDController(Constants.PID.ARM_P, Constants.PID.ARM_I, Constants.PID.ARM_D);

  /** Creates a new Arms. */
  public Arms() {
    this.leftArm = new CANSparkMax(Constants.CAN.LEFT_ARM, MotorType.kBrushless);
    this.leftEncoder = leftArm.getEncoder();
    this.leftEncoder.setPosition(0);
    this.rightArm = new CANSparkMax(Constants.CAN.RIGHT_ARM, MotorType.kBrushless);
    this.rightEncoder = rightArm.getEncoder();
    this.rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}