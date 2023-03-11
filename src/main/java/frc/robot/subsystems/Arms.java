// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void setSpeed(double leftSpeed, double rightSpeed) {
    this.leftArm.set(leftSpeed);
    this.rightArm.set(rightSpeed);
  }

  public void scoreHigh() {
    leftArm.set(
        MathUtil.clamp(this.armPID.calculate(Math.round(this.leftEncoder.getPosition()), 17), -0.3, 0.3));
    rightArm.set(
        MathUtil.clamp(this.armPID.calculate(Math.round(this.rightEncoder.getPosition()), -17), -0.3,
            0.3));
  }

  public void zero() {
    if (Math.abs(leftEncoder.getPosition()) <= 1) {
      leftArm.set(0);
    } else if (Math.abs(rightEncoder.getPosition()) <= 1) {
      rightArm.set(0);
    } else {
      leftArm.set(
          MathUtil.clamp(armPID.calculate(Math.round(leftEncoder.getPosition()), 1), -0.05,
              0.05));
      rightArm.set(
          MathUtil.clamp(armPID.calculate(Math.round(rightEncoder.getPosition()), -1), -0.05,
              0.05));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Arm Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Left Arm Speed", leftArm.get());
    SmartDashboard.putNumber("Right Arm Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Right Arm Speed", rightArm.get());
  }
}
