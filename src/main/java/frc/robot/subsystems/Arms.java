// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arms extends SubsystemBase {

  private final CANSparkMax leftArm;
  private final CANSparkMax rightArm;

  private final PIDController armPID = new PIDController(0.03, 0.04, 0);
  private Joystick rightJoystick;

  /** Creates a new Arms. */
  public Arms(Joystick leftJoystick, Joystick rightJoystick) {
    this.rightJoystick = rightJoystick;
    this.leftArm = new CANSparkMax(Constants.DriveTrain.LEFT, MotorType.kBrushless);
    this.rightArm = new CANSparkMax(Constants.DriveTrain.RIGHT, MotorType.kBrushless);
    this.leftArm.getEncoder().setPosition(0);
    this.rightArm.getEncoder().setPosition(0);
    SmartDashboard.putNumber("armP", armPID.getP());
    SmartDashboard.putNumber("armI", armPID.getI());
    SmartDashboard.putNumber("armD", armPID.getD());
    SmartDashboard.putNumber("armSpeed", 0.2);
  }

  @Override
  public void periodic() {

    armPID.setPID(SmartDashboard.getNumber("armP", 0), SmartDashboard.getNumber("armI", 0),
        SmartDashboard.getNumber("armD", 0));

    SmartDashboard.putNumber("leftArm", leftArm.getEncoder().getPosition());
    SmartDashboard.putNumber("rightArm", rightArm.getEncoder().getPosition());
    // System.out.println(rightJoystick.getPOV());

    if (rightJoystick.getRawButtonPressed(2)) {
      RobotContainer.sliderMode = "zero";
      RobotContainer.armMode = "zero";
    }

    if (rightJoystick.getRawButtonPressed(3)) {
      RobotContainer.armMode = "1st";
    }

    if (rightJoystick.getRawButtonPressed(4)) {
      RobotContainer.armMode = "2nd";
    }

    // if(rightJoystick.getRawButtonPressed(3)) {
    // this.armMode = "low-up";
    // }

    // if(rightJoystick.getRawButtonPressed(4)) {
    // this.armMode = "high-up";
    // }

    if (rightJoystick.getPOV() == 0) {
      RobotContainer.armMode = "off";
      leftArm.set(0.1);
      rightArm.set(-0.1);
    } else if (rightJoystick.getPOV() == 180) {
      RobotContainer.armMode = "off";
      leftArm.set(-0.1);
      rightArm.set(0.1);
    } else if (RobotContainer.armMode.equals("1st")) {
      RobotContainer.sliderMode = "off";
      leftArm.set(
          MathUtil.clamp(this.armPID.calculate(Math.round(this.leftArm.getEncoder().getPosition()), 12), -0.3, 0.3));
      rightArm.set(
          MathUtil.clamp(this.armPID.calculate(Math.round(this.rightArm.getEncoder().getPosition()), -12), -0.3,
              0.3));
    } else if (RobotContainer.armMode.equals("2nd")) {
      leftArm.set(
          MathUtil.clamp(this.armPID.calculate(Math.round(this.leftArm.getEncoder().getPosition()), 17), -0.3, 0.3));
      rightArm.set(
          MathUtil.clamp(this.armPID.calculate(Math.round(this.rightArm.getEncoder().getPosition()), -16.75), -0.3,
              0.3));
    } else if (RobotContainer.armMode.equals("zero")) {
      if (Math.abs(this.leftArm.getEncoder().getPosition()) <= 1) {
        this.leftArm.set(0);
      } else if (Math.abs(this.rightArm.getEncoder().getPosition()) <= 1) {
        this.rightArm.set(0);
      } else {
        leftArm.set(
            MathUtil.clamp(this.armPID.calculate(Math.round(this.leftArm.getEncoder().getPosition()), 1), -0.08,
                0.08));
        rightArm.set(
            MathUtil.clamp(this.armPID.calculate(Math.round(this.rightArm.getEncoder().getPosition()), -1), -0.08,
                0.08));
      }
    } else {
      leftArm.set(0);
      rightArm.set(0);
    }
  }
  // This method will be called once per scheduler run
}
