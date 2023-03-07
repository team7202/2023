// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Slider extends SubsystemBase {

  private final CANSparkMax leftSlider;
  private final CANSparkMax rightSlider;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  private final PIDController sliderPID = new PIDController(0.05, 0, 0);

  private final DigitalInput leftLowerLimit;
  private final DigitalInput leftUpperLimit;
  private final DigitalInput rightLowerLimit;
  private final DigitalInput rightUpperLimit;

  /** Creates a new Slider. */
  public Slider(Joystick leftJoystick, Joystick rightJoystick) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.leftLowerLimit = new DigitalInput(0);
    this.leftUpperLimit = new DigitalInput(1);
    this.rightLowerLimit = new DigitalInput(2);
    this.rightUpperLimit = new DigitalInput(3);
    this.leftSlider = new CANSparkMax(Constants.DriveTrain.LEFT_SLIDER, MotorType.kBrushless);
    this.rightSlider = new CANSparkMax(Constants.DriveTrain.RIGHT_SLIDER, MotorType.kBrushless);
    this.leftSlider.getEncoder().setPosition(0);
    this.rightSlider.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("leftSlider", leftSlider.getEncoder().getPosition());
    SmartDashboard.putNumber("rightSlider", rightSlider.getEncoder().getPosition());

    SmartDashboard.putNumber("POV", leftJoystick.getPOV());

    SmartDashboard.putBoolean("LEFT_LOWER", leftLowerLimit.get());
    SmartDashboard.putBoolean("LEFT_UPPER", leftUpperLimit.get());
    SmartDashboard.putBoolean("RIGHT_LOWER", rightLowerLimit.get());
    SmartDashboard.putBoolean("RIGHT_UPPER", rightUpperLimit.get());

    if (leftJoystick.getRawButtonPressed(2)) {
      RobotContainer.sliderMode = "zero";
    }

    if (!leftLowerLimit.get()) {
      leftSlider.getEncoder().setPosition(0);
      RobotContainer.sliderMode = "off";
    }

    if (!rightLowerLimit.get()) {
      rightSlider.getEncoder().setPosition(0);
      RobotContainer.sliderMode = "off";
    }

    // if (leftJoystick.getRawButtonPressed(4)) {
    // RobotContainer.sliderMode = "up";
    // }

    // if (rightJoystick.getRawButtonPressed(4)) {
    // RobotContainer.sliderMode = "up";
    // }

    SmartDashboard.putString("sliderMode", RobotContainer.sliderMode);
    SmartDashboard.putString("armMode", RobotContainer.armMode);

    if (leftJoystick.getPOV() == 180) {
      if (!leftLowerLimit.get()) {
        leftSlider.set(0);
        rightSlider.set(0);
      } else {
        leftSlider.set(0.1);
      }
      if (!rightLowerLimit.get()) {
        rightSlider.set(0);
        leftSlider.set(0);
      } else {
        rightSlider.set(-0.1);
      }
      RobotContainer.sliderMode = "off";
    } else if (leftJoystick.getPOV() == 0) {
      if (!leftUpperLimit.get()) {
        leftSlider.set(0);
        rightSlider.set(0);
      } else {
        leftSlider.set(-0.1);
      }
      if (!rightUpperLimit.get()) {
        rightSlider.set(0);
        leftSlider.set(0);
      } else {
        rightSlider.set(0.1);
      }
      RobotContainer.sliderMode = "off";
    } else {
      if (RobotContainer.armMode.equals("1st")) {
        leftSlider.set(0);
        rightSlider.set(0);
      } else if (RobotContainer.armMode.equals("2nd")) {
        // if(leftUpperLimit.get())
        // if(rightLowerLimit.get())
        leftSlider.set(MathUtil
            .clamp(this.sliderPID.calculate(Math.round(this.leftSlider.getEncoder().getPosition()), -30), -.15, .15));
        rightSlider.set(MathUtil
            .clamp(this.sliderPID.calculate(Math.round(this.rightSlider.getEncoder().getPosition()), 30), -.15, .15));
      } else if (RobotContainer.armMode.equals("3rd")) {
        leftSlider.set(MathUtil
            .clamp(this.sliderPID.calculate(Math.round(this.leftSlider.getEncoder().getPosition()), -17.5), -.15, .15));
        rightSlider.set(MathUtil
            .clamp(this.sliderPID.calculate(Math.round(this.rightSlider.getEncoder().getPosition()), 17.5), -.15, .15));
      } else if (RobotContainer.sliderMode.equals("zero")) {
        if (Math.abs(this.leftSlider.getEncoder().getPosition()) <= 1) {
          this.leftSlider.set(0);
        } else if (Math.abs(this.rightSlider.getEncoder().getPosition()) <= 1) {
          this.rightSlider.set(0);
        } else {
          leftSlider.set(
              MathUtil.clamp(this.sliderPID.calculate(Math.round(this.leftSlider.getEncoder().getPosition()), 1),
                  -0.15,
                  0.15));
          rightSlider.set(
              MathUtil.clamp(this.sliderPID.calculate(Math.round(this.rightSlider.getEncoder().getPosition()), -1),
                  -0.15,
                  0.15));
        }
        // leftSlider.set(MathUtil
        // .clamp(this.sliderPID.calculate(Math.round(this.leftSlider.getEncoder().getPosition()),
        // 1), -.15, .15));
        // rightSlider.set(MathUtil
        // .clamp(this.sliderPID.calculate(Math.round(this.rightSlider.getEncoder().getPosition()),
        // -1), -.15, .15));
      } else {
        leftSlider.set(0);
        rightSlider.set(0);
      }
    }
  }

  public CANSparkMax getLeftSlider() {
    return leftSlider;
  }

  public CANSparkMax getRightSlider() {
    return rightSlider;
  }
}