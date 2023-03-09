// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase {

  public final CANSparkMax leftSlider;
  public final CANSparkMax rightSlider;

  public final RelativeEncoder leftEncoder;
  public final RelativeEncoder rightEncoder;

  public final PIDController sliderPID = new PIDController(Constants.PID.SLIDER_P, Constants.PID.SLIDER_I, Constants.PID.SLIDER_D);

  private final DigitalInput leftLowerLimit;
  private final DigitalInput leftUpperLimit;
  private final DigitalInput rightLowerLimit;
  private final DigitalInput rightUpperLimit;

  /** Creates a new Slider. */
  public Slider() {
    this.leftLowerLimit = new DigitalInput(Constants.DI.LEFT_LOWER_SLIDER_SW);
    this.leftUpperLimit = new DigitalInput(Constants.DI.LEFT_UPPER_SLIDER_SW);
    this.rightLowerLimit = new DigitalInput(Constants.DI.RIGHT_LOWER_SLIDER_SW);
    this.rightUpperLimit = new DigitalInput(Constants.DI.RIGHT_UPPER_SLIDER_SW);
    this.leftSlider = new CANSparkMax(Constants.CAN.LEFT_SLIDER, MotorType.kBrushless);
    this.leftEncoder = this.leftSlider.getEncoder();
    this.rightSlider = new CANSparkMax(Constants.CAN.RIGHT_SLIDER, MotorType.kBrushless);
    this.rightEncoder = this.rightSlider.getEncoder();
  }

  @Override
  public void periodic() {
    if(!leftLowerLimit.get() || !rightLowerLimit.get()) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      leftSlider.set(0);
      rightSlider.set(0);
    }

    if(!leftUpperLimit.get() || !rightUpperLimit.get()) {
      leftSlider.set(0);
      rightSlider.set(0);
    }

    SmartDashboard.putNumber("leftSlider", leftEncoder.getPosition());
    SmartDashboard.putNumber("rightSlider", rightEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
