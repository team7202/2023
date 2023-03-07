// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  
  private final CANSparkMax rearLeftMax;
  private final CANSparkMax frontLeftMax;
  private final CANSparkMax frontRightMax; 
  private final CANSparkMax rearRightMax;

  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  private final DifferentialDrive drive;
  
  public DriveTrain() {
    this.rearLeftMax = new CANSparkMax(Constants.CAN.REAR_LEFT_DRIVE, MotorType.kBrushed);
    this.frontLeftMax = new CANSparkMax(Constants.CAN.FRONT_LEFT_DRIVE, MotorType.kBrushed);
    this.frontRightMax = new CANSparkMax(Constants.CAN.FRONT_RIGHT_DRIVE, MotorType.kBrushed);
    this.rearRightMax = new CANSparkMax(Constants.CAN.REAR_RIGHT_DRIVE, MotorType.kBrushed);
    this.leftGroup = new MotorControllerGroup(rearLeftMax, frontLeftMax);
    this.rightGroup = new MotorControllerGroup(rearRightMax, frontRightMax);
    this.rightGroup.setInverted(true);
    this.drive = new DifferentialDrive(leftGroup, rightGroup);
  }

  @Override
  public void periodic() {}

  public void tankDrive(double leftSpeed, double rightSpeed) {
    this.drive.tankDrive(leftSpeed, rightSpeed);
  }
}
