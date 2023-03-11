// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  public final CANSparkMax rearLeftMax;
  public final CANSparkMax frontLeftMax;
  public final CANSparkMax frontRightMax;
  public final CANSparkMax rearRightMax;

  public final RelativeEncoder rearLeftEncoder;
  public final RelativeEncoder frontLeftEncoder;
  public final RelativeEncoder frontRightEncoder;
  public final RelativeEncoder rearRightEncoder;

  public final MotorControllerGroup leftGroup;
  public final MotorControllerGroup rightGroup;

  private final DifferentialDrive drive;

  private MotorType motorType = MotorType.kBrushless;

  public final AHRS gyro;

  public PIDController feetPID = new PIDController(Constants.PID.FEET_P, Constants.PID.FEET_I, Constants.PID.FEET_D);
  public PIDController rotatePID = new PIDController(Constants.PID.ROTATE_P, Constants.PID.ROTATE_I, Constants.PID.ROTATE_D);

  public DriveTrain() {
    this.rearLeftMax = new CANSparkMax(Constants.CAN.REAR_LEFT_DRIVE, motorType);
    this.rearLeftEncoder = rearLeftMax.getEncoder();
    this.frontLeftMax = new CANSparkMax(Constants.CAN.FRONT_LEFT_DRIVE, motorType);
    this.frontLeftEncoder = frontLeftMax.getEncoder();
    this.frontRightMax = new CANSparkMax(Constants.CAN.FRONT_RIGHT_DRIVE, motorType);
    this.frontRightEncoder = frontRightMax.getEncoder();
    this.rearRightMax = new CANSparkMax(Constants.CAN.REAR_RIGHT_DRIVE, motorType);
    this.rearRightEncoder = rearRightMax.getEncoder();
    this.leftGroup = new MotorControllerGroup(rearLeftMax, frontLeftMax);
    this.rightGroup = new MotorControllerGroup(rearRightMax, frontRightMax);
    this.leftGroup.setInverted(true);
    this.drive = new DifferentialDrive(leftGroup, rightGroup);
    this.rotatePID.enableContinuousInput(-180.0f, 180.0f);
    this.gyro = new AHRS(SPI.Port.kMXP);
    this.gyro.reset();
    this.gyro.calibrate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rear Left Wheel Speed", rearLeftMax.get());
    SmartDashboard.putNumber("Front Left Wheel Speed", frontLeftMax.get());
    SmartDashboard.putNumber("Front Right Wheel Speed", frontRightMax.get());
    SmartDashboard.putNumber("Rear Right Wheel Speed", rearRightMax.get());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    if (Math.abs(leftSpeed) > 0.05 || Math.abs(rightSpeed) > 0.05) {
      double difference = Math.abs(leftSpeed - rightSpeed);
      double multiplier;

      if (leftSpeed > rightSpeed) {
        multiplier = Math.abs(leftSpeed) * 0.55;
      } else {
        multiplier = Math.abs(rightSpeed) * 0.55;
      }

      if (leftSpeed < 0 && rightSpeed < 0) {
        if (leftSpeed < rightSpeed) {
          drive.tankDrive(leftSpeed + ((0.2 + multiplier) * difference), rightSpeed);
        } else {
          drive.tankDrive(leftSpeed, rightSpeed + ((0.2 + multiplier) * difference));
        }
      } else if (leftSpeed > 0 && rightSpeed > 0) {
        if (leftSpeed < rightSpeed) {
          drive.tankDrive(leftSpeed + ((0.2 + multiplier) * difference), rightSpeed);
        } else {
          drive.tankDrive(leftSpeed, rightSpeed + ((0.2 + multiplier) * difference));
        }
      } else {
        drive.tankDrive(Math.pow(leftSpeed, 3), Math.pow(rightSpeed, 3));
      }
    }
  }

  public void rotateToAngle(double angle) {
    double val = MathUtil.clamp(this.rotatePID.calculate(Math.round(this.gyro.getYaw()), Math.round(angle)), -.5, .5);
    tankDrive(-val, val);
  }

  public double feetToTicks(double feet) {
    return feet / Constants.MATH.TICKS_FEET_DIVISOR;
  }
}
