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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax rearLeft;
  public final CANSparkMax frontLeft;
  public final CANSparkMax frontRight;
  private final CANSparkMax rearRight;

  public MotorControllerGroup leftGroup;
  public MotorControllerGroup rightGroup;

  private DifferentialDrive drive;

  private Joystick leftJoystick;
  private Joystick rightJoystick;

  public final AHRS gyro;
  private final PIDController pid;
  private double angleToRotate;

  private RelativeEncoder relativeEncoder;

  private float pitch;
  private float roll;
  private float yaw;
  private double angle;

  private boolean tipOverride = false;

  /** Creates a new DriveTrain. */
  public DriveTrain(Joystick leftJoystick, Joystick rightJoystick) {
    this.frontLeft = new CANSparkMax(Constants.DriveTrain.FRONT_LEFT_ID, Constants.DriveTrain.FRONT_LEFT_TYPE);
    this.rearLeft = new CANSparkMax(Constants.DriveTrain.REAR_LEFT_ID, Constants.DriveTrain.REAR_LEFT_TYPE);
    this.frontRight = new CANSparkMax(Constants.DriveTrain.FRONT_RIGHT_ID, Constants.DriveTrain.FRONT_RIGHT_TYPE);
    this.rearRight = new CANSparkMax(Constants.DriveTrain.REAR_RIGHT_ID, Constants.DriveTrain.REAR_RIGHT_TYPE);

    this.leftGroup = new MotorControllerGroup(rearLeft, frontLeft);
    this.rightGroup = new MotorControllerGroup(rearRight, frontRight);
    this.leftGroup.setInverted(Constants.DriveTrain.LEFT_GROUP_INVERTED);
    this.rightGroup.setInverted(Constants.DriveTrain.RIGHT_GROUP_INVERTED);

    this.drive = new DifferentialDrive(leftGroup, rightGroup);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    this.gyro = new AHRS(Constants.DriveTrain.GYRO_PORT);
    gyro.reset();

    SmartDashboard.putNumber("rotate", 0);

    this.pid = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD);
    this.pid.enableContinuousInput(-180.0f, 180.0f);

    this.pitch = gyro.getPitch();
    this.roll = gyro.getRoll();
    this.yaw = gyro.getYaw();
    this.angle = gyro.getAngle();
    this.frontLeft.getEncoder().setPosition(0);
    this.frontRight.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    this.pitch = gyro.getPitch();
    this.roll = gyro.getRoll();
    this.yaw = gyro.getYaw();

    SmartDashboard.putNumber("frontLeftPos", frontLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("frontRightPos", frontRight.getEncoder().getPosition());

    // if(gyro.getPitch()> 10) {
    // tipOverride = true;
    // tankDrive(0.75, 0.75);
    // } else if(gyro.getPitch() < -10) {
    // tipOverride = true;
    // tankDrive(-0.75, -0.75);
    // } else {
    // tipOverride = false;
    // }

    // if (gyro.getPitch() < -9) {
    // drive.tankDrive(0.3, 0.3);
    // } else if (gyro.getPitch() > 9) {
    // drive.tankDrive(-0.3, -0.3);
    // }

    SmartDashboard.putNumber("pitch", gyro.getPitch());

    if (Math.abs(leftJoystick.getY()) >= 0.05 || Math.abs(rightJoystick.getY()) >= 0.05) {
      this.drive.tankDrive(Math.copySign(Math.pow(leftJoystick.getY(), 2), leftJoystick.getY()), Math.copySign(Math.pow(rightJoystick.getY(), 2), rightJoystick.getY()));
    }

    // if (leftJoystick.getRawButtonPressed(7)) {
    // this.left.getEncoder().setPosition(0);
    // this.right.getEncoder().setPosition(0);
    // }

    // if(rightJoystick.getRawButtonPressed()) {
    // this.armMode = "off";
    // }

    SmartDashboard.putNumber("roll", gyro.getPitch());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    SmartDashboard.putNumber("angle", gyro.getAngle());
  }

  public float getPitch() {
    return pitch;
  }

  public float getRoll() {
    return roll;
  }

  public float getYaw() {
    return yaw;
  }

  public double getAngle() {
    return angle;
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    this.drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void rotateToAngle(double angle) {
    var val = MathUtil.clamp(this.pid.calculate(Math.round(this.gyro.getYaw()), Math.round(angle)), -.5, .5);
    // var val = MathUtil.clamp(this.pid.calculate(this.gyro.getYaw(), angle), -0.5,
    // 0.5);
    this.drive.tankDrive(-val, val);
  }

  public double ticksToFeet(double feet) {
    return feet / Constants.DriveTrain.FEET_DIVISOR;
  }

  public double percentToFormula(double percent) {
    if (percent >= 0) {
      return (0.6 * (Math.cbrt(percent - 0.5))) + 0.4762;
    } else {
      return -((0.6 * (Math.cbrt(-percent - 0.5))) + 0.4762);
    }
  }
}
