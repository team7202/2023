// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;

public class AutonomousCommand extends CommandBase {

  private DriveTrain driveTrain;
  private Gripper gripper;
  Timer timer;
  // private boolean backwards = false;
  // private boolean balance = false;
  private String mode = "";

  private PIDController feetPID;
  // boolean needToCenter = false;

  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(DriveTrain driveTrain, Gripper gripper) {
    this.driveTrain = driveTrain;
    this.gripper = gripper;
    this.feetPID = new PIDController(0.5, 0.25, 0.01);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = new Timer();
    this.mode = "forward";
    driveTrain.gyro.reset();
    timer.reset();
    timer.start();
    driveTrain.frontLeft.getEncoder().setPosition(0);
    driveTrain.frontRight.getEncoder().setPosition(0);
    SmartDashboard.putNumber("driveFeet", 6);
    driveTrain.leftGroup.set(0);
    driveTrain.rightGroup.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getString("AutoRoute", "center").equals("center")) {
      runCenterCode();
    } else if (SmartDashboard.getString("AutoRoute", "center").equals("left")) {
      runLeftCode();
      // driveTrain.tankDrive(0.1, -0.1);
    } else if (SmartDashboard.getString("AutoRoute", "center").equals("right")) {
      runRightCode();
      // driveTrain.tankDrive(-0.1, 0.1);
    } else {
      runCenterCode();
    }
  }

  private void runCenterCode() {
    RobotContainer.armMode = "2nd";
    if (timer.get() < 2) {
      gripper.setGripperMode(Value.kForward);
    } else {
      gripper.setGripperMode(Value.kReverse);
      RobotContainer.armMode = "zero";
      RobotContainer.sliderMode = "zero";
      if (timer.get() > 2.5) {
        if (mode.equals("forward")) {
          if (driveTrain.frontLeft.getEncoder().getPosition() > driveTrain.ticksToFeet(-12.5)
              || driveTrain.frontRight.getEncoder().getPosition() < driveTrain.ticksToFeet(12.5)) {
            driveTrain.leftGroup.set(-MathUtil.clamp(
                this.feetPID.calculate(driveTrain.frontLeft.getEncoder().getPosition(), driveTrain.ticksToFeet(-12.5)),
                -0.4,
                0.4));
            driveTrain.rightGroup.set(MathUtil.clamp(
                this.feetPID.calculate(driveTrain.frontRight.getEncoder().getPosition(), driveTrain.ticksToFeet(12.5)),
                -0.4, 0.4));
          } else {
            mode = "backwards";
          }
        }

        else if (mode.equals("backwards")) {
          if (driveTrain.getYaw() > 3 || driveTrain.getYaw() > 3) {
            driveTrain.rotateToAngle(0);
          } else {
            if (driveTrain.frontLeft.getEncoder().getPosition() < driveTrain.ticksToFeet(-6)
                || driveTrain.frontRight.getEncoder().getPosition() > driveTrain.ticksToFeet(6)) {
              driveTrain.leftGroup.set(-MathUtil.clamp(
                  this.feetPID.calculate(driveTrain.frontLeft.getEncoder().getPosition(), driveTrain.ticksToFeet(-6)),
                  -0.4,
                  0.4));
              driveTrain.rightGroup.set(MathUtil.clamp(
                  this.feetPID.calculate(driveTrain.frontRight.getEncoder().getPosition(), driveTrain.ticksToFeet(6)),
                  -0.4, 0.4));
            } else {
              mode = "balance";
            }
          }
        }

        else if (mode.equals("balance")) {
          if (driveTrain.getPitch() < -4.5) {
            driveTrain.tankDrive(-0.26, -0.26);
          } else if (driveTrain.getPitch() > 7.5) {
            driveTrain.tankDrive(0.26, 0.26);
          }
        }
      }
    }
  }

  private void runLeftCode() {
    RobotContainer.armMode = "2nd";
    if (driveTrain.frontLeft.getEncoder().getPosition() > driveTrain.ticksToFeet(-5)
        || driveTrain.frontRight.getEncoder().getPosition() < driveTrain.ticksToFeet(5)) {
      driveTrain.leftGroup.set(-MathUtil.clamp(
          this.feetPID.calculate(driveTrain.frontLeft.getEncoder().getPosition(), driveTrain.ticksToFeet(-5)), -0.25,
          0.25));
      driveTrain.rightGroup.set(MathUtil.clamp(
          this.feetPID.calculate(driveTrain.frontRight.getEncoder().getPosition(), driveTrain.ticksToFeet(5)),
          -0.25, 0.25));
    }
  }

  private void runRightCode() {
    RobotContainer.armMode = "2nd";
    if (driveTrain.frontLeft.getEncoder().getPosition() > driveTrain.ticksToFeet(-5)
        || driveTrain.frontRight.getEncoder().getPosition() < driveTrain.ticksToFeet(5)) {
      driveTrain.leftGroup.set(-MathUtil.clamp(
          this.feetPID.calculate(driveTrain.frontLeft.getEncoder().getPosition(), driveTrain.ticksToFeet(-5)), -0.25,
          0.25));
      driveTrain.rightGroup.set(MathUtil.clamp(
          this.feetPID.calculate(driveTrain.frontRight.getEncoder().getPosition(), driveTrain.ticksToFeet(5)),
          -0.25, 0.25));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
