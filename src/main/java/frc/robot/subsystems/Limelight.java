// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  private final DriveTrain driveTrain;
  private final NetworkTable table;
  private NetworkTableEntry pipeline;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final PIDController rotatePID;
  private final PIDController drivePID;

  /** Creates a new Limelight. */
  public Limelight(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
    this.pipeline = this.table.getEntry("pipeline");
    this.pipeline.setNumber(1);
    this.tx = this.table.getEntry("tx");
    this.ty = this.table.getEntry("ty");
    this.ta = this.table.getEntry("ta");
    this.rotatePID = new PIDController(Constants.Limelight.ROTATE_P, Constants.Limelight.ROTATE_I,
        Constants.Limelight.ROTATE_D);
    this.drivePID = new PIDController(Constants.Limelight.DRIVE_P, Constants.Limelight.DRIVE_I,
        Constants.Limelight.DRIVE_D);
  }

  @Override
  public void periodic() {

    // if (RobotContainer.leftJoystick.getRawButtonPressed(7)) {
    // this.pipeline.setNumber(0);
    // }

    // if (RobotContainer.leftJoystick.getRawButtonPressed(6)) {
    // this.pipeline.setNumber(1);
    // }

    // if (RobotContainer.leftJoystick.getRawButtonPressed(5)) {
    // this.pipeline.setNumber(2);
    // }

    var rotateVal = MathUtil.clamp(this.rotatePID.calculate(Math.round(tx.getDouble(0)), 0), -.4, .4);
    var driveVal = MathUtil.clamp(this.drivePID.calculate(ty.getDouble(0), 0), -.4, .4);

    if (RobotContainer.leftJoystick.getRawButton(3)) {
      if (this.pipeline.getInteger(0) != 0) {
        this.pipeline.setNumber(0);
      }
      // if (Math.abs(tx.getDouble(0)) < 0.75 && (Math.abs(tx.getDouble(0)) > 0)) {
      //   driveTrain.tankDrive(driveVal, driveVal);
      // } else {
      //   driveTrain.tankDrive(-rotateVal, rotateVal);
      // }
      driveTrain.tankDrive(-rotateVal, rotateVal);
    } else {
      if (this.pipeline.getInteger(0) != 1) {
        this.pipeline.setNumber(1);
      }
    }
    // This method will be called once per scheduler run
  }
}
