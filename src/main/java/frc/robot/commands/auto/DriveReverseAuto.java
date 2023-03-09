// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;

public class DriveReverseAuto extends CommandBase {

  private boolean finished = false;

  private final DriveTrain driveTrain;
  private final Arms arms;

  /** Creates a new DriveReverseAuto. */
  public DriveReverseAuto(DriveTrain driveTrain, Arms arms) {
    addRequirements(driveTrain, arms);
    this.driveTrain = driveTrain;
    this.arms = arms;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.zero();
    if (driveTrain.frontLeftMax.getEncoder().getPosition() > driveTrain.feetToTicks(-12.5)
        || driveTrain.frontRightMax.getEncoder().getPosition() < driveTrain.feetToTicks(12.5)) {
      driveTrain.leftGroup.set(-MathUtil.clamp(
          driveTrain.feetPID.calculate(driveTrain.frontLeftMax.getEncoder().getPosition(),
              driveTrain.feetToTicks(-12.5)),
          -0.4,
          0.4));
      driveTrain.rightGroup.set(MathUtil.clamp(
          driveTrain.feetPID.calculate(driveTrain.frontRightMax.getEncoder().getPosition(),
              driveTrain.feetToTicks(12.5)),
          -0.4, 0.4));
    } else {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
