// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class ZeroArms extends CommandBase {
  /** Creates a new ZeroArms. */

  private final Arms arms;

  public ZeroArms(Arms arms) {
    addRequirements(arms);
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
    if (Math.abs(arms.leftArm.getEncoder().getPosition()) <= 1) {
      arms.leftArm.set(0);
    } else if (Math.abs(arms.rightArm.getEncoder().getPosition()) <= 1) {
      arms.rightArm.set(0);
    } else {
      arms.leftArm.set(
          MathUtil.clamp(arms.armPID.calculate(Math.round(arms.leftArm.getEncoder().getPosition()), 1), -0.08,
              0.08));
      arms.rightArm.set(
          MathUtil.clamp(arms.armPID.calculate(Math.round(arms.rightArm.getEncoder().getPosition()), -1), -0.08,
              0.08));
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
