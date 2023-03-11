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

  private boolean finished = false;

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
    if (Math.abs(arms.leftEncoder.getPosition()) <= 1) {
      finished = true;
    } else if (Math.abs(arms.rightEncoder.getPosition()) <= 1) {
      finished = true;
    } else {
      arms.leftArm.set(
          MathUtil.clamp(arms.armPID.calculate(Math.round(arms.leftEncoder.getPosition()), 1), -0.05,
              0.05));
      arms.rightArm.set(
          MathUtil.clamp(arms.armPID.calculate(Math.round(arms.rightEncoder.getPosition()), -1), -0.05,
              0.05));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.leftArm.set(0);
    arms.rightArm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
