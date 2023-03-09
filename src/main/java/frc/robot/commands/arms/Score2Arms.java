// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class Score2Arms extends CommandBase {

  private final Arms arms;

  /** Creates a new Score2Arms. */
  public Score2Arms(Arms arms) {
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
    arms.leftArm.set(
        MathUtil.clamp(this.arms.armPID.calculate(Math.round(this.arms.leftEncoder.getPosition()), 12), -0.3, 0.3));
    arms.rightArm.set(
        MathUtil.clamp(this.arms.armPID.calculate(Math.round(this.arms.rightEncoder.getPosition()), -12), -0.3,
            0.3));
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
    return false;
  }
}
