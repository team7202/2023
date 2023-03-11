// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Gripper;

public class ScoreGamePiece extends CommandBase {
  
  private final Gripper gripper;
  private final Arms arms;
  private final Timer timer;

  private boolean finished = false;

  /** Creates a new DropGamePiece. */
  public ScoreGamePiece(Timer timer, Arms arms, Gripper gripper) {
    addRequirements(gripper, arms);
    this.gripper = gripper;
    this.arms = arms;
    timer.reset();
    timer.start();
    this.timer = timer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.scoreHigh();
    if(timer.get() < 2) {
      gripper.toggleValue();
    } else {
      gripper.toggleValue();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
