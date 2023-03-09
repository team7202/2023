// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {

  private final DriveTrain driveTrain;

  /** Creates a new AutoBalance. */
  public AutoBalance(DriveTrain driveTrain) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.gyro.getPitch() < -4.5) {
      driveTrain.tankDrive(-0.26, -0.26);
    } else if (driveTrain.gyro.getPitch() > 7.5) {
      driveTrain.tankDrive(0.26, 0.26);
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
