// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arms.ZeroArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {

  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(ScoreGamePiece scoreGamePiece, ZeroArms zeroArms, DriveReverseAuto driveReverseAuto, DriveForwardAuto driveForwardAuto, AutoBalance autoBalance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(scoreGamePiece, driveReverseAuto, driveForwardAuto, autoBalance);
  }
}
