// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Slider;

public class Autonomous extends CommandBase {

  private final Arms arms;
  private final DriveTrain driveTrain;
  private final Gripper gripper;
  private final Slider slider;
  private final SendableChooser<String> autoRoute;

  private Timer timer;

  /** Creates a new Autonomous. */

  public Autonomous(Arms arms, DriveTrain driveTrain, Gripper gripper, Slider slider,
      SendableChooser<String> autoRoute) {
    addRequirements(arms, driveTrain, gripper, slider);
    this.arms = arms;
    this.driveTrain = driveTrain;
    this.gripper = gripper;
    this.slider = slider;
    this.autoRoute = autoRoute;
    this.autoRoute.setDefaultOption("CENTER", "center");
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.autoRoute.addOption("CENTER", "center");
    timer.reset();
    driveTrain.frontLeftMax.getEncoder().setPosition(0);
    driveTrain.frontRightMax.getEncoder().setPosition(0);
    driveTrain.leftGroup.set(0);
    driveTrain.rightGroup.set(0);
    timer.start();

    if (autoRoute.getSelected().equals("center")) {
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
