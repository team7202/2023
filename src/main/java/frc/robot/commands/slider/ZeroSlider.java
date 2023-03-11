// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.slider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slider;

public class ZeroSlider extends CommandBase {

  private final Slider slider;

  private boolean finished = false;

  /** Creates a new ZeroSlider. */
  public ZeroSlider(Slider slider) {
    addRequirements(slider);
    this.slider = slider;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(slider.leftEncoder.getPosition()) <= 1) {
      finished = true;
    } else if (Math.abs(slider.rightEncoder.getPosition()) <= 1) {
      finished = true;
    } else {
      slider.leftSlider.set(
          MathUtil.clamp(slider.sliderPID.calculate(Math.round(slider.leftEncoder.getPosition()), 1), -0.15,
              0.05));
      slider.rightSlider.set(
          MathUtil.clamp(slider.sliderPID.calculate(Math.round(slider.rightEncoder.getPosition()), -1), -0.15,
              0.05));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    slider.leftSlider.set(0);
    slider.rightSlider.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
