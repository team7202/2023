// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.slider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slider;

public class Score3Slider extends CommandBase {

  private final Slider slider;

  /** Creates a new Score3Slider. */
  public Score3Slider(Slider slider) {
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
    slider.leftSlider.set(MathUtil
        .clamp(this.slider.sliderPID.calculate(Math.round(this.slider.leftSlider.getEncoder().getPosition()), -30),
            -.15, .15));
    slider.rightSlider.set(MathUtil
        .clamp(this.slider.sliderPID.calculate(Math.round(this.slider.rightSlider.getEncoder().getPosition()), 30),
            -.15, .15));
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
