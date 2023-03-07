// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Colors;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Lights extends SubsystemBase {

  private final PWM lights;
  private String teamColor = "RED";

  /** Creates a new Lights. */
  public Lights() {
    lights = new PWM(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("teamColor", teamColor);
    SmartDashboard.putNumber("pwmLights", lights.getSpeed());

    // if(RobotContainer.leftJoystick.getTrigger() ||
    // RobotContainer.rightJoystick.getTrigger() ||
    // RobotContainer.rightJoystick.getRawButton(2)) {
    // RobotContainer.color = Colors.GREEN;
    // } else if(RobotContainer.rightJoystick.getRawButton(3)){
    // RobotContainer.color = Colors.PURPLE;
    // } else if(RobotContainer.leftJoystick.getRawButton(4)) {
    // RobotContainer.color = Colors.ORANGE;
    // } else {
    // if(RobotContainer.FMS.getEntry("IsRedAlliance").getBoolean(false)) {
    // RobotContainer.color = Colors.RED;
    // } else {
    // RobotContainer.color = Colors.BLUE;
    // }
    // }

    if (RobotContainer.FMS.getEntry("IsRedAlliance").getBoolean(false)) {
      teamColor = "RED";
    } else {
      teamColor = "BLUE";
    }

    if (RobotContainer.controller.getAButtonPressed()) {
      if (RobotContainer.color != Colors.YELLOW) {
        RobotContainer.color = Colors.YELLOW;
      } else {
        RobotContainer.color = Colors.valueOf(teamColor);
      }
    } else if (RobotContainer.controller.getBButtonPressed()) {
      if (RobotContainer.color != Colors.PURPLE) {
        RobotContainer.color = Colors.PURPLE;
      } else {
        RobotContainer.color = Colors.valueOf(teamColor);
      }
    } else {
      if (RobotContainer.color != Colors.YELLOW && RobotContainer.color != Colors.PURPLE) {
        RobotContainer.color = Colors.valueOf(teamColor);
      }
    }

    switch (RobotContainer.color) {
      case BLUE:
        lights.setSpeed(Constants.ColorConstants.BLUE);
        break;
      case RED:
        lights.setSpeed(Constants.ColorConstants.RED);
        break;
      case YELLOW:
        lights.setSpeed(0.65);
        break;
      case PURPLE:
        lights.setSpeed(0.89);
        break;
      default:
        lights.setSpeed(0.99);
        break;
    }
    // This method will be called once per scheduler run
  }
}
