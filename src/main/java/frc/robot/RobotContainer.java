// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arms.ZeroArms;
import frc.robot.commands.drive.DriveForward;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final CommandJoystick leftJoystick = new CommandJoystick(Constants.OI.JOYSTICK1);
  public static final CommandJoystick rightJoystick = new CommandJoystick(Constants.OI.JOYSTICK2);
  public static final CommandXboxController controller = new CommandXboxController(Constants.OI.CONTROLLER);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Arms arms = new Arms();
  
  private final DriveForward driveForward = new DriveForward(driveTrain);
  private final ZeroArms zeroArms = new ZeroArms(arms);
  // The robot's subsystems and commands are defined here...
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  // private final AutonomousCommand autonomousCommand = new
  // AutonomousCommand(driveTrain);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.tankDrive(leftJoystick.getY(), rightJoystick.getY()), driveTrain));
    leftJoystick.trigger().whileTrue(driveForward);
    // zeroArms.andThen
    rightJoystick.button(2).onTrue(zeroArms);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }
}
