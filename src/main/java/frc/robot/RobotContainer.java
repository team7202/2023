// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Slider;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController controller = new XboxController(2);

  public static Colors color = Colors.RED;
  public static NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
  public static String armMode = "off";
  public static String sliderMode = "off";

  // The robot's subsystems and commands are defined here...
  private final CameraSystem cameraSystem = new CameraSystem();
  private final DriveTrain driveTrain = new DriveTrain(leftJoystick, rightJoystick);
  private final Limelight limelight = new Limelight(driveTrain);
  private final Slider slider = new Slider(leftJoystick, rightJoystick);
  private final Arms arms = new Arms(leftJoystick, rightJoystick, slider);
  private final Gripper gripper = new Gripper(leftJoystick, rightJoystick);
  private final Lights lights = new Lights();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutonomousCommand autonomousCommand = new AutonomousCommand(driveTrain, gripper);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return autonomousCommand;
  }
}
