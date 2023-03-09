// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arms.ArmsForward;
import frc.robot.commands.arms.ArmsReverse;
import frc.robot.commands.arms.PickupArms;
import frc.robot.commands.arms.Score2Arms;
import frc.robot.commands.arms.Score3Arms;
import frc.robot.commands.arms.ZeroArms;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.auto.DriveForwardAuto;
import frc.robot.commands.auto.DriveReverseAuto;
import frc.robot.commands.auto.ScoreGamePiece;
import frc.robot.commands.drive.DriveForward;
import frc.robot.commands.slider.Score2Slider;
import frc.robot.commands.slider.Score3Slider;
import frc.robot.commands.slider.ZeroSlider;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Slider;

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

  SendableChooser<String> autoRoute = new SendableChooser<>();
  

  public static final CommandJoystick leftJoystick = new CommandJoystick(Constants.OI.JOYSTICK1);
  public static final CommandJoystick rightJoystick = new CommandJoystick(Constants.OI.JOYSTICK2);
  public static final CommandXboxController controller = new CommandXboxController(Constants.OI.CONTROLLER);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Arms arms = new Arms();
  private final Slider slider = new Slider();
  private final Gripper gripper = new Gripper();
  private final CameraSystem cameraSystem = new CameraSystem();

  private final Timer timer = new Timer();

  private final ScoreGamePiece scoreGamePiece = new ScoreGamePiece(timer, arms, gripper);
  private final DriveReverseAuto driveReverseAuto = new DriveReverseAuto(driveTrain, arms);
  private final DriveForwardAuto driveForwardAuto = new DriveForwardAuto(driveTrain);
  private final AutoBalance autoBalance = new AutoBalance(driveTrain);
  // private final DriveReverseAuto driveReverseAuto = new DriveReverseAuto(timer, );


  private final DriveForward driveForward = new DriveForward(driveTrain);

  private final ZeroArms zeroArms = new ZeroArms(arms);
  private final ArmsForward armsForward = new ArmsForward(arms);
  private final ArmsReverse armsReverse = new ArmsReverse(arms);
  private final PickupArms pickupArms = new PickupArms(arms);
  private final Score2Arms score2Arms = new Score2Arms(arms);
  private final Score3Arms score3Arms = new Score3Arms(arms);

  private final ZeroSlider zeroSlider = new ZeroSlider(slider);
  private final Score2Slider score2Slider = new Score2Slider(slider);
  private final Score3Slider score3Slider = new Score3Slider(slider);

  private final AutonomousCommand autonomous = new AutonomousCommand(scoreGamePiece, zeroArms, driveReverseAuto, driveForwardAuto, autoBalance);


  // private final ToggleGripper openGripper = new ToggleGripper(gripper, Value.kForward);
  // private final ToggleGripper closeGripper = new ToggleGripper(gripper, Value.kReverse);

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
    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.tankDrive(leftJoystick.getY(), rightJoystick.getY()), driveTrain));
    leftJoystick.trigger().whileTrue(driveForward);
    rightJoystick.button(2).onTrue(zeroSlider.alongWith(zeroArms));
    // arms.setDefaultCommand(
    //   new RunCommand(() -> arms.move();, null)
    // );
    rightJoystick.povUp().onTrue(armsForward);
    rightJoystick.povDown().onTrue(armsReverse);
    leftJoystick.button(4).onTrue(pickupArms);
    rightJoystick.button(3).onTrue(score2Slider.alongWith(score2Arms));
    rightJoystick.button(4).onTrue(score3Slider.alongWith(score3Arms));
    // rightJoystick.trigger().toggleOnTrue(closeGripper).toggleOnFalse(openGripper);
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
