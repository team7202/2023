// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.catcher.ToggleCatcher;
import frc.robot.commands.drive.DriveForward;
import frc.robot.commands.gripper.ToggleGripper;
import frc.robot.commands.slider.Score2Slider;
import frc.robot.commands.slider.Score3Slider;
import frc.robot.commands.slider.ZeroSlider;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.Catcher;
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

  private final SendableChooser<String> autoRoute = new SendableChooser<>();

  private final Timer timer = new Timer();

  private final Compressor compressor = new Compressor(Constants.PCM.COMPRESSOR, PneumaticsModuleType.CTREPCM);

  public static final CommandJoystick leftJoystick = new CommandJoystick(Constants.OI.JOYSTICK1);
  public static final CommandJoystick rightJoystick = new CommandJoystick(Constants.OI.JOYSTICK2);
  public static final CommandXboxController controller = new CommandXboxController(Constants.OI.CONTROLLER);

  // Subsystems
  private DriveTrain driveTrain;
  private Arms arms;
  private Slider slider;
  private Gripper gripper;
  private Catcher catcher;
  private CameraSystem cameraSystem;

  // Autonomous Commands
  private ScoreGamePiece scoreGamePiece;
  private DriveReverseAuto driveReverseAuto;
  private DriveForwardAuto driveForwardAuto;
  private AutoBalance autoBalance;

  // Teleop Commands
  // DriveTrain Commands
  private DriveForward driveForward;
  // Arm Commands
  private ZeroArms zeroArms;
  private ArmsForward armsForward;
  private ArmsReverse armsReverse;
  private PickupArms pickupArms;
  private Score2Arms score2Arms;
  private Score3Arms score3Arms;
  // Slider Commands
  private ZeroSlider zeroSlider;
  private Score2Slider score2Slider;
  private Score3Slider score3Slider;
  // Gripper Commands
  private ToggleGripper toggleGripper;
  private ToggleCatcher toggleCatcher;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoRoute.setDefaultOption("CENTER", "center");
    SmartDashboard.putData("Auto Routes", autoRoute);
    compressor.enableDigital();
    configureSubsystems();
    configureAutoCommands();
    configureCommands();
    configureButtonBindings();
  }

  /**
   * Use this method to define your subsystems.
   */
  private void configureSubsystems() {
    this.driveTrain = new DriveTrain();
    this.arms = new Arms();
    this.slider = new Slider();
    this.gripper = new Gripper();
    this.catcher = new Catcher();
    this.cameraSystem = new CameraSystem();
  }

  /**
   * Use this method to define your autonomous commands.
   */
  private void configureAutoCommands() {
    this.scoreGamePiece = new ScoreGamePiece(timer, arms, gripper);
    this.driveReverseAuto = new DriveReverseAuto(driveTrain, arms);
    this.driveForwardAuto = new DriveForwardAuto(driveTrain);
    this.autoBalance = new AutoBalance(driveTrain);
  }

  /**
   * Use this method to define your commands.
   */
  private void configureCommands() {
    this.driveForward = new DriveForward(driveTrain);

    this.zeroArms = new ZeroArms(arms);
    this.armsForward = new ArmsForward(arms);
    this.armsReverse = new ArmsReverse(arms);
    this.pickupArms = new PickupArms(arms);
    this.score2Arms = new Score2Arms(arms);
    this.score3Arms = new Score3Arms(arms);

    this.zeroSlider = new ZeroSlider(slider);
    this.score2Slider = new Score2Slider(slider);
    this.score3Slider = new Score3Slider(slider);

    this.toggleGripper = new ToggleGripper(gripper);
    this.toggleCatcher = new ToggleCatcher(catcher);
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
    
    rightJoystick.button(2).onTrue(zeroSlider.alongWith(zeroArms));
    rightJoystick.povUp().whileTrue(armsForward);
    rightJoystick.povDown().whileTrue(armsReverse);
    
    leftJoystick.button(4).onTrue(pickupArms);
    
    rightJoystick.button(3).onTrue(score2Slider.alongWith(score2Arms));
    rightJoystick.button(4).onTrue(score3Slider.alongWith(score3Arms));
    rightJoystick.trigger().toggleOnTrue(toggleGripper).toggleOnFalse(toggleGripper);
    rightJoystick.button(16).toggleOnTrue(toggleCatcher).toggleOnFalse(toggleCatcher);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonomousCommand(autoRoute, scoreGamePiece, zeroArms, driveReverseAuto,
        driveForwardAuto, autoBalance);
  }
}
