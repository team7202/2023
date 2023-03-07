// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSystem extends SubsystemBase {

  Thread m_visionThread;

  private UsbCamera camera;
  private UsbCamera camera2;
  
  // private final UsbCamera camera;
  /** Creates a new CameraSystem. */
  public CameraSystem() {
    this.camera = CameraServer.startAutomaticCapture();
    this.camera2 = CameraServer.startAutomaticCapture();
    this.camera.setResolution(320, 240);
    this.camera2.setResolution(320, 240);
    // this.camera.setResolution(720, 480);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public UsbCamera getCamera() {
      return camera;
  }

  public UsbCamera getCamera2() {
      return camera2;
  }
}
