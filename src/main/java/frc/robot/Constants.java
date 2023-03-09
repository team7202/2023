// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OI {
    public static final int JOYSTICK1 = 0;
    public static final int JOYSTICK2 = 1;
    public static final int CONTROLLER = 2;
  }

  public static final class CAN {
    public static final int REAR_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int REAR_RIGHT_DRIVE = 3;

    public static final int LEFT_ARM = 13;
    public static final int RIGHT_ARM = 21;

    public static final int LEFT_SLIDER = 31;
    public static final int RIGHT_SLIDER = 32;
  }

  public static final class PID {
    public static final double ARM_P = 0.03;
    public static final double ARM_I = 0.02;
    public static final double ARM_D = 0;

    public static final double SLIDER_P = 0.05;
    public static final double SLIDER_I = 0;
    public static final double SLIDER_D = 0.075;

    public static final double FEET_P = 0.5;
    public static final double FEET_I = 0.25;
    public static final double FEET_D = 0.01;

    public static final double ROTATE_P = 0.05;
    public static final double ROTATE_I = 0;
    public static final double ROTATE_D = 0;
  }

  public static final class DI {
    public static final int LEFT_LOWER_SLIDER_SW = 0;
    public static final int LEFT_UPPER_SLIDER_SW = 1;
    public static final int RIGHT_LOWER_SLIDER_SW = 2;
    public static final int RIGHT_UPPER_SLIDER_SW = 3;
  }

  public static final class PCM {
    public static final int COMPRESSOR = 0;
    public static final int[] SOLENOID1 = { 0, 1 };
  }

  public static final class MATH {
    public static final double TICKS_FEET_DIVISOR = 0.149;
  }
}
