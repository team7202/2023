// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static class DriveTrain {
        /* PID VALUES FOR ROTATING TO ANGLE */
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0.0075;
        /* MOTOR ID NUMBERS */
        public static final int FRONT_LEFT_ID = 1;
        public static final int REAR_LEFT_ID = 2;
        public static final int FRONT_RIGHT_ID = 3;
        public static final int REAR_RIGHT_ID = 4;
        public static final int LEFT = 13;
        public static final int RIGHT = 21;
        public static final int LEFT_SLIDER = 31;
        public static final int RIGHT_SLIDER = 32;
        /* MOTOR TYPES */
        public static final MotorType FRONT_LEFT_TYPE = MotorType.kBrushless;
        public static final MotorType REAR_LEFT_TYPE = MotorType.kBrushless;
        public static final MotorType FRONT_RIGHT_TYPE = MotorType.kBrushless;
        public static final MotorType REAR_RIGHT_TYPE = MotorType.kBrushless;
        /* MOTOR INVERSION */
        public static final boolean LEFT_GROUP_INVERTED = true;
        public static final boolean RIGHT_GROUP_INVERTED = false;
        /* ROBORIO PORTS */
        public static final SPI.Port GYRO_PORT = SPI.Port.kMXP;
        /* MATH */
        public static final double FEET_DIVISOR = 0.149;
    }

    public static class Gripper {
        public static final int COMPRESSOR_ID = 0;
    }

    public static class ColorConstants {
        public static final double RED = -0.31;
        public static final double BLUE = -0.29;
      }

      public static class Limelight {
        public static final float ROTATE_P = -0.1125f;
        public static final float ROTATE_I = 0f;
        public static final float ROTATE_D = -0.0075f;
        
        public static final float DRIVE_P = -0.15f;
        public static final float DRIVE_I = -0f;
        public static final float DRIVE_D = 0.01f;
      }
}
