// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.config.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PneumaticsConstants {
    public static final int PCM_CAN_ID = 30;
    //public static final int LIFTER_SOLENOID_CHANNEL = 0;
    //public static final int RELEASE_SOLENOID_CHANNEL = 1;

    //public static final Boolean EXTENDED = true;
    //public static final Boolean RETRACTED = false; 

  }

  public static class NeoPixelConstants {
    public static final int NEOPIXEL_PWM_PORT = 0;
  }

  public static class LimelightConstants {
    public static final String LIMELIGHT_A_HOSTNAME = "limelight-Blargle";
    public static final double CAMERA_HEIGHT = 14.5;
    public static final double CAMERA_ANGLE = 0;
  }

 /* public static class NoteHandlerConstants {
    public static final int FORE_INTAKE_MOTOR_CANID = 21;
    public static final int AFT_INTAKE_MOTOR_CANID = 22;
    public static final double INTAKE_MOTOR_SPEED = 1.0;

    public static final int LEFT_LAUNCHER_MOTOR_CANID = 23;
    public static final double LEFT_LAUNCHER_MOTOR_SPEED = 1.0;
    public static final int RIGHT_LAUNCHER_MOTOR_CANID = 24;
    public static final double RIGHT_LAUNCHER_MOTOR_SPEED = 1.0;

    public static final int NOTE_LOADED_LIMIT_DIO = 0;
    
  }
  */

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 15;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(14.5);
    public static final double wheelBase = Units.inchesToMeters(24.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0/7.0);//20.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 40;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 40;//20; // meters per second: higher means slower
    public static final double maxAngularVelocity =35.0;//26;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;//false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;//1;
      public static final int angleMotorID = 2;//2;
      public static final int canCoderID = 3;//20;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(354.7);//131.8);//16.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;//3;
      public static final int angleMotorID = 5;//4;
      public static final int canCoderID = 6;//21;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(105.6);//99.7);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;//7;
      public static final int angleMotorID = 8;//8;
      public static final int canCoderID = 9;//22;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.8);//351.0);//39.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 10;//5;
      public static final int angleMotorID = 11;//6;
      public static final int canCoderID = 12;//23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.0);//262.88);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
