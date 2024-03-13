package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  //-private final Pigeon2 gyro;
  private final AHRS ahrs;

  //PowerDistribution powerDistributionModule; 

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
  //-  gyro = new Pigeon2(Constants.Swerve.pigeonID);
      ahrs = new AHRS(SPI.Port.kMXP);
    
  //-  gyro.configFactoryDefault();
    zeroGyro();

    //powerDistributionModule = new PowerDistribution(0, ModuleType.kCTRE);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants, 0),
          new SwerveModule(1, Constants.Swerve.Mod1.constants, 2),
          new SwerveModule(2, Constants.Swerve.Mod2.constants, 12),
          new SwerveModule(3, Constants.Swerve.Mod3.constants, 14)
        };

    // **********
    SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++)
    {
        swervePositions[i] = new SwerveModulePosition(0, mSwerveMods[i].getState().angle);
    }
    swerveOdometry = new SwerveDriveOdometry( Constants.Swerve.swerveKinematics, getYaw(), swervePositions);

    field = new Field2d();
    SmartDashboard.putData("Field", field);


    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    // ***********
   /* */ SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++)
    {
        swervePositions[i] = new SwerveModulePosition(0, mSwerveMods[i].getState().angle);
    }
    
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), /*swervePositions,*/ pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods)
      positions[mod.moduleNumber] = mod.getPosition();

    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds speeds)
  {
    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
    setModuleStates(states);
  }

  public void zeroGyro() {
   //- gyro.setYaw(0);
    ahrs.reset();
  }

  public Rotation2d getYaw() { //return new Rotation2d(); }
     return (Constants.Swerve.invertGyro)
        //? Rotation2d.fromDegrees(360 - gyro.getYaw())
        ? Rotation2d.fromDegrees(360 - ahrs.getYaw())
        //: Rotation2d.fromDegrees(gyro.getYaw());
       : Rotation2d.fromDegrees(ahrs.getYaw());
  }

  

  @Override
  public void periodic() {
    SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++)
    {
        swervePositions[i] = new SwerveModulePosition(0, mSwerveMods[i].getState().angle);
    }
    swerveOdometry.update(getYaw(), swervePositions);
    field.setRobotPose(getPose());

    //--SmartDashboard.putNumber("Yaw ",gyro.getYaw());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      /*
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Current" , mod.getCurrent(powerDistributionModule));
        SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Max Current" , mod.getMaxCurrent());*/
    }
  }
}