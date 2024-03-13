// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.libs.Debug;
import frc.libs.LimelightLib;
import frc.libs.AprilTag;

/** An example command that uses an example subsystem. */
public class AutoSwerve extends Command {

  private final double MAX_DRIVE = 0.4;         // Simple speed limit so we don't drive too fast
  private double leftCommand = 0;
  private double rightCommand = 0;
  private Boolean targetCaught = false;
   
  //  Aim and move to target adjust to smooth motion.
  double kPSteer = 0.009;//.006;
  double kPDistance = 0.1;//.75;
  double minCommand = 0.05;

  /*private final CANSparkMax m_leftFrontMotor = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax m_leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);

  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);
  
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFrontMotor::set, m_rightFrontMotor::set);
 
  private final XboxController m_driverController = new XboxController(0);*/

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  
  private int m_rainbowFirstPixelHue;

  private AutoStates robotState;

  enum AutoStates {
    Stationary,
    Backup,
    TurnToTarget,
    MoveToTarget,
    getCloser;
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Swerve s_Swerve;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoSwerve(Swerve s) {
    //m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    Debug.log("Construct AutoSwerve");
    s_Swerve = s;

    robotState = AutoStates.Stationary;
    //-m_leftBackMotor.follow(m_leftFrontMotor);
    //-m_rightBackMotor.follow(m_rightFrontMotor);

    //-m_rightFrontMotor.setInverted(true);
    //-m_leftFrontMotor.setInverted(false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState = AutoStates.Backup;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    Debug.log("auto...llkhkjgkjgdsfuijfds");
    //Debug.log(LimelightLib.getJSON("limelight-blargle"));
    AprilTag[] visibleTags = LimelightLib.GetVisibleTags(LimelightLib.getJSON("limelight-blargle"));
    //AprilTag newTag = visibleTags[0];

    SmartDashboard.putBoolean("targetCaught", targetCaught);

    //if(visibleTags.length > 0 && !targetCaught)
    //chaseTarget(visibleTags[0]);
    leftCommand = 0;
    rightCommand = 0;
      
    switch(robotState) {
      case Stationary:
      //  code goes here
        break;
      case Backup:
        backup(7);
        break;
      case TurnToTarget:
        turnToTarget(7.0);
        break;
      case MoveToTarget:
        moveToTarget(7.0);
        break;
      //case getCloser: 
        //getCloser(7);
        //break; 
        default:
        break;
    }
    //  Finally move the robot after all the math is done.
    leftCommand = ClampValue(leftCommand);
    rightCommand = ClampValue(rightCommand);

    SmartDashboard.putNumber("leftCommand", leftCommand);
    SmartDashboard.putNumber("rightCommand", rightCommand);

    //m_robotDrive.tankDrive(leftCommand, rightCommand);


    //  translate is the value for distance error
    // rotate is the value for the angle error.

    //  send .drive the values for forward into Translation.x
    //  send .drive the values for rotation.
    s_Swerve.drive(new Translation2d(0.0, 0.0), 26.0, !false, true);
  
  
  }

  // Caedled once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public void turnToTarget(double tagID) {
    AprilTag[] visibleTags = LimelightLib.GetVisibleTags(LimelightLib.getJSON("limelight-blargle"));
    AprilTag vTag = null;
    for(int i = 0; i < visibleTags.length; i++)
    {
      if(visibleTags[i].getfID() == tagID) {  //visibleTags[i].getfID())
        Debug.log("****************************************************Found 7");
        vTag = visibleTags[i];
      }
    }
    if(vTag != null) {
      SmartDashboard.putNumber("FollowingTag", vTag.getfID());
      // Since Valid target exists, get tx and ta from Network Table
      double tx = vTag.gettx();//limelightA.getTx();
      //  Handle the Steering values;
      double headingError = -tx;
      double steeringAdjust = 0.0f;
      //  Use proportional steering to set steering adjust amounts.
      //  steering adjust will speed or slow wheels to steer.
      if(Math.abs(headingError) > 1.0) {
        if(headingError < 0)
          steeringAdjust = kPSteer * headingError - minCommand;
        else
          steeringAdjust = kPSteer * headingError + minCommand;
      }
      if(Math.abs(vTag.gettx()) < 10) {
        //for(int i = 0; i < 10; i++)
          //m_ledBuffer.setRGB(i, 0, 0, 255);
          //m_led.setData(m_ledBuffer);
      }
      else {
        //for(int i = 0; i < 10; i++)
          //m_ledBuffer.setRGB(i, 0, 255, 255);
          //m_led.setData(m_ledBuffer);
      }
      if(Math.abs(steeringAdjust) > 0.2) {
        leftCommand -= steeringAdjust;
        rightCommand += steeringAdjust;
      }
      else
        robotState = AutoStates.MoveToTarget;
    }
  }

  public void moveToTarget(double tagID) {
    AprilTag[] visibleTags = LimelightLib.GetVisibleTags(LimelightLib.getJSON("limelight-blargle"));
    AprilTag vTag = null;

    for(int i = 0; i < visibleTags.length; i++) {
      if(visibleTags[i].getfID() == tagID) {  //visibleTags[i].getfID())
        Debug.log("****************************************************Found 7");
        vTag = visibleTags[i];
      }
    }

    if(vTag != null) {
      double tx = vTag.gettx();
      double headingError = -tx;
      double steeringAdjust = 0.0f;

      SmartDashboard.putNumber("FollowingTag", vTag.getfID());
      //  Process Forward Motion here
      //  may need to reverse the following difference.
        
      double calculatedDistance = LimelightLib.GetDistance(vTag);
      SmartDashboard.putNumber("Distance", calculatedDistance);
      SmartDashboard.putNumber("tX", vTag.gettx());
      SmartDashboard.putNumber("vTag", vTag.getfID());
      double distanceError = -(65 - LimelightLib.GetDistance(vTag)); 
      if(distanceError < 0){
        distanceError = 0;
        targetCaught = true;
        robotState = AutoStates.Stationary;
      } 
        
      if(Math.abs(headingError) > 0) {
        if(headingError < 0.5)
          steeringAdjust = kPSteer * headingError - minCommand;
        else
          steeringAdjust = kPSteer * headingError + minCommand;
      }

        if(Math.abs(steeringAdjust) > 0) {
          leftCommand -= steeringAdjust;
          rightCommand += steeringAdjust;
      }

      double distanceAdjust = kPDistance * distanceError;
      leftCommand += distanceAdjust;
      rightCommand += distanceAdjust;
    } 
     
  } 

  public double ClampValue(double v) {
    if(v > 0) return Math.min(v, MAX_DRIVE);
    if(v < 0) return Math.max(v, -MAX_DRIVE);
    return 0;
  }


  //post to smart dashboard periodically
  Boolean faceTarget = false;
  int ledCounter = 0;
  AprilTag targetTag;
  public void backup(int tagID) {
    //if(ledCounter++ % 1 * 50 == 0)
    //for(int i = 0; i < 10; i++)
    //m_ledBuffer.setRGB(i, 0xff, 0x00, 0x00);
    //m_led.setData(m_ledBuffer);

    AprilTag[] visibleTags = LimelightLib.GetVisibleTags(LimelightLib.getJSON("limelight-blargle"));
    
    leftCommand = -0.4;
    rightCommand = -0.4;

    if (visibleTags.length > 0) {  // if this happens, we don't see a tag!
      for(int i = 0; i < visibleTags.length; i++) {
        if(visibleTags[i].getfID() == tagID) {
          leftCommand = 0.0;
          rightCommand = 0.0;
          robotState = AutoStates.TurnToTarget;
          break;
        }
      } 
    }
    }

}
