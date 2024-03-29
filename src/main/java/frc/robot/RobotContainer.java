// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LifterSubsystem;
//import frc.robot.subsystems.ReleaseSubsystem;

import frc.robot.subsystems.NeoPixelSubsystem;
//import frc.robot.subsystems.NoteIntakeSubsystem;
//import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
/* Controllers */
  private final Joystick driver = new Joystick(0);
  //private final Joystick armer = new Joystick(1);

  private final SendableChooser<Command> autoChooser;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
     new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Swerve s_Swerve = new Swerve();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
    configureButtonBindings();

    //  Configure Autochoose
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));




    //).whileTrue(m_lifterSubsystem.toggleLifter());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //  Might need to adjust to match new controller/button system...
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //m_driverController.x().onTrue(m_lifterSubsystem.toggleLifter());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new exampleAuto(s_Swerve);
    //return AutoSwerve(s_Swerve, () -> robotCentric.getAsBoolean());
    // **  this is the state limelight follower..  -->>>>>   return Autos.competitionAuto(new AutoSwerve(s_Swerve));
    // **return new PathPlannerAuto("Example Auto");
    return autoChooser.getSelected();
  }

public void setTeleopSwerve()
{
   s_Swerve.setDefaultCommand(
         new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));
}

public void ClearTeleopSwerve()
{
  s_Swerve.removeDefaultCommand();
}

  
}
