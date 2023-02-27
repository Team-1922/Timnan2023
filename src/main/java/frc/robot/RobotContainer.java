// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TestArm;
import frc.robot.subsystems.ScoreMode;
import frc.robot.Constants;
import frc.robot.commands.IncrementScoreMode;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TankDrive;
import frc.robot.commands.GatherTheCube;
import frc.robot.commands.Score;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightEmitingDiode;
import edu.wpi.first.cscore.raw.RawSink;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// tryout temp imports

import frc.robot.commands.LedAnimate;
import frc.robot.commands.LedColors;
import frc.robot.commands.ScoreLedCommand;
import frc.robot.commands.CollectLedCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 Animation RainbowAnimation = new RainbowAnimation(
 );
  // joysticks and xboxcontrollers 
 public final static Joystick LeftJoystick = new Joystick(0);
 public final static Joystick RightJoystick = new Joystick(1);


 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);



  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

// Subsystems, put them here or code might not work 
  public static EndEffector m_EndEffector = new EndEffector();
  public static Arm m_Arm = new Arm();
  public static ScoreMode m_ScoreMode = new ScoreMode();

  //private final DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem(m_navX);
private final DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem(m_navX);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private LightEmitingDiode m_LightEmitingDiode = new LightEmitingDiode();
  //arm commands
  private final GatherTheCube m_GatherCube = new GatherTheCube(m_Arm, m_EndEffector);
  private final Score m_Score = new Score(m_Arm, m_EndEffector, m_ScoreMode, m_LightEmitingDiode);
  private final IncrementScoreMode m_ScoreModeIncrement = new IncrementScoreMode(m_ScoreMode, m_LightEmitingDiode);
  private final TestArm m_TestArm = new TestArm(m_Arm);


  

    // Auto drive commands
    //private final AutoBalance m_autoBalance = new AutoBalance(m_DriveTrainSubsystem);


    



  // drive commands 
  //private final TankDrive m_TankDrive = new TankDrive(m_DriveTrainSubsystem, LeftJoystick, RightJoystick);
  //private final DriveStraight m_DriveStraight = new DriveStraight();
  

  //other commands 
private final LedAnimate m_Rainbow = new LedAnimate(m_LightEmitingDiode, RainbowAnimation);
private final LedColors m_Lightoff = new LedColors(m_LightEmitingDiode,0,0,0 );
private final LedColors m_LightUpRed = new LedColors(m_LightEmitingDiode, 255,0,0);
private final LedAnimate m_stopAnimate = new LedAnimate(m_LightEmitingDiode, null);

  // tryouts temp commands
  private final LightEmitingDiode m_ledSubsystem = new LightEmitingDiode();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    //m_DriveTrainSubsystem.setDefaultCommand(m_TankDrive);
    // Configure the trigger bindings
    configureBindings();
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.rightBumper().onTrue(m_ScoreModeIncrement);
    //m_driverController.leftTrigger().whileTrue(m_GatherCube);
    //m_driverController.rightTrigger().whileTrue(m_Score);
    new JoystickButton(RightJoystick, 2).whileTrue(m_GatherCube); //Need to find the button number for the trigger
    new JoystickButton(RightJoystick, 4).whileTrue(m_Score);
    new JoystickButton(RightJoystick, 3).onTrue(m_ScoreModeIncrement);
    new JoystickButton(LeftJoystick, 4).whileTrue(m_TestArm);
    //new JoystickButton(LeftJoystick, 1)
      //.whileTrue(m_DriveStraight);
    //new JoystickButton(LeftJoystick, 5)
      //.whileTrue(m_TankDrive);
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
      new JoystickButton(RightJoystick, 12)
      .onTrue(m_Rainbow);
      new JoystickButton(RightJoystick, 11)
      .onTrue(m_LightUpRed);
      new JoystickButton(RightJoystick, 10)
      .onTrue(m_Lightoff);
      new JoystickButton(RightJoystick, 9)
      .onTrue(m_stopAnimate);
/* 
    new JoystickButton(LeftJoystick, 5)
      .whileTrue(m_TankDrive);*/



  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(null);
  }
}
