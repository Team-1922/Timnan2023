// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.ToggleBrake;
import frc.robot.commands.ToggleFlip;
import frc.robot.subsystems.ScoreMode;

import frc.robot.commands.IncrementScoreMode;
import frc.robot.commands.IncrementScoreModeDown;
import frc.robot.commands.LedAmericaAnimation;
import frc.robot.commands.AnimateStop;
import frc.robot.commands.Apriltag;
import frc.robot.commands.ControllerBuzz;
import frc.robot.commands.CurvyDrive;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FlipTankDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.XBoxTankDrive;
import frc.robot.commands.autocommands.Autos;
import frc.robot.commands.autocommands.TrajectoryDrive;
import frc.robot.commands.GatherTheCube;
import frc.robot.commands.Score;
import frc.robot.commands.SwivelDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoolLedSubsystem;
import frc.robot.subsystems.LightEmittingDiode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// tryout temp imports

import frc.robot.commands.LedAnimate;
import frc.robot.commands.LedColors;
import frc.robot.commands.LedCoolAnimation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  private final SendableChooser<CommandBase> m_autochooser = new SendableChooser<CommandBase>();
 Animation RainbowAnimation = new RainbowAnimation(1,0.5,108);
 Animation RgbFadeAnimation = new RgbFadeAnimation(1, 0.5, 108);
 Animation FireAnimation = new FireAnimation(1, 1,108 , .8, 0); 
 Animation StrobeAnimation = new com.ctre.phoenix.led.StrobeAnimation(255, 0, 0, 0, 0.1, 108) ;
 Animation ColorFlowAnimation = new ColorFlowAnimation(255, 255, 0, 0, 0.3, 108, Direction.Backward);
 Animation TwinkleAnimation = new com.ctre.phoenix.led.TwinkleAnimation(255, 0, 0, 0, 0, 108, TwinklePercent.Percent42);
 Animation SingleFadeAnimation = new com.ctre.phoenix.led.SingleFadeAnimation(255, 255, 0, 0, 0.3, 100);
 // joysticks and xboxcontrollers 
 public final static Joystick LeftJoystick = new Joystick(0);
 public final static Joystick RightJoystick = new Joystick(1);

 private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);



  private static final AHRS m_navX = new AHRS(SPI.Port.kMXP);

// Subsystems, put them here or code might not work 
  public static EndEffector m_EndEffector = new EndEffector();
  public static Arm m_Arm = new Arm();
  public static ScoreMode m_ScoreMode = new ScoreMode();
  public static DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem(m_navX);
  public static LightEmittingDiode m_LightEmittingDiode = new LightEmittingDiode();
  //arm commands
  private final GatherTheCube m_GatherCube = new GatherTheCube(m_Arm, m_EndEffector);
  private final Score m_Score = new Score(m_Arm, m_EndEffector, m_ScoreMode);
  private final IncrementScoreMode m_ScoreModeIncrement = new IncrementScoreMode(m_ScoreMode, m_LightEmittingDiode);
  private final IncrementScoreModeDown m_ScoreModeIncrementDown = new IncrementScoreModeDown(m_ScoreMode, m_LightEmittingDiode);


  

    // Auto drive commands
    //private final AutoBalance m_autoBalance = new AutoBalance(m_DriveTrainSubsystem);
    //private final AutoBalance m_autoBalance2 = new AutoBalance(m_DriveTrainSubsystem);

    //private final AutoStraight m_autoStraight = new AutoStraight(m_DriveTrainSubsystem, 3000);
    //private final  AutoStraightBack m_autoStraightBack = new AutoStraightBack(m_DriveTrainSubsystem, -2700);
    private final TrajectoryDrive m_trajectoryDriveTest = new TrajectoryDrive(m_DriveTrainSubsystem, 
    new Translation2d(.5, 0), 
    new Translation2d(1, 0), 
    new Translation2d(3,.01), 
    new Pose2d(new Translation2d(2.9, 0), Rotation2d.fromDegrees(0)), 
    false);





  // drive commands 
  private final TankDrive m_TankDrive = new TankDrive(m_DriveTrainSubsystem, LeftJoystick, RightJoystick);
  private final XBoxTankDrive m_xBoxTankDrive = new XBoxTankDrive(m_DriveTrainSubsystem, m_driverController);
  private final CurvyDrive m_curvyDrive = new CurvyDrive(m_DriveTrainSubsystem, LeftJoystick, RightJoystick);
  private final DriveStraight m_DriveStraight = new DriveStraight(m_DriveTrainSubsystem, LeftJoystick);
  private final FlipTankDrive m_flipDrive = new FlipTankDrive(m_DriveTrainSubsystem, LeftJoystick, RightJoystick);
    private final ToggleFlip m_toggleFlip = new ToggleFlip(m_DriveTrainSubsystem);
  private final ToggleBrake m_toggleBrake = new ToggleBrake(m_DriveTrainSubsystem);
  private final SwivelDrive m_swivelDrive = new SwivelDrive(m_DriveTrainSubsystem, RightJoystick);
   
  private final Apriltag m_Apriltag = new Apriltag(m_DriveTrainSubsystem);
  

  //other commands 
  private final ControllerBuzz m_buzz = new ControllerBuzz(m_driverController);
  //LED commands
private final LedAnimate m_Rainbow = new LedAnimate(m_LightEmittingDiode, RainbowAnimation,1);
private final LedAnimate m_RGBAnimation = new LedAnimate(m_LightEmittingDiode, RgbFadeAnimation, 1);
private final LedAnimate m_FireAnimation = new LedAnimate(m_LightEmittingDiode, FireAnimation, 0);
private final LedColors m_Lightoff = new LedColors(m_LightEmittingDiode,0,0,0 );
private final LedColors m_LightUpRed = new LedColors(m_LightEmittingDiode, 255,0,0);
private final static LedCoolAnimation m_CoolAnimation = new LedCoolAnimation(m_LightEmittingDiode);
private final AnimateStop m_AnimateStop = new AnimateStop(m_LightEmittingDiode);
private final LedAnimate m_StrobeAnimation = new LedAnimate(m_LightEmittingDiode, StrobeAnimation, 0);
private final LedAnimate m_ColorFlowAnimation = new LedAnimate(m_LightEmittingDiode, ColorFlowAnimation, 0);  
private final LedAnimate m_TwinkleAnimation = new LedAnimate(m_LightEmittingDiode, TwinkleAnimation, 0);
private final LedAnimate m_SingleFadeAnimation = new LedAnimate(m_LightEmittingDiode, SingleFadeAnimation, 0);
private final LedAmericaAnimation m_AmericaAnimation = new LedAmericaAnimation(m_LightEmittingDiode);


public static CoolLedSubsystem m_CoolLedSubsystem = new CoolLedSubsystem(m_LightEmittingDiode, m_CoolAnimation);


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    m_DriveTrainSubsystem.setDefaultCommand(m_curvyDrive);

    // Configure the trigger bindings
    configureBindings();




 initNetworkTable();
   
   autochooser();
   



    SmartDashboard.putNumber("Deadzone", .125);

    SmartDashboard.putNumber("Balance P", .015);
    SmartDashboard.putNumber("Balance D", .01);
  }

  private void initNetworkTable(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Ozram");
NetworkTableEntry visionPGain = table.getEntry("visionPGain");
visionPGain.setNumber(0.02);

NetworkTableEntry visionDGain = table.getEntry("visionDGain");
visionDGain.setNumber(0.002);



  }
  public void autochooser(){

 m_autochooser.setDefaultOption("(LEFT) Trajectory 2-Cube", Autos.m_trajectoryAutoLEFT);
 m_autochooser.addOption("(RIGHT) Trajectory 2-Cube", Autos.m_trajectoryAutoRIGHT);
 m_autochooser.addOption("(CENTER) Balance w/o Mobility", Autos.m_autoStraightToBalance);

 m_autochooser.addOption("Back Up", Autos.m_autoBackup);
 m_autochooser.addOption("Balance w/ Mobility", Autos.m_autoStraightGroup);
 m_autochooser.addOption("Test run", Autos.m_test);



 SmartDashboard.putData("autochooser",m_autochooser); }
 

 

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



    //                       *** OPERATOR CONTROLS ***

    // X Button
    m_driverController.button(3).onTrue(m_ScoreModeIncrement); 
    // Y Button
    m_driverController.button(4).onTrue(m_ScoreModeIncrementDown);
    // Left Bumper - 5
    m_driverController.button(5).whileTrue(m_GatherCube); 
    // RightBumper - 6
    m_driverController.button(6).whileTrue(m_Score); 




    //                       *** DRIVER CONTROLS ***
    
    // Left Trigger
    new JoystickButton(LeftJoystick, 1).whileTrue(m_DriveStraight);
    // Left 3
    new JoystickButton(LeftJoystick, 3).onTrue(m_toggleFlip);
    // Left 4
    new JoystickButton(LeftJoystick, 4).onTrue(m_Apriltag);


    // Right Trigger
    new JoystickButton(RightJoystick, 1).whileTrue(m_swivelDrive);
    // Right Side
    new JoystickButton(RightJoystick, 2).onTrue(m_toggleBrake);
    // Right 3
    new JoystickButton(RightJoystick, 3).whileTrue(m_buzz);


    // new JoystickButton(RightJoystick, 5).onTrue(m_trajectoryDriveTest);





      
    //                       *** LED CONTROLS ***

      //LED buttons
      
      
      new JoystickButton(RightJoystick, 12)
      .onTrue(m_Rainbow);
      new JoystickButton(RightJoystick, 11)
      .onTrue(m_LightUpRed);
      new JoystickButton(RightJoystick, 10)
      .onTrue(m_Lightoff);
      new JoystickButton(RightJoystick, 9)
      .onTrue(m_AnimateStop);
      new JoystickButton(RightJoystick, 8)
      .onTrue(m_RGBAnimation);
      new JoystickButton(RightJoystick, 7)
      .onTrue(m_CoolAnimation);

       new JoystickButton(LeftJoystick, 12) 
       .onTrue(m_StrobeAnimation);
       new JoystickButton(LeftJoystick, 11)
      .onTrue(m_ColorFlowAnimation);
       new JoystickButton(LeftJoystick, 10)
       .onTrue(m_TwinkleAnimation);
       new JoystickButton(LeftJoystick, 9)
       .onTrue(m_SingleFadeAnimation);
       new JoystickButton(LeftJoystick, 8)
       .onTrue(m_AmericaAnimation);
       new JoystickButton(LeftJoystick, 7)
       .onTrue(m_FireAnimation); 
       
       
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
    // An example command will be run in autonomous
    //return Autos.m_autoBackup;
    //return Autos.m_autoStraightHalf;
 //   return Autos.m_autoStraightGroup;
}

}
