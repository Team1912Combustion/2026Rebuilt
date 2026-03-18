// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.CandleCommand;
import frc.robot.commands.ClimbAlign;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DuckHood;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeArmToggle;
import frc.robot.commands.MoveHood;
import frc.robot.commands.MoveTurret;
import frc.robot.commands.PointAtThing;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunReverseIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.SlowMode;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.TuningCommands.ManualHoodDown;
import frc.robot.commands.TuningCommands.ManualHoodUp;
import frc.robot.commands.TuningCommands.ShooterSpeedDown;
import frc.robot.commands.TuningCommands.ShooterSpeedUp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightClimberLeft;
import frc.robot.subsystems.LimelightClimberRight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretHood;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(0);
  private final CommandXboxController operatorController = 
      new CommandXboxController(1);

  DriveTrain driveTrain;
  LimelightClimberLeft limelightClimberLeft;
  LimelightClimberRight limelightClimberRight;
  Spindexer spindexer;
  Intake intake;
  Turret turret;
  TurretHood turretHood;
  Shooter shooter;
  LEDs leds;
  //Climber climber;

  CandleCommand candleCommand;

  ZeroHeading zeroHeading;
  SlowMode slowMode;
  //ClimbAlign climbAlignLeft;
  //ClimbAlign climbAlignRight;
  AutoIntake autoIntake;
  ResetPose resetPose;

  ShootFuel shootFuel;
  RunIntake runIntake;
  RunReverseIntake runReverseIntake;
  IntakeArmToggle intakeArmToggle;
  
  MoveTurret moveTurret;
  MoveHood moveHood;

  DuckHood duckHood;

  //ClimberUp climberUp;
  //ClimberDown climberDown;
  //ClimberClimb climberClimb;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limelightClimberLeft = new LimelightClimberLeft();
    limelightClimberRight = new LimelightClimberRight();

    driveTrain = new DriveTrain(limelightClimberLeft, limelightClimberRight);
    spindexer = new Spindexer();
    intake = new Intake();
    turret = new Turret(driveTrain);
    turretHood = new TurretHood(turret);
    shooter = new Shooter(turret);
    leds = new LEDs(driveTrain);
    //climber = new Climber();

    candleCommand = new CandleCommand(leds, turret, turretHood, shooter);
    leds.setDefaultCommand(candleCommand);

    driveTrain.setDefaultCommand(new RunCommand( () -> driveTrain.drive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(), 
        driveTrain.fieldRelative),
      driveTrain));

    zeroHeading = new ZeroHeading(driveTrain);
    slowMode = new SlowMode(driveTrain);
    //climbAlignLeft = new ClimbAlign(driveTrain, false);
    //climbAlignRight = new ClimbAlign(driveTrain, true);
    autoIntake = new AutoIntake(driveTrain, intake, limelightClimberLeft);
    resetPose = new ResetPose(driveTrain);

    shootFuel = new ShootFuel(shooter, spindexer, turret, turretHood);
    runIntake = new RunIntake(intake);
    runReverseIntake = new RunReverseIntake(intake);
    intakeArmToggle = new IntakeArmToggle(intake);

    moveTurret = new MoveTurret(turret);
    turret.setDefaultCommand(moveTurret);

    moveHood = new MoveHood(turretHood, turret);
    turretHood.setDefaultCommand(moveHood);

    duckHood = new DuckHood(turretHood);

    //climberUp = new ClimberUp(climber);
    //climberDown = new ClimberDown(climber);
    //climberClimb = new ClimberClimb(climber);

    NamedCommands.registerCommand("RunIntake", runIntake);
    NamedCommands.registerCommand("ShootFuel", shootFuel);

    driveTrain.autoChooser.setDefaultOption("Right Outpost", new PathPlannerAuto("Right Outpost"));
    driveTrain.autoChooser.addOption("Right Shoot", new PathPlannerAuto("Right Shoot"));
    driveTrain.autoChooser.addOption("Right Lob", new PathPlannerAuto("Right Lob"));
    driveTrain.autoChooser.addOption("Left Shoot", new PathPlannerAuto("Left Shoot"));
    driveTrain.autoChooser.addOption("Left Lob", new PathPlannerAuto("Left Lob"));

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

    // DRIVER //

    driverController.start().onTrue(zeroHeading);
    driverController.back().onTrue(resetPose);

    driverController.rightBumper().whileTrue(shootFuel);
    driverController.leftStick().whileTrue(intakeArmToggle);
    driverController.rightStick().whileTrue(slowMode);
    driverController.x().whileTrue(runIntake);
    driverController.y().whileTrue(runReverseIntake);

    //driverController.pov(180).whileTrue(autoIntake);

    // OPERATOR //

    operatorController.rightBumper().whileTrue(shootFuel);
    operatorController.leftStick().whileTrue(intakeArmToggle);
    operatorController.rightStick().whileTrue(runIntake);

    operatorController.start().onTrue(resetPose);

    operatorController.a().whileTrue(duckHood);

    operatorController.y().whileTrue(runReverseIntake);
    // INSERT MANUAL COMMANDS FOR TUNING SPEEDS, HOOD ANGLES, AND TIMES

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
