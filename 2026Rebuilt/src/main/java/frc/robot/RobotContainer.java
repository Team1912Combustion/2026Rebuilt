// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.BoostDown;
import frc.robot.commands.BoostUp;
import frc.robot.commands.CandleCommand;
import frc.robot.commands.DuckHood;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeArmToggle;
import frc.robot.commands.MoveHood;
import frc.robot.commands.PointAtTarget;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunReverseIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.SlowMode;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.AutoCommands.RevUpShooter;
import frc.robot.commands.TuningCommands.ManualHoodDown;
import frc.robot.commands.TuningCommands.ManualHoodUp;
import frc.robot.commands.TuningCommands.ShooterSpeedDown;
import frc.robot.commands.TuningCommands.ShooterSpeedUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.LimelightLeft;
import frc.robot.subsystems.LimelightRight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  LimelightLeft limelightLeft;
  LimelightRight limelightRight;
  LimelightShooter limelightShooter;
  Floor spindexer;
  Intake intake;
  Hood hood;
  Shooter shooter;
  LEDs leds;
  //Climber climber;

  CandleCommand candleCommand;

  PointAtTarget pointAtTarget;

  ZeroHeading zeroHeading;
  ZeroGyro zeroGyro;
  SlowMode slowMode;
  //ClimbAlign climbAlignLeft;
  //ClimbAlign climbAlignRight;
  AutoIntake autoIntake;
  ResetPose resetPose;

  ShootFuel shootFuel;
  BoostUp boostUp;
  BoostDown boostDown;
  RevUpShooter revUpShooter;

  RunIntake runIntake;
  RunReverseIntake runReverseIntake;
  IntakeArmToggle intakeArmToggle;

  MoveHood moveHood;
  DuckHood duckHood;

  //ClimberUp climberUp;
  //ClimberDown climberDown;
  //ClimberClimb climberClimb;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limelightLeft = new LimelightLeft();
    limelightRight = new LimelightRight();
    limelightShooter = new LimelightShooter();

    driveTrain = new DriveTrain(limelightLeft, limelightRight, limelightShooter);
    spindexer = new Floor();
    intake = new Intake();
    hood = new Hood(driveTrain);
    shooter = new Shooter(driveTrain);
    leds = new LEDs(driveTrain);
    //climber = new Climber();

    candleCommand = new CandleCommand(leds, hood, shooter);
    leds.setDefaultCommand(candleCommand);

    pointAtTarget = new PointAtTarget(driveTrain);

    driveTrain.setDefaultCommand(new RunCommand( () -> driveTrain.drive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(), 
        driveTrain.fieldRelative),
      driveTrain));

    zeroHeading = new ZeroHeading(driveTrain);
    zeroGyro = new ZeroGyro(driveTrain);
    slowMode = new SlowMode(driveTrain);
    //climbAlignLeft = new ClimbAlign(driveTrain, false);
    //climbAlignRight = new ClimbAlign(driveTrain, true);
    autoIntake = new AutoIntake(driveTrain, intake, limelightLeft);
    resetPose = new ResetPose(driveTrain);

    shootFuel = new ShootFuel(shooter, spindexer, driveTrain, hood);
    boostUp = new BoostUp(shooter);
    boostDown = new BoostDown(shooter);
    revUpShooter = new RevUpShooter(shooter);

    runIntake = new RunIntake(intake);
    runReverseIntake = new RunReverseIntake(intake);
    intakeArmToggle = new IntakeArmToggle(intake);

    moveHood = new MoveHood(hood, driveTrain);
    hood.setDefaultCommand(moveHood);

    duckHood = new DuckHood(hood);

    //climberUp = new ClimberUp(climber);
    //climberDown = new ClimberDown(climber);
    //climberClimb = new ClimberClimb(climber);

    NamedCommands.registerCommand("RunIntake", runIntake);
    NamedCommands.registerCommand("ArmToggle", intakeArmToggle);
    NamedCommands.registerCommand("ShootFuel", shootFuel);
    NamedCommands.registerCommand("RunReverseIntake", runReverseIntake);
    NamedCommands.registerCommand("RevUpShooter", revUpShooter);
    NamedCommands.registerCommand("PointAtTarget", pointAtTarget);
    NamedCommands.registerCommand("ResetPose", resetPose);

    driveTrain.autoChooser = AutoBuilder.buildAutoChooser("");
    driveTrain.autoChooser.addOption("Right Outpost", new PathPlannerAuto("Right Outpost"));
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
    driverController.rightStick().whileTrue(pointAtTarget);
    //driverController.x().whileTrue(runIntake);
    driverController.y().whileTrue(runReverseIntake);

    //driverController.pov(180).whileTrue(autoIntake);

    // OPERATOR //

    operatorController.rightBumper().whileTrue(shootFuel);
    //operatorController.leftStick().whileTrue(intakeArmToggle);
    operatorController.leftStick().whileTrue(shootFuel);
    operatorController.rightStick().whileTrue(runIntake);
    operatorController.x().whileTrue(runIntake);
    operatorController.y().whileTrue(runReverseIntake);

    operatorController.start().onTrue(resetPose);
    operatorController.back().multiPress(2, 0.5).onTrue(zeroGyro);

    operatorController.pov(0).whileTrue(boostUp);
    operatorController.pov(180).whileTrue(boostDown);

    operatorController.a().whileTrue(duckHood);
    // INSERT MANUAL COMMANDS FOR TUNING SPEEDS, HOOD ANGLES, AND TIMES

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveTrain.autoChooser.getSelected();
  }
}
