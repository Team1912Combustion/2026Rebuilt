// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmWiggle;
import frc.robot.commands.BoostDown;
import frc.robot.commands.BoostUp;
import frc.robot.commands.DuckHood;
import frc.robot.commands.IntakeAdjustIn;
import frc.robot.commands.IntakeAdjustOut;
import frc.robot.commands.IntakeArmToggle;
import frc.robot.commands.MoveHood;
import frc.robot.commands.PointAtTarget;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunReverseIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.SlowMode;
import frc.robot.commands.ToggleTargetMode;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.AutoCommands.DriveToPosition;
import frc.robot.commands.AutoCommands.RevUpShooter;
import frc.robot.commands.TuningCommands.FloorSpeedDown;
import frc.robot.commands.TuningCommands.FloorSpeedUp;
import frc.robot.commands.TuningCommands.ManualHoodDown;
import frc.robot.commands.TuningCommands.ManualHoodUp;
import frc.robot.commands.TuningCommands.ShooterSpeedDown;
import frc.robot.commands.TuningCommands.ShooterSpeedUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  DriveTrain driveTrain;
  LimelightShooter limelightShooter;
  Floor floor;
  IntakeRollers intakeRollers;
  IntakeArm intakeArm;
  Hood hood;
  Shooter shooter;
  LEDs leds;

  PointAtTarget pointAtTarget;
  ToggleTargetMode toggleTargetMode;

  ZeroHeading zeroHeading;
  ZeroGyro zeroGyro;
  SlowMode slowMode;
  ResetPose resetPose;

  ShootFuel shootFuel;
  BoostUp boostUp;
  BoostDown boostDown;
  RevUpShooter revUpShooter;

  IntakeAdjustIn intakeAdjustIn;
  IntakeAdjustOut intakeAdjustOut;

  RunIntake runIntake;
  RunReverseIntake runReverseIntake;
  IntakeArmToggle intakeArmToggle;
  ArmWiggle armWiggle;

  MoveHood moveHood;
  DuckHood duckHood;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limelightShooter = new LimelightShooter();

    driveTrain = new DriveTrain(limelightShooter);
    floor = new Floor();
    intakeRollers = new IntakeRollers();
    intakeArm = new IntakeArm();
    hood = new Hood(driveTrain);
    shooter = new Shooter(driveTrain);
    leds = new LEDs(driveTrain);

    pointAtTarget = new PointAtTarget(driveTrain, limelightShooter);
    toggleTargetMode = new ToggleTargetMode(driveTrain);

    driveTrain.setDefaultCommand(new RunCommand( () -> driveTrain.drive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(), 
        driveTrain.fieldRelative
      ), driveTrain));

    zeroHeading = new ZeroHeading(driveTrain);
    zeroGyro = new ZeroGyro(driveTrain);
    slowMode = new SlowMode(driveTrain);
    resetPose = new ResetPose(driveTrain);

    shootFuel = new ShootFuel(shooter, floor, driveTrain, hood, intakeRollers, intakeArm);
    boostUp = new BoostUp(shooter);
    boostDown = new BoostDown(shooter);
    revUpShooter = new RevUpShooter(shooter);

    intakeAdjustIn = new IntakeAdjustIn(intakeArm);
    intakeAdjustOut = new IntakeAdjustOut(intakeArm);

    runIntake = new RunIntake(intakeRollers, intakeArm, floor, shooter);
    runReverseIntake = new RunReverseIntake(intakeRollers, floor);
    intakeArmToggle = new IntakeArmToggle(intakeArm);
    armWiggle = new ArmWiggle(intakeArm);

    moveHood = new MoveHood(hood, driveTrain);
    hood.setDefaultCommand(moveHood);

    duckHood = new DuckHood(hood);

    NamedCommands.registerCommand("RunIntake", runIntake);
    NamedCommands.registerCommand("ArmToggle", intakeArmToggle);
    NamedCommands.registerCommand("ShootFuel", shootFuel);
    NamedCommands.registerCommand("RunReverseIntake", runReverseIntake);
    NamedCommands.registerCommand("RevUpShooter", revUpShooter);
    NamedCommands.registerCommand("PointAtTarget", pointAtTarget);
    NamedCommands.registerCommand("ResetPose", resetPose);
    NamedCommands.registerCommand("ArmWiggle", armWiggle);

    NamedCommands.registerCommand("Drive to (3.0, 7.4, 0)", new DriveToPosition(driveTrain, new Pose2d(new Translation2d(3.0, 8), Rotation2d.fromDegrees(0))));
    NamedCommands.registerCommand("Drive to (3.0, 0.6, 0)", new DriveToPosition(driveTrain, new Pose2d(new Translation2d(3.0, 0), Rotation2d.fromDegrees(0))));

    driveTrain.autoChooser = AutoBuilder.buildAutoChooser("");
    driveTrain.autoChooser.addOption("Right Shoot Trench", new PathPlannerAuto("Right Shoot Trench"));
    driveTrain.autoChooser.addOption("Left Shoot Trench", new PathPlannerAuto("Left Shoot Trench"));
    driveTrain.autoChooser.addOption("Right Shoot Bump", new PathPlannerAuto("Right Shoot Bump"));
    driveTrain.autoChooser.addOption("Left Shoot Bump", new PathPlannerAuto("Left Shoot Bump"));
    driveTrain.autoChooser.addOption("Right Shoot Trench Long Swipe", new PathPlannerAuto("Right Shoot Trench Long Swipe"));
    driveTrain.autoChooser.addOption("Left Shoot Trench Long Swipe", new PathPlannerAuto("Left Shoot Trench Long Swipe"));
    driveTrain.autoChooser.addOption("Right Shoot Outpost", new PathPlannerAuto("Right Shoot Outpost"));
    driveTrain.autoChooser.addOption("Center Shoot", new PathPlannerAuto("Center Shoot"));

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
    driverController.leftStick().whileTrue(pointAtTarget);
    //driverController.rightStick().whileTrue(shootFuel);
    driverController.x().whileTrue(runIntake);
    driverController.y().whileTrue(runReverseIntake);

    // OPERATOR //
    operatorController.rightBumper().whileTrue(shootFuel);
    operatorController.leftBumper().whileTrue(intakeArmToggle);
    operatorController.leftTrigger(0.2).whileTrue(armWiggle);
    operatorController.x().whileTrue(runIntake);
    operatorController.y().whileTrue(runReverseIntake);

    Command leftOffsetIn = new InstantCommand(()-> {
      IntakeArm.offsetIn(intakeArm.left_arm);
      intakeArm.resetStallState();
    }, intakeArm);
    operatorController.axisLessThan(1, -.2).whileTrue(leftOffsetIn);

    Command rightOffsetIn = new InstantCommand(()-> {
      IntakeArm.offsetIn(intakeArm.right_arm);
      intakeArm.resetStallState();
    },intakeArm);
    operatorController.axisLessThan(5, -.2).whileTrue(rightOffsetIn);

    Command leftOffsetOut = new InstantCommand(()-> {
      IntakeArm.offsetOut(intakeArm.left_arm);
      intakeArm.resetStallState();
    }, intakeArm);
    operatorController.axisGreaterThan(1, .2).whileTrue(leftOffsetOut);

    Command rightOffsetOut = new InstantCommand(()-> {
      IntakeArm.offsetOut(intakeArm.right_arm);
      intakeArm.resetStallState();
    }, intakeArm);
    operatorController.axisGreaterThan(5, .2).whileTrue(rightOffsetOut);

    operatorController.start().onTrue(resetPose);
    operatorController.back().multiPress(2, 0.5).onTrue(zeroGyro);

    operatorController.pov(0).whileTrue(boostUp);
    operatorController.pov(180).whileTrue(boostDown);
    operatorController.pov(90).whileTrue(new FloorSpeedUp(floor));
    operatorController.pov(270).whileTrue(new FloorSpeedDown(floor));

    operatorController.a().whileTrue(duckHood);
    operatorController.b().onTrue(toggleTargetMode);
    // INSERT MANUAL COMMANDS FOR TUNING SPEEDS, HOOD ANGLES, AND TIMES

    /*
     * BFR - do we need these?
    driverController.pov(0).whileTrue(new ShooterSpeedUp(shooter));
    driverController.pov(180).whileTrue(new ShooterSpeedDown(shooter));
    driverController.pov(90).whileTrue(new ManualHoodUp(hood));
    driverController.pov(270).whileTrue(new ManualHoodDown(hood));
    */
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
