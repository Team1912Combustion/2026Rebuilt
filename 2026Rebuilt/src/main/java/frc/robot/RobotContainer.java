// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbAlign;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeArmToggle;
import frc.robot.commands.PointAtThing;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ZeroHeading;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightClimberLeft;
import frc.robot.subsystems.LimelightClimberRight;
import frc.robot.subsystems.Spindexer;

import com.pathplanner.lib.auto.NamedCommands;

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
  Climber climber;

  ZeroHeading zeroHeading;
  ClimbAlign climbAlignLeft;
  ClimbAlign climbAlignRight;
  AutoIntake autoIntake;
  PointAtThing pointAtThing;

  ShootFuel shootFuel;
  RunIntake runIntake;
  IntakeArmToggle intakeArmToggle;

  ClimberUp climberUp;
  ClimberDown climberDown;
  ClimberClimb climberClimb;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limelightClimberLeft = new LimelightClimberLeft();
    limelightClimberRight = new LimelightClimberRight();

    driveTrain = new DriveTrain(limelightClimberLeft, limelightClimberRight);
    spindexer = new Spindexer();
    intake = new Intake();
    climber = new Climber();

    driveTrain.setDefaultCommand(new RunCommand( () -> driveTrain.drive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(), 
        driveTrain.fieldRelative),
      driveTrain));

    zeroHeading = new ZeroHeading(driveTrain);
    climbAlignLeft = new ClimbAlign(driveTrain, false);
    climbAlignRight = new ClimbAlign(driveTrain, true);
    autoIntake = new AutoIntake(driveTrain, intake, limelightClimberLeft);
    pointAtThing = new PointAtThing(driveTrain);

    shootFuel = new ShootFuel(spindexer);
    runIntake = new RunIntake(intake);
    intakeArmToggle = new IntakeArmToggle(intake);

    climberUp = new ClimberUp(climber);
    climberDown = new ClimberDown(climber);
    climberClimb = new ClimberClimb(climber);

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

    driverController.start().onTrue(zeroHeading);

    driverController.rightBumper().whileTrue(shootFuel);
    driverController.leftStick().whileTrue(runIntake);
    driverController.rightStick().onTrue(intakeArmToggle);

    driverController.y().onTrue(climberUp);
    driverController.a().onTrue(climberDown);
    driverController.b().onTrue(climberClimb);

    driverController.pov(270).whileTrue(climbAlignLeft);
    driverController.pov(90).whileTrue(climbAlignRight);
    driverController.rightTrigger(0.1).whileTrue(pointAtThing);
    
    driverController.pov(180).whileTrue(autoIntake);
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
