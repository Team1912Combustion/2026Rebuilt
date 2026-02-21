// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MovePose;
import frc.robot.commands.MoveTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightTurret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

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
  Turret turret;
  Shooter shooter;
  LimelightTurret limelightTurret;

  MovePose movePose;
  MoveTurret moveTurret;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limelightTurret = new LimelightTurret();
    driveTrain = new DriveTrain(limelightTurret);
    turret = new Turret(driveTrain);
    shooter= new Shooter(turret);

    movePose = new MovePose(driveTrain);
    moveTurret = new MoveTurret(turret);

    /*driveTrain.setDefaultCommand(new RunCommand( () -> driveTrain.drive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(), 
        driveTrain.fieldRelative),
      driveTrain));*/
    driveTrain.setDefaultCommand(movePose);

    turret.setDefaultCommand(moveTurret);

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
    //driverController.a().whileTrue(movePose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
