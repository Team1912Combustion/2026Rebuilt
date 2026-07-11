// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  IntakeRollers intakeRollers;
  IntakeArm intakeArm;
  Floor floor;
  Shooter shooter;
  /** Creates a new RunIntake. */
  public RunIntake(IntakeRollers ir, IntakeArm ia, Floor f, Shooter s) {
    intakeRollers = ir;
    intakeArm = ia;
    floor = f;
    shooter = s;
    addRequirements(intakeRollers, floor, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRollers.intake();
    floor.setSpeed(10);
    shooter.kickerReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRollers.setRollerSpeed(0);
    floor.setSpeed(0);
    shooter.kickerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
