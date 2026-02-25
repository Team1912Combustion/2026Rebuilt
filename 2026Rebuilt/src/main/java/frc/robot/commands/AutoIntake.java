// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightClimberLeft;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  DriveTrain driveTrain; 
  Intake intake;
  LimelightClimberLeft limelight;

  PIDController pid;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveTrain dt, Intake i, LimelightClimberLeft llcl) {
    driveTrain = dt;
    intake = i;
    limelight = llcl;
    addRequirements(driveTrain, intake);

    pid = new PIDController(0, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.2);
    pid.setSetpoint(0);
    limelight.setPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = limelight.getXOffset();

    driveTrain.drive(0.1, 0, pid.calculate(tx), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(0);
    driveTrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
