// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  Shooter shooter;
  Floor floor;
  DriveTrain driveTrain;
  Hood hood;
  IntakeRollers intakeRollers;
  IntakeArm intakeArm;
  /** Creates a new ShootFuel. */
  public ShootFuel(Shooter s, Floor f, DriveTrain dt, Hood h, IntakeRollers ir, IntakeArm ia) {
    shooter = s;
    floor = f;
    driveTrain = dt;
    hood = h;
    intakeRollers = ir;
    intakeArm = ia;
    addRequirements(shooter, floor, intakeRollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.duckHood = false;
    //intakeArm.armOut = false;
    intakeRollers.intakeSlow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(shooter.calculateSpeedContinuous(driveTrain.getDistance(driveTrain.getPose().getTranslation(), driveTrain.getTarget().getTranslation())));
    if (hood.isInPosiiton() && shooter.shooterAtSpeed() && !hood.duckHood()) {
      shooter.kickerOn();
      floor.setSpeed(90);
    } else {
      shooter.kickerOff();
      floor.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.duckHood = true;
    shooter.shooterOff();
    floor.floorOff();
    shooter.kickerOff();
    intakeArm.armOut = true;
    intakeRollers.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
