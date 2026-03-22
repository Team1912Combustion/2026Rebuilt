// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  Shooter shooter;
  Spindexer spindexer;
  //DriveTrain driveTrain;
  Turret turret;
  TurretHood turretHood;
  /** Creates a new ShootFuel. */
  public ShootFuel(Shooter s, Spindexer sp, Turret t, TurretHood th) {
    shooter = s;
    spindexer = sp;
    //driveTrain = dt;
    turret = t;
    turretHood = th;
    addRequirements(spindexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretHood.duckHood = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(shooter.calculateSpeedContinuous(turret.getDistance(turret.getTurretPose().getTranslation(), turret.getTarget().getTranslation())));
    //shooter.setSpeed(-40);
    if (turret.isAimed() && turretHood.isInPosiiton() && shooter.shooterAtSpeed() && !turretHood.duckHood()) {
      shooter.kickerOn();
      spindexer.setSpeed(-40);
    } else {
      shooter.kickerOff();
      spindexer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretHood.duckHood = true;
    shooter.shooterOff();
    spindexer.spindexerOff();
    shooter.kickerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
