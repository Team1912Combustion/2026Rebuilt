// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CandleCommand extends Command {
  LEDs leds;
  Hood hood;
  Shooter shooter;
  /** Creates a new CandleCommand. */
  public CandleCommand(LEDs l, Hood h, Shooter s) {
    leds = l;
    hood = h;
    shooter = s;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (leds.getMatchPeriod() <= 2) {
      leds.setOrangeFlashing();
    } else if (leds.getMatchPeriod() <= 6) {
      if (leds.getAlliance() == leds.getActiveHub()) {
        leds.setOrangeFlashing();
      } else {
        leds.setBlueStatic();
      }
    } else if (leds.getMatchPeriod() == 7) {
      leds.setOrangeFlashing();
    }*/

    //leds.setBlueStatic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
