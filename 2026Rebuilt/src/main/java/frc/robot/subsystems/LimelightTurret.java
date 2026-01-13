// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class LimelightTurret extends LimelightFrontLeft {
  /** Creates a new LimelightTurret. */
  @Override
    // IP IS - 10.19.12.13:5801 //

    /**
    * Gets the name of the limelight.
    * @return The name of the limelight as a string
    */
    public String getName() {
        return "limelight-t";
    }
    
    @Override

    /**
    * Gets the position of the limelight.
    * @return The position of the limelight as a string
    */
    public String getPosition() {
        return "turret";
    }
}