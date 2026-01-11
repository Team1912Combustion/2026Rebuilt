// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldZone {    
    Translation2d topLeft;
    Translation2d bottomRight;

    public FieldZone(Translation2d topLeftPose2d, Translation2d bottomRightPose2d) {
        topLeft = topLeftPose2d;
        bottomRight = bottomRightPose2d;
    }

    public boolean withinXLimit(Pose2d robotPose) {
        return (robotPose.getX() > topLeft.getX() && robotPose.getX() < bottomRight.getX());
    }

    public boolean withinYLimit(Pose2d robotPose) {
        return (robotPose.getY() > bottomRight.getY() && robotPose.getY() < topLeft.getY());
    }

    public boolean isInZone(Pose2d robotPose) {
        if (withinXLimit(robotPose) && withinYLimit(robotPose)) {
            return true;
        } else {
            return false;
        }
    }
}
