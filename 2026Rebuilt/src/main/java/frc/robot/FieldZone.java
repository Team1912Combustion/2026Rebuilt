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
    Translation2d shotPoint;

    /**
     * Creates a FieldZone. 
     * The FieldZone class is meant for getting information about the robot's presence in certain areas of the field.
     * It also allows for the creation of a "Shot Point" per FieldZone, which is a pose where a turret should be aiming.
     * @param topLeftTranslation2d The position of the top left of the field zone, with the origin starting on the right of blue's side
     * @param bottomRightTranslation2d The position of the bottom right of the field zone, with the origin starting on the right of blue's side
     */
    public FieldZone(Translation2d topLeftTranslation2d, Translation2d bottomRightTranslation2d) {
        topLeft = topLeftTranslation2d;
        bottomRight = bottomRightTranslation2d;
        shotPoint = new Translation2d();
    }

    /**
     * Returns whether or not a given pose is within the X limits of the FieldZone.
     * @param robotPose The current pose of the robot
     * @return Whether or not the robot is within the X limits of the FieldZone
     */
    public boolean withinXLimit(Pose2d robotPose) {
        return (robotPose.getX() > topLeft.getX() && robotPose.getX() < bottomRight.getX());
    }

    /**
     * Returns whether or not a given pose is within the Y limits of the FieldZone.
     * @param robotPose The current pose of the robot
     * @return Whether or not the robot is within the Y limits of the FieldZone
     */
    public boolean withinYLimit(Pose2d robotPose) {
        return (robotPose.getY() > bottomRight.getY() && robotPose.getY() < topLeft.getY());
    }

    /**
     * Returns whether or not a given pose is within the FieldZone
     * @param robotPose The current pose of the robot
     * @return Whether or not the robot is within the FieldZone
     */
    public boolean isInZone(Pose2d robotPose) {
        if (withinXLimit(robotPose) && withinYLimit(robotPose)) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Defines the shot point for the FieldZone.
     * @param desiredShotPoint The translation of the shot point
     */
    public void setShotPoint(Translation2d desiredShotPoint) {
        shotPoint = desiredShotPoint;
    }

    /**
     * Gets the shot point of the FieldZone.
     * @return The shot point as a Translation2d
     */
    public Translation2d getShotPoint() {
        return shotPoint;
    }

    /**
     * Gets the distance from a given pose to the shot point.
     * @param robotPose The current pose of the robot
     * @return The distance, in meters, from the shot point
     */
    public double getDistanceFromShotPoint(Pose2d robotPose) {
        return shotPoint.getDistance(robotPose.getTranslation());
    }
}
