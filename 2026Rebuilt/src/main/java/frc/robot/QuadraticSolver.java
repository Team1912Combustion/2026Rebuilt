// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class QuadraticSolver {

    public QuadraticSolver() {}

    public double findZeros(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;

        if (discriminant > 0) {
            double root1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root2 = (-b - Math.sqrt(discriminant)) / (2 * a);
            return Math.max(root1, root2);
        } else if (discriminant == 0) {
            double root = -b / (2 * a);
            return root;
        } else {
            // Handle complex roots or indicate no real roots
            return 0;
        }
    }

}
