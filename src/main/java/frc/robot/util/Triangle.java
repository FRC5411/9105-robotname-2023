
package frc.robot.util;

public class Triangle {
    
    double sideA;
    double sideB;
    double sideC;

    public Triangle(double side_a, double side_b, double side_c) {
        sideA = side_a;
        sideB = side_b;
        sideC = side_c;
    }

    public double getAngleA() {
        return Math.acos((Math.pow(sideB, 2) - Math.pow(sideA, 2) - Math.pow(sideC, 2)) / (-2 * sideA * sideC));
    }

    public double getAngleB() {
        return Math.acos((Math.pow(sideC, 2) - Math.pow(sideA, 2) - Math.pow(sideB, 2)) / (-2 * sideA * sideB));
    }

    public double getAngleC() {
        return Math.acos((Math.pow(sideA, 2) - Math.pow(sideB, 2) - Math.pow(sideC, 2)) / (-2 * sideB * sideC));
    }
}