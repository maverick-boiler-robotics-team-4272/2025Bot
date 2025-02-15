// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.positions;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmevatorPositions {
    public static final ArmevatorPosition example = new ArmevatorPosition(Rotation2d.fromDegrees(90), Meters.convertFrom(30, Inches));
    
    public static class ArmevatorPosition {
        private Rotation2d armAngle;
        private double elevatorHeight;

        public ArmevatorPosition(Rotation2d armAngle, double elevatorHeight) {
            this.armAngle = armAngle;
            this.elevatorHeight = elevatorHeight;
        }

        public Rotation2d getArmAngle() {
            return armAngle;
        }

        public double getElevatorHeight() {
            return elevatorHeight;
        }
    }
}