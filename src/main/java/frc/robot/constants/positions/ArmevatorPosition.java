// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.positions;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmevatorPosition {
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