// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.positions;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmevatorPositions {
   // public static final ArmevatorPosition example = new ArmevatorPosition(Rotation2d.fromDegrees(90), Meters.convertFrom(30, Inches));
   // example is 30 in and 90 deg

    public static final ArmevatorPosition HOME = new ArmevatorPosition(Rotation2d.fromDegrees(5), Meters.convertFrom(0.1, Inches));

    public static final ArmevatorPosition BARGE_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(160), Meters.convertFrom(50,Inches));
    public static final ArmevatorPosition BARGE_PREP_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(120), Meters.convertFrom(50,Inches));

    public static final ArmevatorPosition L1_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(33), Meters.convertFrom(3.0, Inches));
    public static final ArmevatorPosition L2_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(15), Meters.convertFrom(17.5, Inches));
    public static final ArmevatorPosition L3_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(15), Meters.convertFrom(32.5, Inches));
    public static final ArmevatorPosition L4_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(147.775 - 360.0), Meters.convertFrom(30, Inches));
    public static final ArmevatorPosition L4_PREP_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(-30.0), Meters.convertFrom(25, Inches));

    public static final ArmevatorPosition FEEDING_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(0), Meters.convertFrom(0.0, Inches));

    //Changed the position to be lower for testing, might be changed back at worlds
    //public static final ArmevatorPosition ALGAE_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(37), Meters.convertFrom(4.0, Inches));
    public static final ArmevatorPosition ALGAE_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(37), Meters.convertFrom(3.0, Inches));
    public static final ArmevatorPosition ALGAE_ARMEVATOR_POSITION_TWO = new ArmevatorPosition(Rotation2d.fromDegrees(40), Meters.convertFrom(15, Inches));

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