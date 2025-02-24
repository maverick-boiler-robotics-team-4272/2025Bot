// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.positions;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmevatorPositions {
   // public static final ArmevatorPosition example = new ArmevatorPosition(Rotation2d.fromDegrees(90), Meters.convertFrom(30, Inches));
   //barge is 50 in and 190 deg

    public static final ArmevatorPosition HOME = new ArmevatorPosition(Rotation2d.fromDegrees(10), Meters.convertFrom(0.1, Inches));

    public static final ArmevatorPosition BARGE_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(160), Meters.convertFrom(50,Inches));  

    public static final ArmevatorPosition L1_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(47), Meters.convertFrom(0, Inches));
    public static final ArmevatorPosition L2_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(30), Meters.convertFrom(11.5, Inches));
    public static final ArmevatorPosition L3_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(20), Meters.convertFrom(30.5, Inches));
    public static final ArmevatorPosition L4_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(147.775 - 360.0), Meters.convertFrom(30, Inches));
    // public static final ArmevatorPosition L4_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(147.775), Meters.convertFrom(30, Inches));

    public static final ArmevatorPosition FEEDING_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(2), Meters.convertFrom(0, Inches));

    public static final ArmevatorPosition ALGAE_ARMEVATOR_POSITION = new ArmevatorPosition(Rotation2d.fromDegrees(40), Meters.convertFrom(4.0, Inches));
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