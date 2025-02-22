package frc.robot.subsystems.coralManipulator.states;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralIndexState extends State<CoralManipulator> {
    private BooleanSupplier lidarTripped;
    public CoralIndexState(CoralManipulator coralManipulator, BooleanSupplier lidarTripped) {
        super(coralManipulator);
        this.lidarTripped = lidarTripped;
    }

    @Override
    public void initialize() {
        requiredSubsystem.setCoralPower(-0.2);
        System.out.println("Init");
    }

    @Override
    public boolean isFinished() {
        if (lidarTripped.getAsBoolean()) {
            requiredSubsystem.setCoralPower(0);
            System.out.println("Done");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.setWheelRotation(Rotation2d.fromRotations(1));
    }
}
