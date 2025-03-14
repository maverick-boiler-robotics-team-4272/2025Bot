package frc.robot.subsystems.coralManipulator.states;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralIndexState extends State<CoralManipulator> {
    private BooleanSupplier lidarTripped;
    private boolean tripped;

    public CoralIndexState(CoralManipulator coralManipulator, BooleanSupplier lidarTripped) {
        super(coralManipulator);
        this.lidarTripped = lidarTripped;
        tripped = false;
    }

    @Override
    public void initialize() {
        requiredSubsystem.setCoralPower(-0.1);
    }

    @Override
    public void execute() {
        if (lidarTripped.getAsBoolean()) {
            requiredSubsystem.setCoralPower(0);
            requiredSubsystem.addWheelRotations(Rotation2d.fromRotations(0.125));
            tripped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return tripped && (Math.abs(requiredSubsystem.getWheelError().getDegrees()) <= 10.0);
    }
}
