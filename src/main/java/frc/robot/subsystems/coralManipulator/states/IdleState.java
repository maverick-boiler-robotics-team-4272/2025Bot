package frc.robot.subsystems.coralManipulator.states;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class IdleState extends State<CoralManipulator> {
    private Rotation2d startArmRotation;
    private Supplier<Rotation2d> currentArmRotation;

    public IdleState(CoralManipulator coralManipulator, Supplier<Rotation2d> armRotation) {
        super(coralManipulator);
        this.currentArmRotation = armRotation;
    }

    @Override
    public void initialize() {
        startArmRotation = currentArmRotation.get();
        requiredSubsystem.resetWheelRotation();
    }

    @Override
    public void execute() {
        requiredSubsystem.setWheelRotation(Rotation2d.fromDegrees((currentArmRotation.get().getDegrees() - startArmRotation.getDegrees()) * 4.5));
    }
}
