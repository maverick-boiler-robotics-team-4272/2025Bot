package frc.robot.subsystems.feeder.states;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.utils.commandUtils.State;

public class DeJamState extends State<Feeder> {
    double power;
    boolean unJamming;
    Timer jamClock;

    public DeJamState(Feeder feed, double power) {
        super(feed);

        this.power = power;
        jamClock = new Timer();
        unJamming = false;
    }

    @Override
    public void initialize() {
        requiredSubsystem.setFeederPower(power);

        jamClock.reset();
    }

    @Override
    public void execute() {
        // Start the jam clock if conditions are met
        if (requiredSubsystem.lidarBackTripped() &&
                requiredSubsystem.lidarFrontNotTripped() &&
                !jamClock.isRunning() &&
                !unJamming) {
            jamClock.start();
        }

        // Stop the jam clock and set feeder power if necessary
        if ((requiredSubsystem.lidarFrontTripped() || requiredSubsystem.lidarBackNotTripped()) &&
                jamClock.isRunning()) {
            jamClock.stop();
            requiredSubsystem.setFeederPower(power);
        }

        // Handle unjamming behavior with a 0.5-second delay
        if (jamClock.hasElapsed(0.5)) {
            unJamming = true;
            requiredSubsystem.setFeederPower(-0.1);
            jamClock.reset();
        }

        // Reset unjamming state after a 0.2-second delay
        if (unJamming && jamClock.hasElapsed(0.2)) {
            unJamming = false;
            jamClock.restart();
            requiredSubsystem.setFeederPower(power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.setFeederPower(0.0);
    }
}
