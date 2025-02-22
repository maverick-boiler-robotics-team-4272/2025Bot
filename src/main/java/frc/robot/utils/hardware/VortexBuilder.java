package frc.robot.utils.hardware;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Robot;

public class VortexBuilder {
    private Vortex motor;
    private SparkFlexConfig config;

    private VortexBuilder(int id) {
        motor = new Vortex(id);
        config = new SparkFlexConfig();
    }

    public VortexBuilder asFollower(SparkFlex leaderMotor, boolean inverted) {
        this.config.follow(leaderMotor, inverted);

        return this;
    }

    public VortexBuilder asFollower(int leaderId, boolean inverted) {
        this.config.follow(leaderId, inverted);

        return this;
    }

    public VortexBuilder withVoltageCompensation(int nominalVoltage) {
        config.voltageCompensation(nominalVoltage);

        return this;
    }

    public VortexBuilder withCurrentLimit(int currentLimit) {
        config.smartCurrentLimit(currentLimit);

        return this;
    }

    public VortexBuilder withIdleMode(IdleMode mode) {
        config.idleMode(mode);

        return this;
    }

    public VortexBuilder withInversion(boolean inverted) {
        config.inverted(inverted);

        return this;
    }

    public VortexBuilder withPIDParams(double p, double i, double d) {
        config.closedLoop.pid(p, i, d);

        return this;
    }

    public VortexBuilder withPIDFParams(double p, double i, double d, double f) {
        config.closedLoop.pidf(p, i, d, f);

        return this;
    }

    public VortexBuilder withPIDClamping(double min, double max) {
        config.closedLoop.minOutput(min);
        config.closedLoop.maxOutput(max);

        return this;
    }

    public VortexBuilder withMaxIAccum(double max) {
        config.closedLoop.iMaxAccum(max);
        
        return this;
    }

    public VortexBuilder withPositionConversionFactor(double factor) {
        config.encoder.positionConversionFactor(factor);

        return this;
    }

    public VortexBuilder withPosition(double position) {
        motor.getEncoder().setPosition(position);

        return this;
    }

    public VortexBuilder withVelocityConversionFactor(double factor) {
        config.encoder.velocityConversionFactor(factor);

        return this;
    }

    public VortexBuilder withForwardSoftlimit(double limit) {
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(limit);

        return this;
    }

    public VortexBuilder withReverseSoftLimit(double limit) {
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(limit);

        return this;
    }

    public VortexBuilder withSoftLimits(double forwardLimit, double reverseLimit) {
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(reverseLimit);

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(forwardLimit);

        return this;
    }

    public VortexBuilder withOutputRange(double min, double max) {
        config.closedLoop.outputRange(min, max);

        return this;
    }

    public VortexBuilder withClosedLoopRampRate(double rate) {
        config.closedLoopRampRate(rate);

        return this;
    }

    public VortexBuilder withOpenLoopRampRate(double rate) {
        config.openLoopRampRate(rate);

        return this;
    }

    public VortexBuilder withPIDPositionWrapping(double min, double max) {
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(min, max);

        return this;
    }

    public VortexBuilder withAbsoluteEncoderConfig(AbsoluteEncoderConfig sensorConfig) {
        this.config.apply(sensorConfig);

        return this;
    }

    public VortexBuilder withAnalogConfig(AnalogSensorConfig sensorConfig) {
        this.config.apply(sensorConfig);

        return this;
    }

    public VortexBuilder withLimitSwitch(LimitSwitchConfig limitConfig) {
        this.config.limitSwitch.setSparkMaxDataPortConfig();
        this.config.apply(limitConfig);

        return this;
    }

    public VortexBuilder withLimitSwitch() {
        config.limitSwitch.setSparkMaxDataPortConfig();

        return this;
    }

    public VortexBuilder maxSpeeds(double vel, double acc) {
        config.closedLoop.maxMotion.maxVelocity(vel).maxAcceleration(acc);

        return this;
    }

    public VortexBuilder withInputFramesMs(
        int position,
        int velocity,
        int busVoltage,
        int externalEncoder,
        int current,
        int warnings,
        int faults
    ) {
        this.config.signals.busVoltagePeriodMs(busVoltage);
        this.config.signals.externalOrAltEncoderPosition(externalEncoder);
        this.config.signals.outputCurrentPeriodMs(current);
        this.config.signals.warningsPeriodMs(warnings);
        this.config.signals.faultsPeriodMs(faults);
        this.config.signals.primaryEncoderPositionPeriodMs(position);
        this.config.signals.primaryEncoderVelocityPeriodMs(velocity);

        return this;
    }

    public VortexBuilder positionFrameMs(int ms) {
        this.config.signals.primaryEncoderPositionPeriodMs(ms);

        return this;
    }
    
    public Vortex build() {
        if(Robot.isReal()) {
            motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        return motor;
    }

    public static VortexBuilder create(int id) {
        return new VortexBuilder(id);
    }

    public static VortexBuilder createWithDefaults(int id) {
        return new VortexBuilder(id)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake);
    }
}