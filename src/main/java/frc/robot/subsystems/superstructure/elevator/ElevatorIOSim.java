package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.RobotConstants;

public class ElevatorIOSim extends ElevatorIOKraken {
    private final ElevatorSim elevatorSim = new ElevatorSim(
        20,
        12,
        DCMotor.getKrakenX60(1),
        ElevatorConstants.minLength.in(Meters),
        ElevatorConstants.stageExtension.in(Meters),
        false,
        ElevatorConstants.minLength.in(Meters)
    );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        elevatorSim.setInputVoltage(-motorSimState.getMotorVoltage());
        elevatorSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(elevatorSim.getPositionMeters() / ElevatorConstants.sprocketRadius.in(Meters));
        var velocity = RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.sprocketRadius.in(Meters));

        cancoderSimState.setRawPosition(position.div(-ElevatorConstants.sensorToMechanism.ratio()));
        cancoderSimState.setVelocity(velocity.div(-ElevatorConstants.sensorToMechanism.ratio()));

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        Logger.recordOutput("DEBUG/leftsimstate voltage", -motorSimState.getMotorVoltage());
        Logger.recordOutput("DEBUG/sim position", position);
        Logger.recordOutput("DEBUG/sim velocity", velocity);
        Logger.recordOutput("DEBUG/ratio", -ElevatorConstants.sensorToMechanism.ratio());

        super.updateInputs(inputs);
    }
}
