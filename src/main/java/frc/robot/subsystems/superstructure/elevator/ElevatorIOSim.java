package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.RobotConstants;

public class ElevatorIOSim extends ElevatorIOKraken {
    private final ElevatorSim elevatorSim = new ElevatorSim(
        1,
        1,
        DCMotor.getKrakenX60(1),
        0,
        ElevatorConstants.stageExtension.in(Meters),
        false,
        0
    );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        elevatorSim.setInputVoltage(motorSimState.getMotorVoltage());
        elevatorSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(elevatorSim.getPositionMeters() / ElevatorConstants.sprocketRadius.in(Meters));
        var velocity = RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.sprocketRadius.in(Meters));

        cancoderSimState.setRawPosition(position);
        cancoderSimState.setVelocity(velocity);

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
