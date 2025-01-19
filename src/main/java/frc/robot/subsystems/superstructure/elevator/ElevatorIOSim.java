package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.RobotConstants;

public class ElevatorIOSim extends ElevatorIOFalcon {
    private final ElevatorSim elevatorSim = new ElevatorSim(
        1,
        1,
        DCMotor.getFalcon500(2),
        0,
        2,
        false,
        0
    );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var leftSimState = leftMotor.getSimState();
        var rightSimState = rightMotor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        elevatorSim.setInputVoltage(leftSimState.getMotorVoltage());
        elevatorSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(elevatorSim.getPositionMeters() / ElevatorConstants.sprocketRadius.in(Meters));
        var velocity = RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.sprocketRadius.in(Meters));

        cancoderSimState.setRawPosition(position);
        cancoderSimState.setVelocity(velocity);

        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
