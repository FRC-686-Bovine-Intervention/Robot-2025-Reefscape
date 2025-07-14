package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.RobotConstants;

public class ElevatorIOSim extends ElevatorIOKraken {
    private final ElevatorSim elevatorSim = new ElevatorSim(
        8,
        2,
        DCMotor.getKrakenX60(1).withReduction(ElevatorConstants.motorToMechanism.reductionUnsigned()),
        ElevatorConstants.minLengthPhysical.in(Meters),
        ElevatorConstants.stageExtension.in(Meters),
        false,
        ElevatorConstants.minLengthPhysical.in(Meters)
    );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        elevatorSim.setInputVoltage(-motorSimState.getMotorVoltage());
        elevatorSim.update(RobotConstants.rioUpdatePeriodSecs);

        var sprocketPosition = ElevatorConstants.stage1LinearRelation.distanceToAngle(Meters.of(elevatorSim.getPositionMeters()));
        var sprocketVelocity = ElevatorConstants.stage1LinearRelation.linearVelocityToAngularVelocity(MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond()));

        cancoderSimState.setRawPosition(ElevatorConstants.sensorToMechanism.inverse().applyUnsigned(sprocketPosition));
        cancoderSimState.setVelocity(ElevatorConstants.sensorToMechanism.inverse().applyUnsigned(sprocketVelocity));

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
