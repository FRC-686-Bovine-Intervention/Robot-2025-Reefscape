package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class WristIOSim extends WristIOKraken {
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(5, 5),
        DCMotor.getKrakenX60(1),
        WristConstants.motorToMechanism.reductionUnsigned(),
        0.2,
        WristConstants.minAngle.in(Radians),
        WristConstants.maxAngle.in(Radians),
        false,
        1
    );

    @Override
    public void updateInputs(WristIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        wristSim.setInputVoltage(motorSimState.getMotorVoltage());
        wristSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(wristSim.getAngleRads());
        var velocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(WristConstants.sensorToMechanism.applyUnsigned(position));
        cancoderSimState.setVelocity(WristConstants.sensorToMechanism.applyUnsigned(velocity));

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
