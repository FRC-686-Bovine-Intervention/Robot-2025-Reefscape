package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class WristIOSim extends WristIOFalcon {
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(0.1, 0.1),
        DCMotor.getFalcon500(1),
        WristConstants.motorToMechanism.ratio(),
        0.2,
        Degrees.of(150).unaryMinus().in(Radians),
        Degrees.of(150).in(Radians),
        false,
        0
    );

    @Override
    public void updateInputs(WristIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        wristSim.setInputVoltage(motorSimState.getMotorVoltage());
        wristSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(wristSim.getAngleRads());
        var velocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(position);
        cancoderSimState.setVelocity(velocity);

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
