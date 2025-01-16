package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class PivotIOSim extends PivotIOFalcon {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(2),
        100,
        1,
        1,
        0,
        Degrees.of(100).in(Radians),
        true,
        0
    );

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        var leftSimState = leftMotor.getSimState();
        var rightSimState = rightMotor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        pivotSim.setInputVoltage(leftSimState.getMotorVoltage());
        pivotSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(pivotSim.getAngleRads());
        var velocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(position);
        cancoderSimState.setVelocity(velocity);

        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
