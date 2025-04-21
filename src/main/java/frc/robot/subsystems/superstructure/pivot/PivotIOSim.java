package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class PivotIOSim extends PivotIOFalcon {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(17, 5),
        DCMotor.getFalcon500(2).withReduction(237.6),
        PivotConstants.motorToMechanism.inverse().ratio(),
        1,
        PivotConstants.minAngle.in(Radians),
        PivotConstants.maxAngle.in(Radians),
        false,
        PivotConstants.minAngle.in(Radians)
    );

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        var leftSimState = leftMotor.getSimState();
        var rightSimState = rightMotor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        pivotSim.setInputVoltage(-leftSimState.getMotorVoltage()+rightSimState.getMotorVoltage());
        pivotSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(pivotSim.getAngleRads());
        var velocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(position.unaryMinus());
        cancoderSimState.setVelocity(velocity.unaryMinus());

        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
