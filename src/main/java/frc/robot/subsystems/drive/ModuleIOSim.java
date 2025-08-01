package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private final FlywheelSim driveSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.0025 / DriveConstants.driveRatio.reductionUnsigned() / DriveConstants.driveRatio.reductionUnsigned(), 1),
        DCMotor.getFalcon500(1)
    );
    private final FlywheelSim azimuthSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.004, DriveConstants.azimuthRatio.reductionUnsigned()),
        DCMotor.getFalcon500(1)
    );

    public ModuleIOSim(ModuleConstants moduleConstants) {
        super(moduleConstants);
    }

    private final MutAngle driveRelativePosition = Radians.mutable(0);
    private final MutAngle azimuthAbsolutePosition = Radians.mutable(0);
    private final MutVoltage azimuthAppliedVolts = Volts.mutable(0);

    public void updateInputs(ModuleIOInputs inputs) {
        var driveSimState = driveMotor.getSimState();
        if (DriverStation.isDisabled()) {
            azimuthAppliedVolts.mut_setBaseUnitMagnitude(0);
        }
        driveSim.setInputVoltage(driveSimState.getMotorVoltage());
        azimuthSim.setInputVoltage(azimuthAppliedVolts.in(Volts));
        
        driveSim.update(RobotConstants.rioUpdatePeriodSecs);
        azimuthSim.update(RobotConstants.rioUpdatePeriodSecs);

        var angleDiff = azimuthSim.getAngularVelocity().times(RobotConstants.rioUpdatePeriod);
        azimuthAbsolutePosition.mut_acc(angleDiff);
        azimuthAbsolutePosition.mut_setMagnitude(MathUtil.angleModulus(azimuthAbsolutePosition.in(Radians)));

        var driveAngularDiff = driveSim.getAngularVelocity().times(RobotConstants.rioUpdatePeriod);
        driveRelativePosition.mut_acc(driveAngularDiff);
        driveSimState.setRawRotorPosition(driveRelativePosition);
        driveSimState.setRotorVelocity(driveSim.getAngularVelocity());
        driveSimState.setSupplyVoltage(12 - driveSimState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);

        inputs.azimuthMotor.updateFrom(azimuthSim, azimuthAppliedVolts);
        inputs.azimuthMotor.encoder.position.mut_replace(azimuthAbsolutePosition);
        inputs.azimuthMotor.encoder.velocity.mut_replace(azimuthSim.getAngularVelocity());
    }
    
    @Override
    public void setAzimuthVolts(double volts) {
        azimuthAppliedVolts.mut_replace(MathUtil.clamp(volts, -12, 12), Volts);
    }
    @Override
    public void setAzimuthAngle(Measure<AngleUnit> angle) {
        setAzimuthVolts(
            azimuthPID.calculate(
                azimuthAbsolutePosition.in(Rotations),
                angle.in(Rotations)
            )
        );
    }
}
