package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
    private final DCMotor driveMotorModel = DCMotor.getFalcon500(1);
    private final DCMotor azimuthMotorModel = DCMotor.getNeo550(1);

    private final DCMotorSim driveSims = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(this.driveMotorModel, 0.025, DriveConstants.driveMotorToWheelRatio.reductionUnsigned()),
        this.azimuthMotorModel
    );
    private final DCMotorSim azimuthSims = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(this.azimuthMotorModel, 0.004, DriveConstants.azimuthMotorToCarriageRatio.reductionUnsigned()),
        this.azimuthMotorModel
    );

    public ModuleIOSim(ModuleConstants moduleConstants) {
        super(moduleConstants);
    }

    private final MutVoltage azimuthAppliedVolts = Volts.mutable(0);

    public void updateInputs(ModuleIOInputs inputs) {
        var driveSimState = this.driveMotor.getSimState();
        if (DriverStation.isDisabled()) {
            this.azimuthAppliedVolts.mut_setBaseUnitMagnitude(0);
        }
        this.driveSims.setInputVoltage(driveSimState.getMotorVoltage());
        this.azimuthSims.setInputVoltage(this.azimuthAppliedVolts.in(Volts));
        
        this.driveSims.update(RobotConstants.rioUpdatePeriodSecs);
        this.azimuthSims.update(RobotConstants.rioUpdatePeriodSecs);

        var wheelAngle = this.driveSims.getAngularPosition();
        var wheelVelocity = this.driveSims.getAngularVelocity();
        driveSimState.setRawRotorPosition(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelAngle));
        driveSimState.setRotorVelocity(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelVelocity));
        driveSimState.setSupplyVoltage(12 - driveSimState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);

        var carriageAngle = this.azimuthSims.getAngularPosition();
        var carriageVelocity = this.azimuthSims.getAngularVelocity();
        inputs.azimuthEncoder.position.mut_replace(DriveConstants.azimuthEncoderToCarriageRatio.inverse().applyUnsigned(carriageAngle));
        inputs.azimuthEncoder.velocity.mut_replace(DriveConstants.azimuthEncoderToCarriageRatio.inverse().applyUnsigned(carriageVelocity));
        inputs.azimuthMotor.encoder.position.mut_replace(DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageAngle));
        inputs.azimuthMotor.encoder.velocity.mut_replace(DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageVelocity));

        inputs.odometryDriveRads = new double[] {inputs.driveMotor.encoder.position.in(Radians)};
        inputs.odometryAzimuthRads = new double[] {inputs.azimuthEncoder.position.in(Radians)};
    }
    
    @Override
    public void setAzimuthVolts(double volts) {
        this.azimuthAppliedVolts.mut_replace(MathUtil.clamp(volts, -12, 12), Volts);
    }
    @Override
    public void setAzimuthAngle(Measure<AngleUnit> angle) {
        this.setAzimuthVolts(
            this.azimuthPID.calculate(
                this.azimuthSims.getAngularPosition().in(Rotations),
                angle.in(Rotations)
            )
        );
    }
}
