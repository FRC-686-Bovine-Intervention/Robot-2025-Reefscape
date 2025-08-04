package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedMotor;


public class IntakeIOFalcon implements IntakeIO {
    protected final TalonFX motor = HardwareDevices.intakeMotorID.talonFX();
    protected final DigitalInput coralSensor = HardwareDevices.coralSensor.input();
    protected final DigitalInput algaeSensor = HardwareDevices.algaeSensor.input();

    public IntakeIOFalcon(){
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
        ;
        motorConfig.CurrentLimits
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true)
        ;

        this.motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motor.getStatorCurrent());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), LoggedMotor.getStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        this.motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.motor.updateFrom(this.motor);
        // inputs.motorFaults.updateFrom(this.motor);

        inputs.coralSensor = this.coralSensor.get() ^ IntakeConstants.coralSensorInverted;
        inputs.algaeSensor = this.algaeSensor.get() ^ IntakeConstants.algaeSensorInverted;
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> voltage) {
        this.motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void clearMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.motor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.motor);
                }
            }
        }
    }
}
