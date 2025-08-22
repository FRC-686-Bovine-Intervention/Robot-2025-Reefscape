package frc.util.loggerUtil.inputs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import java.nio.ByteBuffer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedMotor implements StructSerializable {
    public final MutVoltage appliedVoltage = Volts.mutable(0);
    public final MutCurrent statorCurrent = Amps.mutable(0);
    public final MutTemperature deviceTemperature = Celsius.mutable(0);

    public static record MotorStatusSignalCache(
        StatusSignal<Voltage> appliedVoltage,
        StatusSignal<Current> statorCurrent,
        StatusSignal<Temperature> deviceTemperature
    ) {
        public static MotorStatusSignalCache from(TalonFX talonFX) {
            return new MotorStatusSignalCache(talonFX.getMotorVoltage(), talonFX.getStatorCurrent(), talonFX.getDeviceTemp());
        }
        public static MotorStatusSignalCache from(TalonFXS talonFXS) {
            return new MotorStatusSignalCache(talonFXS.getMotorVoltage(), talonFXS.getStatorCurrent(), talonFXS.getDeviceTemp());
        }

        public BaseStatusSignal[] getStatusSignals() {
            return new BaseStatusSignal[] {
                this.appliedVoltage(),
                this.statorCurrent(),
                this.deviceTemperature(),
            };
        }
    }

    public void updateFrom(MotorStatusSignalCache statusSignals) {
        this.appliedVoltage.mut_replace(statusSignals.appliedVoltage().getValue());
        this.statorCurrent.mut_replace(statusSignals.statorCurrent().getValue());
        this.deviceTemperature.mut_replace(statusSignals.deviceTemperature().getValue());
    }
    
    public void updateFrom(TalonSRX talonSRX) {
        this.appliedVoltage.mut_replace(talonSRX.getMotorOutputVoltage(), Volts);
        this.statorCurrent.mut_replace(talonSRX.getStatorCurrent(), Amps);
        this.deviceTemperature.mut_replace(talonSRX.getTemperature(), Celsius);
    }

    public void updateFrom(SparkMax spark) {
        this.appliedVoltage.mut_replace(spark.getAppliedOutput() * 12, Volts);
        this.statorCurrent.mut_replace(spark.getOutputCurrent(), Amps);
    }

    public void updateFrom(DCMotorSim sim) {
        this.statorCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(DCMotorSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public void updateFrom(FlywheelSim sim) {
        this.statorCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(FlywheelSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.statorCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(SingleJointedArmSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public static final LoggedMotorStruct struct = new LoggedMotorStruct();

    public static class LoggedMotorStruct implements Struct<LoggedMotor> {
        @Override
        public Class<LoggedMotor> getTypeClass() {
            return LoggedMotor.class;
        }

        @Override
        public String getTypeName() {
            return "Motor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double AppliedVolts;double CurrentAmps;double TempKelvin";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var motor = new LoggedMotor();
            motor.appliedVoltage.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.statorCurrent.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.deviceTemperature.mut_setBaseUnitMagnitude(bb.getDouble());
            return motor;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.appliedVoltage.baseUnitMagnitude());
            bb.putDouble(value.statorCurrent.baseUnitMagnitude());
            bb.putDouble(value.deviceTemperature.baseUnitMagnitude());
        }
    }
}
