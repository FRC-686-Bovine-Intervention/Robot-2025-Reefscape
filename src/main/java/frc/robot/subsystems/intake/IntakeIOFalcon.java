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
import frc.robot.subsystems.drive.DriveConstants;


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

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            motor.getStatorCurrent()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency.div(2),
            motor.getMotorVoltage(),
            motor.getStatorCurrent(),
            motor.getDeviceTemp()
        );
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.motor.updateFrom(motor);

        inputs.coralSensor = coralSensor.get() ^ IntakeConstants.coralSensorInverted;
        inputs.algaeSensor = algaeSensor.get() ^ IntakeConstants.algaeSensorInverted;
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }
}
