package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

public class IntakeIOSim extends IntakeIOFalcon {
    private final BooleanSupplier coralSim;
    private final BooleanSupplier algaeSim;

    public IntakeIOSim(BooleanSupplier coralSim, BooleanSupplier algaeSim) {
        this.coralSim = coralSim;
        this.algaeSim = algaeSim;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // super.updateInputs(inputs);
        inputs.coralSensor = coralSim.getAsBoolean();
        inputs.algaeSensor = algaeSim.getAsBoolean();
    }
}
