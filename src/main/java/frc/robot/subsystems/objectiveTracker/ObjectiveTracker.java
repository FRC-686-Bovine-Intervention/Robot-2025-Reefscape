package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Branch;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.robot.constants.FieldConstants.Reef.StagedAlgae;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.wrist.WristConstants;
import frc.util.VirtualSubsystem;
import frc.util.robotStructure.Root;
import frc.util.robotStructure.angle.ArmMech;
import frc.util.robotStructure.linear.ExtenderMech;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ObjectiveSelectorIO io;
    private final ObjectiveSelectorIOInputsAutoLogged inputs =
        new ObjectiveSelectorIOInputsAutoLogged();

    public enum AlgaeGoal {
        NET,
        PROCESSOR,
        OPPONENT_PROCESSOR,
        ;
    }

    private final ArrayList<Branch> placedCoral = new ArrayList<>(36);
    private Branch selectedCoral = FieldConstants.Reef.branches[0];
    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;
    private Optional<StagedAlgae> selectedIntakeGoal = Optional.empty();

    private final Root structureRoot = new Root();
    private final ArmMech pivotMech = new ArmMech(PivotConstants.pivotBase);
    private final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
    private final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
    private final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);
    private final ArmMech wristMech = new ArmMech(WristConstants.wristBase);

    public ObjectiveTracker(ObjectiveSelectorIO io) {
        System.out.println("[Init] Instantiating ObjectiveTracker");
        this.io = io;

        structureRoot
            .addChild(pivotMech
                .addChild(stage2Mech
                    .addChild(stage3Mech
                        .addChild(stage4Mech
                            .addChild(wristMech)
                        )
                    )
                )
            )
        ;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Objective Tracker", inputs);

        if (inputs.coral != -1) {
            selectedCoral = FieldConstants.Reef.branches[inputs.coral];
            inputs.coral = -1;
        }
        if (inputs.algae != -1) {
            selectedAlgaeGoal = AlgaeGoal.values()[inputs.algae];
            inputs.algae = -1;
        }
        if (inputs.intake != -1) {
            selectedIntakeGoal = (inputs.intake > 0) ? (
                Optional.of(FieldConstants.Reef.stagedAlgae[inputs.intake - 1])
            ) : (
                Optional.empty()
            );
            inputs.intake = -1;
        }

        io.setCoral(selectedCoral.getIndex());
        io.setAlgae(selectedAlgaeGoal.ordinal());
        io.setIntake(selectedIntakeGoal.isEmpty() ? 0 : selectedIntakeGoal.get().getIndex() + 1);
        
        Logger.recordOutput("Objective Tracker/Selected Branch", selectedCoral.branchPose.getOurs());
        structureRoot.setPose(selectedCoral.robotPose.getOurs());
        var setpointState = SuperstructureState.fromRobotSpace(selectedCoral.level.forwardBranchRobotSpace.transformBy(Superstructure.forwardCoralTransform));
        pivotMech.set(setpointState.pivotAngle);
        stage2Mech.set(setpointState.elevatorLength.div(ElevatorConstants.movingStages));
        stage3Mech.set(setpointState.elevatorLength.div(ElevatorConstants.movingStages));
        stage4Mech.set(setpointState.elevatorLength.div(ElevatorConstants.movingStages));
        wristMech.set(setpointState.wristAngle);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Robot", structureRoot.getFieldRelative());
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Mechs",
            pivotMech.getRobotRelative(),
            stage2Mech.getRobotRelative(),
            stage3Mech.getRobotRelative(),
            stage4Mech.getRobotRelative(),
            wristMech.getRobotRelative()
        );
    }

    public void moveSelectedCoral(int x, int y) {
        var horiz = Math.floorMod(((selectedCoral.rack.ordinal() * Side.values().length) + selectedCoral.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedCoral.level.ordinal() + y), Level.values().length);
        selectedCoral = FieldConstants.Reef.getBranch(Rack.values()[horiz / Side.values().length], Level.values()[height], Side.values()[Math.floorMod(horiz, Side.values().length)]);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedCoral)) {
            placedCoral.add(selectedCoral);
        }
    }

    public Branch getSelectedBranch() {
        return selectedCoral;
    }

    public boolean intakeFromCoralStation() {
        return selectedIntakeGoal.isEmpty();
    }

    public Optional<StagedAlgae> getSelectedStagedAlgae() {
        return selectedIntakeGoal;
    }

    public AlgaeGoal getAlgaeGoal() {
        return selectedAlgaeGoal;
    }
}