package frc.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.util.flipping.AllianceFlipUtil;

public class Perspective {
    protected Matrix<N2, N2> perspectiveToField;
    protected Matrix<N2, N2> fieldToPerspective;

    private Perspective(Matrix<N2, N2> perspectiveToField) {
        this.perspectiveToField = perspectiveToField;
        this.fieldToPerspective = this.perspectiveToField.inv();
    }

    public Vector<N2> toField(Vector<N2> perspectiveVector) {
        return (Vector<N2>) this.perspectiveToField.times(perspectiveVector);
    }

    public Vector<N2> toPerspective(Vector<N2> fieldVector) {
        return (Vector<N2>) this.fieldToPerspective.times(fieldVector);
    }

    private static final Perspective posX = new Perspective(Rotation2d.kZero.toMatrix());
    private static final Perspective negX = new Perspective(Rotation2d.k180deg.toMatrix());
    private static final Perspective posY = new Perspective(Rotation2d.kCCW_90deg.toMatrix());
    private static final Perspective negY = new Perspective(Rotation2d.kCW_90deg.toMatrix());
    private static final Perspective custom = new Perspective(Rotation2d.kZero.toMatrix()) {
        private final LoggedNetworkNumber customDegrees = new LoggedNetworkNumber("Perspective/Custom", 0.0);
        private double lastDegrees = this.customDegrees.get();

        private boolean hasChanged() {
            return this.customDegrees.get() != this.lastDegrees;
        }

        private void setPerspectiveDegs(double degrees) {
            var rads = Units.degreesToRadians(this.customDegrees.get());
            var cos = Math.cos(rads);
            var sin = Math.sin(rads);
            this.perspectiveToField = MatBuilder.fill(Nat.N2(), Nat.N2(),
                +cos, -sin,
                +sin, +cos
            );
            this.fieldToPerspective = this.perspectiveToField.inv();
            this.lastDegrees = degrees;
        }

        private void updateIfChanged() {
            if (this.hasChanged()) {
                this.setPerspectiveDegs(lastDegrees);
            }
        }

        @Override
        public Vector<N2> toField(Vector<N2> vector) {
            this.updateIfChanged();
            return super.toField(vector);
        }
    };

    private static final LoggedDashboardChooser<Perspective> chooser;

    static {
        chooser = new LoggedDashboardChooser<>("Perspective/Chooser");
        chooser.addDefaultOption("Blue Alliance (+X)", posX);
        chooser.addOption("Blue Alliance (+X)", negX);
        chooser.addOption("Blue Alliance (+X)", posY);
        chooser.addOption("Blue Alliance (+X)", negY);
        chooser.addOption("Custom", custom);
    }

    public static Perspective getAlliance() {
        return AllianceFlipUtil.shouldFlip() ? negX : posX;
    }

    public static Perspective getCurrent() {
        return chooser.get();
    }
}
