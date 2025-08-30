package frc.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import frc.util.flipping.AllianceFlipUtil;

public class Perspective {
    protected Rotation2d forwardDirection;
    protected Matrix<N2, N2> perspectiveToField;
    protected Matrix<N2, N2> fieldToPerspective;

    private Perspective(Rotation2d forwardDirection) {
        this.forwardDirection = forwardDirection;
        this.perspectiveToField = this.forwardDirection.toMatrix();
        this.fieldToPerspective = this.perspectiveToField.inv();
    }

    public Rotation2d getForwardDirection() {
        return this.forwardDirection;
    }

    public Vector<N2> toField(Vector<N2> perspectiveVector) {
        return (Vector<N2>) this.perspectiveToField.times(perspectiveVector);
    }

    public Vector<N2> toPerspective(Vector<N2> fieldVector) {
        return (Vector<N2>) this.fieldToPerspective.times(fieldVector);
    }

    private static final Perspective posX = new Perspective(Rotation2d.kZero);
    private static final Perspective negX = new Perspective(Rotation2d.k180deg);
    private static final Perspective posY = new Perspective(Rotation2d.kCCW_90deg);
    private static final Perspective negY = new Perspective(Rotation2d.kCW_90deg);
    private static final Perspective custom = new Perspective(Rotation2d.kZero) {
        private final LoggedNetworkNumber customDegrees = new LoggedNetworkNumber("SmartDashboard/Perspective/Custom", 0.0);

        private boolean hasChanged() {
            return this.customDegrees.get() != this.forwardDirection.getDegrees();
        }

        private void setPerspectiveDegs(double degrees) {
            this.forwardDirection = Rotation2d.fromDegrees(degrees);
            this.perspectiveToField = this.forwardDirection.toMatrix();
            this.fieldToPerspective = this.perspectiveToField.inv();
        }

        private void updateIfChanged() {
            if (this.hasChanged()) {
                this.setPerspectiveDegs(this.customDegrees.get());
            }
        }

        @Override
        public Rotation2d getForwardDirection() {
            this.updateIfChanged();
            return super.getForwardDirection();
        }

        @Override
        public Vector<N2> toField(Vector<N2> vector) {
            this.updateIfChanged();
            return super.toField(vector);
        }

        @Override
        public Vector<N2> toPerspective(Vector<N2> fieldVector) {
            this.updateIfChanged();
            return super.toPerspective(fieldVector);
        }
    };

    private static final LoggedDashboardChooser<Perspective> chooser;

    static {
        chooser = new LoggedDashboardChooser<>("Perspective/Chooser");
        chooser.addOption("Blue Alliance (+X)", posX);
        chooser.addOption("Red Alliance (-X)", negX);
        chooser.addDefaultOption("Blue Left (+Y)", posY);
        chooser.addOption("Red Left (-Y)", negY);
        chooser.addOption("Custom", custom);
    }

    public static Perspective getAlliance() {
        return AllianceFlipUtil.shouldFlip() ? negX : posX;
    }

    public static Perspective getCurrent() {
        return chooser.get();
    }
}
