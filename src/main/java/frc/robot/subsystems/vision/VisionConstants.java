package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.robotStructure.CameraMount;

public final class VisionConstants {
    public static class CameraConstants {
        public final String hardwareName;
        public final CameraMount mount;

        public CameraConstants(String hardwareName, CameraMount mount) {
            this.hardwareName = hardwareName;
            this.mount = mount;
        }
    }

    public static final CameraMount frontLeftMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.083008),
            Meters.of(+0.244626),
            Meters.of(+0.396240)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(-7.5)
        )
        .rotateBy(
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(+0)
            )
        )
    ));
    public static final CameraMount frontRightMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.083008),
            Meters.of(-0.244626),
            Meters.of(+0.396240)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(+7.5)
        )
        .rotateBy(
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(+0)
            )
        )
    ));
    public static final CameraMount backLeftMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.175207),
            Meters.of(+0.254154),
            Meters.of(+0.403667)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(+175)
        )
        .rotateBy(
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(-15),
                Degrees.of(+0)
            )
        )
    ));
    public static final CameraMount backRightMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.175207),
            Meters.of(-0.254154),
            Meters.of(+0.403667)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(-175)
        )
        .rotateBy(
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(-15),
                Degrees.of(+0)
            )
        )
    ));
    public static final CameraMount driveCamMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.219548),
            Meters.of(-0.195904),
            Meters.of(+1.147068)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(+10)
        )
        .rotateBy(new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+30),
            Degrees.of(+0)
        ))
    ));
}
