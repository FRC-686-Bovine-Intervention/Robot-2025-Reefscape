package frc.util.robotStructure;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class GamepiecePose extends ChildBase {
    public static final String KEY = "Gamepieces";

    private static GamepiecePose[] gamepiecePoses;
    public static void registerMechs(GamepiecePose... gamepieces) {
        gamepiecePoses = gamepieces;
    }
    public static void logAscopeAxes() {
        Logger.recordOutput(KEY + "/Axes", Arrays.stream(gamepiecePoses).map(GamepiecePose::getFieldRelative).toArray(Pose3d[]::new));
    }

    public GamepiecePose(Transform3d base) {
        super(base);
    }
}
