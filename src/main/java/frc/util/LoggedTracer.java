package frc.util;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class LoggedTracer {
    /*
        * reset
        * ObjectiveTracker/UpdateCoral/GrabData    0
        * ObjectiveTracker/UpdateCoral/Process     2
        * ObjectiveTracker/UpdateCoral             2
        * ObjectiveTracker/UpdateAlgae/GrabData    1
        * ObjectiveTracker/UpdateAlgae/Process     2
        * ObjectiveTracker/UpdateAlgae             2
        * ObjectiveTracker                         1
        * GameState/UpdateTimestamps               0
        * GameState                                1
     */

    // private static final ArrayList<Double> startTimes = new ArrayList<>(8);
    private static double[] startTimes = new double[1];
    private static String[] lastEpochPath = new String[0];

    public static void reset() {
        var now = Timer.getFPGATimestamp();
        Arrays.fill(startTimes, now);
        lastEpochPath = new String[0];
    }

    public static void logEpoch(String epochName) {
        var now = Timer.getFPGATimestamp();
        var epochPath = epochName.split("/");

        if (epochPath.length > startTimes.length) {
            var newArray = Arrays.copyOf(startTimes, epochPath.length);
            for (int i = startTimes.length; i < newArray.length; i++) {
                newArray[i] = startTimes[startTimes.length - 1];
            }
            startTimes = newArray;
        }

        // if (epochPath.length > lastEpochPath.length) {

        // }

        var lastTime = startTimes[epochPath.length - 1];
        for (int i = epochPath.length - 1; i < startTimes.length; i++) {
            startTimes[i] = now;
        }

        Logger.recordOutput("LoggedTracer/" + epochName, (now - lastTime) * 1000.0);

        // var mismatch = Arrays.mismatch(epochPath, lastEpochPath);
        // if (mismatch < 0) return;
        // if (mismatch >= epochPath.length) {

        // } else {
        //     for (int i = mismatch; i < epochPath.length; i++) {
                
        //     }
        // }

        lastEpochPath = epochPath;
    }
}
