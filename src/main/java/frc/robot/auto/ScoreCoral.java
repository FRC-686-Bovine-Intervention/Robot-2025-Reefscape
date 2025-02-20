package frc.robot.auto;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.Branch;
import frc.util.misc.MathExtraUtil;
import frc.robot.constants.FieldConstants.Reef.Pipe;

public class ScoreCoral extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)

    private static final Map.Entry<String, Pipe> pipeA = Settings.option("Pipe A", FieldConstants.Reef.pipes[0]); 
    private static final Map.Entry<String, Pipe> pipeB = Settings.option("Pipe B", FieldConstants.Reef.pipes[1]); 
    private static final Map.Entry<String, Pipe> pipeC = Settings.option("Pipe C", FieldConstants.Reef.pipes[2]); 
    private static final Map.Entry<String, Pipe> pipeD = Settings.option("Pipe D", FieldConstants.Reef.pipes[3]); 
    private static final Map.Entry<String, Pipe> pipeE = Settings.option("Pipe E", FieldConstants.Reef.pipes[4]); 
    private static final Map.Entry<String, Pipe> pipeF = Settings.option("Pipe F", FieldConstants.Reef.pipes[5]); 
    private static final Map.Entry<String, Pipe> pipeG = Settings.option("Pipe G", FieldConstants.Reef.pipes[6]); 
    private static final Map.Entry<String, Pipe> pipeH = Settings.option("Pipe H", FieldConstants.Reef.pipes[7]); 
    private static final Map.Entry<String, Pipe> pipeI = Settings.option("Pipe I", FieldConstants.Reef.pipes[8]); 
    private static final Map.Entry<String, Pipe> pipeJ = Settings.option("Pipe J", FieldConstants.Reef.pipes[9]); 
    private static final Map.Entry<String, Pipe> pipeK = Settings.option("Pipe K", FieldConstants.Reef.pipes[10]); 
    private static final Map.Entry<String, Pipe> pipeL = Settings.option("Pipe L", FieldConstants.Reef.pipes[11]); 
    
    private static boolean isRightCoralStation(Pipe pipe){
        return MathExtraUtil.isWithin(pipe.getIndex(), 1, 6);
    }

    private static final AutoQuestion<Pipe> scorePreloadPipe = new AutoQuestion<Pipe>("Score Preload Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            return Settings.from(
                pipeA,
                pipeA, pipeB, pipeC, pipeD, pipeE, pipeF, pipeG, pipeH, pipeI, pipeJ, pipeK, pipeL
            );
        }

    };

    private static final AutoQuestion<Pipe> scoreCoral1 = new AutoQuestion<Pipe>("Score Second Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            return (isRightCoralStation(scorePreloadPipe.getResponse())) ? (
                Settings.from(pipeB, 
                pipeB, pipeC, pipeD, pipeE, pipeF, pipeG)
            ) : (
                Settings.from(pipeA, 
                pipeA, pipeH, pipeI, pipeJ, pipeK, pipeL)
            );
        }
    };

    private static final AutoQuestion<Pipe> scoreCoral2 = new AutoQuestion<Pipe>("Score Third Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            return (isRightCoralStation(scorePreloadPipe.getResponse())) ? (
                Settings.from(pipeB, 
                pipeB, pipeC, pipeD, pipeE, pipeF, pipeG)
            ) : (
                Settings.from(pipeA, 
                pipeA, pipeH, pipeI, pipeJ, pipeK, pipeL)
            );
        }
    };

    private enum CoralStationPosition{
        CLOSE,
        MID,
        FAR
    }
    private static final AutoQuestion<CoralStationPosition> stationPosition = new AutoQuestion<CoralStationPosition>("Coral Station Position") {
        private static final Map.Entry<String, CoralStationPosition> stationFar = Settings.option("Far", CoralStationPosition.FAR);
        private static final Map.Entry<String, CoralStationPosition> stationMid = Settings.option("Mid", CoralStationPosition.MID);
        private static final Map.Entry<String, CoralStationPosition> stationClose = Settings.option("Close", CoralStationPosition.CLOSE);
        
        @Override
        protected Settings<CoralStationPosition> generateSettings() {
            return Settings.from(stationClose, stationClose, stationMid, stationFar);
        }
    };

    private enum StartPosition{
        CLOSE,
        REMOTE
    }
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<StartPosition>("Starting Position") {
        private static final Map.Entry<String, StartPosition> startRemoteLeft = Settings.option("Remote (Left)", StartPosition.REMOTE);
        private static final Map.Entry<String, StartPosition> startRemoteRight = Settings.option("Remote (Right)", StartPosition.REMOTE);
        private static final Map.Entry<String, StartPosition> startFarLeft = Settings.option("Close (Far Left)", StartPosition.CLOSE);
        private static final Map.Entry<String, StartPosition> startFarRight = Settings.option("Close (Far Right)", StartPosition.CLOSE);
        private static final Map.Entry<String, StartPosition> startLeftCage = Settings.option("Close (Left Cage)", StartPosition.CLOSE);
        private static final Map.Entry<String, StartPosition> startRightCage = Settings.option("Close(Right Cage)", StartPosition.CLOSE);
        private static final Map.Entry<String, StartPosition> startLeftCenter = Settings.option("Close (Left Center)", StartPosition.CLOSE);
        private static final Map.Entry<String, StartPosition> startRightCenter = Settings.option("Close (Right Center)", StartPosition.CLOSE);
        
        @Override
        protected Settings<StartPosition> generateSettings() {
            switch(scorePreloadPipe.getResponse().getIndex()){
                case 0:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                case 1:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 2:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 3:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 4:
                return Settings.from(startFarRight, startFarRight);
                case 5:
                return Settings.from(startRightCage, startRightCage);
                case 6:
                return Settings.from(startRightCenter, startRightCenter);
                case 7:
                return Settings.from(startLeftCenter, startLeftCenter);
                case 8:
                return Settings.from(startLeftCage, startLeftCage);
                case 9:
                return Settings.from(startFarLeft, startFarLeft);
                case 10:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                case 11:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                default:
                return null;
            }
        }
    };

    public ScoreCoral(RobotContainer robot) {
        super("ScoreCoral", List.of(scorePreloadPipe, startPosition, scoreCoral1, scoreCoral2, stationPosition));
    }

    @Override
    public Command generateCommand() {
        return Commands.none();
    }
}
