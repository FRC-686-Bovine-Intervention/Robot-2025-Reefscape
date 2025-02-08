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

public class ScoreCoral extends AutoRoutine {
    // scoring preload (reef branches)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef branches - 1)
    // scoring coral 2 (1/2 reef branches - 2)
    // which part of the coral station (close, mid, far)

    private static final Map.Entry<String, Branch> branchA = Settings.option("Branch A", FieldConstants.Reef.branches[0]); 
    private static final Map.Entry<String, Branch> branchB = Settings.option("Branch B", FieldConstants.Reef.branches[1]); 
    private static final Map.Entry<String, Branch> branchC = Settings.option("Branch C", FieldConstants.Reef.branches[2]); 
    private static final Map.Entry<String, Branch> branchD = Settings.option("Branch D", FieldConstants.Reef.branches[3]); 
    private static final Map.Entry<String, Branch> branchE = Settings.option("Branch E", FieldConstants.Reef.branches[4]); 
    private static final Map.Entry<String, Branch> branchF = Settings.option("Branch F", FieldConstants.Reef.branches[5]); 
    private static final Map.Entry<String, Branch> branchG = Settings.option("Branch G", FieldConstants.Reef.branches[6]); 
    private static final Map.Entry<String, Branch> branchH = Settings.option("Branch H", FieldConstants.Reef.branches[7]); 
    private static final Map.Entry<String, Branch> branchI = Settings.option("Branch I", FieldConstants.Reef.branches[8]); 
    private static final Map.Entry<String, Branch> branchJ = Settings.option("Branch J", FieldConstants.Reef.branches[9]); 
    private static final Map.Entry<String, Branch> branchK = Settings.option("Branch K", FieldConstants.Reef.branches[10]); 
    private static final Map.Entry<String, Branch> branchL = Settings.option("Branch L", FieldConstants.Reef.branches[11]); 
    
    private static boolean isRightCoralStation(Branch branch){
        return MathExtraUtil.isWithin(branch.getIndex(), 1, 6);
    }

    private static final AutoQuestion<Branch> scorePreloadBranch = new AutoQuestion<Branch>("Score Preload Branch") {
        @Override
        protected Settings<Branch> generateSettings() {
            return Settings.from(
                branchA,
                branchA, branchB, branchC, branchD, branchE, branchF, branchG, branchH, branchI, branchJ, branchK, branchL
            );
        }

    };

    private static final AutoQuestion<Branch> scoreCoral1 = new AutoQuestion<Branch>("Score Second Branch") {
        @Override
        protected Settings<Branch> generateSettings() {
            return (isRightCoralStation(scorePreloadBranch.getResponse())) ? (
                Settings.from(branchB, 
                branchB, branchC, branchD, branchE, branchF, branchG)
            ) : (
                Settings.from(branchA, 
                branchA, branchH, branchI, branchJ, branchK, branchL)
            );
        }
    };

    private static final AutoQuestion<Branch> scoreCoral2 = new AutoQuestion<Branch>("Score Third Branch") {
        @Override
        protected Settings<Branch> generateSettings() {
            return (isRightCoralStation(scorePreloadBranch.getResponse())) ? (
                Settings.from(branchB, 
                branchB, branchC, branchD, branchE, branchF, branchG)
            ) : (
                Settings.from(branchA, 
                branchA, branchH, branchI, branchJ, branchK, branchL)
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
            switch(scorePreloadBranch.getResponse().getIndex()){
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
        super("ScoreCoral", List.of(scorePreloadBranch, startPosition, scoreCoral1, scoreCoral2, stationPosition));
    }

    @Override
    public Command generateCommand() {
        return Commands.none();
    }
}
