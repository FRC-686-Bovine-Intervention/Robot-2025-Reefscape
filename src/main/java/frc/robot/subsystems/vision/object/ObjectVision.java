package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class ObjectVision {
    private final ObjectPipeline[] pipelines;

    private static final LoggedTunableMeasure<DistanceUnit> updateDistanceThreshold = new LoggedTunableMeasure<>("Vision/Bucket/Updating/Update Distance Threshold", Meters.of(5), Inches);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Bucket/Updating/Pos Updating Filtering Factor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Bucket/Confidence/Updating Filtering Factor", 0.5);
    private static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/Bucket/Confidence/Per Area Percent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/Bucket/Confidence/Decay Per Second", 3);
    private static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber("Vision/Bucket/Priority/Priority Per Confidence", 4);
    private static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber("Vision/Bucket/Priority/Priority Per Distance", -2);
    private static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber("Vision/Bucket/Target Threshold/Acquire", -2);
    private static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber("Vision/Bucket/Target Threshold/Detarget", -3);

    public ObjectVision(ObjectPipeline... pipelines) {
        System.out.println("[Init ObjectVision] Instantiating ObjectVision");
        this.pipelines = pipelines;
    }

    // @Override
    public void periodic() {
        Arrays.stream(this.pipelines)
            .flatMap(
                (pipeline) -> Arrays.stream(pipeline.getFrames()).flatMap((frame) -> Arrays.stream(frame.targets).map((target) -> {
                    var minX = target.corners[0].getX();
                    var maxX = target.corners[0].getX();
                    for (var corner : target.corners) {
                        minX = Math.min(minX, corner.getX());
                        maxX = Math.max(maxX, corner.getX());
                    }
                    var pixelWidth = maxX - minX;
                    return target;
                }))
            )
            // .sorted((a, b) -> Double.compare(a.timestamp, b.timestamp))
            .iterator()
        ;


        // var connections = new ArrayList<TargetMemoryConnection>(bucketMemories.size() * frameTargets.size());
        // bucketMemories.forEach(
        //     (memory) -> frameTargets.forEach(
        //         (target) -> {
        //             if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.in(Meters)) {
        //                 connections.add(new TargetMemoryConnection(memory, target));
        //             }
        //         }
        //     )
        // );
        // connections.sort((a, b) -> Double.compare(a.getDistance(), b.getDistance()));
        // var unusedMemories = new ArrayList<>(bucketMemories);
        // var unusedTargets = new ArrayList<>(frameTargets);
        // while(!connections.isEmpty()) {
        //     var confirmedConnection = connections.get(0);
        //     confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.cameraTarget);
        //     confirmedConnection.memory.updateConfidence();
        //     unusedMemories.remove(confirmedConnection.memory);
        //     unusedTargets.remove(confirmedConnection.cameraTarget);
        //     connections.removeIf((connection) -> 
        //         connection.memory == confirmedConnection.memory
        //         || connection.cameraTarget == confirmedConnection.cameraTarget
        //     );
        // }
        // unusedMemories.forEach((memory) -> {
        //     if(RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) > RobotConstants.robotLength.in(Meters)*0.5) {
        //         memory.decayConfidence(1);
        //     }
        // });
        // unusedTargets.forEach(bucketMemories::add);
        // bucketMemories.removeIf((memory) -> memory.confidence <= 0);
        // bucketMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()) || Double.isNaN(memory.fieldPos.getY()));
        // bucketMemories.removeIf((memory) -> RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) <= 0.07);

        // if(
        //     optIntakeTarget.isPresent()
        //     && (
        //         optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()
        //         || !bucketMemories.contains(optIntakeTarget.get())
        //     )
        // ) {
        //     optIntakeTarget = Optional.empty();
        // }
        // if(optIntakeTarget.isEmpty() || !intakeTargetLocked) {
        //     optIntakeTarget = bucketMemories
        //         .stream()
        //         .filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get())
        //         .sorted((a,b) -> Double.compare(b.getPriority(), a.getPriority()))
        //         .findFirst()
        //     ;
        // }
        
        // // Leds.getInstance().visionAcquired.setFlag(hasTarget());
        // // Leds.getInstance().visionLocked.setFlag(targetLocked());
        
        // Logger.recordOutput("Vision/Bucket/Bucket Memories", bucketMemories.stream().map(TrackedBucket::toASPose).toArray(Pose3d[]::new));
        // Logger.recordOutput("Vision/Bucket/Bucket Confidence", bucketMemories.stream().mapToDouble((note) -> note.confidence).toArray());
        // Logger.recordOutput("Vision/Bucket/Bucket Priority", bucketMemories.stream().mapToDouble(TrackedBucket::getPriority).toArray());
        // Logger.recordOutput("Vision/Bucket/Target", LoggerUtil.toArray(optIntakeTarget.map(TrackedBucket::toASPose), Pose3d[]::new));
        // Logger.recordOutput("Vision/Bucket/Locked Target", LoggerUtil.toArray(optIntakeTarget.filter((a) -> intakeTargetLocked).map(TrackedBucket::toASPose).map(Pose3d::getTranslation), Translation3d[]::new));
    }

    public static class TrackedObject {
        public Translation2d fieldTranslation;
        public double timestamp;
        public double confidence;
    }
}
