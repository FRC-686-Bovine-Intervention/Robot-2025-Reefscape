package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.VirtualSubsystem;
import frc.util.led.animation.AllianceColorAnimation;
import frc.util.led.animation.AutonomousFinishedAnimation;
import frc.util.led.animation.BarAnimation;
import frc.util.led.animation.FillAnimation;
import frc.util.led.animation.FlashingAnimation;
import frc.util.led.animation.StatusLightAnimation;
import frc.util.led.functions.Gradient;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.hardware.AddressableStrip;
import frc.util.led.strips.hardware.HardwareStrip;

public class Leds extends VirtualSubsystem {
    private static Leds instance;
    public static Leds getInstance() {if(instance == null) {instance = new Leds();} return instance;}
    
    private final HardwareStrip hardwareStrip;

    private final Notifier loadingNotifier;

    public Leds() {
        System.out.println("[Init Leds] Instantiating Leds");
        hardwareStrip = new AddressableStrip(HardwareDevices.ledPort, 29 + 22 + 29);

        var rawLeftStrip = hardwareStrip.substrip(0, 29);
        var rawBackStrip = hardwareStrip.substrip(29, 29+22);
        var rawRightStrip = hardwareStrip.substrip(29+22, 29+22+29);

        var rawBackLeftStrip = rawBackStrip.substrip(0, 11);
        var rawBackRightStrip = rawBackStrip.substrip(11, 22);

        var leftStrip = rawLeftStrip;
        var rightStrip = rawRightStrip.reverse();
        var backLeftStrip = rawBackLeftStrip;
        var backRightStrip = rawBackRightStrip.reverse();
        var backMirrorStrip = backLeftStrip.parallel(backRightStrip);

        var sideStrips = leftStrip.parallel(rightStrip);

        var fullLeftStrip = leftStrip.concat(backLeftStrip);
        var fullRightStrip = rightStrip.concat(backRightStrip);
        
        var fullSideStrips = fullLeftStrip.parallel(fullRightStrip);

        autonomousFinishedAnimation = new AutonomousFinishedAnimation(sideStrips, hardwareStrip);
        estopped = new FillAnimation(hardwareStrip, Color.kRed);
        allianceColorAnimation = new AllianceColorAnimation(fullSideStrips);
        driverStationConnection = new StatusLightAnimation(sideStrips.substrip(0, 2), Color.kOrange, Color.kGreen);
        flAprilConnection = new StatusLightAnimation(leftStrip.substrip(3, 4), Color.kOrange, Color.kGreen);
        blAprilConnection = new StatusLightAnimation(leftStrip.substrip(2, 3), Color.kOrange, Color.kGreen);
        frAprilConnection = new StatusLightAnimation(rightStrip.substrip(3, 4), Color.kOrange, Color.kGreen);
        brAprilConnection = new StatusLightAnimation(rightStrip.substrip(2, 3), Color.kOrange, Color.kGreen);
        questNavConnection = new StatusLightAnimation(sideStrips.substrip(4, 5), Color.kOrange, Color.kGreen);
        coralSecured = new FillAnimation(sideStrips.substrip(24, 29).concat(backMirrorStrip.substrip(8, 11)), Color.kGreen);
        coralAcquired = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(5), InterpolationFunction.step.gradient(Color.kBlack, Color.kGreen));
        algaeSecured = new FillAnimation(sideStrips.substrip(24, 29).concat(backMirrorStrip.substrip(8, 11)), Color.kAquamarine);
        algaeAcquired = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(5), InterpolationFunction.step.gradient(Color.kBlack, Color.kAquamarine));
        prepareClimbing = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(1), InterpolationFunction.linear.gradient(Color.kBlack, Color.kTeal));
        climbing = new BarAnimation(sideStrips.parallel(backMirrorStrip), InterpolationFunction.linear.gradient(Color.kBlack, Color.kTeal));
        climbingComplete = new FlashingAnimation(fullSideStrips, WaveFunction.Modulo.frequency(0.5), Gradient.rainbow);
        level4Targeted = new FillAnimation(fullSideStrips, Color.kGreenYellow);
        level3Targeted = new FillAnimation(fullSideStrips, Color.kAquamarine);
        level2Targeted = new FillAnimation(fullSideStrips, Color.kYellow);
        level1Targeted = new FillAnimation(fullSideStrips, Color.kRed);
        removeAlgae = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(2), InterpolationFunction.step.gradient(Color.kBlack, Color.kAquamarine));
        

        loadingNotifier = new Notifier(() -> {
            synchronized(this) {
                hardwareStrip.apply(
                    InterpolationFunction.linear.gradient(
                        Color.kBlack,
                        Color.kDimGray
                    )
                    .apply(
                        WaveFunction.Sinusoidal.applyAsDouble(
                            System.currentTimeMillis() / 1000.0
                        )
                    )
                );
                hardwareStrip.refresh();
            }
        });
        System.out.println("[Init Leds] Starting Loading Notifier");
        loadingNotifier.startPeriodic(RobotConstants.rioUpdatePeriodSecs);
    }

    public final AutonomousFinishedAnimation autonomousFinishedAnimation;
    public final FillAnimation estopped;
    public final AllianceColorAnimation allianceColorAnimation;
    public final StatusLightAnimation driverStationConnection;
    public final StatusLightAnimation flAprilConnection;
    public final StatusLightAnimation frAprilConnection;
    public final StatusLightAnimation blAprilConnection;
    public final StatusLightAnimation brAprilConnection;
    public final StatusLightAnimation questNavConnection;
    public final FlashingAnimation coralAcquired;
    public final FillAnimation coralSecured;
    public final FlashingAnimation algaeAcquired;
    public final FillAnimation algaeSecured;
    public final FlashingAnimation prepareClimbing;
    public final BarAnimation climbing;
    public final FlashingAnimation climbingComplete;
    public final FillAnimation level4Targeted;
    public final FillAnimation level3Targeted;
    public final FillAnimation level2Targeted;
    public final FillAnimation level1Targeted;
    public final FlashingAnimation removeAlgae;

    private int skippedFrames = 0;
    private static final int frameSkipAmount = 15;

    @Override
    public void periodic() {
        driverStationConnection.setStatus(DriverStation.isDSAttached());
        estopped.setFlag(DriverStation.isEStopped());
    }

    @Override
    public synchronized void postCommandPeriodic() {
        if (skippedFrames < frameSkipAmount) {
            skippedFrames++;
            return;
        }
        loadingNotifier.stop();

        // Default alliance color scrolling
        allianceColorAnimation.apply();

        if (DriverStation.isDisabled()) {
            driverStationConnection.apply();
            flAprilConnection.apply();
            frAprilConnection.apply();
            blAprilConnection.apply();
            brAprilConnection.apply();
            questNavConnection.apply();
        }

        level1Targeted.applyIfFlagged();
        level2Targeted.applyIfFlagged();
        level3Targeted.applyIfFlagged();
        level4Targeted.applyIfFlagged();
        removeAlgae.applyIfFlagged();

        coralSecured.applyIfFlagged();
        coralAcquired.applyIfFlagged();
        algaeSecured.applyIfFlagged();
        algaeAcquired.applyIfFlagged();

        prepareClimbing.applyIfFlagged();
        climbing.applyIfFlagged();
        climbingComplete.applyIfFlagged();

        autonomousFinishedAnimation.applyIfFlagged();

        estopped.applyIfFlagged();

        //TODO: End game notification
        hardwareStrip.refresh();
    }
}
