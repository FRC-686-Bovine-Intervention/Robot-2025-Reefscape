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
import frc.util.led.animation.WaveAnimation;
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

        var bottomSideStrip = sideStrips.substrip(0, 14);
        var topSideStrip = fullSideStrips.substrip(14, 40);

        var gamepieceSecuredStrip = sideStrips.substrip(24, 29).concat(backMirrorStrip.substrip(8, 11));

        var coralColor = Color.kGreen;
        var algaeColor = new Color(0,0.5,0.1);
        var climbingColor = Color.kTeal;
        var level4Color = Color.kGreen;
        var level3Color = algaeColor;
        var level2Color = Color.kYellow;
        var level1Color = Color.kRed;

        autonomousRunningAnimation = new WaveAnimation(fullSideStrips, (time, pos) -> WaveFunction.Sawtooth.applyAsDouble((time * 4) - (pos * 4)), InterpolationFunction.step.gradient(new Color(0,0,0.2), new Color(0.2,0.2,0)));
        autonomousFinishedAnimation = new AutonomousFinishedAnimation(sideStrips, hardwareStrip);
        estopped = new FillAnimation(hardwareStrip, Color.kRed);
        tipped = new FillAnimation(hardwareStrip, Color.kWhite);
        allianceColorAnimation = new AllianceColorAnimation(fullSideStrips, Color.kFirstBlue, Color.kRed);
        driverStationConnection = new StatusLightAnimation(sideStrips.substrip(0, 2), Color.kOrange, Color.kGreen);
        flAprilConnection = new StatusLightAnimation(sideStrips.substrip(2, 3), Color.kOrange, Color.kGreen);
        frAprilConnection = new StatusLightAnimation(sideStrips.substrip(3, 4), Color.kOrange, Color.kGreen);
        blAprilConnection = new StatusLightAnimation(sideStrips.substrip(4, 5), Color.kOrange, Color.kGreen);
        brAprilConnection = new StatusLightAnimation(sideStrips.substrip(5, 6), Color.kOrange, Color.kGreen);
        questNavConnection = new StatusLightAnimation(sideStrips.substrip(6, 7), Color.kOrange, Color.kGreen);
        coralSecured = new FillAnimation(gamepieceSecuredStrip, coralColor);
        coralAcquired = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(5), InterpolationFunction.step.gradient(Color.kBlack, coralColor));
        algaeSecured = new FillAnimation(gamepieceSecuredStrip, algaeColor);
        algaeAcquired = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(5), InterpolationFunction.step.gradient(Color.kBlack, algaeColor));
        prepareClimbing = new FlashingAnimation(fullSideStrips, WaveFunction.Sawtooth.frequency(1), InterpolationFunction.linear.gradient(Color.kBlack, climbingColor));
        climbing = new BarAnimation(sideStrips.parallel(backMirrorStrip), InterpolationFunction.linear.gradient(Color.kBlack, climbingColor));
        climbingComplete = new FlashingAnimation(fullSideStrips, WaveFunction.Modulo.frequency(0.5), Gradient.rainbow);
        level4Targeted = new FillAnimation(bottomSideStrip, level4Color);
        level3Targeted = new FillAnimation(bottomSideStrip, level3Color);
        level2Targeted = new FillAnimation(bottomSideStrip, level2Color);
        level1Targeted = new FillAnimation(bottomSideStrip, level1Color);
        removeAlgae = new FlashingAnimation(topSideStrip, WaveFunction.Sawtooth.frequency(2), InterpolationFunction.step.gradient(Color.kBlack, algaeColor));
        goToOppositeSideOfReef = new FlashingAnimation(topSideStrip, WaveFunction.Sawtooth.frequency(2), InterpolationFunction.step.gradient(Color.kBlack, Color.kYellow));

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

    public final WaveAnimation autonomousRunningAnimation;
    public final AutonomousFinishedAnimation autonomousFinishedAnimation;
    public final FillAnimation estopped;
    public final FillAnimation tipped;
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
    public final FlashingAnimation goToOppositeSideOfReef;

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
        } else {
            level1Targeted.applyIfFlagged();
            level2Targeted.applyIfFlagged();
            level3Targeted.applyIfFlagged();
            level4Targeted.applyIfFlagged();
            removeAlgae.applyIfFlagged();
            goToOppositeSideOfReef.applyIfFlagged();
        }

        coralSecured.applyIfFlagged();
        coralAcquired.applyIfFlagged();
        algaeSecured.applyIfFlagged();
        algaeAcquired.applyIfFlagged();

        prepareClimbing.applyIfFlagged();
        climbing.applyIfFlagged();
        climbingComplete.applyIfFlagged();

        autonomousRunningAnimation.applyIfFlagged();
        autonomousFinishedAnimation.applyIfFlagged();

        tipped.applyIfFlagged();
        estopped.applyIfFlagged();

        //TODO: End game notification
        hardwareStrip.refresh();
    }
}
