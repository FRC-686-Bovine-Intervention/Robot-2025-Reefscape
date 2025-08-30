package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.util.geometry.GeomUtil;

public class AutoDrive {
    public static Command preciseToPose(Pose2d pose, Drive drive) {
        return new Command() {
            {
                addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
            }
            private final PIDController xController = new PIDController(2, 0, 0);
            private final PIDController yController = new PIDController(15, 0, 0);
            private final PIDController thetaController = new PIDController(3, 0, 0);

            @Override
            public void initialize() {
                xController.reset();
                yController.reset();
                thetaController.reset();
            }

            @Override
            public void execute() {
                var currentPose = RobotState.getInstance().getEstimatedGlobalPose();
                var xOut = xController.calculate(currentPose.getX(), pose.getX());
                var yOut = yController.calculate(currentPose.getY(), pose.getY());
                var thetaOut = thetaController.calculate(currentPose.getRotation().minus(pose.getRotation()).getRadians(), 0);

                drive.runFieldSpeeds(new ChassisSpeeds(
                    xOut,
                    yOut,
                    thetaOut
                ));
            }

            @Override
            public void end(boolean interrupted) {
                drive.stop();
            }
        };
    }
    public static BooleanSupplier withinTolerance(Pose2d pose, Drive drive) {
        return () -> GeomUtil.isNear(pose, RobotState.getInstance().getEstimatedGlobalPose(), Centimeters.of(2), Degrees.of(10));
    }
}
