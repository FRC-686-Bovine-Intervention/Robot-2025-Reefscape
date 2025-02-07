// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = HardwareDevices.pigeonID.pigeon2();

    public GyroIOPigeon2() {
        var config = new Pigeon2Configuration();
        // change factory defaults here
        config.MountPose
            .withMountPoseYaw(Degrees.of(-179.59326171875))
            .withMountPosePitch(Degrees.of(-0.29825273156166077))
            .withMountPoseRoll(Degrees.of(-0.2136882245540619))
        ;

        pigeon.getConfigurator().apply(config);

        // set signals to an appropriate rate
        pigeon.getYaw().setUpdateFrequency(RobotConstants.rioUpdateFrequency);

        pigeon.setYaw(0);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getYaw().getStatus().isOK();

        inputs.rotation = pigeon.getRotation3d();

        inputs.yawVelocity = pigeon.getAngularVelocityZWorld().getValue();   // ccw+
        inputs.pitchVelocity = pigeon.getAngularVelocityYWorld().getValue().unaryMinus();   // up+
        inputs.rollVelocity = pigeon.getAngularVelocityXWorld().getValue().unaryMinus();   // ccw+
    }

    @Override
    public void resetYaw(Measure<AngleUnit> yaw) {
        pigeon.setYaw(yaw.in(Degrees));
    }
}
