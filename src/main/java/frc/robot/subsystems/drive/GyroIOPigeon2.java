// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import frc.robot.constants.HardwareDevices;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = HardwareDevices.pigeonID.pigeon2();

    private final Queue<Double> quatWQueue;
    private final Queue<Double> quatXQueue;
    private final Queue<Double> quatYQueue;
    private final Queue<Double> quatZQueue;

    public GyroIOPigeon2() {
        var config = new Pigeon2Configuration();
        config.MountPose
            .withMountPoseYaw(Degrees.of(-179.59326171875))
            .withMountPosePitch(Degrees.of(-0.29825273156166077))
            .withMountPoseRoll(Degrees.of(-0.2136882245540619))
        ;
        this.pigeon.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency,
            this.pigeon.getQuatW(),
            this.pigeon.getQuatX(),
            this.pigeon.getQuatY(),
            this.pigeon.getQuatZ()
        );
        this.pigeon.setYaw(0);

        this.quatWQueue = OdometryThread.getInstance().registerPhoenixSignal(this.pigeon.getQuatW());
        this.quatXQueue = OdometryThread.getInstance().registerPhoenixSignal(this.pigeon.getQuatX());
        this.quatYQueue = OdometryThread.getInstance().registerPhoenixSignal(this.pigeon.getQuatY());
        this.quatZQueue = OdometryThread.getInstance().registerPhoenixSignal(this.pigeon.getQuatZ());
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = this.pigeon.getYaw().getStatus().isOK();

        var rotation3ds = new Rotation3d[this.quatWQueue.size()];
        for (int i = 0; i < rotation3ds.length; i++) {
            rotation3ds[i] = new Rotation3d(new Quaternion(
                this.quatWQueue.poll(),
                this.quatXQueue.poll(),
                this.quatYQueue.poll(),
                this.quatZQueue.poll()
            ));
        }
        inputs.odometryGyroRotation = rotation3ds;

        inputs.yawVelocity = this.pigeon.getAngularVelocityZWorld().getValue();   // ccw+
        inputs.pitchVelocity = this.pigeon.getAngularVelocityYWorld().getValue().unaryMinus();   // up+
        inputs.rollVelocity = this.pigeon.getAngularVelocityXWorld().getValue().unaryMinus();   // ccw+
    }

    @Override
    public void resetYaw(Measure<AngleUnit> yaw) {
        this.pigeon.setYaw(yaw.in(Degrees));
    }
}
