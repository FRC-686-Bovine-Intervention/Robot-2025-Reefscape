package frc.util.loggerUtil.inputs;

import java.nio.ByteBuffer;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class LoggedFaults implements StructSerializable {
    public long faults;

    public void updateFrom(TalonFX talon) {
        this.faults = talon.getStickyFaultField().getValue().longValue();
    }
    private static final Faults phoenix5Faults = new Faults();
    private static final StickyFaults phoenix5StickyFaults = new StickyFaults();
    public void updateFrom(TalonSRX talon) {
        talon.getFaults(phoenix5Faults);
        talon.getStickyFaults(phoenix5StickyFaults);
        this.faults = Integer.toUnsignedLong(phoenix5StickyFaults.toBitfield()) << 32 | Integer.toUnsignedLong(phoenix5Faults.toBitfield());
    }
    public void updateFrom(TalonFXS talon) {
        this.faults = talon.getStickyFaultField().getValue().longValue() << 32 | talon.getFaultField().getValue().longValue();
    }

    public void updateFrom(SparkMax spark) {
        this.faults = Integer.toUnsignedLong(spark.getStickyFaults().rawBits) << 32 | Integer.toUnsignedLong(spark.getFaults().rawBits);
    }

    public static final LoggedFaultsStruct struct = new LoggedFaultsStruct();

    public static class LoggedFaultsStruct implements Struct<LoggedFaults> {
        @Override
        public Class<LoggedFaults> getTypeClass() {
            return LoggedFaults.class;
        }

        @Override
        public String getTypeName() {
            return "Faults";
        }

        @Override
        public int getSize() {
            return kSizeInt64;
        }

        @Override
        public String getSchema() {
            return "long faults";
        }

        @Override
        public LoggedFaults unpack(ByteBuffer bb) {
            var faults = new LoggedFaults();
            faults.faults = bb.getLong();
            return faults;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedFaults value) {
            bb.putLong(value.faults);
        }
    }
}
