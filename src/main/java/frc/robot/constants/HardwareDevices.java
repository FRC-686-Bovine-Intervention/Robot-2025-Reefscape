package frc.robot.constants;

import frc.util.hardwareID.can.CANBus;
import frc.util.hardwareID.can.CANDevice;
import frc.util.hardwareID.rioPorts.DIOPort;
import frc.util.hardwareID.rioPorts.PWMPort;

public class HardwareDevices {
    /*
     * PDH Ports
     * 10: Left Pivot           9: Mini Power Module
     * 11:                      8: Right Pivot
     * 12: Elevator             7:
     * 13: Cage                 6:
     * 14: Wrist                5: 
     * 15: Intake               4: Chain Winch
     * 16: Back Left Turn       3: Back Right Turn
     * 17: Back Left Drive      2: Back Right Drive
     * 18: Front Left Turn      1: Front Right Turn
     * 19: Front Left Drive     0: Front Right Drive
     * 20: RoboRIO
     * 21: Radio DC
     * 22: Radio POE
     * 23: 
     */
    public static final CANBus rio = CANBus.rio();
    public static final CANBus canivore = CANBus.newBus("canivore");

    // Drive
    public static final CANDevice pigeonID = canivore.id(0);
    // | Front Left
    public static final CANDevice frontLeftDriveMotorID = canivore.id(1);
    public static final CANDevice frontLeftTurnMotorID = rio.id(1);
    // | Front Right
    public static final CANDevice frontRightDriveMotorID = canivore.id(2);
    public static final CANDevice frontRightTurnMotorID = rio.id(2);
    // | Back Left
    public static final CANDevice backLeftDriveMotorID = canivore.id(3);
    public static final CANDevice backLeftTurnMotorID = rio.id(3);
    // | Back Right
    public static final CANDevice backRightDriveMotorID = canivore.id(4);
    public static final CANDevice backRightTurnMotorID = rio.id(4);

    // Pivot
    public static final CANDevice pivotLeftMotorID = rio.id(5);
    public static final CANDevice pivotRightMotorID = rio.id(6);
    public static final CANDevice pivotEncoderID = rio.id(5);

    // Elevator
    public static final CANDevice elevatorMotorID = rio.id(7);
    public static final CANDevice elevatorEncoderID = rio.id(7);

    // Wrist
    public static final CANDevice wristMotorID = rio.id(8);
    public static final CANDevice wristEncoderID = rio.id(8);

    // Intake
    public static final CANDevice intakeMotorID = rio.id(9);
    public static final DIOPort coralSensor = DIOPort.port(1);
    public static final DIOPort algaeSensor = DIOPort.port(2);

    // Climber
    public static final CANDevice climberMotorID = rio.id(10);
    public static final PWMPort climberServoPort = PWMPort.port(1);
    public static final DIOPort climberSensor = DIOPort.port(9);

    // RIO
    public static final PWMPort ledPort = PWMPort.port(0);
    public static final DIOPort button1 = DIOPort.port(0);
    public static final DIOPort button2 = DIOPort.port(0);
    public static final DIOPort button3 = DIOPort.port(0);
}
