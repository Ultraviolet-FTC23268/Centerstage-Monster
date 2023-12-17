package org.firstinspires.ftc.teamcode.Common.Utility;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleMeta;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

import javax.annotation.Nonnegative;

public class RobotHardware {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx intakeMotor;

    public MotorEx armLeftMotor;
    public MotorEx armRightMotor;

    public Servo droneLatch;
    public Servo gateServo;

    public CRServo frontLeftServo;
    public CRServo frontRightServo;
    public CRServo backLeftServo;
    public CRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder leftArmEncoder;
    public Motor.Encoder rightArmEncoder;

    public Servo leftPivotServo;
    public Servo rightPivotServo;
    public Servo leftElbow;
    public Servo rightElbow;

    public Servo wristServo;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;

    public RevBlinkinLedDriver LEDcontroller;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    //public PhotonBHI260IMU imu;
    //public BHI260IMU imu;
    private BNO055IMU imu;

    private Orientation angles;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    public List<LynxModule> modules;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    private final double startingIMUOffset = -Math.PI/2;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;

        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        modules = hardwareMap.getAll(LynxModule.class);

        //for (LynxModule hub : modules) {
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        ///}

        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageTimer = new ElapsedTime();

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        armLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        armLeftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        armRightMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        /*leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");
        turretServo = hardwareMap.get(Servo.class, "turret");
        wristServo = hardwareMap.get(Servo.class, "wrist");*/

        leftElbow = hardwareMap.get(Servo.class, "leftServo");
        rightElbow = hardwareMap.get(Servo.class, "rightServo");
        rightElbow.setDirection(Servo.Direction.REVERSE);

        droneLatch = hardwareMap.get(Servo.class, "droneServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        leftArmEncoder = new MotorEx(hardwareMap, "dr4bLeft").encoder;
        leftArmEncoder.setDirection(Motor.Direction.REVERSE);
        rightArmEncoder = new MotorEx(hardwareMap, "dr4bRight").encoder;
        rightArmEncoder.setDirection(Motor.Direction.REVERSE);

        //LEDcontroller = hardwareMap.get(RevBlinkinLedDriver.class, "LEDcontroller");

        parallelPod = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpindicularPod = new MotorEx(hardwareMap, "frontRightMotor").encoder;
        perpindicularPod.setDirection(Motor.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public void loop(Pose drive, SwerveDrivetrain drivetrain, LiftSubsystem lift) {

        try {
            if (drive != null) {
                drivetrain.set(drive);
            }
            drivetrain.updateModules();
        } catch (Exception ignored) {
        }

        try {
            lift.loop();
        }
        catch (Exception ignored) {}

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

    }

    public void read(SwerveDrivetrain drivetrain, LiftSubsystem lift) {
        try {
            drivetrain.read();
        }
        catch (Exception ignored) {}
        try {
            lift.read();
        } catch (Exception ignored) {}

    }

    public void write(SwerveDrivetrain drivetrain, LiftSubsystem lift) {

        try {
            drivetrain.write();

        } catch (Exception ignored) {}
        try {
            lift.write();
        } catch (Exception ignored) {}

    }

    public void reset() {
        try {
            parallelPod.reset();
            perpindicularPod.reset();
        } catch (Exception e) {
        }
        //imu.resetYaw();
        //imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

    }

    public void clearBulkCache() {
        //for (LynxModule hub : modules) {
        //    hub.clearBulkCache();
        //}
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    @Nonnegative
    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                        imuAngle = angles.firstAngle + startingIMUOffset;
                    }
                }
            });
            imuThread.start();
        }
    }

    public double getVoltage() {
        return voltage;
    }

}
