package org.firstinspires.ftc.teamcode.Funny.Utility;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.hardware.i2c.imu.PhotonBHI260IMU;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

import javax.annotation.Nonnegative;

public class RobotHardware {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx intakeMotor;

    public DcMotorEx armLeftMotor;
    public DcMotorEx armRightMotor;

    public DcMotorEx droneMotor;

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

    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;

    public RevBlinkinLedDriver LEDcontroller;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    //public PhotonBHI260IMU imu;
    public BHI260IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    public List<PhotonLynxModule> modules;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    private final double startingIMUOffset = 0;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        if (Globals.USING_IMU) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BHI260IMU.class, "imu");//PhotonBHI260IMU
                imu.initialize(new IMU.Parameters(orientationOnRobot));
            }
        }

        //for(PhotonLynxModule module : modules) {
        //    module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        //}

        voltageTimer = new ElapsedTime();

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        /*armLeftMotor = hardwareMap.get(DcMotorEx.class, "dr4bLeft");
        armRightMotor = hardwareMap.get(DcMotorEx.class, "dr4bRight");

        droneMotor = hardwareMap.get(DcMotorEx.class, "droneMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");*/

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        /*leftArmEncoder = new MotorEx(hardwareMap, "leftArmEncoder").encoder;
        leftArmEncoder.setDirection(Motor.Direction.FORWARD);
        rightArmEncoder = new MotorEx(hardwareMap, "rightArmEncoder").encoder;
        rightArmEncoder.setDirection(Motor.Direction.FORWARD);

        LEDcontroller = hardwareMap.get(RevBlinkinLedDriver.class, "LEDcontroller");*/

        parallelPod = new MotorEx(hardwareMap, "backLeftMotor").encoder;
        parallelPod.setDirection(Motor.Direction.FORWARD);
        perpindicularPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        perpindicularPod.setDirection(Motor.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);


    }

    public void loop(Pose drive, SwerveDrivetrain drivetrain) {

        try {
            if (drive != null) {
                drivetrain.set(drive);
            }
            drivetrain.updateModules();
        } catch (Exception ignored) {
        }

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

    }

    public void read(SwerveDrivetrain drivetrain) {

        try {
            drivetrain.read();

        } catch (Exception ignored) {
        }

    }

    public void write(SwerveDrivetrain drivetrain) {

        try {
            drivetrain.write();

        } catch (Exception ignored) {
        }

    }

    public void reset() {
        try {
            parallelPod.reset();
            perpindicularPod.reset();
        } catch (Exception e) {
        }
        imu.resetYaw();
        imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void clearBulkCache() {
        //for(PhotonLynxModule module : modules) {
        //    module.clearBulkCache();
        //}
    }

    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+ startingIMUOffset;
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
