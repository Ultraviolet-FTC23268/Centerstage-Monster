package org.firstinspires.ftc.teamcode.Common.Utility;

import android.util.Size;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;
import org.firstinspires.ftc.teamcode.Common.Vision.Pipelines.PreloadDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.ArrayList;

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

    public Servo leftElbow;
    public Servo rightElbow;

    public Servo droneLatch;
    public Servo gateServo;
    public Servo purpleLatch;

    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;

    public RevBlinkinLedDriver LEDcontroller;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    //public PhotonBHI260IMU imu;
    //public BHI260IMU imu;
    private BNO055IMU imu;
    public TwoWheelLocalizer localizer;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public PreloadDetectionPipeline preloadDetectionPipeline;

    private Orientation angles;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double voltage = 12.0;
    private ElapsedTime voltageTimer;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;
    private final double startingIMUOffset = 0;

    public SwerveDrivetrain drivetrain;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public DepositSubsystem deposit;

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

        voltageTimer = new ElapsedTime();

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

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

        leftElbow = hardwareMap.get(Servo.class, "leftServo");
        rightElbow = hardwareMap.get(Servo.class, "rightServo");
        leftElbow.setDirection(Servo.Direction.REVERSE);

        droneLatch = hardwareMap.get(Servo.class, "droneServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        purpleLatch = hardwareMap.get(Servo.class, "purpleServo");
        purpleLatch.setDirection(Servo.Direction.REVERSE);

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
        rightArmEncoder = new MotorEx(hardwareMap, "frontRightMotor").encoder;

        //LEDcontroller = hardwareMap.get(RevBlinkinLedDriver.class, "LEDcontroller");

        parallelPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpindicularPod = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
        perpindicularPod.setDirection(Motor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        drivetrain = new SwerveDrivetrain();
        intake = new IntakeSubsystem();
        lift = new LiftSubsystem();
        deposit = new DepositSubsystem();

        this.preloadDetectionPipeline = new PreloadDetectionPipeline();

        localizer = new TwoWheelLocalizer();

        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }

    public void loop(Pose drive) {

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

        if (Globals.AUTO) localizer.periodic();

    }

    public void read() {
        if(Globals.SWERVE) {
            try {
                drivetrain.read();
            } catch (Exception ignored) {
            }
            try {
                lift.read();
            } catch (Exception ignored) {
            }
        }
    }

    public void write() {

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
        CONTROL_HUB.clearBulkCache();
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

    public Pose getAprilTagPosition() {
        if (aprilTag != null && localizer != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            List<Pose> backdropPositions = new ArrayList<>();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    switch (detection.id) {
                        case 1:
                        case 4:
                            backdropPositions.add(new Pose(detection.ftcPose).add(new Pose(6, 0, 0)));
                            break;
                        case 2:
                        case 5:
                            backdropPositions.add(new Pose(detection.ftcPose));
                            break;
                        case 3:
                        case 6:
                            backdropPositions.add(new Pose(detection.ftcPose).subt(new Pose(6, 0, 0)));
                            break;
                        default:
                            break;
                    }
                }
            }

            Pose backdropPosition = backdropPositions.stream().reduce(Pose::add).orElse(new Pose());
            backdropPosition = backdropPosition.divide(new Pose(backdropPositions.size(), backdropPositions.size(), backdropPositions.size()));


            Pose globalTagPosition = Globals.ALLIANCE == Location.BLUE ?
                    AprilTagLocalizer.convertBlueBackdropPoseToGlobal(backdropPosition) :
                    AprilTagLocalizer.convertRedBackdropPoseToGlobal(backdropPosition);

            if (Double.isNaN(globalTagPosition.x) || Double.isNaN(globalTagPosition.y) || Double.isNaN(globalTagPosition.heading)) return null;

            return globalTagPosition;
        } else {
            return null;
        }
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTag != null && localizer != null) return aprilTag.getDetections();
        System.out.println("Active");
        return null;
    }

    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(877.37, 877.37, 448.296, 272.436)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTag, preloadDetectionPipeline)
                .enableLiveView(false)
                .build();

        visionPortal.setProcessorEnabled(preloadDetectionPipeline, false);
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public boolean isStreaming() { return(this.getCameraState() == VisionPortal.CameraState.STREAMING); }

    public boolean canSeeTag() { return(this.getAprilTagPosition() != null); }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        this.visionPortal.setProcessorEnabled(processor, enabled);
    }

}
