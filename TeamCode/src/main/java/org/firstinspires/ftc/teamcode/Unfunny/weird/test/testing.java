package org.firstinspires.ftc.teamcode.Unfunny.weird.test;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveModule;
//import com.outoftheboxrobotics.photoncore.PhotonCore;

@Disabled
@TeleOp(name="test")

public class testing extends OpMode {

    public BHI260IMU imu;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public CRServo frontLeftServo;
    public CRServo frontRightServo;
    public CRServo backLeftServo;
    public CRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public AbsoluteAnalogEncoder FLencoder;
    public AbsoluteAnalogEncoder FRencoder;
    public AbsoluteAnalogEncoder BLencoder;
    public AbsoluteAnalogEncoder BRencoder;

    public SwerveModule FLmodule;
    public SwerveModule FRmodule;
    public SwerveModule BLmodule;
    public SwerveModule BRmodule;

    public SwerveDrivetrain dt;

    private String FLposition;
    private String FRposition;
    private String BLposition;
    private String BRposition;

    public double targetPosition = 0;


    @Override
    public void init() {

        //PhotonCore.enable();

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        /*imu = hardwareMap.get(BHI260IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/

        FLencoder = new AbsoluteAnalogEncoder(frontLeftEncoder, 3.3);
        FRencoder = new AbsoluteAnalogEncoder(frontRightEncoder, 3.3);
        BLencoder = new AbsoluteAnalogEncoder(backLeftEncoder, 3.3);
        BRencoder = new AbsoluteAnalogEncoder(backRightEncoder, 3.3);

        FLmodule = new SwerveModule(frontLeftMotor, frontLeftServo, FLencoder);
        FRmodule = new SwerveModule(frontRightMotor, frontRightServo, FRencoder);
        BLmodule = new SwerveModule(backLeftMotor, backLeftServo, BLencoder);
        BRmodule = new SwerveModule(backRightMotor, backRightServo, BRencoder);

        telemetry.addData("Status", "Initialized");

        FLposition = FLmodule.getTelemetry(FLposition);
        FRposition = FRmodule.getTelemetry(FRposition);
        BLposition = BLmodule.getTelemetry(BLposition);
        BRposition = BRmodule.getTelemetry(BRposition);

    }

    @Override
    public void loop() {

        FLmodule.update();
        FRmodule.update();
        BLmodule.update();
        BRmodule.update();

        //SwerveDrivetrain.write();
        //double imuAngle = imu.getAngularOrientation().firstAngle;

        if(gamepad1.dpad_left)
            FLmodule.setTargetRotation(2);
        else
            FLmodule.setTargetRotation(0);

        if(gamepad1.dpad_right)
            FRmodule.setTargetRotation(2);
        else
            FRmodule.setTargetRotation(0);

        if(gamepad1.x)
            BLmodule.setTargetRotation(2);
        else
            BLmodule.setTargetRotation(0);

        if(gamepad1.b)
            BRmodule.setTargetRotation(2);
        else
            BRmodule.setTargetRotation(0);


        if(gamepad1.right_bumper) {

            frontLeftServo.setPower(0);
            frontRightServo.setPower(0);
            backLeftServo.setPower(0);
            backRightServo.setPower(0);

        }

        /*FLposition = frontLeftEncoder.getVoltage() / 3.3 * 360;
        FRposition = frontRightEncoder.getVoltage() / 3.3 * 360;
        BLposition = backLeftEncoder.getVoltage() / 3.3 * 360;
        BRposition = backRightEncoder.getVoltage() / 3.3 * 360;

        telemetry.addLine("Front Left: ")
                .addData("", FLposition);
        telemetry.addLine("Front Right: ")
                .addData("", FRposition);
        telemetry.addLine("Back Left: ")
                .addData("", BLposition);
        telemetry.addLine("Back Right: ")
                .addData("", BRposition);*/

    }

}
