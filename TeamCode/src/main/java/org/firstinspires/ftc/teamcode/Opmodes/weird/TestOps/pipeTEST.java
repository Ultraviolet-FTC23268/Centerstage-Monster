package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Other.Pipelines.PropPipeline;
import org.firstinspires.ftc.teamcode.Other.Side;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Disabled
@Autonomous(name = "PipelineTest")
public class pipeTEST extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.COLOR = Side.BLUE;
        Globals.SIDE = Side.LEFT;

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);

        Scalar scale = new Scalar(1/1000000.0,1/1000000.0,1/1000000.0);
        while (opModeInInit()) {
            telemetry.addData("Location", propPipeline.getLocation());
            //telemetry.addData("leftZone", propPipeline.left.mul(scale).toString());
            //telemetry.addData("centerZone", propPipeline.center.mul(scale).toString());
            if(Globals.SIDE == Side.LEFT)
                telemetry.addData("leftZone", propPipeline.leftColor);
            else
                telemetry.addData("rightZone", propPipeline.rightColor);
            telemetry.addData("centerZone", propPipeline.centerColor);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }
    }
}
