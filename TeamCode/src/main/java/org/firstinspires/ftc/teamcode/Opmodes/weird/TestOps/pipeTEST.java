package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;
import org.firstinspires.ftc.teamcode.Common.Vision.Pipelines.PropPipeline;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

//@Disabled
@Autonomous(name = "PipelineTest")
public class pipeTEST extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);

        Scalar scale = new Scalar(1/1000000.0,1/1000000.0,1/1000000.0);
        while (opModeInInit()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("sideZone", propPipeline.sideColor);
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
