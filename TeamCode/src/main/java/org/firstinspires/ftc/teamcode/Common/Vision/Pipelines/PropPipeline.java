package org.firstinspires.ftc.teamcode.Common.Vision.Pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Location location = Location.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Rect sideZoneArea;
    private Rect centerZoneArea;

    private Mat finalMat = new Mat();

    //Close Side

    public static int CloseBlueLeftX = 225;
    public static int CloseBlueLeftY = 480;

    public static int CloseBlueCenterX = 825;
    public static int CloseBlueCenterY = 480;

    public static int CloseRedRightX = 1055;
    public static int CloseRedRightY = 480;

    public static int CloseRedCenterX = 455;
    public static int CloseRedCenterY = 480;

    //Far Side

    public static int FarBlueRightX = 1055;
    public static int FarBlueRightY = 480;

    public static int FarBlueCenterX = 455;
    public static int FarBlueCenterY = 480;

    public static int FarRedLeftX = 225;
    public static int FarRedLeftY = 480;

    public static int FarRedCenterX = 825;
    public static int FarRedCenterY = 480;

    public static int width = 100;
    public static int height = 100;

    public static double redThreshold = 1.73;
    public static double blueThreshold = 0.75;
    public static double threshold = 0;

    public double sideColor = 0.0;
    public double centerColor = 0.0;

    public Scalar side = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (Globals.ALLIANCE == Location.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.ALLIANCE == Location.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        if(Globals.SIDE == Location.CLOSE) {

            sideZoneArea = new Rect(Globals.ALLIANCE == Location.RED ? CloseRedRightX : CloseBlueLeftX, Globals.ALLIANCE == Location.RED ? CloseRedRightY : CloseBlueLeftY, width, height);
            centerZoneArea = new Rect(Globals.ALLIANCE == Location.RED ? CloseRedCenterX : CloseBlueCenterX, Globals.ALLIANCE == Location.RED ? CloseRedCenterY : CloseBlueCenterY, width, height);

            Mat sideZone = finalMat.submat(sideZoneArea);
            Mat centerZone = finalMat.submat(centerZoneArea);

            side = Core.sumElems(sideZone);
            center = Core.sumElems(centerZone);

            sideColor = side.val[0] / 1000000.0;
            centerColor = center.val[0] / 1000000.0;

            if (Globals.ALLIANCE == Location.BLUE) {
                if (sideColor < threshold) {
                    // left zone has it
                    location = Location.LEFT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor < threshold) {
                    // center zone has it
                    location = Location.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // right zone has it
                    location = Location.RIGHT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                }
            } else {
                if (sideColor > threshold) {
                    // right zone has it
                    location = Location.RIGHT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor > threshold) {
                    // center zone has it
                    location = Location.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // left zone has it
                    location = Location.LEFT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                }
            }

            Imgproc.rectangle(finalMat, sideZoneArea, new Scalar(255, 255, 255));
            Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

            Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(finalMat, b);
            lastFrame.set(b);

            sideZone.release();
            centerZone.release();

        }

        //Blue Far or Red Close

        else {

            sideZoneArea = new Rect(Globals.ALLIANCE == Location.RED ? FarRedLeftX : FarBlueRightX, Globals.ALLIANCE == Location.RED ? FarRedLeftY : FarBlueRightY, width, height);
            centerZoneArea = new Rect(Globals.ALLIANCE == Location.RED ? FarRedCenterX: FarBlueCenterX, Globals.ALLIANCE == Location.RED ? FarRedCenterY : FarBlueCenterY, width, height);

            Mat sideZone = finalMat.submat(sideZoneArea);
            Mat centerZone = finalMat.submat(centerZoneArea);

            side = Core.sumElems(sideZone);
            center = Core.sumElems(centerZone);

            sideColor = side.val[0] / 1000000.0;
            centerColor = center.val[0] / 1000000.0;

            if (Globals.ALLIANCE == Location.BLUE) {
                if (sideColor < threshold) {
                    // right zone has it
                    location = Location.RIGHT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor < threshold) {
                    // center zone has it
                    location = Location.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // left zone has it
                    location = Location.LEFT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                }
            } else {
                if (sideColor > threshold) {
                    // left zone has it
                    location = Location.LEFT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor > threshold) {
                    // center zone has it
                    location = Location.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // left zone has it
                    location = Location.RIGHT;
                    Imgproc.rectangle(frame, sideZoneArea, new Scalar(255, 255, 255));
                }
            }

            Imgproc.rectangle(finalMat, sideZoneArea, new Scalar(255, 255, 255));
            Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

            Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(finalMat, b);
            lastFrame.set(b);

            sideZone.release();
            centerZone.release();

        }

        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}