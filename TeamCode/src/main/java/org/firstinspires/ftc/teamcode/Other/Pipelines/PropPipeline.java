package org.firstinspires.ftc.teamcode.Other.Pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Other.Side;
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

    private Side location = Side.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Rect leftZoneArea;
    private Rect centerZoneArea;

    private Rect rightZoneArea;

    private Mat finalMat = new Mat();

    //Backdrop Side

    public static int BackdropBlueLeftX = 90;
    public static int BackdropBlueLeftY = 130;

    public static int BackdropBlueCenterX = 695;
    public static int BackdropBlueCenterY = 80;

    public static int BackdropRedRightX = 975;
    public static int BackdropRedRightY = 40;

    public static int BackdropRedCenterX = 370;
    public static int BackdropRedCenterY = 20;

    //Wing Side

    public static int WingBlueRightX = 975;
    public static int WingBlueRightY = 120;

    public static int WingBlueCenterX = 375;
    public static int WingBlueCenterY = 85;

    public static int WingRedLeftX = 75;
    public static int WingRedLeftY = 65;

    public static int WingRedCenterX = 680;
    public static int WingRedCenterY = 30;

    public static int width = 100;
    public static int height = 100;

    public static double redThreshold = 1.73;
    public static double blueThreshold = 0.75;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public double rightColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    public Scalar right = new Scalar(0,0,0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        //Blue Backdrop or Red Wing

        if(Globals.SIDE == Side.LEFT) {

            leftZoneArea = new Rect(Globals.COLOR == Side.RED ? WingRedLeftX : BackdropBlueLeftX, Globals.COLOR == Side.RED ? WingRedLeftY : BackdropBlueLeftY, width, height);
            centerZoneArea = new Rect(Globals.COLOR == Side.RED ? WingRedCenterX : BackdropBlueCenterX, Globals.COLOR == Side.RED ? WingRedCenterY : BackdropBlueCenterY, width, height);

            Mat leftZone = finalMat.submat(leftZoneArea);
            Mat centerZone = finalMat.submat(centerZoneArea);

            left = Core.sumElems(leftZone);
            center = Core.sumElems(centerZone);

            leftColor = left.val[0] / 1000000.0;
            centerColor = center.val[0] / 1000000.0;

            if (Globals.COLOR == Side.BLUE) {
                if (leftColor < threshold) {
                    // left zone has it
                    location = Side.LEFT;
                    Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor < threshold) {
                    // center zone has it
                    location = Side.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // right zone has it
                    location = Side.RIGHT;
                    Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
                }
            } else {
                if (leftColor > threshold) {
                    // left zone has it
                    location = Side.LEFT;
                    Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor > threshold) {
                    // center zone has it
                    location = Side.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // right zone has it
                    location = Side.RIGHT;
                    Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
                }
            }

            Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
            Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

            Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(finalMat, b);
            lastFrame.set(b);

            leftZone.release();
            centerZone.release();

        }

        //Blue Wing or Red Backdrop

        else {

            rightZoneArea = new Rect(Globals.COLOR == Side.RED ? BackdropRedRightX : WingBlueRightX, Globals.COLOR == Side.RED ? BackdropRedRightY : WingBlueRightY, width, height);
            centerZoneArea = new Rect(Globals.COLOR == Side.RED ? BackdropRedCenterX: WingBlueCenterX, Globals.COLOR == Side.RED ? BackdropRedCenterY : WingBlueCenterY, width, height);

            Mat rightZone = finalMat.submat(rightZoneArea);
            Mat centerZone = finalMat.submat(centerZoneArea);

            right = Core.sumElems(rightZone);
            center = Core.sumElems(centerZone);

            rightColor = right.val[0] / 1000000.0;
            centerColor = center.val[0] / 1000000.0;

            if (Globals.COLOR == Side.BLUE) {
                if (rightColor < threshold) {
                    // right zone has it
                    location = Side.RIGHT;
                    Imgproc.rectangle(frame, rightZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor < threshold) {
                    // center zone has it
                    location = Side.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // left zone has it
                    location = Side.LEFT;
                    Imgproc.rectangle(frame, rightZoneArea, new Scalar(255, 255, 255));
                }
            } else {
                if (rightColor > threshold) {
                    // right zone has it
                    location = Side.RIGHT;
                    Imgproc.rectangle(frame, rightZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor > threshold) {
                    // center zone has it
                    location = Side.CENTER;
                    Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
                } else {
                    // left zone has it
                    location = Side.LEFT;
                    Imgproc.rectangle(frame, rightZoneArea, new Scalar(255, 255, 255));
                }
            }

            Imgproc.rectangle(finalMat, rightZoneArea, new Scalar(255, 255, 255));
            Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

            Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(finalMat, b);
            lastFrame.set(b);

            rightZone.release();
            centerZone.release();

        }

        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getLocation() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}