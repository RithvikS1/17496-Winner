package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BallColumnPipeline extends OpenCvPipeline {

    public int columnCenterX = -1;
    public boolean foundColumn = false;
    public int error = 0;

    // HSV ranges — tune on FTC Dashboard
    private static final Scalar PURPLE_LOW  = new Scalar(125, 80, 65);
    private static final Scalar PURPLE_HIGH = new Scalar(155, 255, 255);

    private static final Scalar GREEN_LOW  = new Scalar(75, 130, 55);
    private static final Scalar GREEN_HIGH = new Scalar(100, 255, 255);


    private static final int IMG_WIDTH = 640;

    @Override
    public Mat processFrame(Mat input) {

        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Color masks
        Mat purpleMask = new Mat();
        Mat greenMask  = new Mat();
        Mat combined   = new Mat();

        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, purpleMask);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);
        Core.bitwise_or(purpleMask, greenMask, combined);

        // =======================
        // MORPHOLOGY (CRITICAL)
        // =======================
        Mat kernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(9, 9)
        );

        // Fill wiffle ball holes
        Imgproc.morphologyEx(combined, combined, Imgproc.MORPH_CLOSE, kernel);

        // Remove small noise
        Imgproc.morphologyEx(combined, combined, Imgproc.MORPH_OPEN, kernel);

        // =======================
        // FIND CONTOURS
        // =======================
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                combined,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        List<Rect> balls = new ArrayList<>();

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < 500) continue;

            Rect r = Imgproc.boundingRect(c);

            // Reject hole-like shapes
            double aspectRatio = (double) r.width / r.height;
            if (aspectRatio < 0.7 || aspectRatio > 1.3) continue;

            balls.add(r);
            Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 2);
        }

        if (balls.size() < 2) {
            foundColumn = false;
            columnCenterX = -1;
            error = 0;
            return input;
        }

        // =======================
        // COLUMN CLUSTERING
        // =======================
        List<List<Rect>> columns = new ArrayList<>();

        for (Rect r : balls) {
            int cx = r.x + r.width / 2;
            boolean added = false;

            for (List<Rect> col : columns) {
                int refCx = col.get(0).x + col.get(0).width / 2;
                if (Math.abs(cx - refCx) < 40) {
                    col.add(r);
                    added = true;
                    break;
                }
            }

            if (!added) {
                List<Rect> newCol = new ArrayList<>();
                newCol.add(r);
                columns.add(newCol);
            }
        }

        // Pick tallest column
        List<Rect> bestColumn = null;
        int maxCount = 0;

        for (List<Rect> col : columns) {
            if (col.size() > maxCount) {
                maxCount = col.size();
                bestColumn = col;
            }
        }

        if (bestColumn == null || bestColumn.size() < 2) {
            foundColumn = false;
            columnCenterX = -1;
            error = 0;
            return input;
        }

        // =======================
        // COLUMN CENTER
        // =======================
        foundColumn = true;
        int sumX = 0;

        for (Rect r : bestColumn) {
            int cx = r.x + r.width / 2;
            sumX += cx;

            Imgproc.circle(
                    input,
                    new Point(cx, r.y + r.height / 2),
                    5,
                    new Scalar(255, 0, 0),
                    -1
            );
        }

        columnCenterX = sumX / bestColumn.size();

        Imgproc.line(
                input,
                new Point(columnCenterX, 0),
                new Point(columnCenterX, input.height()),
                new Scalar(0, 0, 255),
                2
        );

        int imageCenter = IMG_WIDTH / 2;
        error = columnCenterX - imageCenter;

        return input;
    }
}
