package org.firstinspires.ftc.teamcode.Autos.Close_PreReq;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class BallColumnPipeline extends OpenCvPipeline {

    public int columnCenterX = 380;

    public boolean foundColumn = false;
    public int error = 0;

    private static final Scalar PURPLE_LOW  = new Scalar(125, 80, 65);
    private static final Scalar PURPLE_HIGH = new Scalar(155, 255, 255);

    private static final Scalar GREEN_LOW  = new Scalar(75, 130, 55);
    private static final Scalar GREEN_HIGH = new Scalar(100, 255, 255);

    private static final int IMG_WIDTH = 640;
    private static final int X_TOL = 70;

    @Override
    public Mat processFrame(Mat input) {

        // =======================
        // HSV MASK (HOLES)
        // =======================
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, mask);

        Mat greenMask = new Mat();
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);
        Core.bitwise_or(mask, greenMask, mask);

        // =======================
        // FILL HOLES (PER BALL)
        // =======================
        Mat fillKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(11, 11)   // fills holes, does NOT merge balls
        );

        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, fillKernel);
        Imgproc.dilate(mask, mask, fillKernel);

        // =======================
        // SPLIT BALLS BACK APART
        // =======================
        Mat splitKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(5, 5)
        );

        Imgproc.erode(mask, mask, splitKernel);

        // =======================
        // FIND BALL CONTOURS
        // =======================
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                mask,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        List<Rect> balls = new ArrayList<>();

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < 3500) continue;

            Rect r = Imgproc.boundingRect(c);
            if (r.width < 55 || r.height < 55) continue;

            balls.add(r);
            Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 2);
        }

        if (balls.size() < 2) {
            foundColumn = false;
            return input;
        }

        // =======================
        // COLUMN CLUSTERING
        // =======================
        balls.sort(Comparator.comparingInt(r -> r.x));

        List<List<Rect>> columns = new ArrayList<>();

        for (Rect r : balls) {
            int cx = r.x + r.width / 2;
            boolean added = false;

            for (List<Rect> col : columns) {
                Rect ref = col.get(0);
                int refCx = ref.x + ref.width / 2;

                boolean xClose = Math.abs(cx - refCx) < X_TOL;
                boolean yStacked = Math.abs(r.y - ref.y) > ref.height * 0.6;

                if (xClose && yStacked) {
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
            return input;
        }

        // =======================
        // COLUMN CENTER
        // =======================
        foundColumn = true;

        List<Integer> xs = new ArrayList<>();
        for (Rect r : bestColumn) {
            xs.add(r.x + r.width / 2);
        }
        xs.sort(Integer::compareTo);

        columnCenterX = xs.get(xs.size() / 2);

        Imgproc.line(
                input,
                new Point(columnCenterX, 0),
                new Point(columnCenterX, input.height()),
                new Scalar(0, 0, 255),
                2
        );

        error = columnCenterX - (IMG_WIDTH / 2);

        return input;
    }
}