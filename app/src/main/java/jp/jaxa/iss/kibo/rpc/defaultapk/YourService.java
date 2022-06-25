package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        api.startMission();
        //S1 (10.68068,-8.37976,5.29881)
        //S2 (10.68068,-9.2,5.4325)
        //move P1 - S1 - S2 - P2 - S2 - goal position
        double angle = Math.sqrt(2) / 2;
        //Point B = new Point(10.71f, -7.76f, 4.5f);
        //Quaternion QB = new Quaternion(0f, (float)angle, 0f, (float)angle);
        //api.moveTo(B, QB, false);
        //move to point 1
        Point P1 = new Point(10.71f, -7.76f, 4.4f);
        //cam(10.71, -7.7, 4.4)
        Quaternion Q1 = new Quaternion(0f, (float) angle, 0f, (float) angle);
        //specificMoveTo(P1, Q1, "y");
        api.moveTo(P1, Q1, false);
        api.reportPoint1Arrival();
        waiting();
        try {
            Log.d("The star is at", "aim first time");
            aim("target1");
        } catch (Exception ignored) {
        }
        waiting();
        try {
            Log.d("The star is at", "aim second time");
            aim("target1");
        } catch (Exception ignored) {
        }
        waiting();
        aimLaser("target1");
        waiting();
        //shot and take picture
        api.laserControl(true);
        api.takeTarget1Snapshot();
        takePicture("target_1");
        api.laserControl(false);
        waiting();
        //move to S1
        //Point S1 = new Point(10.68068,-8.37976,5.29881);
        Point S1 = new Point(10.55068, -8.37976, 5.4325);
        Quaternion QS1 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S1, QS1, false);
        //MoveTo(S1, QS1,"z");
        //move to S2
        //Point S2 = new Point(10.79276,-10,5.29981);
        Point S2 = new Point(10.55068, -9.35, 5.4325);
        Quaternion QS2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QS2, false);
        //MoveTo(S2,QS2,"z");
        //move to point 2
        Point P2 = new Point(11.21360, -10, 5.4325);
        //11.17460,     ,5.29881
        Quaternion Q2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        //specificMoveTo(P2, Q2,"z");
        api.moveTo(P2, Q2, false);
        waiting();
        Mat img = api.getMatNavCam();
        Mat gray = new Mat();
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_RGB2GRAY);

        Imgproc.medianBlur(gray, gray, 3);
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, 0, 300, 30, 40, 70);

        try {
            Log.d("The star is at", "aim first time");
            aim("target2");
        } catch (Exception ignored) {
        }
        waiting();
        try {
            Log.d("The star is at", "aim second time");
            aim("target2");
        } catch (Exception ignored) {
        }
        waiting();
        aimLaser("target2");
        waiting();
        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target_2");
        api.laserControl(false);
        //p2 - s2 - s1
        Quaternion QG = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QG, false);
        //api.moveTo(S1, QG, false);
        //MoveTo(S2, QG,"z");
        //MoveTo(S1, QG,"z");
        //move to gaol position
        Point PG = new Point(11.27460, -7.89178, 4.96538);
        specificMoveTo(PG, QG, "z");
        takePicture("goal");
        api.reportMissionCompletion();
    }

        private void specificMoveTo (Point p, Quaternion q, String mode) {
        double axile = 0;
        Point robotPos, output;
        double tolerance = 0.3d;
        double error_pos, error_rotate;
        double error_posX, error_posY, error_posZ;
        int time1 = 0;
        int time2 = 0;
        Log.d("startfrom", api.getRobotKinematics().getPosition().toString());
        api.moveTo(p, q, false);
        waiting();
        Point P = api.getRobotKinematics().getPosition();
        do {
            switch (mode) {
                case "x":
                    axile = api.getRobotKinematics().getOrientation().getX();
                    break;
                case "y":
                    axile = api.getRobotKinematics().getOrientation().getY();
                    break;
                case "z":
                    axile = api.getRobotKinematics().getOrientation().getZ();
                    break;
            }
            api.moveTo(P, q, false);
            time1++;
        } while (Math.abs(axile - Math.sqrt(2) / 2) > 0.001 && time1 < 3);
        Quaternion Q = api.getRobotKinematics().getOrientation();
        do {
            double currentX = api.getRobotKinematics().getPosition().getX();
            double currentY = api.getRobotKinematics().getPosition().getY();
            double currentZ = api.getRobotKinematics().getPosition().getZ();
            error_pos = Math.abs(p.getX() - currentX) + Math.abs(p.getY() -
                    currentY) + Math.abs(p.getZ() - currentZ);
            error_posX = Math.abs(p.getX() - currentX);
            error_posY = Math.abs(p.getY() - currentY);
            error_posZ = Math.abs(p.getZ() - currentZ);
            api.relativeMoveTo(new Point(error_posX, error_posY, error_posZ), Q,
                    false);
            time2 ++;
        }while(error_pos > tolerance && time2 < 3);
    }

    private void takePicture(String tag) {
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, tag);
    }

    private void waiting() {
        try {
            Thread.sleep(300);
        } catch (Exception ignored) {
        }
    }

    private void Laser(String s){
        api.laserControl(true);
        switch (s){
            case "target1":
                api.takeTarget1Snapshot();
                break;
            case "target2":
                api.takeTarget2Snapshot();
                break;
        }

        api.laserControl(false);
    }

    private void aim(String mode){
        Mat img = api.getMatNavCam();
        Mat gray = new Mat();
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_RGB2GRAY);

        Imgproc.medianBlur(gray, gray, 3);
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, 0, 300, 30, 40, 70);

        ArrayList<Integer> radius = new ArrayList<>();
        ArrayList<Double> pixelX = new ArrayList<>();
        ArrayList<Double> pixelY = new ArrayList<>();
        for (int i=0; i < circles.cols(); i++){
        double[] vCircle = circles.get(0, i);
        //org.opencv.core.Point center = new org.opencv.core.Point(vCircle[0], vCircle[1]);
        //int Radius = (int) Math.round(vCircle[2]);
        //Imgproc.circle(gray, center, Radius, new Scalar(0, 255, 0), 3, 8,0);
        pixelX.add(new Double(vCircle[0]));
        pixelY.add(new Double(vCircle[1]));
        radius.add(new Integer((int) Math.round(vCircle[2])));
        Log.d("The star is at", "find" + i + "circles");
    }
        //api.saveMatImage(gray, "gray");
        int max_radius = radius.get(0);
        int index = 0;
        for (int i=0; i < radius.size(); i++){
            if (max_radius < radius.get(i)) {
                max_radius = radius.get(i);
                index = i;
                Log.d("The star is at", i + "is the biggest");
                Log.d("The star is at", "radius = " + max_radius);
            }
        }

        double proportion = (double)max_radius / 0.05 ;
        double errorX = (pixelX.get(index) - 640) / proportion;
        double errorY = (pixelY.get(index) - 480) / proportion;
        Log.d("The star is at", "errorX = " + errorX);
        Log.d("The star is at", "errorY = " + errorY);

        switch (mode){
            case "target1":
                double x1 = errorY;
                double y1 = errorX;
                double z1 = api.getRobotKinematics().getPosition().getZ();
                Point p1 = new Point(x1, y1, z1);
                Quaternion q1 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(p1, q1, false);
                break;
            case "target2":
                double x2 = errorX;
                double y2 = api.getRobotKinematics().getPosition().getY();
                double z2 = errorY;
                Point p2 = new Point(x2, y2, z2);
                Quaternion q2 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(p2, q2, false);
                break;

        }
    }


    private void aimLaser(String mode) {
        switch (mode) {
            case "target1":
                Point pj1 = new Point(0.05, -0.1, api.getRobotKinematics().getPosition().getZ());
                Quaternion qj1 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(pj1, qj1, false);
                break;
            case "target2":
                Point pj2 = new Point(-0.08, api.getRobotKinematics().getPosition().getY(), 0.07);
                Quaternion qj2 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(pj2, qj2, false);
                break;
        }
    }


    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

