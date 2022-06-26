package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;

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
        double angle = Math.sqrt(2) / 2;

        //P1
        Point P1 = new Point(10.71f, -7.76f, 4.4f);
        //cam(10.71, -7.7, 4.4)
        Quaternion Q1 = new Quaternion(0f, (float) angle, 0f, (float) angle);
        //specificMoveTo(P1, Q1, "y");
        api.moveTo(P1, Q1, false);
        api.reportPoint1Arrival();
        waiting();

        aimLaser("target1");
        waiting();

        //shot and take picture
        api.laserControl(true);
        api.takeTarget1Snapshot();
        takePicture("target_1");
        api.laserControl(false);
        waiting();


        //Point S1 = new Point(10.68068,-8.37976,5.29881);
        Point S1 = new Point(10.55068, -8.37976, 5.4325);
        Quaternion QS1 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S1, QS1, false);

        //move to S2
        //Point S2 = new Point(10.79276,-10,5.29981);
        Point S2 = new Point(10.55068, -9.8, 5.4325);
        Quaternion QS2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QS2, false);

        //move to point 2
        Point P2 = new Point(11.21360, -10, 5.4825);
        //11.17460,     ,5.29881
        Quaternion Q2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(P2,Q2, false);
        waiting();

        try {
            aim();
            Mat img = api.getMatNavCam();
            api.saveMatImage(img, "aim success");
            waiting();
        }catch (Exception ignored){
            Mat correct = api.getMatNavCam();
            api.saveMatImage(correct, "aim crash");
        }
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
        api.moveTo(S1,QG, false);

        //move to gaol position
        Point PG = new Point(11.27460, -7.89178, 4.96538);
        api.moveTo(PG,QG,false);
        takePicture("goal");
        api.reportMissionCompletion();


    }

    public void takePicture(String tag) {
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, tag);
    }


    private void waiting() {
        try {
            Thread.sleep(300);
        } catch (Exception ignored) {
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
                Point pj2 = new Point(-0.1, api.getRobotKinematics().getPosition().getY(), 0.05);
                Quaternion qj2 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(pj2, qj2, false);
                break;
        }
    }

    private void aim(){
        Mat img = api.getMatNavCam();
        api.saveMatImage(img, "aim start");
        //Mat gray = new Mat();
        //Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(img, img, 3);
        api.saveMatImage(img, "blur success");
        Mat circles = new Mat();
        Imgproc.HoughCircles(img, circles, Imgproc.HOUGH_GRADIENT, 1,
                (double)img.rows()/16, 300, 30, 20, 70);
        api.saveMatImage(img, "find circle success");
        ArrayList<Integer> radius = new ArrayList<>();
        ArrayList<Double> pixelX = new ArrayList<>();
        ArrayList<Double> pixelY = new ArrayList<>();
        for (int i=0;
             i < circles.cols();
             i++){
            double[] vCircle = circles.get(0, i);
            pixelX.add(vCircle[0]);
            pixelY.add(vCircle[1]);
            radius.add((int) Math.round(vCircle[2]));
            int Radius = (int)Math.round(vCircle[2]);
            org.opencv.core.Point center = new org.opencv.core.Point(vCircle[0],
                    vCircle[1]);
            Imgproc.circle(img, center, Radius, new Scalar(0, 255, 0), 3, 8, 0);
            api.saveMatImage(img, "circle" + i);
        }
        api.saveMatImage(img, "get circle value success");
        int max_radius = radius.get(0);
        int index = 0;
        for (int i=0;
             i < radius.size();
             i++){
            if (max_radius < radius.get(i)) {
                max_radius = radius.get(i);
                index = i;
            }
        }
        api.saveMatImage(img, "find biggest circle success");
        double proportion = (double)max_radius / 0.05 ;
        double errorX = (pixelX.get(index) - 640) / proportion;
        double errorY = (pixelY.get(index) - 480) / proportion;
        double x2 = errorX;
        double y2 = api.getRobotKinematics().getPosition().getY();
        double z2 = errorY;
        api.saveMatImage(img, "adjust position success");
        Point p2 = new Point(x2, y2, z2);
        Quaternion q2 = api.getRobotKinematics().getOrientation();
        api.relativeMoveTo(p2, q2, false);
        api.saveMatImage(img, "aim end");
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

