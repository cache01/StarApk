package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
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
    protected void runPlan1(){
        api.startMission();

        double angle = Math.sqrt(2)/2;
        float angleF = (float)angle;

        //move to point 1
        Point p1 = new Point(10.71f,-7.76f,4.4f);
        Quaternion Q1 = new Quaternion(0f, angleF, 0f , angleF);
        specificMoveTo(p1, Q1, "y");

        //shot and take picture
        api.reportPoint1Arrival();
        api.laserControl(true);
        api.takeTarget1Snapshot();
        takePicture("target1");
        api.laserControl(false);

        //move to s1
        Point s1 = new Point(10.68068,-8.37976,5.29881);
        Quaternion Qs1 = new Quaternion(0, 0, -angleF, angleF);
        api.moveTo(s1, Qs1, false);

        //move to s2
        Point s2 = new Point(10.79276,-10,5.29881);
        Quaternion Qs2 = new Quaternion(0, 0, -angleF, angleF);
        api.moveTo(s2, Qs2, false);

        //move to p2
        Point p2 = new Point(11.21360,-10,5.4325);
        Quaternion Q2 = new Quaternion(0, 0, -angleF, angleF);
        specificMoveTo(p2, Q2, "Z");
        waiting();

        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target2");
        api.laserControl(false);

        //p2-s2-s1
        Quaternion QG = new Quaternion(0, 0, -angleF, angleF);
        api.moveTo(s2, QG, false);
        api.moveTo(s1, QG, false);

        //move to the Goal
        Point pG = new Point(11.27460, -7.89178, 4.96538);
        specificMoveTo(pG, QG, "z");
        takePicture("Goal");
        
        api.reportMissionCompletion();
    }


    private void specificMoveTo (Point p, Quaternion q, String mode){
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

        //調整自旋
        Point P = api.getRobotKinematics().getPosition();

        do {
            switch (mode){
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

            time1 ++;
        }while (Math.abs(axile - Math.sqrt(2)/2) > 0.001 && time1 <3);


        //調整座標
        Quaternion Q = api.getRobotKinematics().getOrientation();

        do {
            double currentX = api.getRobotKinematics().getPosition().getX();
            double currentY = api.getRobotKinematics().getPosition().getY();
            double currentZ = api.getRobotKinematics().getPosition().getZ();

            error_pos = Math.abs(p.getX()-currentX) + Math.abs(p.getY()-currentY) + Math.abs(p.getZ()-currentZ);
            error_posX = Math.abs(p.getX()-currentX);
            error_posY = Math.abs(p.getY()-currentY);
            error_posZ = Math.abs(p.getZ()-currentZ);

            api.relativeMoveTo(new Point(error_posX, error_posY, error_posZ), q, false);

            time2 ++;
        }while(error_pos > tolerance && time2 < 3);
    }


    private void takePicture(String tag){
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, tag);
    }

    public void aim(){
        Mat img = api.getMatNavCam();
        Mat gray = new Mat();

        takePicture("aiming");

        Imgproc.cvtColor(img, gray, Imgproc.COLOR_RGB2GRAY);

        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, 100, 440, 50, 0, 345);

        //dp: 檢測圓心的累加器圖像與源圖像之間的比值倒數
        //minDist：檢測到的圓的圓心之間的最小距離
        //param1：method設置的檢測方法對應參數，針對HOUGH_GRADIENT，表示邊緣檢測算子的高閾值（低閾值是高閾值的一半），默認值100
        //param2：method設置的檢測方法對應參數，針對HOUGH_GRADIENT，表示累加器的閾值。值越小，檢測到的無關的圓
        //minRadius：圓半徑的最小半徑，默認為0
        //maxRadius：圓半徑的最大半徑，默認為0（若minRadius和maxRadius都默認為0，則HoughCircles函數會自動計算半徑）

        List<Integer> radius = new ArrayList<>();
        List<Double> pixelX = new ArrayList<>();
        List<Double> pixelY = new ArrayList<>();

        for(int i = 0; i<circles.cols();i++){
            double [] vCircles = circles.get(0, 1);

            pixelX.add(vCircles[0]);
            pixelY.add(vCircles[1]);
            radius.add((int)Math.round(vCircles[2]));
        }

        int max_radius = radius.get(0);
        int index = 0;

        for(int i = 0;i < radius.size();i++){
            if(max_radius<radius.get(i)){
                max_radius = radius.get(i);
                index = i;
            }
        }

        double proportion = max_radius / 0.05 ;
        double errorX = (pixelX.get(index) - 640) / proportion;
        double errorY = (pixelY.get(index) - 480) / proportion;

        double x = errorX;
        double y = api.getRobotKinematics().getPosition().getY();
        double z = errorY;
        Point p = new Point(x, y, z);
        Quaternion q = api.getRobotKinematics().getOrientation();
        api.relativeMoveTo(p, q, false);

    }

    private void waiting() {
        try {
            Thread.sleep(200);
        } catch (Exception ignored) {
        }

    }

    private void aimLaser(){
        Point pi = new Point(0.1, api.getRobotKinematics().getPosition().getY(), -0.05);
        Quaternion qi = api.getRobotKinematics().getOrientation();
        api.relativeMoveTo(pi, qi, false);
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

