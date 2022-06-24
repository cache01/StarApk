package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import java.lang.reflect.Array;
import java.util.ArrayList;

import gov.nasa.arc.astrobee.Kinematics;
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
        Point p2 = new Point(11.21460,-10,5.4625);
        Quaternion Q2 = new Quaternion(0, 0, -angleF, angleF);
        specificMoveTo(p2, Q2, "Z");

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
        api.flashlightControlFront(0.5f);         // 開燈照明
        Mat img = (api.getMatNavCam());
        //img = cv2.cvtColor(img, cv2.VOLOR_BGR2GRAY)

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();    // 不重要，照打

        Imgproc.findContours(img, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);    // 輪廓的資料存入counters

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        double recX = 0;   // 矩形中點x座標
        double recY = 0;   // 矩形中點y座標

        for (int i = 0; i < contours.size(); i++){
            MatOfPoint contour = contours.get(i);
            double area = Imgproc.contourArea(contour);

            if (area > 300){                                                                        // 300要視情況做調整，取出要的圖形面積範圍
                MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
                double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;                // 0.02 可調整
                if (approxDistance > 1){                                                          // 不知道要判斷甚麼XD
                    Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);         // 將結果存到 approxCurve
                    MatOfPoint points = new MatOfPoint(approxCurve.toArray());                  // 將型態轉回MatOfPOint
                    if (points.total() == 4 && Math.abs(Imgproc.contourArea(points)) > 500 && Imgproc.isContourConvex(points)){    // 判斷矩形，最後一個不知到是啥?
                        Rect rect = Imgproc.boundingRect(points);
                        recX = rect.x + rect.width/2;
                        recY = rect.y + rect.height/2;
                    }
                }
                break;
            }
        }

        double errorX = 480 - recX;
        double errorY = 640 - recY;
        Log.println((int) errorY,"errorY","This is errorY");
        Log.println((int) errorX,"errorX","This is errorX");

        double Z = api.getRobotKinematics().getPosition().getZ();
        Point PE = new Point(errorX, errorY, Z);
        Quaternion Q = api.getRobotKinematics().getOrientation();
        api.moveTo(PE, Q, false);
    }

    private void waiting() {
        try {
            Thread.sleep(200);
        } catch (Exception ignored) {
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

