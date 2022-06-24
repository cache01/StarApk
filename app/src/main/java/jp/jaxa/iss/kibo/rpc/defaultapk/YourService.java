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

        //move to point 1
        Point p1 = new Point(10.71000,-7.70000,4.40000);
        Quaternion Q1 = new Quaternion(0f, 0.707f, 0f , 0.707f);
        MoveTo(p1,Q1);


        //shot and take picture
        api.reportPoint1Arrival();
        aim();
        waiting();
        api.laserControl(true);
        api.takeTarget1Snapshot();
        takePicture("target1");
        api.laserControl(false);

        //move to s1
        Point s1 = new Point(10.68068,-8.37976,5.29881);
        Quaternion Qs1 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(s1, Qs1);

        //move to s2
        Point s2 = new Point(10.79276,-9.9173,5.29881);
        Quaternion Qs2 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(s2, Qs2);

        //move to p2
        Point p2 = new Point(11.17460,-10.00000,5.29881);
        Quaternion Q2 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(p2,Q2);
        aim();
        waiting();
        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target2");
        api.laserControl(false);

        //p2-s2-s1
        Quaternion QG = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(s2,QG);
        MoveTo(s1, QG);

        //move to the Goal
        Point pG = new Point(11.27460, -7.89178, 4.96538);
        MoveTo(pG,QG);
        takePicture("Goal");
        
        api.reportMissionCompletion();
    }


    private void MoveTo(Point p, Quaternion q){
        moveToPID(p, q, false);
    }

    private void moveToPID(Point p, Quaternion q, boolean direction){
        Point robotPose, output;
        double x, y, z, error, tolerance = 0.33d;
        int time = 0;
        Log.d("start from", api.getRobotKinematics().getPosition().toString());
        do {
            if(time == 0){
                double outputX = p.getX() * 1.006;   //應該是先移到近似位置
                double outputY = p.getY() * 1.007;   //參數是調出來的
                double outputZ = p.getZ() * 1.005;
                output = new Point(outputX, outputY, outputZ);
            }else{
                output = p;
            }
            Log.d("to", output.toString());

            error = 0;
            Kinematics kinematics = api.getRobotKinematics();  //得到機器人位置資訊
            api.moveTo(output, q, true);
            robotPose = kinematics.getPosition();
            x = Math.abs(p.getX() - robotPose.getX());   //誤差
            y = Math.abs(p.getY() - robotPose.getY());
            z = Math.abs(p.getZ() - robotPose.getZ());
            error += x;
            error += y;
            error += z;
            if(direction){
                double w = Math.abs(kinematics.getOrientation().getW() - q.getW()); //面向角度的誤差
                error += w;
                tolerance = 0.35d;  //應該是可容忍的誤差(?
            }
            ++time;
            if(time > 1){
                tolerance *= 1.1;
            }
        } while (error > tolerance && time < 2);
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
        MoveTo(PE, Q);
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

