package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.Mat;

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
        Point p1 = new Point(10.71f, -7.7f, 4.48f);
        Quaternion Q1 = new Quaternion(0f, 0.707f, 0f , 0.707f);
        MoveTo(p1,Q1);

        //shot and take picture
        api.reportPoint1Arrival();
        api.laserControl(true);
        api.takeTarget1Snapshot();
        takePicture("target1");
        api.laserControl(false);

        //move to s1
        Point s1 = new Point(0.68068,-8.37976,5.29881);
        Quaternion Qs1 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(s1, Qs1);

        //move to s2
        Point s2 = new Point(10.79276,-9.9173,5.29881);
        Quaternion Qs2 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(s2, Qs2);

        //move to p2
        Point p2 = new Point(11.17460, -9.92284, 5.29881);
        Quaternion Q2 = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(p2,Q2);

        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target");
        api.laserControl(false);

        //p2-s2-s1
        Quaternion QG = new Quaternion(0, 0, -0.707f, 0.707f);
        MoveTo(p2,QG);
        MoveTo(s1, QG);
        //move to the Goal
        takePicture("Goal");

        //tack picture
        Mat imagePG = api.getMatNavCam();
        api.saveMatImage(imagePG, "Goal");
        
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



    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

