package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import org.opencv.core.Mat;


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
        Point p;
        Quaternion q;

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

