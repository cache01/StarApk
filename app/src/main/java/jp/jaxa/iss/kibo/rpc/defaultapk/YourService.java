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

        aimLaser("target");
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

        //move to S2
        //Point S2 = new Point(10.79276,-10,5.29981);
        Point S2 = new Point(10.55068, -9.35, 5.4325);
        Quaternion QS2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QS2, false);

        //move to point 2
        Point P2 = new Point(11.21360, -10, 5.4825);
        //11.17460,     ,5.29881
        Quaternion Q2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        specificMoveTo(P2, Q2,"z");
        waiting();

        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target_2");
        api.laserControl(false);

        //p2 - s2
        Quaternion QG = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QG, false);

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

