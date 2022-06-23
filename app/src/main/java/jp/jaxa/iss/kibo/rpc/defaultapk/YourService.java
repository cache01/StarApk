package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

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

        Point p1 = new Point(10.71f, -7.7f, 4.48f);
        Quaternion Q1 = new Quaternion(0f, 0.707f, 0f , 0.707f);
        specificMoveTo(p1,Q1,true, true, true,true);

        api.reportPoint1Arrival();

        Point p2 = new Point(11.17460, -9.92284, 5.29881);
        Quaternion Q2 = new Quaternion(0, 0, -0.707f, 0.707f);
        specificMoveTo(p2,Q2,true, true, true,true);

        api.reportPoint1Arrival();

        Point pG = new Point(11.27460, -7.89178, 4.96538);
        Quaternion QG = new Quaternion(0, 0, -0.707f, 0.707f);
        specificMoveTo(pG,QG,true, true, true,true);

        api.reportMissionCompletion();
    }

    public void specificMoveTo(Point p, Quaternion q, boolean ax, boolean ay, boolean az, boolean direction){
        Point robotPose, output;
        double error, tolerance = 0.11d;
        int time = 0;
        Log.d("start from", api.getRobotKinematics().getPosition().toString());
        do {
            output = p;
            Log.d("to", output.toString());

            error = 0;
            Kinematics kinematics = api.getRobotKinematics();
            api.moveTo(output, q, true);
            robotPose = kinematics.getPosition();    //機器人現在的位置
            if(ax)
                error += Math.abs(p.getX() - robotPose.getX());
            //是否計入x方向誤差

            if(ay)
                error += Math.abs(p.getY() - robotPose.getY());
            //是否計入y方向誤差
            if(az)
                error += Math.abs(p.getZ() - robotPose.getZ());
            //是否計入z方向誤差
            if(direction){
                double w = Math.abs(kinematics.getOrientation().getW() - q.getW());
                //翻轉的誤差(?
                error += w;
                tolerance = 0.33d;
            }
            ++time;   //增加次數
        } while (error > tolerance && time < 2);
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

