package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class MainActivity extends AppCompatActivity {

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    public static double[] Cul(Mat img) {

        ArrayList<Integer> radius = new ArrayList<>();
        ArrayList<Double> pixelX = new ArrayList<>();
        ArrayList<Double> pixelY = new ArrayList<>();

        Mat gray = new Mat();

        Imgproc.cvtColor(img, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.medianBlur(gray, gray, 3);
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, 0, 300, 30, 40, 70);

        for (int i = 0; i < circles.cols(); i++) {
            double[] vCircle = circles.get(0, i);
            pixelX.add(vCircle[0]);
            pixelY.add(vCircle[1]);
            radius.add((int) Math.round(vCircle[2]));
            }

        int max_radius = radius.get(0);
        int index = 0;
        for (int i = 0; i < radius.size(); i++) {
            if (max_radius < radius.get(i)) {
                max_radius = radius.get(i);
                index = i;
                Log.d("The star is at", i + "is the biggest");
                Log.d("The star is at", "radius = " + max_radius);
                }
            }

        double proportion = (double) max_radius / 0.05;
        double errorX = (pixelX.get(index) - 640) / proportion;
        double errorY = (pixelY.get(index) - 480) / proportion;

        double []result = {errorX, errorY};

        return result;



    }
}

