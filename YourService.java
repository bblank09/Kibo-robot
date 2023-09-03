package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

//import java.util.Vector;
//import gov.nasa.arc.astrobee.android.gs.MessageType;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();
        Log.i(TAG,"start mission");
        int loop_counter = 1;
        Point intersecpoint = new Point(10.4000d, -10.0000d, 4.4700d);
        Quaternion intersecquaternion = new Quaternion(0f, 0f, 0f, 1f);
        /*Result result = api.moveTo(intersecpoint, intersecquaternion, true);
        final int LOOP_MAX = 5;

        int loopcounter = 0;
        while(!result.hasSucceeded()&&loopcounter<LOOP_MAX){
            result=api.moveTo(intersecpoint,intersecquaternion,true);
            ++loopcounter;
        }*/

        Point[] point_adjust7 = {new Point(11.20d, -9.8d, 5.47d),
                new Point(10.45d, -9.16d, 4.4d),
                new Point(10.65d, -7.77d, 4.35d),
                new Point(10.51d, -6.65d, 5.1804d),
                new Point(11.035d, -7.9d, 5.3393d), // เพิ่ม y ลด x
                new Point(11.355d, -9.04d, 4.93d)}; //ลด y เพิ่ม z

        Point point_adjust3_2 = new Point(10.735d, -7.77d, 4.4d);



        Point[] pointdown = {new Point(11.2746d, -9.92284d, 5.35d),
                new Point(10.612d, -9.0709d, 5.35d),
                new Point(10.71d, -7.7d, 5.57d),
                new Point(10.51d, -6.7185d, 5.35d),
                new Point(11.114d, -7.9756d, 5.35d),
                new Point(11.355d, -8.9929d, 5.35d)};

        Quaternion[] quaternion= {new Quaternion(0f, 0f, -0.707f, 0.707f),
                new Quaternion(0.5f, 0.5f, -0.5f, 0.5f),
                new Quaternion(0f, 0.707f, 0f, 0.707f),
                new Quaternion(0f, 0f, -1f, 0f),
                new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f),
                new Quaternion(0f, 0f, 0f, 1f)};

        Point QRP = new Point(11.369d, -8.5518d, 4.4600d);
        Quaternion QRQ = new Quaternion(0f,0.707f,0f,0.707f);

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Point QRPD = new Point(11.369d, -8.5518d, 5.4d);
        Point Goalp = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion Goalq = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Point Goalpd = new Point(11.143d, -6.7607d, 5.40d);
        int k=0;
        String qrscan;

        Point intersecpointdown = new Point(10.4000d, -10.0000d, 5.47d);
        api.moveTo(intersecpointdown,intersecquaternion,true);


        while (true){
            // get the list of active target id
            List<Integer> list = api.getActiveTargets();
            for(int i=0;i<list.size();i++){
                List<Long> timeRemaining = api.getTimeRemaining();
                if (timeRemaining.get(1) < 140000){
                    break;
                }
                api.moveTo(pointdown[list.get(i)-1],quaternion[list.get(i)-1],true);
                List<Long> timeRemaining__ = api.getTimeRemaining();
                if (timeRemaining__.get(1) < 150000  ) {
                    break;
                }
                api.moveTo(point_adjust7[list.get(i)-1],quaternion[list.get(i)-1],true);
                int target3 = list.get(i);
                if (target3 == 3){
                    api.moveTo(point_adjust3_2,quaternion[list.get(i)-1],true);
                }
                // irradiate the laser
                api.laserControl(true);
                // take active target snapshots
                api.takeTargetSnapshot(list.get(i));

                if (target3 == 3){
                    api.moveTo(point_adjust7[list.get(i)-1],quaternion[list.get(i)-1],true);
                }

                api.moveTo(pointdown[list.get(i)-1],quaternion[list.get(i)-1],true);
                k++;
                Log.i(TAG,"Time" + timeRemaining);
                List<Long> timeRemaining_ = api.getTimeRemaining();
                if (timeRemaining_.get(1) < 166000  ) {
                    break;
                }

            }



            List<Long> timeRemaining = api.getTimeRemaining();

            if (timeRemaining.get(1) < 166000){
                break;
            }

        }

        api.moveTo(QRPD,QRQ,true);
        api.moveTo(QRP,QRQ,true);
        qrscan=yourMethod();
        api.moveTo(QRPD,QRQ,true);
        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        /* ****************************************************** */

        api.moveTo(Goalpd,Goalq,true);
        api.moveTo(Goalp,Goalq,true);
        /* ****************************************************** */

        // send mission completion
        api.reportMissionCompletion(qrscan);
    }

    @Override
    protected void runPlan2(){

    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private String yourMethod(){

        Point QRP = new Point(11.369d, -8.5518d, 4.4600d);
        Quaternion QRQ = new Quaternion(0f, 0.707f, 0f, 0.707f);
        for(int i=0 ; i<5; i++) {
            api.moveTo(QRP, QRQ, true);
        }
        Mat Qrcode = api.getMatNavCam();
        Mat perp = new Mat();
        Size size = new Size(500,500);
        MatOfPoint2f pt1 = new MatOfPoint2f(new org.opencv.core.Point(577,436),
                new org.opencv.core.Point(748,437),
                new org.opencv.core.Point(579,672),
                new org.opencv.core.Point(745,671));
        MatOfPoint2f pt2 = new MatOfPoint2f(new org.opencv.core.Point(0,0),
                new org.opencv.core.Point(500,0),
                new org.opencv.core.Point(0,500),
                new org.opencv.core.Point(500,500));
        Mat perspectiveTransform = Imgproc.getPerspectiveTransform(pt1,pt2);
        Imgproc.warpPerspective(Qrcode,perp,perspectiveTransform,size,Imgproc.INTER_LINEAR);
        QRCodeDetector qrCodeDetector = new QRCodeDetector();
        String text =qrCodeDetector.detectAndDecode(perp);
        String report;
        if(text.compareTo("JEM")==0){
            report = "STAY_AT_JEM";
        }else if(text.compareTo("COLUMBUS")==0){
            report = "GO_TO_COLUMBUS";
        }else if(text.compareTo("RACK1")==0){
            report = "CHECK_RACK_1";
        }else if(text.compareTo("ASTROBEE")==0){
            report = "I_AM_HERE";
        }else if(text.compareTo("INTBALL")==0){
            report = "LOOKING_FORWARD_TO_SEE_YOU";
        }else if(text.compareTo("BLANK")==0){
            report = "NO_PROBLEM";
        }else{
            report = "None";
        }
        return report;
    }
}
