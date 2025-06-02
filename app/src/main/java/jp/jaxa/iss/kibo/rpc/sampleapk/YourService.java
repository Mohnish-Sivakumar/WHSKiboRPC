package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.Size;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    private final String[] TEMPLATE_FILE_NAME = {
            "coin.png",
            "compass.png",
            "coral.png",
            "crystal.png",
            "emerald.png",
            "fossil.png",
            "key.png",
            "letter.png",
            "shell.png",
            "treasure_box.png"
    };
    private final String[] TEMPLATE_NAME = {
            "coin.png",
            "compass.png",
            "coral.png",
            "crystal.png",
            "emerald",
            "fossil",
            "key",
            "letter",
            "shell",
            "treasure_box"
    };
    @Override
    protected void runPlan1(){
        Log.i(TAG, "start mission");
        // The mission starts.
        //will start a timer
        api.startMission();

        // Move to a point.
        Point point = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        //makes astrobee move
        api.moveTo(point, quaternion, false);
        // Get a camera image.
        Mat image = api.getMatNavCam();

        api.saveMatImage(image, "file_name.png");

        /* ******************************************************************************** */
        /* Write your code to recognize the type and number of landmark items in each area! */
        /* If there is a treasure item, remember it.                                        */
        /* ******************************************************************************** */

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        //get cmera matrix
        Mat cameraMatrix = new Mat(3,3, CvType.CV_64F);
        cameraMatrix.put(0,0,api.getNavCamIntrinsics()[0]);
        Mat cameraCoefficients = new Mat(1,5,CvType.CV_64F);
        cameraCoefficients.put(0,0,api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i < TEMPLATE_FILE_NAME.length; i++){
            try {
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                //convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                //Assign to an array of templates
                templates[i] = mat;
                inputStream.close();
            }catch (IOException e){
                e.printStackTrace();

            }
        }
        int templateMatchCnt[] = new int[templates.length];

        //Get the number of template matches
        for (int tempNum = 0;tempNum < templates.length; tempNum++){
            int matchCnt = 0;
            //coordinates of the matched location
            List<org.opencv.core.Point> matches = new ArrayList<>();

            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();
            //pattern matching
            int widthMin = 20;
            int widthMax = 100;
            int changeWidth = 5;
            int changeAngle = 45;

            for (int size = widthMin; size <= widthMax; size += changeWidth){
                for (int angle = 0; angle< 360; angle += changeAngle){
                    Mat resizedTemp = resizeImg(template, size);
                    Mat rotResizedTemp = rotImg(resizedTemp, angle);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    //Get coordiantes with similarity greater than or equal to the threshold
                    double threshold = 0.7;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold){
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);
                        for (int y= 0; y<thresholdedResult.rows(); y++){
                            for (int x = 0; x< thresholdedResult.cols(); x++){
                                if (thresholdedResult.get(y,x)[0]>0){
                                    matches.add(new org.opencv.core.Point(x,y));
                                }
                            }
                        }
                    }
                }

            }
            //Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt += filteredMatches.size();
            templateMatchCnt[tempNum] = matchCnt;
        }
        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);

        // When you recognize landmark items, let’s set the type and number.
        api.setAreaInfo(1, TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.
        point = new Point(11.143d, -6.7607d, 4.9654d);
        quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
       // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // You can add your method.
    private String yourMethod(){
        return "your method";
    }

    private Mat resizeImg(Mat img, int width){
        int height = (int) (img.rows() * ((double) width/img.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, height));
        return resizedImg;
    }
    //Rotate image
    private Mat rotImg(Mat img, int angle){
        org.opencv.core.Point  center = new org.opencv.core.Point(img.cols()/2.0, img.rows()/2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());
        return rotatedImg;
    }
    //Remove multiple detections
    private List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points){
        double length = 10;
        List <org.opencv.core.Point> filteredList = new ArrayList<>();
        for (org.opencv.core.Point point : points){
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList){
                double distance = calculateDistance(point, checkPoint);
                if (distance <= length){
                    isInclude = true;
                    break;
                }
            }
            if (!isInclude){
                filteredList.add(point);
            }
        }
        return filteredList;
    }
    private double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2){
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
    }
    //Get the maximum value of an array
    private int getMaxIndex (int[] array){
        int max = 0;
        int maxIndex = 0;
        //Find the index of the element with the largest vlaue
        for (int i = 0; i< array.length; i++){
            if (array[i] > max){
                max = array[i];
                maxIndex = i;

            }
        }
        return maxIndex;
    }
}
