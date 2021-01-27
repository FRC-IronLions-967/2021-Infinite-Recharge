import java.util.*;

import org.opencv.core.*;
import org.opencv.imgproc.*;

import edu.wpi.first.vision.VisionPipeline;

public class LemonPipeline implements VisionPipeline {
    public static volatile int val;

    public volatile Mat result;
    public volatile double tx;
    public volatile double ty;
    public volatile double width;
    public volatile double height;
    public volatile double area;

    @Override
    public void process(Mat mat) {
        val++;
  
        // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
  
        //inRange(mat, Scalar(0, 127, 127), Scalar(15, 255, 255), mat); //tracks red
        Core.inRange(mat, new Scalar(0, 50, 50), new Scalar(40, 235, 235), mat);

        // this.result = mat.clone();
  
        // Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        // Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
  
        // Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        // Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
  
        Imgproc.Canny(mat, mat, 100, 200, 3, false);
  
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
  
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
  
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect boundRects[] = new Rect[contours.size()];
  
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRects[i] = Imgproc.boundingRect(contoursPoly[i]);
        }
  
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
  
        Rect bRect = new Rect();
  
        System.out.println("boundRects length: " + boundRects.length);
        if (boundRects.length > 0) {
            int maxIndex = 0;
            for (int i = 0; i < boundRects.length; i++) {
                maxIndex = (boundRects[i].area() > boundRects[maxIndex].area()) ? i : maxIndex;
            }
            Imgproc.rectangle(mat, boundRects[maxIndex].tl(), boundRects[maxIndex].br(), new Scalar(0, 0, 255));
            bRect = boundRects[maxIndex];
        }
  
        if (bRect.width != 0 && bRect.height != 0) {
            tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
            ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
            width = (double) bRect.width;
            height = (double) bRect.height;
        }
  
        this.result = mat.clone();
      
      }
}