import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;

public class TargetPipeline implements VisionPipeline {
  public static Mat drawing;
  public static volatile int val;
  public static volatile double tx;
  public static volatile double ty;
  public static volatile double width;
  public static volatile double height;
  public static volatile double minH = 0.0;
  public static volatile double maxH = 255.0;
  public static volatile double minS = 0.0;
  public static volatile double maxS = 255.0;
  public static volatile double minV = 0.0;
  public static volatile double maxV = 255.0;

  public static final double WHITE_THRESH = 230.0;

  public Mat result;

  @Override
  public void process(Mat mat) {

    minH = SmartDashboard.getNumber("minH", minH);
    maxH = SmartDashboard.getNumber("maxH", maxH);
    minS = SmartDashboard.getNumber("minS", minS);
    maxS = SmartDashboard.getNumber("maxS", maxS);
    minV = SmartDashboard.getNumber("minV", minV);
    maxV = SmartDashboard.getNumber("maxV", maxV);

    val++;
    
    // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
    // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
    for(int i = 0; i < mat.rows(); i++) {
        for(int j = 0; j < mat.cols(); j++) {
            double[] tmp = mat.get(i, j);
            if(tmp[0] > WHITE_THRESH && tmp[1] > WHITE_THRESH && tmp[2] > WHITE_THRESH) {
                mat.put(i, j, new double[] {0.0, 0.0, 0.0});
            }
        }
    }
    Core.extractChannel(mat, mat, 1);

    Imgproc.blur(mat, mat, new Size(3, 3));

    Imgproc.threshold(mat, mat, 240, 255, 0);
    // Core.inRange(mat, new Scalar(minH, minS, minV), new Scalar(maxH, maxS, maxV), mat);

    Imgproc.Canny(mat, mat, 50, 200, 3, false);

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();

    Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

    MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
    Rect boundRects[] = new Rect[contours.size()];

    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

    for (int i = 0; i < contours.size(); i++) {
      contoursPoly[i] = new MatOfPoint2f();
      Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
      boundRects[i] = Imgproc.boundingRect(contoursPoly[i]);
    }

//   Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

  //for (int i = 0; i < contours.size(); i++) {
  //    drawContours(*dest, contours, i, Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point());
  //}
  Rect bRect = new Rect();

  if (boundRects.length > 0) {
      System.out.println("Finding bounding rectangles");
      int maxIndex = 0;
      for (int i = 0; i < boundRects.length; i++) {
          maxIndex = (boundRects[i].area() > boundRects[maxIndex].area()) ? i : maxIndex;
      }
      Imgproc.rectangle(mat, boundRects[maxIndex].tl(), boundRects[maxIndex].br(), new Scalar(0, 0, 255));
      bRect = boundRects[maxIndex];
  }

//   mat.copyTo(TargetPipeline.drawing);
  this.result = mat.clone();

  double tx, ty; //pixels offset from center of image

    if (bRect.width != 0 && bRect.height != 0) {
      tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
      ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
      width = (double) bRect.width;
      height = (double) bRect.height;
      
    }
    }
}