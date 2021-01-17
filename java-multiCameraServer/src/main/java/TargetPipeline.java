import edu.wpi.first.vision.VisionPipeline;

import java.util.*;

import org.opencv.core.*;
import org.opencv.imgproc.*;

public class TargetPipeline implements VisionPipeline {
  public static Mat drawing;
  public static volatile int val;
  public static volatile double tx;
  public static volatile double ty;
  public static volatile double width;
  public static volatile double height;

  public Mat result;

  @Override
  public void process(Mat mat) {

    val++;
    
    // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

    Imgproc.blur(mat, mat, new Size(3, 3));

    Imgproc.threshold(mat, mat, 200, 255, 0);
    // Core.inRange(mat, new Scalar(110, v1, v2), upperb, dst);

    Imgproc.Canny(mat, mat, 50, 200, 3, false);

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

  //double tx, ty; //pixels offset from center of image

    if (bRect.width != 0 && bRect.height != 0) {
      tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
      ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
      width = (double) bRect.width;
      height = (double) bRect.height;
      
    }
    }
}