import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;

public class TargetPipeline implements VisionPipeline {
  public static Mat drawing;
  public static volatile int val;
  public volatile double tx;
  public volatile double ty;
  public volatile double reliability;
  public volatile double contourArea;
  public volatile double area;

  public Mat result;

  @Override
  public void process(Mat mat) {

    val++;
    
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);

    Imgproc.blur(mat, mat, new Size(3, 3));

    Imgproc.threshold(mat, mat, 230, 255, 0);

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
      int maxIndex = 0;
      for (int i = 0; i < boundRects.length; i++) {
          maxIndex = (boundRects[i].area() > boundRects[maxIndex].area()) ? i : maxIndex;
      }
      contourArea = Imgproc.contourArea(contours.get(maxIndex));
      bRect = boundRects[maxIndex];
      area = bRect.area();

      if(contourArea / area < 0.20 && contourArea / area > 0.15) {
        Imgproc.rectangle(mat, boundRects[maxIndex].tl(), boundRects[maxIndex].br(), new Scalar(0, 0, 255));
        tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
        ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
        reliability = (bRect.width >= 20) ? 1.0 : 0.0;
      } else {
        reliability = 0.0;
      }
  }

  this.result = mat.clone();

  // double tx, ty; //pixels offset from center of image

    // if (bRect.width != 0 && bRect.height != 0) {
    //   tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
    //   ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
    //   reliability = (bRect.width >= 20) ? 1.0 : 0.0;
    // } else {
    //   reliability = 0.0;
    // }
    }
}