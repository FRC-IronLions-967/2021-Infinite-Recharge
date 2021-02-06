import edu.wpi.first.vision.VisionPipeline;

import java.util.*;

import org.opencv.core.*;
import org.opencv.imgproc.*;

public class TargetPipeline implements VisionPipeline {
  public static Mat drawing;
  public static volatile int val;
  public volatile double tx;
  public volatile double ty;
  public volatile double reliability;
  public volatile double contourArea;
  public volatile double area;
  public volatile double height;
  public volatile double width;
  public volatile boolean hasTarget = false;
  public final double MIN_AREA_PROP = 0.12;
  public final double MAX_AREA_PROP = 0.23;

  public volatile Mat result;

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

    Rect bRect = new Rect();

    if (boundRects.length > 0) {
      int maxIndex = 0;
      for (int i = 0; i < boundRects.length; i++) {
          maxIndex = (boundRects[i].area() > boundRects[maxIndex].area()) ? i : maxIndex;
      }
      contourArea = Imgproc.contourArea(contours.get(maxIndex));
      bRect = boundRects[maxIndex];
      area = bRect.area();

      if(contourArea / area < MAX_AREA_PROP && contourArea / area > MIN_AREA_PROP) {
        Imgproc.rectangle(mat, boundRects[maxIndex].tl(), boundRects[maxIndex].br(), new Scalar(0, 0, 255));
        tx = ((double)bRect.tl().x - (((double)mat.cols() / 2.0) - (double)bRect.width / 2.0));
        ty = ((((double)mat.rows() / 2.0) - (double)bRect.height / 2.0) - (double)bRect.tl().y);
        height = (double) bRect.height;
        width = (double) bRect.width;
        area = bRect.area();
        reliability = (bRect.width >= 20) ? 1.0 : 0.0;
        hasTarget = true;
      } else {
        reliability = 0.0;
        hasTarget = false;
      }
    }

    this.result = mat.clone();
  }
}