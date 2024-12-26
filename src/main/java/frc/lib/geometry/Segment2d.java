package frc.lib.geometry;

public class Segment2d {
  public final Point2d p1;
  public final Point2d p2;

  public Segment2d() {
    this(0, 0, 0, 0);
  }

  public Segment2d(Point2d p1, Point2d p2) {
    this.p1 = p1;
    this.p2 = p2;
  }

  public Segment2d(double x1, double y1, double x2, double y2) {
    this(new Point2d(x1, y1), new Point2d(x2, y2));
  }
}
