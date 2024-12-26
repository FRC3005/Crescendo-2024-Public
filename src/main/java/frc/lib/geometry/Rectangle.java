package frc.lib.geometry;

public class Rectangle {
  public final Point2d tl;
  public final Point2d br;
  private final double m_minX, m_maxX, m_minY, m_maxY;

  public Rectangle(Point2d tl, Point2d br) {
    this.tl = tl;
    this.br = br;

    m_minX = Math.min(tl.x, br.x);
    m_minY = Math.min(tl.y, br.y);
    m_maxX = Math.max(tl.x, br.x);
    m_maxY = Math.max(tl.y, br.y);
  }

  public Rectangle(double x1, double x2, double y1, double y2) {
    this(new Point2d(x1, y1), new Point2d(x2, y2));
  }

  public boolean containsPoint(double x, double y) {
    return x <= m_maxX && x >= m_minX && y <= m_maxY && y >= m_minY;
  }

  public boolean containsPoint(Point2d point) {
    return containsPoint(point.x, point.y);
  }
}
