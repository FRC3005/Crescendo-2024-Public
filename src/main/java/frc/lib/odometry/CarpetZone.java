package frc.lib.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.geometry.Point2d;
import frc.lib.geometry.Rectangle;

public class CarpetZone {
  private final Rectangle m_bounds;
  private final Rotation2d m_carpetDirection;

  /**
   * Create a carpet zone specifying the x,y coordinate rectangle on the field where the carpet has
   * the same nap direction. Always use the Red alliance zero for the xy zero, and the carpet
   * direction vector should start at the origin.
   *
   * @param bounds the rectangle bounds where the carpet direction is the same
   * @param carpetDirection a rotation, starting from the red relative origin and point in the
   *     direciton that the weave in the carpet is laying down. Looking at the carpet you can find
   *     this by seeing which way the nap in the carpet lies. You can also gently push down with
   *     your finger on the carpet and drag it in each direction. The direction with the least
   *     resistence is the direciton we care about.
   */
  public CarpetZone(Rectangle bounds, Rotation2d carpetDirection) {
    m_bounds = bounds;
    m_carpetDirection = carpetDirection;
  }

  public CarpetZone(Point2d tl, Point2d br, Rotation2d carpetDirection) {
    this(new Rectangle(tl, br), carpetDirection);
  }

  public CarpetZone(
      double tl_x, double br_x, double tl_y, double br_y, Rotation2d carpetDirection) {
    this(new Rectangle(tl_x, br_x, tl_y, br_y), carpetDirection);
  }

  public Rectangle getBounds() {
    return m_bounds;
  }

  public Rotation2d getCarpetDirection() {
    return m_carpetDirection;
  }

  public boolean containsPoint(double x, double y) {
    return m_bounds.containsPoint(x, y);
  }

  public boolean containsPoint(Point2d p) {
    return containsPoint(p.x, p.y);
  }
}
