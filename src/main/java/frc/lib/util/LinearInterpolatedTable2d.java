package frc.lib.util;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Creates a 2d lookup table where points are added as pairs in the form <x, y>. The lookup is done
 * by the x, and returns an interpolated y. This is a peicewise linear structure.
 *
 * <p>This is for small numbers of points. Inserts are expensive and are only done on init, and
 * lookups use linear search. If you need it to be better, this is the wrong structure.
 *
 * <p>spotless:off
 * Example:
 *     LinearInterpolatedTable2d table = new LinearInterpolatedTable2d()
 *    .withPair(0.0, 1.0)
 *    .withPair(1.0, 3.0)
 *    .withPair(0.5, 2.0)
 *    .withPair(2.0, 5.0);
 * spotless:on
 */
public class LinearInterpolatedTable2d {
  private final List<Pair<Double, Double>> m_table = new ArrayList<>();

  private class SortFcn implements Comparator<Pair<Double, Double>> {
    @Override
    public int compare(Pair<Double, Double> arg0, Pair<Double, Double> arg1) {
      assert arg0.getFirst() != arg1.getFirst();
      if (arg0.getFirst() < arg1.getFirst()) {
        return -1;
      } else {
        return 1;
      }
    }
  }

  public LinearInterpolatedTable2d withPair(double x, double y) {
    m_table.add(Pair.of(x, y));
    Collections.sort(m_table, new SortFcn());
    return this;
  }

  private double lerp(int tableIdx, double value) {
    var p1 = m_table.get(tableIdx);
    var p2 = m_table.get(tableIdx + 1);

    double pct = (value - p1.getFirst()) / (p2.getFirst() - p1.getFirst());
    return p1.getSecond() + (p2.getSecond() - p1.getSecond()) * pct;
  }

  public double get(double value) {
    assert m_table.size() > 1;

    // Two edge cases
    if (value < m_table.get(0).getFirst()) {
      return lerp(0, value);
    }

    // This is a small table, linear search will be faster
    for (int i = 0; i < m_table.size() - 1; i++) {
      if (value >= m_table.get(i).getFirst() && value < m_table.get(i + 1).getFirst()) {
        return lerp(i, value);
      }
    }

    return lerp(m_table.size() - 2, value);
  }

  @Override
  public String toString() {
    String result = "";
    for (var pair : m_table) {
      result += pair.getFirst() + " " + pair.getSecond() + "\r\n";
    }
    return result;
  }
}
