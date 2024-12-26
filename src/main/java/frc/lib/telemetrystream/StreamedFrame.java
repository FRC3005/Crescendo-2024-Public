package frc.lib.telemetrystream;

public interface StreamedFrame {
  public void processOnMatch(int arbid, long timestamp, byte[] data, int length);
}
