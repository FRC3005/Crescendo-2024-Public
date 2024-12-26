package frc.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import org.tinylog.Logger;

public class Version {
  private static String m_version = "";

  static {
    try {
      File deployDir = Filesystem.getDeployDirectory();
      File versionFile = new File(deployDir, "version.txt");
      var reader = new BufferedReader(new FileReader(versionFile));
      m_version = reader.readLine();
      reader.close();
    } catch (Exception e) {
      Logger.tag("Version").error(e);
    }
  }

  public static String get() {
    return m_version;
  }
}
