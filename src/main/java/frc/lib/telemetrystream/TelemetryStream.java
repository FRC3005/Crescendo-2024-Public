package frc.lib.telemetrystream;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStreamOverflowException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class TelemetryStream {
  private final int m_canStreamSession;
  private final int m_bufferSizeWords;

  // Preallocate and keep this buffer to prevent object creation each loop
  CANStreamMessage[] m_messageBuffer;

  private static final int DEFAULT_BUFFER_SIZE = 64;

  private final List<StreamedFrame> m_frames = new ArrayList<>();

  private AtomicBoolean m_enabled = new AtomicBoolean(false);

  public enum ErrorCode {
    OK,
    BUFFER_OVERLOW
  }

  /*
   * Create a telemetry stream with a buffer size of DEFAULT_BUFFER_SIZE
   *
   * See TelemetryStream 3 args for details
   */
  public TelemetryStream(int arbId, int idMask) {
    this(arbId, idMask, DEFAULT_BUFFER_SIZE);
  }

  /*
   * Create a telemetry stream based on an arbId and idMask
   *
   * @param arbId The CAN ArbID to match against. The bits of the arbID are
   * and with idMask
   *
   * @param idMask the CAN id Mask is a bit-wise mask of bits in the arbId to match
   * against. This allows matching against multiple frames. For example, providing
   * an an arbId of 0x2050001 and a mask of 0x0x1FFF003F would match _all_ REV
   * motor contorller frames for a device with CAN ID 1. Providing a mask of 0x1FFFFFFF means
   * that only the exact arbID will be matched. Providing a mask of 0 would match
   * any frame of any type.
   *
   * @param bufferSizeWords the number of frames in the stream buffer. Frames are removed
   * in each call to poll() or at a rate specified if a thread is given.
   */
  public TelemetryStream(int arbId, int idMask, int bufferSizeWords) {
    m_bufferSizeWords = bufferSizeWords;
    m_canStreamSession = CANJNI.openCANStreamSession(arbId, idMask, m_bufferSizeWords);
    m_messageBuffer = new CANStreamMessage[m_bufferSizeWords];
    for (int i = 0; i < m_bufferSizeWords; i++) {
      m_messageBuffer[i] = new CANStreamMessage();
    }

    if (m_canStreamSession == 0) {
      // TODO: Should this throw? I believe WPILib will already send some kind of message
      // throw new Exception("Failed to open CAN Stream");
    }
  }

  private void messageMatch(CANStreamMessage message) {
    for (StreamedFrame frame : m_frames) {
      frame.processOnMatch(message.messageID, message.timestamp, message.data, message.length);
    }
  }

  // Lazy way to test...
  public void injectAndPoll(CANStreamMessage[] messages) {
    for (CANStreamMessage message : messages) {
      messageMatch(message);
    }
  }

  /*
   * If sychronous, call this method periodically to flush the buffer and trigger
   * callbacks.
   */
  public ErrorCode poll() {
    ErrorCode errorCode = ErrorCode.OK;
    CANStreamMessage messages[];

    int numToRead;
    try {
      numToRead =
          CANJNI.readCANStreamSession(m_canStreamSession, m_messageBuffer, m_bufferSizeWords);
      messages = m_messageBuffer;
    } catch (CANStreamOverflowException e) {
      numToRead = e.getMessagesRead();
      errorCode = ErrorCode.BUFFER_OVERLOW;
      messages = e.getMessages();
    }

    if (!m_enabled.get()) {
      return ErrorCode.OK;
    }

    for (int i = 0; i < numToRead; i++) {
      var message = messages[i];
      if (message != null) {
        // TODO: mutex this
        messageMatch(message);
      }
    }

    return errorCode;
  }

  /*
   * Add a StreamedFrame implementation to this stream.
   */
  public TelemetryStream addFrame(StreamedFrame frame) {
    m_frames.add(frame);
    return this;
  }

  /*
   * Start collecting frames and enable all callbacks.
   */
  public void start() {
    // Flush the buffer first.
    poll();
    m_enabled.set(true);
    ;
  }

  /*
   * Stop collecting frames and disable all callbacks.
   */
  public void stop() {
    m_enabled.set(false);
  }
}
