/*
 * Copyright (c) 2018-2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package frc.lib.vendor.motorcontroller.stream;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class SparkMaxFrames {
  /*********CAN ID Defines***********/

  // CANSpark API class;
  public static final int CMD_API_SETPNT_SET = 0x001;

  public static final int CMD_API_DC_SET = 0x002;
  public static final int CMD_API_SPD_SET = 0x012;
  public static final int CMD_API_POS_SET = 0x032;
  public static final int CMD_API_VOLT_SET = 0x042;
  public static final int CMD_API_CURRENT_SET = 0x043;
  public static final int CMD_API_SMARTMOTION_SET = 0x052;
  public static final int CMD_API_STAT0 = 0x060;
  public static final int CMD_API_STAT1 = 0x061;
  public static final int CMD_API_STAT2 = 0x062;
  public static final int CMD_API_CLEAR_FAULTS = 0x06E;
  public static final int CMD_API_DRV_STAT = 0x06A;
  public static final int CMD_API_BURN_FLASH = 0x072;
  public static final int CMD_API_SET_FOLLOWER = 0x073;
  public static final int CMD_API_FACTORY_DEFAULT = 0x074;
  public static final int CMD_API_FACTORY_RESET = 0x075;
  public static final int CMD_API_NACK = 0x080;
  public static final int CMD_API_ACK = 0x081;
  public static final int CMD_API_BROADCAST = 0x090;
  public static final int CMD_API_HEARTBEAT = 0x092;
  public static final int CMD_API_SYNC = 0x093;
  public static final int CMD_API_ID_QUERY = 0x094;
  public static final int CMD_API_ID_ASSIGN = 0x095;
  public static final int CMD_API_FIRMWARE = 0x098;
  public static final int CMD_API_ENUM = 0x099;
  public static final int CMD_API_MECH_POS = 0x0A0;
  public static final int CMD_API_I_ACCUM = 0x0A2;
  public static final int CMD_API_PARAM_ACCESS = 0x300;

  public static int packFloat32(double val) {
    return Float.floatToIntBits((float) val);
  }

  public static double unpackFloat32(int val) {
    return (double) Float.intBitsToFloat(val);
  }

  public interface DataFrame {
    public default int arbId(int canId) {
      return 0;
    }
    ;

    public byte[] Serialize();

    public void Deserialize(byte[] buf);
  }

  public static class SetpointOut implements DataFrame {
    public float setpoint;
    public short auxSetpoint;
    public byte pidSlot; // Upper 2 bits only
    public byte rsvd;

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putFloat(setpoint);
      buffer.putShort(auxSetpoint);
      buffer.put(pidSlot);
      buffer.put(rsvd);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {}
  }

  public static class SetParamOut implements DataFrame {
    public int parameter;
    public byte parameterType;

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(5);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putInt(parameter);
      buffer.put(parameterType);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      buffer.get(buf);
      buffer.position(0);

      parameter = buffer.getInt(0);
      parameterType = buffer.get(4);
    }
  }

  public static class StatusConfigOut implements DataFrame {
    public short updateRate;

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(2);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putShort(updateRate);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {}
  }

  public static class GetParamIn implements DataFrame {
    public int parameter0;
    public byte parameterType;
    public byte parameterResponse;

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putInt(parameter0);
      buffer.put(parameterType);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.wrap(buf);
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      parameter0 = buffer.getInt();
      parameterType = buffer.get();
    }
  }

  public static class BurnFlashOut implements DataFrame {
    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.put((byte) 0xA2);
      buffer.put((byte) 0x2A);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {}
  }

  public static class FollowerOut implements DataFrame {
    public int followerID;
    public int followerCfg;

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putInt(followerID);
      buffer.putInt(followerCfg);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      buffer.get(buf);
      buffer.position(0);

      followerID = buffer.getInt(0);
      followerCfg = buffer.getInt(4);
    }
  }

  public static class FirmwareIn implements DataFrame {
    public byte firmwareMajor;
    public byte firmwareMinor;
    public short firmwareBuild;
    public byte debugBuild;
    public byte hardwareRevision;

    public byte[] Serialize() {
      return new byte[] {(byte) 0};
    }

    public void Deserialize(byte[] buf) {
      if (buf.length < 5) {
        firmwareMajor = 0;
        firmwareMinor = 0;
        firmwareBuild = 0;
        debugBuild = 0;
        hardwareRevision = 0;
        return;
      }

      ByteBuffer buffer = ByteBuffer.wrap(buf);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      firmwareMajor = buffer.get();
      firmwareMinor = buffer.get();
      buffer.order(ByteOrder.BIG_ENDIAN);
      firmwareBuild = buffer.getShort();
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      debugBuild = buffer.get();

      if (buf.length < 6) {
        return;
      }
      hardwareRevision = buffer.get();
    }
  }

  public static class Status0In implements DataFrame {
    public short appliedOutput;
    public short faults;
    public short stickyFaults;
    public byte sensorInv;
    public byte setpointInv;
    public byte idleMode;
    public byte mtrType;
    public byte isFollower;

    public int arbId(int canId) {
      return 0x2051800 | canId;
    }

    public int arbId() {
      return arbId(0);
    }

    public double getAppliedOutput() {
      return (double) appliedOutput / 32767.0;
    }

    public void setAppliedOutput(double ao) {
      appliedOutput = (short) (ao * 32767.0);
    }

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putShort(appliedOutput);
      buffer.putShort(faults);
      buffer.putShort(stickyFaults);

      byte bits = 0;
      bits |= (sensorInv != 0) ? (byte) 0x01 : 0;
      bits |= (setpointInv != 0) ? (byte) 0x02 : 0;
      bits |= (mtrType != 0) ? (byte) 0x10 : 0;
      bits |= (isFollower != 0) ? (byte) 0x20 : 0;

      buffer.put(bits);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.wrap(buf);
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      byte bits;

      appliedOutput = buffer.getShort();
      faults = buffer.getShort();
      stickyFaults = buffer.getShort();
      bits = buffer.get();
      ;

      sensorInv = ((bits & (byte) 0x01) != 0) ? (byte) 1 : (byte) 0;
      setpointInv = ((bits & (byte) 0x02) != 0) ? (byte) 1 : (byte) 0;
      mtrType = ((bits & (byte) 0x10) != 0) ? (byte) 1 : (byte) 0;
      isFollower = ((bits & (byte) 0x20) != 0) ? (byte) 1 : (byte) 0;
    }
  }

  public static class Status1In implements DataFrame {
    public float sensorVel;
    public byte mtrTemp;
    public short mtrVoltage;
    public short mtrCurrent;

    public int arbId(int canId) {
      return 0x2051840 | canId;
    }

    public int arbId() {
      return arbId(0);
    }

    public double getVelocity() {
      return (double) sensorVel;
    }

    public void setVelocity(double vel) {
      sensorVel = (float) vel;
    }

    public double getBusVoltage() {
      return (double) mtrVoltage / 128.0;
    }

    public void setBusVoltage(double voltage) {
      mtrVoltage = (short) (voltage * 128.0);
    }

    public double getOutputCurrent() {
      return (double) mtrCurrent / 32.0;
    }

    public void setOutputCurrent(double current) {
      mtrCurrent = (short) (current * 32.0);
    }

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putFloat(sensorVel);
      buffer.put(mtrTemp);

      // Pack mtrVoltage and mtrCurrent into 3 bytes
      int mtrVoltage12 = mtrVoltage & 0x0FFF;
      int mtrCurrent12 = mtrCurrent & 0x0FFF;

      buffer.put((byte) (mtrVoltage12 & 0xFF)); // Least significant 8 bits of mtrVoltage
      buffer.put(
          (byte)
              ((mtrVoltage12 >> 8)
                  | ((mtrCurrent12 & 0x0F) << 4))); // 4 bits each from mtrVoltage and mtrCurrent
      buffer.put((byte) (mtrCurrent12 >> 4)); // Most significant 8 bits of mtrCurrent

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.wrap(buf);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      sensorVel = buffer.getFloat(0);
      mtrTemp = buffer.get(4);

      // Extracting 12-bit mtrVoltage and 12-bit mtrCurrent from 3 bytes
      short combined1 = buffer.get(5);
      short combined2 = buffer.getShort(6);

      mtrVoltage = (short) ((combined1 & 0xFF) | ((combined2 & 0x0F) << 8));
      mtrCurrent = (short) ((combined2 >> 4) & 0x0FFF);
    }
  }

  public static class Status2In implements DataFrame {
    public float sensorPos;
    public float iAccum;

    public int arbId(int canId) {
      return 0x2051880 | canId;
    }

    public int arbId() {
      return arbId(0);
    }

    public double getPosition() {
      return (double) sensorPos;
    }

    public void setPosition(double position) {
      sensorPos = (float) position;
    }

    public byte[] Serialize() {
      ByteBuffer buffer = ByteBuffer.allocate(8); // 4 bytes for each float
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      buffer.putFloat(sensorPos);
      buffer.putFloat(iAccum);

      return buffer.array();
    }

    public void Deserialize(byte[] buf) {
      ByteBuffer buffer = ByteBuffer.wrap(buf);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      sensorPos = buffer.getFloat(0);
      iAccum = buffer.getFloat(4);
    }
  }
}
