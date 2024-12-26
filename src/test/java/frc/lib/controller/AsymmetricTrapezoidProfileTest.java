package frc.lib.controller;

import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.*;

public class AsymmetricTrapezoidProfileTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  class ExpectedState {
    public final AsymmetricTrapezoidProfile.State state;
    public final double time;

    public ExpectedState(double pos, double vel, double time) {
      this.state = new AsymmetricTrapezoidProfile.State(pos, vel);
      this.time = time;
    }
  }

  public double kDt = 0.002;

  public void assertLessThanOrEqual(double expected, double value) {
    assertLessThanOrEqual(expected, value, "");
  }

  public void assertLessThanOrEqual(double expected, double value, String context) {
    if (value > (expected + kEpsilon)) {
      int line = Thread.currentThread().getStackTrace()[2].getLineNumber();
      String function = Thread.currentThread().getStackTrace()[2].getMethodName();
      String output =
          "Assertion failed at "
              + function
              + ":"
              + line
              + " Expected "
              + value
              + " to be less than or equal to "
              + expected
              + "\r\n\tContext: "
              + context;
      System.out.println(output);
      Assertions.fail(output, new Throwable().fillInStackTrace());
    }
  }

  public void testProfileCase(
      double startPosition,
      double startVelocity,
      double goalPosition,
      double goalVelocity,
      double maxVelocity,
      double maxAcceleration,
      double maxDeceleration,
      List<ExpectedState> expectedStates) {
    AsymmetricTrapezoidProfile putest =
        new AsymmetricTrapezoidProfile(
            new AsymmetricTrapezoidProfile.Constraints(
                maxVelocity, maxAcceleration, maxDeceleration),
            new AsymmetricTrapezoidProfile.State(goalPosition, goalVelocity),
            new AsymmetricTrapezoidProfile.State(startPosition, startVelocity));

    /*
     * Verify no discontinuities (TODO)
     */
    double prevAccel = putest.accelerationAt(0);
    for (double t = kDt; !putest.isFinished(t); t += kDt) {
      var state = putest.calculate(t);
      var prevState = putest.calculate(t - kDt);

      /*
       * Check that max velocity is never violated using velocity & distance
       */
      assertLessThanOrEqual(maxVelocity, Math.abs(state.velocity));
      assertLessThanOrEqual(maxVelocity, (Math.abs(prevState.position - state.position) / kDt));

      /*
       * Check that max acceleration is never violated using velocity
       */
      double accel = putest.accelerationAt(t);
      // Discard points where acceleration changes.
      if (accel == prevAccel) {
        assertLessThanOrEqual(
            Math.abs(accel), (Math.abs(prevState.velocity - state.velocity) / kDt), "Time: " + t);
      }
      prevAccel = accel;
    }

    /*
     * Check that end state is reached
     */
    var endState = putest.calculate(putest.totalTime());
    Assertions.assertEquals(goalPosition, endState.position, kEpsilon);
    Assertions.assertEquals(goalVelocity, endState.velocity, kEpsilon);

    /*
     * Check that we start at the right spot
     *
     * Relax the starting point a bit for when starting the profile with a starting velocity
     * and position that are not close to where they should be.
     */
    var startState = putest.calculate(0);
    Assertions.assertEquals(startPosition, startState.position, 0.1);
    Assertions.assertEquals(startVelocity, startState.velocity, 0.1);

    /*
     * Check any intermediate states (these are eyeball'd from python calc)
     */
    if (expectedStates != null) {
      for (var expectedState : expectedStates) {
        Assertions.assertEquals(
            expectedState.state.position, putest.calculate(expectedState.time).position, 0.25);
        Assertions.assertEquals(
            expectedState.state.velocity, putest.calculate(expectedState.time).velocity, 0.25);
      }
    }
  }

  @Test
  public void profiles() {
    /*
     * These are painstakenly written out to give different cases for each
     */
    /*
     * Zero initial conditions
     */
    {
      /*
       * Asymmetric faster acceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              0.0,
              0.0,
              5.0,
              0.0,
              20.0,
              10.0,
              1.0,
              Arrays.asList(new ExpectedState(1, 2.8, 0.5), new ExpectedState(4.9, 0.5, 3.0)));
          /*
           * negative distance
           */
          testProfileCase(
              0.0,
              0.0,
              -5.0,
              0.0,
              20.0,
              10.0,
              1.0,
              Arrays.asList(new ExpectedState(-1, -2.8, 0.5), new ExpectedState(-4.9, -0.5, 3.0)));
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              0.0,
              0.0,
              10.0,
              0.0,
              4.0,
              6.0,
              2.0,
              Arrays.asList(new ExpectedState(2.6, 4.0, 1.0), new ExpectedState(9.3, 1.75, 3.0)));
          /*
           * negative distance
           */
          testProfileCase(
              0.0,
              0.0,
              -10.0,
              0.0,
              4.0,
              6.0,
              2.0,
              Arrays.asList(
                  new ExpectedState(-2.6, -4.0, 1.0), new ExpectedState(-9.3, -1.75, 3.0)));
        }
      }
      /*
       * Asymmetric faster deceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              0.0,
              0.0,
              5.0,
              0.0,
              20.0,
              2.0,
              10.0,
              Arrays.asList(new ExpectedState(1.0, 2.0, 1.0), new ExpectedState(4.8, 2.0, 2.25)));
          /*
           * negative distance
           */
          testProfileCase(
              0.0,
              0.0,
              -5.0,
              0.0,
              20.0,
              2.0,
              10.0,
              Arrays.asList(
                  new ExpectedState(-1.0, -2.0, 1.0), new ExpectedState(-4.8, -2.0, 2.25)));
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              0.0,
              0.0,
              5.0,
              0.0,
              3.0,
              1.5,
              5.5,
              Arrays.asList(new ExpectedState(0.6, 1.5, 1.0), new ExpectedState(3.5, 3.0, 2.25)));
          /*
           * negative distance
           */
          testProfileCase(
              0.0,
              0.0,
              -5.0,
              0.0,
              3.0,
              1.5,
              5.5,
              Arrays.asList(
                  new ExpectedState(-0.6, -1.5, 1.0), new ExpectedState(-3.5, -3.0, 2.25)));
        }
      }
    }

    /*
     * Non-0 starting velocity
     */
    {
      /*
       * Asymmetric faster acceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              -1.0,
              1.0,
              5.0,
              0.0,
              20.0,
              10.0,
              1.0,
              Arrays.asList(new ExpectedState(1.75, 2.5, 1.0), new ExpectedState(4.45, 1.0, 2.5)));
          /*
           * negative distance
           */
          testProfileCase(
              -1.0,
              -1.0,
              -5.0,
              0.0,
              20.0,
              10.0,
              1.0,
              Arrays.asList(
                  new ExpectedState(-3.2, -1.8, 1.0), new ExpectedState(-4.9, -0.3, 2.5)));
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(
              1.0,
              1.0,
              10.0,
              0.0,
              4.0,
              6.0,
              2.0,
              Arrays.asList(new ExpectedState(4.25, 4.0, 1.0), new ExpectedState(9.1, 1.8, 2.5)));
          /*
           * negative distance
           */
          testProfileCase(
              1.0,
              1.0,
              -10.0,
              0.0,
              4.0,
              6.0,
              2.0,
              Arrays.asList(
                  new ExpectedState(-0.9, -4.0, 1.0), new ExpectedState(-6.8, -3.5, 2.5)));
        }
      }
      /*
       * Asymmetric faster deceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(1.0, 2.0, 5.0, 0.0, 20.0, 2.0, 10.0, null);
          /*
           * negative distance
           */
          testProfileCase(1.0, 1.5, -5.0, 0.0, 20.0, 2.0, 10.0, null);
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(0.0, -1.0, 5.0, 0.0, 3.0, 1.5, 5.5, null);
          /*
           * negative distance
           */
          testProfileCase(0.0, -1.0, -5.0, 0.0, 3.0, 1.5, 5.5, null);
        }
      }
    }
    /*
     * Non-0 ending velocity
     */
    {
      /*
       * Asymmetric faster acceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(0.0, 0.0, 5.0, 1.0, 20.0, 10.0, 1.0, null);
          /*
           * negative distance
           */
          testProfileCase(-1.0, 0.0, -5.0, -1.0, 20.0, 10.0, 1.0, null);
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(1.0, 0.0, 10.0, -1.0, 4.0, 6.0, 2.0, null);
          /*
           * negative distance
           */
          testProfileCase(1.0, 0.0, -10.0, 3.0, 4.0, 6.0, 2.0, null);
        }
      }
      /*
       * Asymmetric faster deceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(1.0, 0.0, 5.0, -1.0, 20.0, 2.0, 10.0, null);
          /*
           * negative distance
           */
          testProfileCase(1.0, 0.0, -5.0, -1.0, 20.0, 2.0, 10.0, null);
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(0.0, 0.0, 5.0, 1.5, 3.0, 1.5, 5.5, null);
          /*
           * negative distance
           */
          testProfileCase(0.0, 0.0, -5.0, 1.0, 3.0, 1.5, 5.5, null);
        }
      }
    }
    /*
     * Non-0 starting and ending velocity
     */
    {
      /*
       * Asymmetric faster acceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(0.0, -3.0, 5.0, 1.0, 20.0, 10.0, 1.0, null);
          /*
           * negative distance
           */
          testProfileCase(-1.0, 1.0, -5.0, -1.0, 20.0, 10.0, 1.0, null);
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(1.0, 2.0, 10.0, -1.0, 4.0, 6.0, 2.0, null);
          /*
           * negative distance
           */
          testProfileCase(1.0, -2.0, -10.0, 3.0, 4.0, 6.0, 2.0, null);
        }
      }
      /*
       * Asymmetric faster deceleration
       */
      {
        /*
         * no cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(1.0, -1.0, 5.0, -1.0, 20.0, 2.0, 10.0, null);
          /*
           * negative distance
           */
          testProfileCase(1.0, -1.0, -5.0, -1.0, 20.0, 2.0, 10.0, null);
        }
        /*
         * with cruise time
         */
        {
          /*
           * positive distance
           */
          testProfileCase(0.0, 1.0, 5.0, 1.5, 3.0, 1.5, 5.5, null);
          /*
           * negative distance
           */
          testProfileCase(0.0, -3.0, -5.0, 1.0, 3.0, 1.5, 5.5, null);
        }
      }
    }

    /*
     * Curise + decel only
     */
    {
      {
        /*
         * positive distance
         */
        testProfileCase(2.0, 4.0, 10.0, -1.0, 4.0, 6.0, 2.0, null);
        /*
         * negative distance
         */
        testProfileCase(1.0, -4.0, -10.0, 3.0, 4.0, 6.0, 2.0, null);
      }
    }

    /*
     * decel only
     */
    {
      {
        /*
         * positive distance
         */
        testProfileCase(8.3, 4.0, 10.0, 3.0, 4.0, 6.0, 2.0, null);
        /*
         * negative distance
         */
        testProfileCase(-8.3, -4.0, -10.0, -3.0, 4.0, 6.0, 2.0, null);
      }
    }
  }
}
