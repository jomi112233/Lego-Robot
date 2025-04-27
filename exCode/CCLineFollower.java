import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * LineFollower: follows a black line on white, avoids obstacles by
 * backing up, turning right, driving forward, then re-aligning.
 */
public class CCLineFollower {
    public static void main(String[] args) {
        // Prompt user to start
        LCD.drawString("Press any key", 0, 0);
        Button.waitForAnyPress();

        // Start ultrasonic thread (reads S1 continuously)
        UltraThread ultra = new UltraThread();

        // Calibrate color sensor on S2 (black vs. white)
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S2);
        float[] cal = calibrate(color);
        float black = cal[0], white = cal[1];

        // PID and motor controller
        PIDController pid = new PIDController(black, white, 250f, 10f, 150f);
        MotorController mc = new MotorController(pid, ultra);

        // Launch threads:
        new Thread(new SensorPoller(color, pid)).start(); // color → PID
        new Thread(ultra).start();                        // ultrasonic → distance
        new Thread(mc).start();                           // drive & avoid

        // Run until ESC pressed
        while (!Button.ESCAPE.isDown()) {
            Delay.msDelay(100);
        }
        color.close();
    }

    /** Calibrate by sampling line vs. background reflectivity */
    private static float[] calibrate(EV3ColorSensor cs) {
        SampleProvider sp = cs.getRedMode();
        float[] buf = new float[sp.sampleSize()];

        LCD.clear(); LCD.drawString("On line", 0, 0);
        Button.waitForAnyPress();
        sp.fetchSample(buf, 0); float onLine = buf[0];

        LCD.clear(); LCD.drawString("Off line", 0, 0);
        Button.waitForAnyPress();
        sp.fetchSample(buf, 0); float offLine = buf[0];

        LCD.clear();
        return new float[]{ onLine, offLine };
    }
}

/** Simple PID controller: updateValue(), calculate(), getOutput() */
class PIDController {
    private final float KP, KI, KD, target;
    private volatile float current, output;
    private float integral, lastError;

    public PIDController(float onLine, float offLine, float kp, float ki, float kd) {
        this.KP = kp; this.KI = ki; this.KD = kd;
        this.target = (onLine + offLine) / 2;
        this.integral = this.lastError = 0;
    }

    public void updateValue(float v) {
        current = v;
    }

    public void calculate() {
        float err = target - current;
        integral = integral * 0.9f + err;    // decay + accumulate
        float deriv = err - lastError;
        output = KP * err + KI * integral + KD * deriv;
        lastError = err;
    }

    public float getOutput() {
        return output;
    }
}

/** Reads the ultrasonic sensor in its own thread. */
class UltraThread implements Runnable {
    private volatile float distance = Float.MAX_VALUE;
    private final SampleProvider sp;
    private final float[] buf;

    public UltraThread() {
        EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(SensorPort.S1);
        this.sp  = sensor.getDistanceMode();
        this.buf = new float[sp.sampleSize()];
    }

    public float getDistance() {
        return distance;
    }

    @Override
    public void run() {
        while (true) {
            sp.fetchSample(buf, 0);
            distance = buf[0];
            Delay.msDelay(20);
        }
    }
}

/**
 * Drives the robot:
 *  - followLine() when clear
 *  - avoid() when distance < threshold
 */
class MotorController implements Runnable {
    private static final int BASE = 200, MAX = 300;
    private static final float OBSTACLE_DIST = 0.20f; // 20 cm

    private final PIDController pid;
    private final UltraThread    ultra;

    public MotorController(PIDController pid, UltraThread ultra) {
        this.pid   = pid;
        this.ultra = ultra;
        Motor.A.setAcceleration(2000);
        Motor.B.setAcceleration(2000);
    }

    @Override
    public void run() {
        while (!Button.ESCAPE.isDown()) {
            if (ultra.getDistance() < OBSTACLE_DIST) {
                avoid();
            } else {
                followLine();
            }
            Delay.msDelay(20);
        }
    }

    
    private void followLine() {
        pid.calculate();
        int corr  = (int)pid.getOutput();
        int left  = clamp(BASE + corr, -MAX, MAX);
        int right = clamp(BASE - corr, -MAX, MAX);
        drive(Motor.A, left);
        drive(Motor.B, right);
    }

    /**
     * Deterministic right side detour:
     *  1) Stop & back up
     *  2) Turn right 90°
     *  3) Drive forward
     *  4) Turn left 90° (face original)
     *  5) Drive straight to clear
     *  6) Turn left ~70° to aim at line
     */
    private void avoid() {
        // 1) back up
        Motor.A.stop(true); Motor.B.stop();
        Delay.msDelay(200);
        Motor.A.backward(); Motor.B.backward();
        Delay.msDelay(400);
        Motor.A.stop(true); Motor.B.stop();

        // 2) turn right 90°
        Motor.A.rotate( 90, true);
        Motor.B.rotate(-90);
        Delay.msDelay(200);

        // 3) drive forward
        Motor.A.forward(); Motor.B.forward();
        Delay.msDelay(800);
        Motor.A.stop(true); Motor.B.stop();

        // 4) turn left 90°
        Motor.A.rotate(-90, true);
        Motor.B.rotate( 90);
        Delay.msDelay(200);

        // 5) drive straight
        Motor.A.forward(); Motor.B.forward();
        Delay.msDelay(500);
        Motor.A.stop(true); Motor.B.stop();

        // 6) turn left ~70° to re-aim
        Motor.A.rotate(-70, true);
        Motor.B.rotate( 70);
        Delay.msDelay(200);
    }

    /** Clamp value to [lo..hi] */
    private int clamp(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /** Drive a motor at ±speed */
    private void drive(BaseRegulatedMotor m, int speed) {
        m.setSpeed(Math.abs(speed));
        if (speed >= 0) m.forward();
        else            m.backward();
    }
}

/** Polls the color sensor and updates the PID controller. */
class SensorPoller implements Runnable {
    private final SampleProvider sp;
    private final float[] buf;
    private final PIDController pid;

    public SensorPoller(EV3ColorSensor cs, PIDController pid) {
        this.sp  = cs.getRedMode();
        this.buf = new float[sp.sampleSize()];
        this.pid = pid;
    }

    @Override
    public void run() {
        while (!Button.ESCAPE.isDown()) {
            sp.fetchSample(buf, 0);
            pid.updateValue(buf[0]);
            Delay.msDelay(20);
        }
    }
}
