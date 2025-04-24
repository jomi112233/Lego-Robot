import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    public static void main(String[] args) {
        LCD.drawString("Press any key", 0, 0);
        Button.waitForAnyPress();

        // 1) Initialize the ultrasonic‐reading thread (no ctor args)
        UltraThread ultraThread = new UltraThread();

        // 2) Initialize color sensor & calibrate
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        float[] cal = calibrate(colorSensor);
        float lineVal = cal[0], bgVal = cal[1];

        // 3) Set up PID + motor controller
        PIDController pid = new PIDController(lineVal, bgVal, 250f, 10f, 150f);
        MotorController mc = new MotorController(pid, ultraThread);

        // 4) Start threads
        new Thread(new SensorPoller(colorSensor, pid)).start();
        new Thread(ultraThread).start();
        new Thread(mc).start();

        // 5) Exit on ESC
        while (!Button.ESCAPE.isDown()) {
            Delay.msDelay(100);
        }
        colorSensor.close();
    }

    private static float[] calibrate(EV3ColorSensor cs) {
        SampleProvider sp = cs.getRedMode();
        float[] buf = new float[sp.sampleSize()];

        LCD.clear(); LCD.drawString("On line", 0, 0);
        Button.waitForAnyPress();
        sp.fetchSample(buf, 0); float line = buf[0];

        LCD.clear(); LCD.drawString("Off line", 0, 0);
        Button.waitForAnyPress();
        sp.fetchSample(buf, 0); float bg = buf[0];

        LCD.clear();
        return new float[]{ line, bg };
    }
}

class PIDController {
    private final float KP, KI, KD, target;
    private volatile float current, output;
    private float integral = 0, lastError = 0;

    public PIDController(float line, float bg, float p, float i, float d) {
        this.KP = p; this.KI = i; this.KD = d;
        this.target = (line + bg) / 2;
    }
    public void updateValue(float v) { current = v; }
    public void calculate() {
        float err = target - current;
        integral = integral * 0.9f + err;
        float deriv = err - lastError;
        output = KP * err + KI * integral + KD * deriv;
        lastError = err;
    }
    public float getOutput() { return output; }
}

class UltraThread implements Runnable {
    private volatile float distance = Float.MAX_VALUE;
    private final EV3UltrasonicSensor sensor;
    private final SampleProvider sp;
    private final float[] buf;

    public UltraThread() {
        // no args: hardcode S1 here
        sensor = new EV3UltrasonicSensor(SensorPort.S1);
        sp     = sensor.getDistanceMode();
        buf    = new float[sp.sampleSize()];
    }

    public float getDistance() {
        return distance;
    }

    public void run() {
        while (true) {
            sp.fetchSample(buf, 0);
            distance = buf[0];
            Delay.msDelay(20);
        }
    }
}

class MotorController implements Runnable {
    private static final int BASE = 200, MAX = 300;
    private static final float DETECT = 0.25f, CLEAR = 0.35f;
    private static final int BUF_SIZE = 5;

    private final PIDController pid;
    private final UltraThread ultra;
    private final float[] distBuf = new float[BUF_SIZE];
    private int bufIdx = 0;
    private boolean inObstacle = false;

    public MotorController(PIDController pid, UltraThread u) {
        this.pid = pid;
        this.ultra = u;
        Motor.A.setAcceleration(2000);
        Motor.B.setAcceleration(2000);
    }

    public void run() {
        while (!Button.ESCAPE.isDown()) {
            float d = ultra.getDistance();
            // sliding average
            distBuf[bufIdx] = d;
            bufIdx = (bufIdx + 1) % BUF_SIZE;
            float sum = 0;
            for (float v : distBuf) sum += v;
            float avg = sum / BUF_SIZE;

            // hysteresis
            if (!inObstacle && avg < DETECT) {
                inObstacle = true;
                avoid();
                inObstacle = false;
            } else if (inObstacle && avg > CLEAR) {
                inObstacle = false;
            }

            if (!inObstacle) {
                follow();
            }

            Delay.msDelay(20);
        }
    }

    private void follow() {
        pid.calculate();
        float corr = pid.getOutput();
        int a = clamp(BASE + (int) corr, -MAX, MAX);
        int b = clamp(BASE - (int) corr, -MAX, MAX);
        drive(Motor.A, a);
        drive(Motor.B, b);
    }

    private void avoid() {
        // stop & back up
        Motor.A.stop(true); Motor.B.stop();
        Delay.msDelay(200);
        Motor.A.backward(); Motor.B.backward();
        Delay.msDelay(400);
        Motor.A.stop(true); Motor.B.stop();

        // scan ±45°
        Motor.A.rotate(-90, true); Motor.B.rotate(90);
        Delay.msDelay(200);
        float left = ultra.getDistance();
        Motor.A.rotate(180, true); Motor.B.rotate(-180);
        Delay.msDelay(200);
        float right = ultra.getDistance();

        // re-center
        Motor.A.rotate(-90, true); Motor.B.rotate(90);
        Delay.msDelay(200);

        // pick best direction
        if (left > right) {
            Motor.A.rotate(-180, true); Motor.B.rotate(180);
        } else {
            Motor.A.rotate(180, true); Motor.B.rotate(-180);
        }
        Delay.msDelay(200);

        // drive around
        Motor.A.forward(); Motor.B.forward();
        Delay.msDelay(600);

        // turn back toward line
        if (left > right) {
            Motor.A.rotate(180, true); Motor.B.rotate(-180);
        } else {
            Motor.A.rotate(-180, true); Motor.B.rotate(180);
        }
        Delay.msDelay(200);
    }

    private int clamp(int v, int lo, int hi) {
        return v < lo ? lo : v > hi ? hi : v;
    }

    private void drive(BaseRegulatedMotor m, int s) {
        m.setSpeed(Math.abs(s));
        if (s >= 0) m.forward();
        else        m.backward();
    }
}

class SensorPoller implements Runnable {
    private final SampleProvider sp;
    private final float[] buf;
    private final PIDController pid;

    public SensorPoller(EV3ColorSensor cs, PIDController p) {
        sp  = cs.getRedMode();
        buf = new float[sp.sampleSize()];
        pid = p;
    }

    public void run() {
        while (!Button.ESCAPE.isDown()) {
            sp.fetchSample(buf, 0);
            pid.updateValue(buf[0]);
            Delay.msDelay(20);
        }
    }
}




// & "C:\Program Files\Java\jdk1.7.0_80\bin\javac.exe" -classpath "..\lib\ev3classes.jar" -d "bin" "src\LineFollower.java"
// & "C:\Program Files\Java\jdk1.7.0_80\bin\jar.exe" cfe "LineFollower.jar" LineFollower -C bin .

