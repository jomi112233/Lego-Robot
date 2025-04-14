import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    public static final float KP = 500.0f;  // Reduced from 1000 for smoother control
    public static final float KI = 0.0f;
    public static final float KD = 50.0f;   // Reduced from 100 for smoother control
    public static final int BASE_SPEED = 150;
    public static final int MAX_SPEED = 300;
    public static final float OBSTACLE_DISTANCE = 0.5f;

    public static void main(String[] args) {
        LCD.drawString("Press any button", 0, 0);
        LCD.drawString("to start", 0, 1);
        Button.waitForAnyPress();
        LCD.clear();

        UltraThread ultraThread = new UltraThread();
        LightThread lightThread = new LightThread();

        Thread motorThread = new Thread(new MotorThread(ultraThread, lightThread));
        Thread ultraSensorThread = new Thread(ultraThread);
        Thread lightSensorThread = new Thread(lightThread);

        ultraSensorThread.start();
        lightSensorThread.start();
        motorThread.start();
    }
}

class MotorThread implements Runnable {
    private UltraThread ultraThread;
    private LightThread lightThread;
    private float integral = 0;
    private float lastError = 0;
    private float targetValue = 0.5f;

    public MotorThread(UltraThread ultraThread, LightThread lightThread) {
        this.ultraThread = ultraThread;
        this.lightThread = lightThread;
    }

    public void run() {
        while (!Button.ESCAPE.isDown()) {
            float distance = ultraThread.getDistance();
            float currentValue = lightThread.getLight();

            if (distance <= LineFollower.OBSTACLE_DISTANCE) {
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                Delay.msDelay(1000);
                continue;
            }

            float error = targetValue - currentValue;
            integral = integral * 0.5f + error;
            float derivative = error - lastError;
            float correction = LineFollower.KP * error + LineFollower.KI * integral + LineFollower.KD * derivative;

            int speedA = (int)(LineFollower.BASE_SPEED + correction);
            int speedB = (int)(LineFollower.BASE_SPEED - correction);

            speedA = Math.min(Math.max(speedA, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);
            speedB = Math.min(Math.max(speedB, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);

            Motor.A.setSpeed(Math.abs(speedA));
            Motor.B.setSpeed(Math.abs(speedB));

            if (speedA > 0) {
                Motor.A.forward();
            } else {
                Motor.A.backward();
            }

            if (speedB > 0) {
                Motor.B.forward();
            } else {
                Motor.B.backward();
            }

            lastError = error;
            Delay.msDelay(10);
        }

        Motor.A.stop();
        Motor.B.stop();
    }
}

class LightThread implements Runnable {
    private float lightValue = Float.MAX_VALUE;
    
    public float getLight() {
        return lightValue;
    }

    public void run() {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider light = colorSensor.getAmbientMode();
        float[] lightSample = new float[light.sampleSize()];

        while (!Button.ESCAPE.isDown()) {
            light.fetchSample(lightSample, 0);
            lightValue = lightSample[0];
            LCD.drawString("Light: " + (int)(lightValue), 4, 0);
            Delay.msDelay(100);
        }
        colorSensor.close();
    }
}

class UltraThread implements Runnable {
    private float distance = Float.MAX_VALUE;

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (!Button.ESCAPE.isDown()) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0];
            Delay.msDelay(40);
        }
        ultrasonicSensor.close();
    }
}
