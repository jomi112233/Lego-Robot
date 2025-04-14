import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    private static final float KP = 500.0f;  // Reduced from 1000 for smoother control
    private static final float KI = 0.0f;
    private static final float KD = 50.0f;   // Reduced from 100 for smoother control
    private static final int BASE_SPEED = 150;
    private static final int MAX_SPEED = 300;
    private static final float OBSTACLE_DISTANCE = 0.5f;
    
    private static float integral = 0;
    private static float lastError = 0;
    private static float targetValue = 0.5f; // Default target value, will be calibrated

    public static void main(String[] args) {
        UltraThread ultraThread = new UltraThread(); // Shared instance

        Thread motorThread = new Thread(new MotorThread(ultraThread)); // Pass it to MotorThread
        Thread ultraSensorThread = new Thread(ultraThread);

        ultraSensorThread.start();
        lineSensorThread.start();
        motorThread.start();
        LightThread.start();
    }
}

class MotorThread implements Runnable {
    private UltraThread ultraThread;
    private LineThread lineThread;
    private float integral = 0;
    private float lastError = 0;

    public MotorThread(UltraThread ultraThread, LineThread lineThread) {
        this.ultraThread = ultraThread;
        this.lineThread = lineThread;
    }

    public void run() {
        while (true) {
            float distance = ultraThread.getDistance(); // safe getter
            if (distance <= 0.5f) {
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                Delay.msDelay(1000);
                continue;
            }

            // PID control for line following
            float error = targetValue - currentValue;
            integral = integral * 0.5f + error;
            float derivative = error - lastError;
            float correction = LineFollower.KP * error + LineFollower.KI * integral + LineFollower.KD * derivative;

            int speedA = (int)(LineFollower.BASE_SPEED + correction);
            int speedB = (int)(LineFollower.BASE_SPEED - correction);

            // Limit speeds
            speedA = Math.min(Math.max(speedA, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);
            speedB = Math.min(Math.max(speedB, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);

            // Set motor speeds and directions
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
    private float LightValue = Float.MAX_VALUE;
    
    public float GetLight(){
        return LightValue;
    }

    public void run(){
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider Light = colorSensor.getAmbientMode();

        float[] LightSample = new float[Light.sampleSize()];

        while (true) {
            Light.fetchSample(LightSample, 0);

            
            LCD.drawString("Light: "  + (int)(LightSample[0]), 4, 0);

            try 
            {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}


class UltraThread implements Runnable  {
    private float distance = Float.MAX_VALUE; // Default large distance

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (!Button.ESCAPE.isDown()) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0]; // Update shared variable

            //LCD.clear();
            //LCD.drawString("Distance: " + distance, 0, 0);

            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }     
    }
}
