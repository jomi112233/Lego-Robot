import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollowrA {

    public static void main(String[] args) {
        UltraThread ultraThread = new UltraThread();

        Thread motorThread = new Thread(new MotorThread(ultraThread)); 
        Thread ultraSensorThread = new Thread(ultraThread);
        Thread LightThread = new Thread();

        ultraSensorThread.start();
        motorThread.start();
        LightThread.start();
    }
}

class MotorThread implements Runnable  {
    private UltraThread ultraThread;

    public MotorThread(UltraThread ultraThread) {
        this.ultraThread = ultraThread;
    }

    public void run() {
        while (true) {
            float distance = ultraThread.getDistance();
            if (distance <= 0.5f) {
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                LCD.drawString("toimii", 0, 1);
            } else {
                Motor.A.forward();
                Motor.B.forward();
            }
            LCD.drawString("Tacho: " + Motor.A.getTachoCount(), 0, 2);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }      
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
    private float distance = Float.MAX_VALUE;

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (true) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0];

            //LCD.clear();
            LCD.drawString("Distance: " + distance, 0, 0);

            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }     
    }
}
