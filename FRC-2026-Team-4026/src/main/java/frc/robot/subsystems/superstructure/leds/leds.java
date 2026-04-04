package frc.robot.subsystems.superstructure.leds;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    
    // an led strip
    private AddressableLED led;
    // how many leds on the strip
    private int length;
    // a buffer with data on the led strip states
    private AddressableLEDBuffer buffer;


    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);

    private static final Distance spacing = Meters.of(1 / 120.0);
    private final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing);

    private Supplier<Double> loadingBar = ()->5.0;
    LEDPattern loadingBarPattern = LEDPattern.progressMaskLayer(()->loadingBar.get()/ length);

    LEDPattern shooterwindUPattern = rainbow.mask(LEDPattern.progressMaskLayer(()->loadingBar.get()/ length));
    



    Map<Double, Color> maskSteps = Map.of(0.0,Color.kWhite,0.125,Color.kBlack,0.25,Color.kWhite,0.375,Color.kBlack,0.5,Color.kWhite,0.625,Color.kBlack,0.75,Color.kWhite,0.875,Color.kBlack);
    LEDPattern base = LEDPattern.rainbow(255, 255);
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Hertz.of(250));

    LEDPattern pulsing = base.mask(mask);


    public Leds(){
        this.length = 60;

        this.led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();

        setDefaultCommand(loadingBarCommand());
    }

    public void updateData(){
        led.setData(buffer);
    }

    @Override
    public void periodic(){
        led.setData(buffer);
    }


    public Command runPattern(LEDPattern pattern){
        return run(()-> pattern.applyTo(buffer));
    }

    public Command rainbowCommand(){
        return run(()-> scrollingRainbow.applyTo(buffer));
    }

    public Command loadingBarCommand(){
        loadingBar = ()-> (DriverStation.getMatchTime() - getShiftEnd())/ getShiftLength() *length;
        loadingBarPattern = LEDPattern.progressMaskLayer(()->loadingBar.get()/ length);
        return run(()->loadingBarPattern.applyTo(buffer));
    }


    public Command shooterWindUpCommand(double currentVelocity, double targetVelocity){
        shooterwindUPattern = rainbow.mask(LEDPattern.progressMaskLayer(()->currentVelocity/targetVelocity));
        return run(()-> shooterwindUPattern.applyTo(buffer));
    }

 

    public Command pulsingCommand(){
        LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(100));    

        LEDPattern pulsing = LEDPattern.solid(Color.kGold).mask(mask);
        return run(()->pulsing.applyTo(buffer));
    }


    public Command breathCommand(Color color){
        return run(()->LEDPattern.solid(color).breathe(Seconds.of(1)).applyTo(buffer));
    }

    public Command correctCommand(){
       return runOnce(()->LEDPattern.solid(Color.kGreen).breathe(Seconds.of(0.25)).applyTo(buffer));
    }

    public Command aligningCommand(){
        return runOnce(()->LEDPattern.solid(Color.kRed).breathe(Seconds.of(1)).applyTo(buffer));
    }

    public void changeShiftCommand(){
        if(DriverStation.getMatchTime() < 133 && DriverStation.getMatchTime() > 130){
            run(()->LEDPattern.solid(Color.kPurple).breathe(Seconds.of(1)).applyTo(buffer));
        }
    }


    private int getShiftLength(){
        int shiftLength = 0;
        if(DriverStation.getMatchTime()< 140 && DriverStation.getMatchTime() >= 130){
            shiftLength = 10;
        } else if(DriverStation.getMatchTime() < 30){
            shiftLength = 30;
        } else{
            shiftLength = 25;
        }

        return shiftLength;
    }

    private int getShiftEnd(){
               int shiftEnd = 0;
        if(DriverStation.getMatchTime()< 140 && DriverStation.getMatchTime() >= 130){
            shiftEnd = 130;
        } else if(DriverStation.getMatchTime() < 130 && DriverStation.getMatchTime() >= 105 ){
            shiftEnd = 105;
        } else if(DriverStation.getMatchTime() <105 && DriverStation.getMatchTime()>= 80){
            shiftEnd = 80;
        } else if (DriverStation.getMatchTime() < 80 && DriverStation.getMatchTime()>=55){
            shiftEnd = 55;
        } else if (DriverStation.getMatchTime() < 55 && DriverStation.getMatchTime()>= 30){
            shiftEnd = 30;
        } else{
            shiftEnd = 0;
        }

        return shiftEnd; 
    }

}
