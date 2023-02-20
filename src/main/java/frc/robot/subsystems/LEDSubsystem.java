
/*
 * TO-DO: Get num of LEDs and update constants
 */

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED LED;
    private AddressableLEDBuffer LEDBuffer;

    private int rainbowFirstHue;

  public LEDSubsystem() {
    LED = new AddressableLED(Constants.LEDsConstants.LED_PORT);
    LEDBuffer = new AddressableLEDBuffer(Constants.LEDsConstants.LED_NUMBER);

    LED.setLength(LEDBuffer.getLength());

    LED.setData(LEDBuffer);
    LED.start();

    rainbowFirstHue = 1;
  }
  
  public Command setBlue() {
    return new SequentialCommandGroup(
      new InstantCommand( () -> {
          for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 47, 194, 235);
          }

          LED.setData(LEDBuffer);
        }
      )
    );
  }

  public Command setRed() {
    return new SequentialCommandGroup(
      new InstantCommand( () -> {
          for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 235, 47, 47);
          }

          LED.setData(LEDBuffer);
        }
      )
    );
  }

  public Command setYellow() {
    return new SequentialCommandGroup(
      new InstantCommand( () -> {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
          LEDBuffer.setRGB(i, 235, 192, 52);
        }

          LED.setData(LEDBuffer);
        }
      )
    );
  }

  public Command setPurple() {
    return new SequentialCommandGroup(
      new InstantCommand( () -> {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
          LEDBuffer.setRGB(i, 208, 52, 235);
        }

          LED.setData(LEDBuffer);
        }
      )
    );
  }
  
  public Command RGBMode() {
    return new SequentialCommandGroup(
      new InstantCommand( () -> {
          for (int i = 0; i < LEDBuffer.getLength(); i++) {
            final var hue = (rainbowFirstHue + (i * 180 / LEDBuffer.getLength())) % 180;

            LEDBuffer.setHSV(i, hue, 255, 128);
          }
          rainbowFirstHue += 3;
          rainbowFirstHue %= 180;

          LED.setData(LEDBuffer);
        }
      )
    );
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
