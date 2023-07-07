// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  public static enum LED_MODE {
    Off, Solid, FIRST, Alliance, BounceAlliance
  }

  private AddressableLED _led;
  private AddressableLEDBuffer _ledBuffer;
  private final int _numLEDs;
  private Color _currentColor = Color.kBlue;

  private double _lastTime = Timer.getFPGATimestamp();
  private double _interval = 0.35;

  private LED_MODE _mode = LED_MODE.Off;
  private int _index;

  private int _bouncePosition = 0;
  private boolean _bounceGoingUp = true;
  private double _bounceInterval = 0.05;
  private final int BOUNCE_LENGTH = 12;

  public LEDStrip(int numberOfLeds, int port) {
    _led = new AddressableLED(port);
    _numLEDs = numberOfLeds;

    // Length is expensive to set, so only set it once, then just update data
    _ledBuffer = new AddressableLEDBuffer(_numLEDs);
    _led.setLength(_ledBuffer.getLength());
    _led.setData(_ledBuffer);
    _led.start();

    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, 0, 0, 0);
    }

    _led.setData(_ledBuffer);
  }

  public LEDStrip(int numberOfLeds, int port, LED_MODE defaultMode) {
    this(numberOfLeds, port);

    setMode(defaultMode);
  }

  public LEDStrip(int numberOfLeds, int port, LED_MODE defaultMode, Color defaultColor) {
    this(numberOfLeds, port);

    setColor(defaultColor);
    setMode(defaultMode);
  }

  public void setMode(LED_MODE mode) {
    _mode = mode;
  }

  public void setColor(Color color) {
    _mode = LED_MODE.Solid;
    _currentColor = color;
  }

  private void increment() {
    _index++;
    if (_index >= _ledBuffer.getLength()) {
      _index = 0;
    }
  }

  private void off() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void solid() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
        _ledBuffer.setLED(i, _currentColor);
    }
  }

  private void alliance() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      if(i % 2 == 0)
        if (DriverStation.getAlliance() == Alliance.Blue) {
          _ledBuffer.setLED(i, Color.kBlue);
        } else {
          _ledBuffer.setLED(i, Color.kRed);
        }
      else
        _ledBuffer.setLED(i, Color.kBlack);
    }
  }

  private void first() {
    double currentTime = Timer.getFPGATimestamp();
    if ((currentTime - _lastTime) > _interval) {
      _lastTime = currentTime;
      for (int i = 0; i < _ledBuffer.getLength(); i++) {
        if ((i + _index) % 3 == 0) {
          _ledBuffer.setLED(i, Color.kWhite);
        } else if ((i + _index) % 3 == 1) {
          _ledBuffer.setLED(i, Color.kFirstBlue);
        } else {
          _ledBuffer.setLED(i, Color.kFirstRed);
        }
      }

      increment();
    }
  }

  private void bounceAlliance() {
    // Credit for the core of this logic
    // https://github.com/roboblazers7617/2023Robot/blob/07953cae19044f9b52374ad3d4be689ccf277dd2/src/main/java/frc/robot/subsystems/Leds.java

    double currentTime = Timer.getFPGATimestamp();
    if ((currentTime - _lastTime) < _bounceInterval) return;
    _lastTime = currentTime;

    for (int i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setLED(i, new Color(0, 0, 0));
    }

    // the following allows the pattern to completely flow off the end; to make it stop
    // at each end, the first if statement should subtract BOUNCE_LENGTH from _ledBuffer.getLength()
    // and the else if should not subtract BOUNCE_LENGTH from 0
    if (_bounceGoingUp && _bouncePosition >= _ledBuffer.getLength()) {
      _bounceGoingUp = false;
    } else if (!_bounceGoingUp && _bouncePosition == 0 - BOUNCE_LENGTH) {
      _bounceGoingUp = true;
    }

    if (_bounceGoingUp) {
      _bouncePosition++;
    } else {
      _bouncePosition--;
    }

    int c = 255;
    for (int i = _bouncePosition; i < BOUNCE_LENGTH + _bouncePosition; i++) {
      // the following will create a gradient tail; the numbers are a tad magic, but
      // the intent is to start somewhere around 150 and end around 25 (the leds
      // we're using don't really show much difference between 200 to 255)
      if(_bounceGoingUp && i <= _bouncePosition + 5) {
        c = 220 - 33 * (6 - (i - _bouncePosition));
      } else if(!_bounceGoingUp && i >= BOUNCE_LENGTH + _bouncePosition - 5) {
        c = 385 - 33 * (i - _bouncePosition);
      }

      if(i >= 0 && i < _ledBuffer.getLength()) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
          _ledBuffer.setLED(i, new Color(0, 0, c));
        } else {
          _ledBuffer.setLED(i, new Color(c, 0, 0));
        }
      }
    }
  }

  @Override
  public void periodic() {
    switch (_mode) {
      case Solid:
        solid();
        break;
      case FIRST:
        first();
        break;
      case Alliance:
        alliance();
        break;
      case BounceAlliance:
        bounceAlliance();
        break;
      default:
        off();
        break;
    }
    _led.setData(_ledBuffer);
  }
}