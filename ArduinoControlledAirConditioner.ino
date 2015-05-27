#include <Arduino.h>

#include "CommandHandler.h"
#include "ArduinoTimer.h"
#include "MegunoLink.h"

#include "Streaming.h" // Easier sending stuff to serial port

// Libraries for Dallas temperature sensors and one wire communication that they use
#include "OneWire.h"
#include "DallasTemperature.h"
#include "Servo.h"
#include "EEPROMStore.h"



// -----------------------------------------------------------
// Heart beat LED flashes every now and then so we know the
// program is still alive!
const int HeartBeatLEDPin = 13; // Pin the LED is connected to
ArduinoTimer HeartBeatTimer;
const int HeartBeatInterval = 300; // Time between toggling LED [milliseconds]


// -----------------------------------------------------------
// Persisted settings. We save a few things into the eeprom 
// so that if power is lost we can get the configuration back 
// again. Configuration includes things like the position of 
// the servo and temperature target. 
// All the settings to be saved get put into a structure so we 
// can use the MegunoLink Pro library to store the data in the
// eeprom
const int EEPROMAddressForConfiguration = 0; // Address where settings are saved in the eeprom. 
struct ConfigurationData 
{
  // When true, we'll control the room temperature. If false, the user has
  // disabled the controller. 
  bool EnableTemperatureController; 

  // Servo position to use to turn the air condition on, fan
  // low cooling, and high cooling. 
  int OffPosition;
  int FanPosition;
  int LowPosition;
  int HighPosition;

  // Controls speed that the servo moves to turn the air-con dial
  int ServoDelay;

  // Target operating temperature [degrees Celsius]
  float TargetTemperature;

  // Temperature change required to change the air con position. 
  float Hysteresis;

  // Setup sensible default values. 
  void Reset()
  {
    // By default, temperature control is active. 
    EnableTemperatureController = true; 

    OffPosition = 0; 
    FanPosition = 0;
    LowPosition = 0; 
    HighPosition = 0; 
    ServoDelay = 50; // ms
    TargetTemperature = 18.0; // deg C
    Hysteresis = 1; // deg C
  }
};

EEPROMStore<ConfigurationData> Configuration;

// -----------------------------------------------------------
// Temperature measurement
const int TemperatureSensorPin = 10; // pin for one-wire interface to temperature sensor
const uint8_t TemperatureSensorBusIndex = 0; // position of the temperature sensor on the one wire bus

OneWire OneWireInterface(TemperatureSensorPin);
DallasTemperature DallasTemperatureBus(&OneWireInterface);


void SetupTemperatureSensor();
float GetCurrentTemperature();

/* Configures the Dallas temperature sensor and finds its address */
void SetupTemperatureSensor()
{
  DallasTemperatureBus.begin();
  DeviceAddress TemperatureSensorAddress;
  Serial << F("Searching for Dallas temperature sensor at index ") << TemperatureSensorBusIndex << '\n';
  bool FoundSensor = DallasTemperatureBus.getAddress(TemperatureSensorAddress, TemperatureSensorBusIndex);
  if (FoundSensor)
  {
    // Change the resolution of the temperature sensor. Give the desired
    // resolution as the number of bits: 8, 9, 10, 11, or 12 are usually
    // supported. 
    DallasTemperatureBus.setResolution(TemperatureSensorAddress, 12);

    Serial.println(F("Found Dallas temperature sensor"));
    Serial << F("Current temperature = ") << GetCurrentTemperature() << '\n';
  }
  else
  {
    Serial.println(F("Unable to find Dallas temperature sensor"));
  }
}

/* Returns the current temperature in degrees Celsius */
float GetCurrentTemperature()
{
  DallasTemperatureBus.requestTemperatures();
  return DallasTemperatureBus.getTempCByIndex(TemperatureSensorBusIndex);
}

// -----------------------------------------------------------
// Air conditioner control

// Pin that the servo is connected to.
const int ServoPin = 9;
Servo DialServo;

enum DialPositions { DialOff, DialFanOnly, DialLow, DialHigh };

int GetServoPosition(DialPositions NewPosition);
void SetDialPosition(DialPositions NewPosition);

void BeginDialServo()
{
  DialServo.attach(ServoPin);
}

/* Move the servo. We detach afterwards to avoid noisy jitters. */
void MoveDialServoTo(int nNewServoPosition)
{
  DialServo.write(nNewServoPosition);
  delay(Configuration.Data.ServoDelay);
}

void EndDialServo()
{
  delay(1000); // Give the servo time to finish moving. 
  DialServo.detach();
}


/* Get the servo position that corresponds to the given dial position. */
int GetServoPosition(DialPositions NewPosition)
{
  switch (NewPosition)
  {
  case DialOff:
  default:
    return Configuration.Data.OffPosition;

  case DialFanOnly:
    return Configuration.Data.FanPosition;

  case DialLow:
    return Configuration.Data.LowPosition;

  case DialHigh:
    return Configuration.Data.HighPosition;
  }
}

/* Set the AC dial position */
void SetDialPosition(DialPositions NewPosition)
{
  static DialPositions CurrentPosition = DialOff;

  int nStartPosition = GetServoPosition(CurrentPosition);
  int nEndPosition = GetServoPosition(NewPosition);

  if (nStartPosition != nEndPosition)
  {
    BeginDialServo();

    if (nStartPosition < nEndPosition)
    {
      for (int nCurrentPosition = nStartPosition; nCurrentPosition < nEndPosition; ++nCurrentPosition)
      {
        MoveDialServoTo(nCurrentPosition);
      }
    }
    else if (nStartPosition > nEndPosition)
    {
      for (int nCurrentPosition = nStartPosition; nCurrentPosition > nEndPosition; --nCurrentPosition)
      {
        MoveDialServoTo(nCurrentPosition);
      }
    }

    EndDialServo();
  }

  CurrentPosition = NewPosition;
}

void UpdateInterfacePanel()
{
  static ArduinoTimer InterfacePanelTimer;

  if (InterfacePanelTimer.TimePassed_Milliseconds(300))
  {
    InterfacePanel ACControllerUIPanel;

    if (Configuration.Data.EnableTemperatureController)
    {
      ACControllerUIPanel.HideControl(F("DisabledIndicatorPicture"));
      ACControllerUIPanel.ShowControl(F("EnabledIndicatorPicture"));
    }
    else
    {
      ACControllerUIPanel.ShowControl(F("DisabledIndicatorPicture"));
      ACControllerUIPanel.HideControl(F("EnabledIndicatorPicture"));
    }
  }
}

// -----------------------------------------------------------
// Temperature control 

// keep track of the time we last updated the controller
ArduinoTimer ControlUpdateTimer;
const int UpdateRate = 1000; // How often we check to update the dial position [milliseconds]

void UpdateTemperatureControl()
{
  if (Configuration.Data.EnableTemperatureController && ControlUpdateTimer.TimePassed_Milliseconds(UpdateRate))
  {
    float CurrentTemperature = GetCurrentTemperature();
    float HighThreshold = Configuration.Data.TargetTemperature + Configuration.Data.Hysteresis;
    float LowThreshold = Configuration.Data.TargetTemperature - Configuration.Data.Hysteresis;

    if (CurrentTemperature < LowThreshold)
    {
      SetDialPosition(DialOff);
    }
    else if (CurrentTemperature > HighThreshold)
    {
      SetDialPosition(DialLow);
    }
  }
}

// -----------------------------------------------------------
// Command handler receives serial messages from MegunoLink Pro
// to control the air conditioner

// The CommandHandler receives, decodes and dispatches serial commands. 
CommandHandler<> SerialHandler(Serial);

InterfacePanel ACControllerUIPanel;

// !EnableController 1/0\r to enable/disable the controller. 
void Cmd_EnableController(CommandParameter &p)
{
  if (p.NextParameterAsInteger(0) == 1)
  {
    Configuration.Data.EnableTemperatureController = true;
    ACControllerUIPanel.HideControl(F("DisabledIndicatorPicture"));
    ACControllerUIPanel.ShowControl(F("EnabledIndicatorPicture"));
  }
  else
  {
    Configuration.Data.EnableTemperatureController = false;
    ACControllerUIPanel.ShowControl(F("DisabledIndicatorPicture"));
    ACControllerUIPanel.HideControl(F("EnabledIndicatorPicture"));
  }

  Serial << F("Temperature controller is ") 
    << (Configuration.Data.EnableTemperatureController ? F("ON") : F("OFF"))
    << '\n';
}

// !SetDialPosition 0|1|2|3\n
// When the dial position is set manually, we also disable temperature control.
void Cmd_SetDialPosition(CommandParameter &p)
{
  int nDialPosition = p.NextParameterAsInteger(0);
  Configuration.Data.EnableTemperatureController = false;
  switch (nDialPosition)
  {
  case 0:
  default:
    SetDialPosition(DialOff);
    break;

  case 1:
    SetDialPosition(DialFanOnly);
    break;

  case 2:
    SetDialPosition(DialLow);
    break;

  case 3:
    SetDialPosition(DialHigh);
    break;
  }
  Serial << F("Set dial position to ") << nDialPosition << '\n';
}

// !SetServoDialPosition (dial position: 0|1|2|3) (servo position)\n
// Updates the servo position that the servo moves to for a dial position. 
void Cmd_SetServoPositionForDialPosition(CommandParameter & p)
{
  int nDialPosition = p.NextParameterAsInteger(0);
  int nServoPosition = p.NextParameterAsInteger(0);

  Serial.print(F("Set servo position for dial "));

  switch (nDialPosition)
  {
  case 0:
  default:
    Configuration.Data.OffPosition = nServoPosition;
    Serial.print(F("off"));
    break;

  case 1:
    Configuration.Data.FanPosition = nServoPosition;
    Serial.print(F("fan-only"));
    break;

  case 2:
    Configuration.Data.LowPosition = nServoPosition;
    Serial.print(F("low"));
    break;

  case 3:
    Configuration.Data.HighPosition = nServoPosition;
    Serial.print(F("high"));
    break;
  }

  Configuration.Save();

  BeginDialServo();
  MoveDialServoTo(nServoPosition);
  EndDialServo();

  Serial << F(" to ") << nServoPosition << '\n';
}

// !SetTemperature value\r: sets the temperature set-point. 
void Cmd_SetTemperature(CommandParameter &p)
{
  Configuration.Data.TargetTemperature = (float) p.NextParameterAsDouble();
  Serial <<  F("Temperature set point is: ") << Configuration.Data.TargetTemperature << F(" degrees Celsius\n");
  Configuration.Save();
}

// !SetServoDelay value\n
// Sets the servo movement speed as a delay between steps. The delay is in milliseconds
void Cmd_SetServoDelay(CommandParameter &p)
{
  Configuration.Data.ServoDelay = p.NextParameterAsInteger(50);
  Serial << F("Set servo delay to ") << Configuration.Data.ServoDelay << '\n';
  Configuration.Save();
}

// !SetHysteresis value\n
// Sets the minimum temperature change required before the controller acts. 
void Cmd_SetHysteresis(CommandParameter &p)
{
  Configuration.Data.Hysteresis = (float)p.NextParameterAsDouble(1.0);
  Serial << F("Set hysteresis to ") << Configuration.Data.Hysteresis << '\n';
  Configuration.Save();
}

void Cmd_DumpConfig(CommandParameter &p)
{
  Serial << F("Current Configuration:\n");
  Serial << F("Temperature control enabled: ") << (Configuration.Data.EnableTemperatureController ? 'Y' : 'N') << '\n';
  Serial << F("Off position: ") << Configuration.Data.OffPosition << '\n';
  Serial << F("Fan position: ") << Configuration.Data.FanPosition << '\n';
  Serial << F("Low position: ") << Configuration.Data.LowPosition << '\n';
  Serial << F("High position: ") << Configuration.Data.HighPosition << '\n';
  Serial << F("Servo delay: ") << Configuration.Data.ServoDelay << '\n';
  Serial << F("Hysteresis: ") << Configuration.Data.Hysteresis << '\n';

}

/* Setup the serial commands we'll handle. These are sent my MegunoLink Pro. 
The SerialHandler takes care of matching the command messages and calling
the function registered here with AddCommand */
void SetupSerialCommands()
{
  SerialHandler.AddCommand(F("EnableController"), Cmd_EnableController);
  SerialHandler.AddCommand(F("SetDialPosition"), Cmd_SetDialPosition);
  SerialHandler.AddCommand(F("SetServoDialPosition"), Cmd_SetServoPositionForDialPosition);
  SerialHandler.AddCommand(F("SetServoDelay"), Cmd_SetServoDelay);
  SerialHandler.AddCommand(F("SetTemperature"), Cmd_SetTemperature);
  SerialHandler.AddCommand(F("SetHysteresis"), Cmd_SetHysteresis);
  SerialHandler.AddCommand(F("DumpConfig"), Cmd_DumpConfig);
}

// -----------------------------------------------------------
// Send data to MegunoLink Pro for plotting
// Periodically send data to MegunoLink Pro for plotting
const int PlotUpdateInterval = 300; // Time between plot updates [milliseconds]
ArduinoTimer PlotTimer;

void PlotData()
{
  if (PlotTimer.TimePassed_Milliseconds(PlotUpdateInterval))
  {
    TimePlot TemperatureData;

    TemperatureData.SendData("Hysteresis+", Configuration.Data.TargetTemperature + Configuration.Data.Hysteresis, Plot::Black, Plot::Dashed);
    TemperatureData.SendData("Hysteresis-", Configuration.Data.TargetTemperature - Configuration.Data.Hysteresis, Plot::Black, Plot::Dashed);
    TemperatureData.SendData("SetPoint", Configuration.Data.TargetTemperature, Plot::Blue);
    TemperatureData.SendData("Temperature", GetCurrentTemperature(), Plot::Red);

    Table TemperatureTable;
    TemperatureTable.SendData("Hysteresis", Configuration.Data.Hysteresis);
    TemperatureTable.SendData("SetPoint", Configuration.Data.TargetTemperature);
    TemperatureTable.SendData("Temperature", GetCurrentTemperature());


  }


}

// -----------------------------------------------------------
// Main Arduino setup routine
void setup()
{
  Serial.begin(9600);
  pinMode(HeartBeatLEDPin, OUTPUT);

  Configuration.Load();

  Serial.println(F("Configuration"));
  Serial << F("Controller enabled: ") << (Configuration.Data.EnableTemperatureController ? 'Y' : 'N') << '\n';
  Serial << F("Servo off position: ") << Configuration.Data.ServoDelay << '\n';
  Serial << F("Servo fan position: ") << Configuration.Data.FanPosition << '\n';
  Serial << F("Servo low position: ") << Configuration.Data.LowPosition << '\n';
  Serial << F("Servo high position: ") << Configuration.Data.HighPosition << '\n';
  Serial << F("Servo delay: ") << Configuration.Data.ServoDelay << '\n';
  Serial << F("Target temperature: ") << Configuration.Data.TargetTemperature << '\n';
  Serial << F("Hysteresis: ") << Configuration.Data.Hysteresis << '\n';


  SetupTemperatureSensor();
  SetupSerialCommands();
}

// -----------------------------------------------------------
// Main Arduino loop routine
void loop()
{
  // Flash led to let humans know we are still alive!
  if (HeartBeatTimer.TimePassed_Milliseconds(HeartBeatInterval))
  {
    digitalWrite(HeartBeatLEDPin, !digitalRead(HeartBeatLEDPin));
  }

  // Handle and dispatch any serial commands
  SerialHandler.Process(); 

  // Adjust the temperature controller. 
  UpdateTemperatureControl();

  // Send plot data to MegunoLink Pro
  PlotData();

  // Refresh controls on the interface panel
  UpdateInterfacePanel();

  if (ControlUpdateTimer.TimePassed_Seconds(3))
  {
    Serial <<F("Current temperature is ") << GetCurrentTemperature() << F(" degrees Celsius\n");
  }

}
