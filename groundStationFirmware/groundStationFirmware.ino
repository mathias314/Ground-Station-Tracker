// todo: test removing backlash

/*
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Mathew Clutter
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------------
*/

#include <Tic.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>


TicI2C tic1(14);
TicI2C tic2(15);

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

float oldAzimuth, newAzimuth = 0;
float oldElevation, newElevation = 0;

// these are used for finding the shortest path to a new point
bool negativeOldAzimuth, negativeNewAzimuth = false;

String bytes_in = "";
String GPS_String = "";


#define GPSECHO false
void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.setTimeout(5);

  Wire.begin();
  delay(20);

  tic1.exitSafeStart();
  tic2.exitSafeStart();

  // this will hopefully clear any errors present
  // if one motor mysteriously stops working,
  // try plugging into the tic controller and clearing any errors from the pololu tic controller software
  tic1.clearDriverError();
  tic2.clearDriverError();

  tic1.haltAndSetPosition(0);
  tic2.haltAndSetPosition(0);

  // end of motor setup, start of GPS setup

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 5 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);  // 5 Hz Fix update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  while(!GPS.LOCUS_ReadStatus());
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


void resetCommandTimeout()
{
  tic1.resetCommandTimeout();
  tic2.resetCommandTimeout();
}


void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}


void waitForPosition1(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic1.getCurrentPosition() != targetPosition && Serial.peek() != 'E'); // and serial read is not E (Serial.peek() ?)

  if(negativeOldAzimuth == true)
  {
    oldAzimuth = oldAzimuth + 360; // move back to 0-360 range
    negativeOldAzimuth = false;
  }
  if(negativeNewAzimuth == true)
  {
    newAzimuth = newAzimuth + 360; // move back to 0-360 range
    negativeNewAzimuth = false;
  }
  oldAzimuth = newAzimuth;
  tic1.haltAndSetPosition(0);
}


void waitForPosition2(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic2.getCurrentPosition() != targetPosition && Serial.peek() != 'E');

  oldElevation = newElevation;
  tic2.haltAndSetPosition(0);
}


void movePanAndTilt(String bytes_in)
{
 for (int index = 1; index < bytes_in.length(); index++)
      {
        if (bytes_in[index] == ',')
        {
          newAzimuth = bytes_in.substring(1, index).toFloat();
          newElevation = bytes_in.substring(index + 1).toFloat();
          break;
        }
      }

      tic1.exitSafeStart();
      tic2.exitSafeStart(); // these seem to prevent safe start error
      // the exitSafeStart only clears flag for 200ms

      int pan = movePan();
      tic1.setTargetPosition(pan);
      waitForPosition1(pan);
      
      int tilt = moveTilt();
      tic2.setTargetPosition(tilt);
      waitForPosition2(tilt);
      
      delayWhileResettingCommandTimeout(125); 
}

int movePan()
{
  tic1.exitSafeStart();

  if(abs(newAzimuth - (oldAzimuth - 360)) < abs(newAzimuth - oldAzimuth)) // check for shortest path to positions
  {
    oldAzimuth = oldAzimuth - 360;
    negativeOldAzimuth = true;
  }
  else if(abs((newAzimuth - 360) - oldAzimuth) < abs(newAzimuth - oldAzimuth))
  {
    newAzimuth = newAzimuth - 360;
    negativeNewAzimuth = true;
  }

  float delta = newAzimuth - oldAzimuth;

  return int(-119.33333 * delta); // needs to be negative due to wiring
}

int moveTilt()
{
  tic2.exitSafeStart();

  float delta = newElevation - oldElevation;

  return int(119.33333 * delta); 
}

void calibrate(String bytes_in)
{
 for (int index = 1; index < bytes_in.length(); index++)
      {
        if (bytes_in[index] == ',')
        {
          oldAzimuth = bytes_in.substring(1, index).toFloat();
          oldElevation = bytes_in.substring(index + 1).toFloat();
          break;
        }
      } 
      return;
}

void adjustPanPos()
{
  tic1.exitSafeStart();

  int adjustedPos = bytes_in.substring(1).toFloat() * 119.3333;
  tic1.setTargetPosition(adjustedPos);
  waitForPosition1(adjustedPos);
}

void adjustPanNeg()
{
  tic1.exitSafeStart();
  
  int adjustedPos = bytes_in.substring(1).toFloat() * 119.3333;
  tic1.setTargetPosition(-adjustedPos);
  waitForPosition1(-adjustedPos);
}

void adjustTiltUp(String bytes_in)
{
  tic2.exitSafeStart();

  int adjustedPos = bytes_in.substring(1).toFloat() * 119.3333;
  tic2.setTargetPosition(adjustedPos);
  waitForPosition2(adjustedPos);
}

void adjustTiltDown()
{
  tic2.exitSafeStart();

  int adjustedPos = bytes_in.substring(1).toFloat() * 119.3333;
  tic2.setTargetPosition(-adjustedPos);
  waitForPosition2(-adjustedPos);
}

void eStop()
{
  tic1.setTargetPosition(tic1.getCurrentPosition());
  tic2.setTargetPosition(tic1.getCurrentPosition());

  // maybe have to change the current azimuth/elevation after estop?
}


void getGPS()
{
  
    if(GPS.fix)
    {
      GPS_String = "" + String(GPS.latitudeDegrees, 4) + GPS.lat 
      + "," + String(GPS.longitudeDegrees, 4) + GPS.lon + "," 
      + String(GPS.altitude);

      Serial.println(GPS_String);
    }
  }



void loop()
{
  // internal gear ratio is 26.85:1 and external gear ratio is 8:1
  // this gives 180 degree rotation of 21480 microsteps
  // each degree is 119.3333 microsteps
  // each microstep is .00838 degrees
  // both gear ratios should be the same for pan and tilt, thus this should be consistent


  if (Serial.available() > 0) // request for something to happen
  {
    bytes_in = Serial.readString();
    if (bytes_in[0] == 'M') // note move is 'M' for now
    {
      movePanAndTilt(bytes_in);
    }
    else if(bytes_in[0] == 'G' && GPS.fix)
    {
      getGPS();
    }
    else if(bytes_in[0] == 'C')
    {
      calibrate(bytes_in);
    }
    else if(bytes_in[0] == 'W')
    {
      adjustTiltUp(bytes_in);
    }
    else if(bytes_in[0] == 'S')
    {
      adjustTiltDown();
    }
    else if(bytes_in[0] == 'A')
    {
      adjustPanPos();
    }
    else if(bytes_in[0] == 'D')
    {
      adjustPanNeg();
    }
    else if(bytes_in[0] == 'E')
    {
      eStop();
    }
    else
    {
      return;
    }
  }

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  // if ((c) && (GPSECHO))
  // Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
}
  
