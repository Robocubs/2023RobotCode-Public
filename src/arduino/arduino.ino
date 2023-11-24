#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define LED_PIN_LEFT 2
#define LED_PIN_RIGHT 4
#define LED_COUNT 26

#define MAX_STACK_HEIGHT 4

Adafruit_NeoPixel controllerLeft = Adafruit_NeoPixel(
    LED_COUNT,
    LED_PIN_LEFT,
    NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel controllerRight = Adafruit_NeoPixel(
    LED_COUNT,
    LED_PIN_RIGHT,
    NEO_GRB + NEO_KHZ800);

void setup()
{
  Serial.begin(9600);
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  controllerLeft.begin();
  controllerRight.begin();
  clearAll();
  update();
}

void loop()
{
}

void receiveEvent(int bytes)
{
  String output = "";
  Serial.println(output + bytes);

  // Clear if partial signal
  if (bytes % 4 != 0)
  {
    clearWire();
    return;
  }

  for (int i = 0; i * 4 < bytes; i++)
  {
    byte id = Wire.read();
    byte red = Wire.read();
    byte green = Wire.read();
    byte blue = Wire.read();

    long color = 0x000000;
    color += (long)red << 16;
    color += (long)green << 8;
    color += (long)blue;

    if (id == 4)
    {
      setColorFrontLeft(color);
    }
    else if (id == 3)
    {
      setColorFrontRight(color);
    }
    else if (id == 2)
    {
      setColorBackLeft(color);
    }
    else if (id == 1)
    {
      setColorBackRight(color);
    }
  }

  update();
}

void clearWire()
{
  while (Wire.available() > 0)
  {
    Wire.read();
  }
}

void setColor(int led, long color)
{
  controllerLeft.setPixelColor(led, color);
  controllerRight.setPixelColor(led, color);
}

void setColorFrontLeft(long color)
{
  for (int i = LED_COUNT / 2; i < LED_COUNT; i++)
  {
    controllerLeft.setPixelColor(i, color);
  }
}

void setColorFrontRight(long color)
{
  for (int i = LED_COUNT / 2; i < LED_COUNT; i++)
  {
    controllerRight.setPixelColor(i, color);
  }
}

void setColorBackLeft(long color)
{
  for (int i = 0; i < LED_COUNT / 2; i++)
  {
    controllerLeft.setPixelColor(i, color);
  }
}

void setColorBackRight(long color)
{
  for (int i = 0; i < LED_COUNT / 2; i++)
  {
    controllerRight.setPixelColor(i, color);
  }
}

void setMultiple(int count, long color)
{
  for (int i = 0; i < count; i++)
  {
    setColor(i, color);
  }
}

void setAll(long color)
{
  setMultiple(LED_COUNT, color);
}

void clearColor(int led)
{
  setColor(led, 0);
}

void clearAll()
{
  setAll(0);
}

void update()
{
  controllerLeft.show();
  controllerRight.show();
}
