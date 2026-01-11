#pragma once

class Adafruit_ILI9341;
struct Palette;

void initUi(Adafruit_ILI9341& tft, const Palette* palette);
void renderStatic();
void renderDynamic();
