#pragma once

#include<cmath>

constexpr float delta_t = 0.0003f;

float FPS = 60.f;

float SPF = 1.f / FPS;

float totalTime = 10.f;

float elapsedTime = 0.0f;

int savedPerFrames = static_cast<int>(ceil(SPF / delta_t));

bool saveMesh = true;
bool saveImage = false;

bool hasWind = true;
bool lineMode = false;
bool drawNormal = false;
bool collisionDetect = true;
bool drawExternalBBox = false;
bool drawBBox = false;
bool drawVelocity = false;
bool bodyLine = true;
bool clothLine = true;