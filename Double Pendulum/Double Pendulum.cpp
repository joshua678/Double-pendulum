#include <SDL.h>
#include <iostream>
#include <windows.h>
#include <vector>
#include <cmath>
#include <array>
#include <algorithm>
#include <random>
#include <chrono>
#include <omp.h>

const int screenWidth = 1000;
const int screenHeight = 1000;

const double timescale = 1;
// more time steps per frame means the simulation will become be more accurate
const int timeSteps = 100;

//scale of 1 metre in pixels
const double scale = screenWidth / 4.5;

struct RGB {
    UINT8 red = 0;
    UINT8 green = 0;
    UINT8 blue = 0;
};

struct circle {
public:
    double x = 0;
    double y = 0;
    double radius = 0.04;
    RGB colour;
};

struct doublePendulum {
public:
    double x = screenWidth / (2 * scale);
    double y = screenHeight / (2 * scale);
    circle topCircle;
    circle bottomCircle;
    double angle1 = 0, angle2 = 0;
    double angularVel1 = 0, angularVel2 = 0;
    double angularAccel1 = 0, angularAccel2 = 0;
    const double point1Mass = 2, point2Mass = 2;
    const double rod1Length = 1, rod2Length = 1;
    const double rod1Thickness = 5, rod2Thickness = 5; //in pixels, not metres

    void setColour(UINT8 red, UINT8 green, UINT8 blue) {
        topCircle.colour.red = red;
        topCircle.colour.green = green;
        topCircle.colour.blue = blue;
        bottomCircle.colour.red = red;
        bottomCircle.colour.green = green;
        bottomCircle.colour.blue = blue;
    }
};

void shuffleVector(std::vector<doublePendulum*>& vec) {
    // Use random_device to obtain a seed for the random number engine
    std::random_device rd;
    std::default_random_engine engine(rd());

    // Shuffle the vector of pointers
    std::shuffle(vec.begin(), vec.end(), engine);
}

void drawLine(SDL_Renderer* renderer, SDL_Texture* squareTexture, double x1, double y1, double x2, double y2, double thickness) {
    double xDist = x2 - x1;
    double yDist = y2 - y1;
    double dist = sqrt(xDist * xDist + yDist * yDist);
    double angle = atan2(yDist, xDist) * (180/M_PI);

    SDL_Rect dstRect = { (int)x1, (int)y1, (int)dist, (int)thickness};

    SDL_Point pivot = { 0, 0 };

    // Render the line with 45-degree rotation
    SDL_RenderCopyEx(renderer, squareTexture, NULL, &dstRect, angle, &pivot, SDL_FLIP_NONE);
}

inline std::array<double, 2> doublePendulumAngularAccelerations(double angle1, double angle2, double angularVel1, double angularVel2, const double point1Mass, const double point2Mass, const double rod1Length, const double rod2Length) {
    const double g = 9.81;
    double angularAccelerationUpper =
        (-g * (2 * point1Mass + point2Mass) * sin(angle1)
            - point2Mass * g * sin(angle1 - 2 * angle2)
            - 2 * sin(angle1 - angle2) * point2Mass * (angularVel2 * angularVel2 + angularVel1 * angularVel1 * rod1Length * cos(angle1 - angle2)))
        / (rod2Length * (2 * point1Mass + point2Mass - point2Mass * cos(2 * angle1 - 2 * angle2)));

    double angularAccelerationLower =
        (2 * sin(angle1 - angle2)
            * (angularVel1 * angularVel1 * rod1Length * (point1Mass + point2Mass)
                + g * (point1Mass + point2Mass) * cos(angle1)
                + angularVel2 * angularVel2 * rod2Length * point2Mass * cos(angle1 - angle2)))
        / (rod2Length * (2 * point1Mass + point2Mass
            - point2Mass * cos(2 * angle1 - 2 * angle2)));

    return { angularAccelerationUpper, angularAccelerationLower };
}

inline void wrapAngle(double& angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Double Pendulum", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screenHeight, screenHeight, SDL_WINDOW_SHOWN);
    if (!window) {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_Log("Failed to create renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Event e;
    bool quit = false;

    SDL_Surface* whiteCircleSurface = SDL_LoadBMP("../images/White_Circle.bmp");
    SDL_Texture* whiteCircleTexture = SDL_CreateTextureFromSurface(renderer, whiteCircleSurface);
    SDL_SetTextureBlendMode(whiteCircleTexture, SDL_BLENDMODE_BLEND);

    SDL_Surface* squareSurface = SDL_CreateRGBSurface(0, 1, 1, 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
    Uint32 color = SDL_MapRGB(squareSurface->format, 0, 0, 0);
    ((Uint32*)squareSurface->pixels)[0] = color;
    SDL_Texture* squareTexture = SDL_CreateTextureFromSurface(renderer, squareSurface);

    Uint64 start, end, elapsed;
    double deltaTime = 0;

    //the objects at the end of the vector are drawn first
    std::vector<doublePendulum*> pendulums;

    int totalPendulums = 100;
    double frequencyR = 5.0 * (2 * M_PI) / totalPendulums;
    double frequencyG = 7.0 * (2 * M_PI) / totalPendulums;
    double frequencyB = 11.0 * (2 * M_PI) / totalPendulums;

    for (int i = 0; i < totalPendulums; i++) {
        doublePendulum* pendulum = new doublePendulum();

        int r = (sin(frequencyR * i) + 1) * 127.5;
        int g = (sin(frequencyG * i) + 1) * 127.5;
        int b = (cos(frequencyB * i) + 1) * 127.5;

        pendulum->setColour(r, g, b);
        pendulum->angle1 = M_PI/1.5 + i * 0.00000000001;
        pendulum->angle2 = M_PI/1.5 + i * 0.00000000001;

        pendulums.push_back(pendulum);
    }




    shuffleVector(pendulums);

    int mousePosX, mousePosY;

    Uint64 beginning = SDL_GetPerformanceCounter();

    double timer = 0;

    while (!quit) {
        start = SDL_GetPerformanceCounter();

        Uint32 mouseState = SDL_GetMouseState(&mousePosX, &mousePosY);

        // Event handling
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }


        if (timer > 2) {
#pragma omp parallel for
            for (int i = 0; i < pendulums.size(); i++) {
                doublePendulum& currentPendulum = *pendulums[i];
                for (int j = 0; j < timeSteps; j++) {
                    std::array<double, 2> angularAccelerations = { 0,0 };
                    angularAccelerations = doublePendulumAngularAccelerations(
                        currentPendulum.angle1, currentPendulum.angle2, currentPendulum.angularVel1,
                        currentPendulum.angularVel2, currentPendulum.point1Mass, currentPendulum.point2Mass,
                        currentPendulum.rod1Length, currentPendulum.rod2Length);
                    currentPendulum.angularAccel1 = angularAccelerations[0];
                    currentPendulum.angularAccel2 = angularAccelerations[1];
                    currentPendulum.angularVel1 += (currentPendulum.angularAccel1 * deltaTime * timescale) / (double)timeSteps;
                    currentPendulum.angularVel2 += (currentPendulum.angularAccel2 * deltaTime * timescale) / (double)timeSteps;
                    currentPendulum.angle1 += (currentPendulum.angularVel1 * deltaTime * timescale) / (double)timeSteps;
                    currentPendulum.angle2 += (currentPendulum.angularVel2 * deltaTime * timescale) / (double)timeSteps;
                }
                wrapAngle(currentPendulum.angle1);
                wrapAngle(currentPendulum.angle2);

                currentPendulum.topCircle.x = currentPendulum.rod1Length * sin(currentPendulum.angle1);
                currentPendulum.topCircle.y = -currentPendulum.rod1Length * cos(currentPendulum.angle1);

                currentPendulum.bottomCircle.x = currentPendulum.topCircle.x + currentPendulum.rod2Length * sin(currentPendulum.angle2);
                currentPendulum.bottomCircle.y = currentPendulum.topCircle.y - currentPendulum.rod2Length * cos(currentPendulum.angle2);
                //SDL_Log("\nangle1: %f\nangle2: %f", currentPendulum.angle1, currentPendulum.angle2);
            }
        }

        // Clear the screen
        SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
        SDL_RenderClear(renderer);

        // Draw the objects
        for (int i = 0; i < pendulums.size(); i++) {
            doublePendulum& currentPendulum = *pendulums[i];

            drawLine(renderer, squareTexture,
                currentPendulum.x * scale - (currentPendulum.rod1Thickness / 2) * sin(currentPendulum.angle1 - (M_PI / 2)), screenHeight - currentPendulum.y * scale - (currentPendulum.rod1Thickness / 2) * cos(currentPendulum.angle1 - (M_PI / 2)),
                (currentPendulum.topCircle.x + currentPendulum.x) * scale - (currentPendulum.rod1Thickness / 2) * sin(currentPendulum.angle1 - (M_PI / 2)), screenHeight - (currentPendulum.topCircle.y + currentPendulum.y) * scale - (currentPendulum.rod1Thickness / 2) * cos(currentPendulum.angle1 - (M_PI / 2)),
                currentPendulum.rod1Thickness);
            drawLine(renderer, squareTexture,
                (currentPendulum.topCircle.x + currentPendulum.x) * scale - (currentPendulum.rod2Thickness / 2) * sin(currentPendulum.angle2 - (M_PI / 2)), screenHeight - (currentPendulum.topCircle.y + currentPendulum.y) * scale - (currentPendulum.rod2Thickness / 2) * cos(currentPendulum.angle2 - (M_PI / 2)),
                (currentPendulum.bottomCircle.x + currentPendulum.x) * scale - (currentPendulum.rod2Thickness / 2) * sin(currentPendulum.angle2 - (M_PI / 2)), screenHeight - (currentPendulum.bottomCircle.y + currentPendulum.y) * scale - (currentPendulum.rod2Thickness / 2) * cos(currentPendulum.angle2 - (M_PI / 2)),
                currentPendulum.rod2Thickness);
        }

        for (int i = 0; i < pendulums.size(); i++) {
            doublePendulum& currentPendulum = *pendulums[i];

            SDL_Rect dstRect = { (currentPendulum.topCircle.x - currentPendulum.topCircle.radius + currentPendulum.x) * scale,
                screenHeight -(currentPendulum.topCircle.y + currentPendulum.topCircle.radius + currentPendulum.y) * scale,
                2 * currentPendulum.topCircle.radius * scale, 2 * currentPendulum.topCircle.radius * scale };
            SDL_SetTextureColorMod(whiteCircleTexture, currentPendulum.topCircle.colour.red, currentPendulum.topCircle.colour.green, currentPendulum.topCircle.colour.blue);
            SDL_RenderCopy(renderer, whiteCircleTexture, NULL, &dstRect);

            SDL_Rect dstRect2 = { (currentPendulum.bottomCircle.x - currentPendulum.bottomCircle.radius + currentPendulum.x) * scale,
                screenHeight -(currentPendulum.bottomCircle.y + currentPendulum.bottomCircle.radius + currentPendulum.y) * scale,
                2 * currentPendulum.bottomCircle.radius * scale, 2 * currentPendulum.bottomCircle.radius * scale };
            SDL_SetTextureColorMod(whiteCircleTexture, currentPendulum.bottomCircle.colour.red, currentPendulum.bottomCircle.colour.green, currentPendulum.bottomCircle.colour.blue);
            SDL_RenderCopy(renderer, whiteCircleTexture, NULL, &dstRect2);
        }

        // Update the screen
        SDL_RenderPresent(renderer);

        //get frame time
        end = SDL_GetPerformanceCounter();
        elapsed = end - start;

        deltaTime = (double)elapsed / SDL_GetPerformanceFrequency();
        timer += deltaTime;

        if ((int)(1 / deltaTime) % 100) {
            SDL_Log("FPS: %d", (int)(1 / deltaTime));
        }
    }

    // Cleanup
    SDL_DestroyTexture(whiteCircleTexture);
    SDL_FreeSurface(whiteCircleSurface);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}