#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>

// Function to calculate the Bezier point
float bezierPoint(float p0, float p1, float p2, float t) {
    return pow(1 - t, 2) * p0 + 2 * (1 - t) * t * p1 + pow(t, 2) * p2;
}

int main() {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Create a window
    SDL_Window* window = SDL_CreateWindow("Bezier Curve", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 500, 500, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // Create a renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Control points for the quadratic Bezier curve
    float p0x = 100, p0y = 300; // Start point
    float p1x = 250, p1y = 100; // Control point
    float p2x = 400, p2y = 300; // End point

    // Clear the screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw the Bezier curve
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (float t = 0; t <= 1; t += 0.001) {
        float x = bezierPoint(p0x, p1x, p2x, t);
        float y = bezierPoint(p0y, p1y, p2y, t);
        SDL_RenderDrawPoint(renderer, static_cast<int>(x), static_cast<int>(y));
    }

    // Render the scene
    SDL_RenderPresent(renderer);

    // Wait for a few seconds
    SDL_Delay(5000);

    // Clean up
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}