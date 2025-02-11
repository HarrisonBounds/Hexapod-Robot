#include <SDL2/SDL.h>
#include <iostream>

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Check for joysticks
    if (SDL_NumJoysticks() < 1) {
        std::cerr << "No joysticks connected!" << std::endl;
    } else {
        // Open the first joystick
        SDL_Joystick* joystick = SDL_JoystickOpen(2);
        if (joystick == nullptr) {
            std::cerr << "Unable to open joystick! SDL_Error: " << SDL_GetError() << std::endl;
        } else {
            std::cout << "Opened joystick: " << SDL_JoystickName(joystick) << std::endl;

            // Main loop flag
            bool quit = false;

            // Event handler
            SDL_Event e;

            // While application is running
            while (!quit) {
                // Handle events on queue
                while (SDL_PollEvent(&e) != 0) {
                    // User requests quit
                    if (e.type == SDL_QUIT) {
                        quit = true;
                    }

                    // Handle joystick axis motion
                    if (e.type == SDL_JOYAXISMOTION) {
                        std::cout << "Joystick " << (int)e.jaxis.which << " axis " << (int)e.jaxis.axis << " value: " << (int)e.jaxis.value << std::endl;
                    }

                    // Handle joystick button down
                    if (e.type == SDL_JOYBUTTONDOWN) {
                        std::cout << "Joystick " << (int)e.jbutton.which << " button " << (int)e.jbutton.button << " down" << std::endl;
                    }

                    // Handle joystick button up
                    if (e.type == SDL_JOYBUTTONUP) {
                        std::cout << "Joystick " << (int)e.jbutton.which << " button " << (int)e.jbutton.button << " up" << std::endl;
                    }
                }
            }

            // Close the joystick
            SDL_JoystickClose(joystick);
        }
    }

    // Quit SDL
    SDL_Quit();

    return 0;
}