#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_ttf.h>
#include <iostream>
#include <vector>
#include <string>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define ROAD_WIDTH 200
#define MAIN_FONT "C:/Windows/Fonts/arial.ttf" // Ensure this path is correct

// --- Structs ---
struct TrafficLight {
    float x, y;
    bool isGreen;
    bool horizontal;
};

// --- Global State ---
TrafficLight lights[4];
int lightPhase = 0; // 0: N/S Green, 1: E/W Green
Uint64 lastSwitch = 0;

// --- Function Declarations ---
void drawJunction(SDL_Renderer* renderer, TTF_Font* font);
void drawSignal(SDL_Renderer* renderer, TrafficLight& light);
void displayText(SDL_Renderer* renderer, TTF_Font* font, const char* text, float x, float y);

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return -1;
    TTF_Init();

    SDL_Window* window = SDL_CreateWindow("Modernized Junction", WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    TTF_Font* font = TTF_OpenFont(MAIN_FONT, 28);

    // Initialize Lights (N, S, E, W)
    lights[0] = { 260, 240, true, false };  // North
    lights[1] = { 515, 510, true, false };  // South
    lights[2] = { 510, 260, false, true };  // East
    lights[3] = { 240, 515, false, true };  // West

    bool running = true;
    SDL_Event event;
    lastSwitch = SDL_GetTicks();

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) running = false;
        }

        // Logic: Switch lights every 5 seconds
        if (SDL_GetTicks() - lastSwitch > 5000) {
            lightPhase = !lightPhase;
            lights[0].isGreen = lights[1].isGreen = (lightPhase == 0);
            lights[2].isGreen = lights[3].isGreen = (lightPhase == 1);
            lastSwitch = SDL_GetTicks();
        }

        // Background
        SDL_SetRenderDrawColor(renderer, 40, 100, 40, 255); // Grass
        SDL_RenderClear(renderer);

        drawJunction(renderer, font);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 0;
}

void drawJunction(SDL_Renderer* renderer, TTF_Font* font) {
    float mid = WINDOW_WIDTH / 2.0f;
    float rh = ROAD_WIDTH / 2.0f;

    // 1. Draw Asphalt
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_FRect vRoad = { mid - rh, 0, ROAD_WIDTH, WINDOW_HEIGHT };
    SDL_FRect hRoad = { 0, mid - rh, WINDOW_WIDTH, ROAD_WIDTH };
    SDL_RenderFillRect(renderer, &vRoad);
    SDL_RenderFillRect(renderer, &hRoad);

    // 2. Draw Yellow Center Lines
    SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
    SDL_FRect vLine = { mid - 2, 0, 4, WINDOW_HEIGHT };
    SDL_FRect hLine = { 0, mid - 2, WINDOW_WIDTH, 4 };
    // Clip center lines so they don't overlap the middle box
    SDL_RenderFillRect(renderer, &vLine);
    SDL_RenderFillRect(renderer, &hLine);

    // 3. Draw Stop Lines (White Bars)
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_FRect stopN = { mid - rh, mid - rh - 10, ROAD_WIDTH / 2, 10 };
    SDL_FRect stopS = { mid, mid + rh, ROAD_WIDTH / 2, 10 };
    SDL_FRect stopW = { mid - rh - 10, mid, 10, ROAD_WIDTH / 2 };
    SDL_FRect stopE = { mid + rh, mid - rh, 10, ROAD_WIDTH / 2 };
    SDL_RenderFillRect(renderer, &stopN);
    SDL_RenderFillRect(renderer, &stopS);
    SDL_RenderFillRect(renderer, &stopW);
    SDL_RenderFillRect(renderer, &stopE);

    // 4. Draw Central Intersection Box (Island look)
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_FRect island = { mid - rh, mid - rh, ROAD_WIDTH, ROAD_WIDTH };
    SDL_RenderFillRect(renderer, &island);

    // 5. Labels and Signals
    displayText(renderer, font, "NORTH", mid - 40, 20);
    displayText(renderer, font, "SOUTH", mid - 40, WINDOW_HEIGHT - 50);

    for (int i = 0; i < 4; i++) {
        drawSignal(renderer, lights[i]);
    }
}

void drawSignal(SDL_Renderer* renderer, TrafficLight& light) {
    // Housing
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_FRect body = { light.x, light.y, light.horizontal ? 50.0f : 25.0f, light.horizontal ? 25.0f : 50.0f };
    SDL_RenderFillRect(renderer, &body);

    // Red Lamp
    if (!light.isGreen) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else SDL_SetRenderDrawColor(renderer, 80, 0, 0, 255);
    SDL_FRect red = { light.x + 5, light.y + 5, 15, 15 };
    SDL_RenderFillRect(renderer, &red);

    // Green Lamp
    if (light.isGreen) SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    else SDL_SetRenderDrawColor(renderer, 0, 80, 0, 255);
    SDL_FRect green = { 
        light.horizontal ? light.x + 30 : light.x + 5, 
        light.horizontal ? light.y + 5 : light.y + 30, 
        15, 15 
    };
    SDL_RenderFillRect(renderer, &green);
}

void displayText(SDL_Renderer* renderer, TTF_Font* font, const char* text, float x, float y) {
    if (!font) return;
    SDL_Color color = { 255, 255, 255, 255 };
    SDL_Surface* surf = TTF_RenderText_Blended(font, text, 0, color);
    SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
    SDL_FRect dst = { x, y, (float)surf->w, (float)surf->h };
    SDL_RenderTexture(renderer, tex, NULL, &dst);
    SDL_DestroySurface(surf);
    SDL_DestroyTexture(tex);
}