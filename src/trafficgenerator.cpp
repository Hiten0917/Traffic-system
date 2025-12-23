#define SDL_MAIN_HANDLED
#include <winsock2.h>
#include <SDL3/SDL.h>
#include <iostream>
#include <thread>
#include <atomic>

#pragma comment(lib, "ws2_32.lib")

std::atomic<bool> lightTrigger{false};

// Background thread to listen for the Generator
void networkListener() {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr = {AF_INET, htons(5000), INADDR_ANY};
    bind(s, (struct sockaddr*)&addr, sizeof(addr));
    listen(s, 3);

    while (true) {
        SOCKET client = accept(s, NULL, NULL);
        if (client != INVALID_SOCKET) {
            char buf[10];
            if (recv(client, buf, 10, 0) > 0) {
                lightTrigger = true; // Signal the main loop to switch lights
            }
            closesocket(client);
        }
    }
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Connection Test", 600, 600, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, NULL);
    
    std::thread(networkListener).detach();
    bool isNorthGreen = true;

    while (true) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) if (e.type == SDL_EVENT_QUIT) return 0;

        // If generator sent a message, swap the lights
        if (lightTrigger) {
            isNorthGreen = !isNorthGreen;
            lightTrigger = false;
            std::cout << "Generator triggered a light change!" << std::endl;
        }

        SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);
        SDL_RenderClear(ren);

        // Draw simple visual feedback (Two circles)
        // North/South Light
        SDL_SetRenderDrawColor(ren, isNorthGreen ? 0 : 255, isNorthGreen ? 255 : 0, 0, 255);
        SDL_FRect rectNS = {275, 100, 50, 50};
        SDL_RenderFillRect(ren, &rectNS);

        // East/West Light
        SDL_SetRenderDrawColor(ren, isNorthGreen ? 255 : 0, isNorthGreen ? 0 : 255, 0, 255);
        SDL_FRect rectEW = {275, 450, 50, 50};
        SDL_RenderFillRect(ren, &rectEW);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    return 0;
}