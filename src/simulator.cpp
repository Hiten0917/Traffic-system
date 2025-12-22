#define SDL_MAIN_HANDLED
#include <winsock2.h> // Must come before SDL or other headers
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_ttf.h>
#include <iostream>
#include <vector>
#include <string>

// Vehicle Structure
struct Vehicle {
    float x, y;
    int lane;
};

// Global State
std::vector<Vehicle> vehicles;
SDL_Mutex* vehicleMutex = nullptr;
bool running = true;

// Networking Thread (Windows Sockets)
int socketThread(void* data) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
    
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    
    sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(5000);

    bind(s, (struct sockaddr*)&server, sizeof(server));
    listen(s, 3);

    while (running) {
        SOCKET client = accept(s, NULL, NULL);
        if (client != INVALID_SOCKET) {
            char buffer[10] = {0};
            int bytes = recv(client, buffer, 10, 0);
            if (bytes > 0) {
                int lane = atoi(buffer);
                
                SDL_LockMutex(vehicleMutex);
                // Lane mapping (4 lanes across 800px)
                float spawnX = (float)((lane - 1) * 200 + 80); 
                vehicles.push_back({ spawnX, -50.0f, lane });
                SDL_UnlockMutex(vehicleMutex);
            }
            closesocket(client);
        }
    }
    closesocket(s);
    WSACleanup();
    return 0;
}

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        return -1;
    }
    TTF_Init();
    
    SDL_Window* window = SDL_CreateWindow("Queue Simulator", 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    
    vehicleMutex = SDL_CreateMutex();
    SDL_Thread* netThread = SDL_CreateThread(socketThread, "NetThread", NULL);

    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) running = false;
        }

        // Draw Background
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);

        // Draw Lane Dividers
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
        for(int i = 1; i < 4; i++) {
            SDL_RenderLine(renderer, (float)(i * 200), 0.0f, (float)(i * 200), 600.0f);
        }

        // Update and Draw Vehicles
        SDL_LockMutex(vehicleMutex);
        SDL_SetRenderDrawColor(renderer, 255, 215, 0, 255); // Gold/Yellow
        for (auto it = vehicles.begin(); it != vehicles.end();) {
            SDL_FRect rect = { it->x, it->y, 40.0f, 60.0f };
            SDL_RenderFillRect(renderer, &rect);
            
            it->y += 3.0f; // Movement
            
            if (it->y > 650.0f) it = vehicles.erase(it);
            else ++it;
        }
        SDL_UnlockMutex(vehicleMutex);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    // Cleanup
    SDL_WaitThread(netThread, NULL);
    SDL_DestroyMutex(vehicleMutex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 0;
}