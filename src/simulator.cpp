#define SDL_MAIN_HANDLED
#define _USE_MATH_DEFINES
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* * --- CONFIGURATION & GEOMETRY ---
 * Junction Layout: 4 Roads (A, B, C, D) 
 * Each Road: L1 (Left Turn), L2 (Straight), L3 (Free-Right)
 */
const int SZ = 900;
const int LW = 70;         // Lane Width
const int RW = 210;        // Road Width (3 * LW)
const int CTR = 450;       // Center point
const int ENT = 345;       // Entry line
const int EXT = 555;       // Exit line
const float MAX_SPD = 2.6f;
const float ACCEL = 0.04f;
const float BRAKE = 0.15f;
const float SAFE_DIST = 90.0f;

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum LightState { RED, YELLOW, GREEN }; 

struct Point { float x, y; };

struct Vehicle {
    Point pos;
    float angle;
    float speed;
    int laneIdx;     // 0-11
    Direction origin;
    SDL_Color color;
    bool active;
    bool turning;
    float turnT;     // Interpolation factor 0.0 to 1.0
    float length = 56.0f, width = 32.0f;

    // Sub-lane helper: 0=Left, 1=Straight, 2=Right
    int sub() const { return laneIdx % 3; }
};

// --- GLOBAL STATE ---
std::vector<Vehicle> traffic;
LightState lights[4] = { RED, RED, RED, RED };
int laneWeights[12] = { 0 };
int activePhase = NORTH;
Uint64 lastLightSwitch = 0;
float currentPhaseDuration = 5000.0f;
bool isYellowPending = false;

// --- MATH & UTILITY ---
Point lerp(Point p1, Point p2, float t) {
    return { p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t };
}

Point getPathPoint(Point p0, Point p1, Point p2, float t) {
    Point a = lerp(p0, p1, t);
    Point b = lerp(p1, p2, t);
    return lerp(a, b, t);
}

// --- CORE LOGIC ---
bool isPathObstructed(Vehicle& v) {
    // 1. Traffic Light Logic
    if (v.sub() != 2) {
        if (lights[v.origin] == RED || (lights[v.origin] == YELLOW && !v.turning)) {
            if (v.origin == NORTH && v.pos.y > ENT - 50 && v.pos.y < ENT) return true;
            if (v.origin == SOUTH && v.pos.y < EXT + 50 && v.pos.y > EXT) return true;
            if (v.origin == EAST  && v.pos.x < EXT + 50 && v.pos.x > EXT) return true;
            if (v.origin == WEST  && v.pos.x > ENT - 50 && v.pos.x < ENT) return true;
        }
    }

    // 2. Proximity Sensor for Vehicles in front
    for (auto& other : traffic) {
        if (&v == &other || !other.active) continue;
        float dx = v.pos.x - other.pos.x;
        float dy = v.pos.y - other.pos.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < SAFE_DIST) {
            if (v.origin == NORTH && other.pos.y > v.pos.y && std::abs(dx) < 25) return true;
            if (v.origin == SOUTH && other.pos.y < v.pos.y && std::abs(dx) < 25) return true;
            if (v.origin == EAST  && other.pos.x < v.pos.x && std::abs(dy) < 25) return true;
            if (v.origin == WEST  && other.pos.x > v.pos.x && std::abs(dy) < 25) return true;
        }
    }
    return false;
}

void spawnVehicle() {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> laneDist(0, 11);
    
    int lane, sub;
    Direction ori;
    bool valid = false;

    // Retry loop to enforce asymmetric layout
    while(!valid) {
        lane = laneDist(gen);
        ori = (Direction)(lane / 3);
        sub = lane % 3;
        valid = true;

        // --- MODIFICATION: Road A (North) Configuration ---
        // Only allow spawning in Lane 1 (AL2/Center). 
        // Lanes 0 and 2 are blocked for incoming traffic (used as Outgoing).
        if (ori == NORTH && sub != 1) valid = false;

        // --- MODIFICATION: Road C (South) Balance ---
        // Since North Lane 1 is incoming, South Lane 1 (Straight) would cause a head-on collision.
        // We restrict South to lanes 0 and 2 so they flow into the empty North outgoing lanes.
        if (ori == SOUTH && sub == 1) valid = false; 
    }

    float laneOffset = (sub * LW) + (LW / 2.0f);

    Vehicle v;
    v.laneIdx = lane;
    v.origin = ori;
    v.speed = 0.0f;
    v.active = true;
    v.turning = false;
    v.turnT = 0.0f;
    v.color = { (Uint8)(100 + rand() % 155), (Uint8)(100 + rand() % 155), (Uint8)(100 + rand() % 155), 255 };

    if (ori == NORTH) { v.pos = { (float)ENT + laneOffset, -60 }; v.angle = 90.0f; }
    else if (ori == SOUTH) { v.pos = { (float)EXT - laneOffset, SZ + 60 }; v.angle = 270.0f; }
    else if (ori == EAST)  { v.pos = { SZ + 60, (float)ENT + laneOffset }; v.angle = 180.0f; }
    else if (ori == WEST)  { v.pos = { -60, (float)EXT - laneOffset }; v.angle = 0.0f; }

    // Collision check at spawn
    for (auto& existing : traffic) {
        if (std::abs(existing.pos.x - v.pos.x) < 70 && std::abs(existing.pos.y - v.pos.y) < 70) return;
    }
    traffic.push_back(v);
}

void manageTrafficLights() {
    Uint64 now = SDL_GetTicks();
    // Use Lane 1 index for North since sub 0 and 2 are empty now
    int northWeight = laneWeights[1]; 
    bool al2Priority = (northWeight > 5 && activePhase != NORTH);

    if (now - lastLightSwitch > currentPhaseDuration || al2Priority) {
        if (!isYellowPending) {
            lights[activePhase] = YELLOW;
            currentPhaseDuration = 2000.0f; 
            lastLightSwitch = now;
            isYellowPending = true;
        } else {
            lights[activePhase] = RED;
            activePhase = al2Priority ? NORTH : (activePhase + 1) % 4;
            lights[activePhase] = GREEN;
            
            int count = 0;
            for (int i = activePhase * 3; i < (activePhase * 3) + 3; i++) count += laneWeights[i];
            currentPhaseDuration = std::max(4000.0f, count * 700.0f);

            lastLightSwitch = now;
            isYellowPending = false;
        }
    }
}

void drawRoundedRect(SDL_Renderer* ren, float x, float y, float w, float h, float angle, SDL_Color col) {
    SDL_FRect r = { x - w / 2, y - h / 2, w, h };
    SDL_SetRenderDrawColor(ren, col.r, col.g, col.b, 255);
    SDL_RenderFillRect(ren, &r);
    
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 200); 
    SDL_FRect win;
    if (angle == 0 || angle == 180) win = { x - w / 4, y - h / 3, w / 2, h / 1.5f };
    else win = { x - w / 3, y - h / 4, w / 1.5f, h / 2 };
    SDL_RenderFillRect(ren, &win);
}

int main(int argc, char** argv) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Advanced Queue Traffic Simulator", SZ, SZ, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, NULL);

    bool running = true;
    lastLightSwitch = SDL_GetTicks();
    lights[activePhase] = GREEN;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) if (e.type == SDL_EVENT_QUIT) running = false;

        if (rand() % 60 == 0) spawnVehicle();

        std::fill_n(laneWeights, 12, 0);
        for (auto& v : traffic) if (v.speed < 0.2f) laneWeights[v.laneIdx]++;

        manageTrafficLights();

        // Update Physics
        for (auto& v : traffic) {
            if (!v.active) continue;

            bool blocked = isPathObstructed(v);
            if (blocked) v.speed = std::max(0.0f, v.speed - BRAKE);
            else v.speed = std::min(MAX_SPD, v.speed + ACCEL);

            if (v.sub() == 1) { 
                // Straight Lane
                float rad = v.angle * (M_PI / 180.0f);
                v.pos.x += std::cos(rad) * v.speed;
                v.pos.y += std::sin(rad) * v.speed;
            } else {
                // Turning Logic
                float distToBox = 0;
                if (v.origin == NORTH) distToBox = v.pos.y;
                else if (v.origin == SOUTH) distToBox = SZ - v.pos.y;
                else if (v.origin == EAST) distToBox = SZ - v.pos.x;
                else distToBox = v.pos.x;

                if (distToBox > ENT - 10 && !v.turning) v.turning = true;

                if (v.turning) {
                    v.turnT += 0.006f * v.speed;
                    Point pStart = v.pos, pEnd;
                    
                    if (v.sub() == 0) { // Left Turn
                        if (v.origin == NORTH) pEnd = { (float)SZ + 100, (float)EXT - 35 };
                        else if (v.origin == SOUTH) pEnd = { -100, (float)ENT + 35 };
                        
                        // --- MODIFICATION: West turning Left to North ---
                        // Target North Lane 0 (AL1) which is now Outgoing
                        else if (v.origin == WEST) pEnd = { (float)ENT + 35, -100 }; 
                        
                        else pEnd = { (float)ENT + 35, (float)SZ + 100 };
                    } else { // Right Turn
                        if (v.origin == NORTH) pEnd = { -100, (float)ENT + 35 };
                        else if (v.origin == SOUTH) pEnd = { (float)SZ + 100, (float)EXT - 35 };
                        
                        // --- MODIFICATION: East turning Right to North ---
                        // Target North Lane 2 (AL3) which is now Outgoing
                        // ENT + 175 corresponds to the far right lane of North road
                        else if (v.origin == EAST)  pEnd = { (float)ENT + 175, -100 };
                        
                        else pEnd = { (float)ENT + 35, (float)SZ + 100 };
                    }

                    Point old = v.pos;
                    v.pos = getPathPoint(v.pos, { (float)CTR, (float)CTR }, pEnd, v.turnT);
                    v.angle = std::atan2(v.pos.y - old.y, v.pos.x - old.x) * (180.0f / M_PI);
                    if (v.turnT >= 1.0f) v.active = false;
                } else {
                    float rad = v.angle * (M_PI / 180.0f);
                    v.pos.x += std::cos(rad) * v.speed;
                    v.pos.y += std::sin(rad) * v.speed;
                }
            }
            if (v.pos.x < -150 || v.pos.x > SZ + 150 || v.pos.y < -150 || v.pos.y > SZ + 150) v.active = false;
        }

        // --- DRAWING ---
        SDL_SetRenderDrawColor(ren, 25, 60, 25, 255);
        SDL_RenderClear(ren);

        SDL_SetRenderDrawColor(ren, 45, 45, 45, 255);
        SDL_FRect roadV = { (float)ENT, 0, (float)RW, (float)SZ };
        SDL_FRect roadH = { 0, (float)ENT, (float)SZ, (float)RW };
        SDL_RenderFillRect(ren, &roadV);
        SDL_RenderFillRect(ren, &roadH);

        // Lane Markings - Visualizing the North Road Difference
        SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
        for (int i = 1; i < 3; i++) {
            for (int j = 0; j < SZ; j += 40) {
                // North Road (0 to ENT) has distinct markings logic? 
                // For simplicity, we keep standard dashed lines everywhere.
                SDL_RenderLine(ren, ENT + (i * LW), j, ENT + (i * LW), j + 20);
                SDL_RenderLine(ren, j, ENT + (i * LW), j + 20, ENT + (i * LW));
            }
        }
        
        // --- REMOVED BLUE LANE HIGHLIGHT HERE ---

        for (auto& v : traffic) {
            if (!v.active) continue;
            drawRoundedRect(ren, v.pos.x, v.pos.y, (v.angle == 0 || v.angle == 180) ? v.length : v.width, 
                                                  (v.angle == 0 || v.angle == 180) ? v.width : v.length, v.angle, v.color);
        }

        // Lights
        for (int i = 0; i < 4; i++) {
            float lx, ly;
            if (i == NORTH) { lx = ENT - 40; ly = ENT - 40; }
            else if (i == SOUTH) { lx = EXT + 15; ly = EXT + 15; }
            else if (i == EAST)  { lx = EXT + 15; ly = ENT - 40; }
            else { lx = ENT - 40; ly = EXT + 15; }

            SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
            SDL_FRect housing = { lx, ly, 25, 25 };
            SDL_RenderFillRect(ren, &housing);

            if (lights[i] == RED) SDL_SetRenderDrawColor(ren, 255, 0, 0, 255);
            else if (lights[i] == YELLOW) SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
            else SDL_SetRenderDrawColor(ren, 0, 255, 0, 255);
            
            SDL_FRect lamp = { lx + 5, ly + 5, 15, 15 };
            SDL_RenderFillRect(ren, &lamp);
        }

        SDL_RenderPresent(ren);
        SDL_Delay(16);
        traffic.erase(std::remove_if(traffic.begin(), traffic.end(), [](const Vehicle& v) { return !v.active; }), traffic.end());
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}