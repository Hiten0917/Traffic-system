#define SDL_MAIN_HANDLED
#define _USE_MATH_DEFINES
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <iostream>
#include <string>
#include <iomanip>

// ==================================================================================
//                                  CONFIG & CONSTANTS
// ==================================================================================

// Screen & Geometry
const int SZ = 950;                // Slightly larger window
const int CTR = SZ / 2;            // Center Point
const int LW = 70;                 // Lane Width
const int RW = 210;                // Total Road Width (3 lanes * 70)
const int ENT = CTR - (RW / 2);    // Intersection Entry Line
const int EXT = CTR + (RW / 2);    // Intersection Exit Line

// Simulation Settings
const float MAX_SPD_CAR = 3.2f;
const float MAX_SPD_TRUCK = 2.0f;
const float MAX_SPD_BIKE = 4.0f;
const float ACCEL_RATE = 0.09f;
const float BRAKE_RATE = 0.25f;
const float SAFE_DIST = 90.0f;
const int SPAWN_RATE_DEFAULT = 80; // Lower = Faster Spawns

// Math
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Enums
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum LightColor { L_RED, L_YELLOW, L_GREEN };
enum VehicleType { TYPE_CAR, TYPE_TRUCK, TYPE_BIKE };

// ==================================================================================
//                                  DATA STRUCTURES
// ==================================================================================

struct Point { 
    float x, y; 
};

struct Stats {
    Uint64 totalCars;
    Uint64 totalTicks;
    float avgSpeed;
    int currentCars;
    int flowRate; // Cars per minute
};

struct Decoration {
    Point pos;
    int type; // 0 = Tree, 1 = Rock/Bush
    float size;
    SDL_Color color;
};

struct Particle {
    Point pos;
    float alpha;
    float size;
    float velX, velY;
    bool active;
    SDL_Color color;
};

// ==================================================================================
//                                  MATH HELPERS
// ==================================================================================

Point lerp(Point p1, Point p2, float t) {
    return { p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t };
}

Point getBezierPoint(Point p0, Point p1, Point p2, float t) {
    Point a = lerp(p0, p1, t);
    Point b = lerp(p1, p2, t);
    return lerp(a, b, t);
}

float getDistance(Point a, Point b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

float getAngle(Point a, Point b) {
    return std::atan2(b.y - a.y, b.x - a.x) * (180.0f / (float)M_PI);
}

// Get the center coordinate of a specific lane at the boundary
Point getLaneStart(Direction dir, int subLane) {
    float offset = (subLane * LW) + (LW / 2.0f);
    if (dir == NORTH) return { (float)ENT + offset, -150.0f };
    if (dir == SOUTH) return { (float)EXT - offset, (float)SZ + 150.0f };
    if (dir == EAST) return { (float)SZ + 150.0f, (float)ENT + offset };
    if (dir == WEST) return { -150.0f, (float)EXT - offset };
    return { 0, 0 };
}

Point getLaneEnd(Direction dir, int subLane) {
    // End points are just the start points of the opposite flow
    // But we need to map the Direction to where it *exits*
    float offset = (subLane * LW) + (LW / 2.0f);
    if (dir == NORTH) return { (float)ENT + offset, -150.0f };      // Exiting North
    if (dir == SOUTH) return { (float)EXT - offset, (float)SZ + 150.0f }; // Exiting South
    if (dir == EAST) return { (float)SZ + 150.0f, (float)ENT + offset };  // Exiting East
    if (dir == WEST) return { -150.0f, (float)EXT - offset };       // Exiting West
    return { 0, 0 };
}

// ==================================================================================
//                                  TRAFFIC SYSTEMS
// ==================================================================================

class TrafficManager {
public:
    Direction activePhase;
    LightColor currentLight;
    Uint64 lastSwitchTime;
    
    // Cycle Durations
    Uint32 durGreen = 4500;
    Uint32 durYellow = 2000;
    Uint32 durAllRed = 1000; 

    TrafficManager() : activePhase(NORTH), currentLight(L_GREEN), lastSwitchTime(0) {}

    void update(Uint32 dt) {
        Uint64 now = SDL_GetTicks();
        Uint32 elapsed = now - lastSwitchTime;

        if (currentLight == L_GREEN && elapsed > durGreen) {
            currentLight = L_YELLOW;
            lastSwitchTime = now;
        } else if (currentLight == L_YELLOW && elapsed > durYellow) {
            currentLight = L_RED;
            lastSwitchTime = now;
        } else if (currentLight == L_RED && elapsed > durAllRed) {
            // Switch Phase: N -> E -> S -> W
            activePhase = (Direction)((activePhase + 1) % 4);
            currentLight = L_GREEN;
            lastSwitchTime = now;
        }
    }

    LightColor getSignal(Direction dir) {
        if (dir == activePhase) return currentLight;
        return L_RED;
    }
};

// ==================================================================================
//                                  VEHICLE CLASS
// ==================================================================================

class Vehicle {
public:
    int id;
    Point pos;
    float angle;
    float speed;
    float maxSpeed;
    
    int laneIdx;     
    Direction origin;
    VehicleType type;
    
    SDL_Color color;
    float length, width;
    
    // State Flags
    bool braking;
    bool signalingLeft;
    bool signalingRight;
    bool active;
    
    // Pathfinding
    bool turning;
    float turnProgress;
    int targetLaneSub;
    Point turnStart;
    Point turnEnd;
    Point turnControl;

    // Stats
    Uint64 spawnTime;

    int sub() const { return laneIdx % 3; }

    Vehicle(int _id, int _laneIdx, Direction _origin) {
        id = _id;
        laneIdx = _laneIdx;
        origin = _origin;
        active = true;
        turning = false;
        turnProgress = 0.0f;
        braking = false;
        signalingLeft = false;
        signalingRight = false;
        spawnTime = SDL_GetTicks();
        
        // Randomize Properties
        int rType = rand() % 100;
        if (rType < 65) {
            type = TYPE_CAR; maxSpeed = MAX_SPD_CAR; length = 52; width = 28;
            color = { (Uint8)(50 + rand()%150), (Uint8)(100 + rand()%155), 255, 255 };
        } else if (rType < 85) {
            type = TYPE_TRUCK; maxSpeed = MAX_SPD_TRUCK; length = 85; width = 36;
            color = { (Uint8)(200 + rand()%20), (Uint8)(140 + rand()%20), 60, 255 };
        } else {
            type = TYPE_BIKE; maxSpeed = MAX_SPD_BIKE; length = 25; width = 12;
            color = { 100, 255, 100, 255 };
        }
        speed = maxSpeed;

        // Set Start Position
        Point start = getLaneStart(origin, sub());
        pos = start;
        
        if (origin == NORTH) angle = 90;
        else if (origin == SOUTH) angle = 270;
        else if (origin == EAST) angle = 180;
        else angle = 0;
    }
};

// ==================================================================================
//                                  SIMULATION STATE
// ==================================================================================

std::vector<Vehicle> traffic;
std::vector<Particle> particles;
std::vector<Decoration> scenery;
TrafficManager tm;
Stats stats = {0, 0, 0, 0, 0};
int carIdCounter = 0;
Uint64 frameCount = 0;

// ==================================================================================
//                                  AI LOGIC
// ==================================================================================

// Helper to count cars in a target area (for smart spawning and routing)
int getLaneDensity(Direction dir, int subLane) {
    int count = 0;
    // Scan exit lanes
    Point exitPoint = getLaneEnd(dir, subLane);
    
    for (const auto& v : traffic) {
        if (!v.active) continue;
        float d = getDistance(v.pos, exitPoint);
        // If car is within 300px of the exit point of that lane
        if (d < 300.0f) {
            // Rough direction check to ensure it's actually in that lane
            // (Simplified spatial hash)
            count++;
        }
    }
    return count;
}

bool checkYielding(Vehicle& v) {
    // Only applies if turning LEFT (Lane 0) and light is green
    if (v.sub() != 0 || !v.turning) return false;

    Direction opposingDir = (Direction)((v.origin + 2) % 4); // The guy opposite us
    
    // Check for oncoming cars going STRAIGHT
    for (const auto& other : traffic) {
        if (!other.active || other.origin != opposingDir) continue;
        
        // If oncoming car is going straight (Lane 1) or Turning Right (Lane 2)
        // And is CLOSE to the intersection
        if (other.sub() != 0) { // Assuming lane 0 is left turn only
            float distToCenter = getDistance(other.pos, { (float)CTR, (float)CTR });
            if (distToCenter < 200.0f && distToCenter > 30.0f) {
                // Potential collision
                return true; 
            }
        }
    }
    return false;
}

bool checkSafeties(Vehicle& v) {
    bool shouldStop = false;

    // 1. Traffic Lights
    LightColor sig = tm.getSignal(v.origin);
    
    // Rule: Right turn on Red (Lane 2) allowed
    bool rightOnRed = (v.sub() == 2 && v.turning);
    // Rule: Left turn Yield (Handled separately, but stops here if Red)
    
    if (sig != L_GREEN && !rightOnRed) {
        float dist = 0;
        bool approaching = false;

        if (v.origin == NORTH) { dist = ENT - v.pos.y; approaching = (v.pos.y < ENT); }
        else if (v.origin == SOUTH) { dist = v.pos.y - EXT; approaching = (v.pos.y > EXT); }
        else if (v.origin == EAST) { dist = v.pos.x - EXT; approaching = (v.pos.x > EXT); }
        else { dist = ENT - v.pos.x; approaching = (v.pos.x < ENT); }

        if (approaching && dist < 120.0f && dist > -20.0f) {
            if (sig == L_RED) shouldStop = true;
            else if (sig == L_YELLOW && dist > 50.0f) shouldStop = true; // Stop if safe
        }
    }

    // 2. Yielding for Left Turns
    if (!shouldStop && sig == L_GREEN && v.sub() == 0 && v.turning) {
        if (checkYielding(v)) shouldStop = true;
    }

    // 3. Car-Following (Collision Avoidance)
    if (!shouldStop) {
        for (const auto& other : traffic) {
            if (v.id == other.id || !other.active) continue;
            
            float d = getDistance(v.pos, other.pos);
            float safe = SAFE_DIST;
            if (v.type == TYPE_TRUCK) safe = 140.0f;
            if (v.speed > 2.5f) safe += 30.0f;

            if (d < safe) {
                // Is 'other' in front?
                float ang = std::atan2(other.pos.y - v.pos.y, other.pos.x - v.pos.x) * (180.0f/M_PI);
                float diff = std::abs(ang - v.angle);
                if (diff > 180) diff = 360 - diff;
                
                if (diff < 60.0f) {
                    shouldStop = true;
                    break;
                }
            }
        }
    }

    return shouldStop;
}

void attemptSpawn() {
    static std::mt19937 gen(std::random_device{}());
    
    // Try to find a valid lane
    for (int attempts = 0; attempts < 10; ++attempts) {
        int r = rand() % 12;
        Direction dir = (Direction)(r / 3);
        int sub = r % 3;

        // Check for spawn blocking (visual overlap)
        bool blocked = false;
        Point spawnPt = getLaneStart(dir, sub);
        for (const auto& v : traffic) {
            if (!v.active) continue;
            if (getDistance(v.pos, spawnPt) < 180.0f) {
                blocked = true; 
                break;
            }
        }
        
        if (blocked) continue;

        // Spawn Vehicle
        Vehicle newCar(++carIdCounter, r, dir);
        traffic.push_back(newCar);
        stats.totalCars++;
        return;
    }
}

void initScenery() {
    // Generate trees in the green zones
    for (int i = 0; i < 20; i++) {
        Decoration d;
        d.type = 0;
        d.size = 20.0f + (rand() % 20);
        d.color = { (Uint8)(20+rand()%40), (Uint8)(80+rand()%50), 20, 255 };
        
        // Pick a quadrant
        int q = rand() % 4;
        float x, y;
        if (q == 0) { x = rand() % ENT; y = rand() % ENT; } // Top Left
        else if (q == 1) { x = EXT + rand() % (SZ - EXT); y = rand() % ENT; } // Top Right
        else if (q == 2) { x = rand() % ENT; y = EXT + rand() % (SZ - EXT); } // Bot Left
        else { x = EXT + rand() % (SZ - EXT); y = EXT + rand() % (SZ - EXT); } // Bot Right

        d.pos = { x, y };
        scenery.push_back(d);
    }
}

// ==================================================================================
//                                  RENDERER
// ==================================================================================

void drawTree(SDL_Renderer* ren, const Decoration& d) {
    // Shadow
    SDL_SetRenderDrawColor(ren, 10, 30, 10, 100);
    SDL_FRect shad = { d.pos.x + 5, d.pos.y + 5, d.size, d.size };
    SDL_RenderFillRect(ren, &shad);

    // Leaves (Layered)
    SDL_SetRenderDrawColor(ren, d.color.r, d.color.g, d.color.b, 255);
    SDL_FRect l1 = { d.pos.x, d.pos.y, d.size, d.size };
    SDL_RenderFillRect(ren, &l1);
    
    SDL_SetRenderDrawColor(ren, d.color.r+20, d.color.g+20, d.color.b+20, 255);
    SDL_FRect l2 = { d.pos.x + d.size*0.2f, d.pos.y + d.size*0.2f, d.size*0.6f, d.size*0.6f };
    SDL_RenderFillRect(ren, &l2);
}

// Helper to draw lane names (L, S, R) without external fonts
void drawLaneLabel(SDL_Renderer* ren, char type, float x, float y) {
    float s = 25.0f; // Size of letter
    float hs = s / 2.0f;
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 120); // Semi-transparent white

    // Center the coordinate
    x -= hs; y -= hs;

    if (type == 'L') {
        SDL_RenderLine(ren, x, y, x, y+s);      // Vert
        SDL_RenderLine(ren, x, y+s, x+s, y+s);  // Horiz
    } 
    else if (type == 'S') {
        SDL_RenderLine(ren, x+s, y, x, y);          // Top
        SDL_RenderLine(ren, x, y, x, y+hs);         // Top-Left Vert
        SDL_RenderLine(ren, x, y+hs, x+s, y+hs);    // Mid
        SDL_RenderLine(ren, x+s, y+hs, x+s, y+s);   // Bot-Right Vert
        SDL_RenderLine(ren, x+s, y+s, x, y+s);      // Bot
    } 
    else if (type == 'R') {
        SDL_RenderLine(ren, x, y+s, x, y);          // Vert
        SDL_RenderLine(ren, x, y, x+s-5, y);        // Top
        SDL_RenderLine(ren, x+s-5, y, x+s, y+hs/2); // Curve 1
        SDL_RenderLine(ren, x+s, y+hs/2, x+s-5, y+hs); // Curve 2
        SDL_RenderLine(ren, x+s-5, y+hs, x, y+hs);  // Mid
        SDL_RenderLine(ren, x, y+hs, x+s, y+s);     // Leg
    }
}

void drawRoadMarkings(SDL_Renderer* ren) {
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);

    // Dashed Lines
    for (int i = 1; i < 3; i++) {
        float off = ENT + i * LW;
        // Vert
        for (int k = 0; k < SZ; k += 50) 
            if (k < ENT || k > EXT) SDL_RenderLine(ren, off, k, off, k + 25);
        // Horiz
        for (int k = 0; k < SZ; k += 50) 
            if (k < ENT || k > EXT) SDL_RenderLine(ren, k, off, k + 25, off);
    }

    // Stop Lines (Thick)
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_FRect stopN = { ENT, ENT - 4, RW, 4 }; SDL_RenderFillRect(ren, &stopN);
    SDL_FRect stopS = { ENT, EXT, RW, 4 }; SDL_RenderFillRect(ren, &stopS);
    SDL_FRect stopW = { ENT - 4, ENT, 4, RW }; SDL_RenderFillRect(ren, &stopW);
    SDL_FRect stopE = { EXT, ENT, 4, RW }; SDL_RenderFillRect(ren, &stopE);

    // Zebra Crossings
    SDL_SetRenderDrawColor(ren, 180, 180, 180, 255);
    for(float i = 0; i < RW; i+=20) {
        // North
        SDL_FRect zN = { (float)ENT + i + 5, (float)ENT - 30, 10, 26 }; SDL_RenderFillRect(ren, &zN);
        // South
        SDL_FRect zS = { (float)ENT + i + 5, (float)EXT + 4, 10, 26 }; SDL_RenderFillRect(ren, &zS);
        // West
        SDL_FRect zW = { (float)ENT - 30, (float)ENT + i + 5, 26, 10 }; SDL_RenderFillRect(ren, &zW);
        // East
        SDL_FRect zE = { (float)EXT + 4, (float)ENT + i + 5, 26, 10 }; SDL_RenderFillRect(ren, &zE);
    }

    // Lane Naming (L = Left, S = Straight, R = Right)
    float halfLn = LW / 2.0f;
    
    // North Lanes (Heading South) - Labels placed above stop line
    drawLaneLabel(ren, 'L', ENT + (0 * LW) + halfLn, ENT - 80);
    drawLaneLabel(ren, 'S', ENT + (1 * LW) + halfLn, ENT - 80);
    drawLaneLabel(ren, 'R', ENT + (2 * LW) + halfLn, ENT - 80);

    // South Lanes (Heading North) - Labels placed below stop line
    drawLaneLabel(ren, 'L', EXT - (0 * LW) - halfLn, EXT + 80);
    drawLaneLabel(ren, 'S', EXT - (1 * LW) - halfLn, EXT + 80);
    drawLaneLabel(ren, 'R', EXT - (2 * LW) - halfLn, EXT + 80);

    // West Lanes (Heading East) - Labels placed left of stop line
    drawLaneLabel(ren, 'L', ENT - 80, EXT - (0 * LW) - halfLn);
    drawLaneLabel(ren, 'S', ENT - 80, EXT - (1 * LW) - halfLn);
    drawLaneLabel(ren, 'R', ENT - 80, EXT - (2 * LW) - halfLn);

    // East Lanes (Heading West) - Labels placed right of stop line
    drawLaneLabel(ren, 'L', EXT + 80, ENT + (0 * LW) + halfLn);
    drawLaneLabel(ren, 'S', EXT + 80, ENT + (1 * LW) + halfLn);
    drawLaneLabel(ren, 'R', EXT + 80, ENT + (2 * LW) + halfLn);
}

void drawVehicleShape(SDL_Renderer* ren, const Vehicle& v) {
    SDL_SetRenderDrawColor(ren, v.color.r, v.color.g, v.color.b, 255);
    float w = v.length, h = v.width;
    bool vert = (std::abs(std::sin(v.angle * M_PI / 180.0f)) > 0.7f);
    if (vert) std::swap(w, h);

    float cx = v.pos.x; float cy = v.pos.y;
    SDL_FRect body = { cx - w/2, cy - h/2, w, h };
    SDL_RenderFillRect(ren, &body);

    // Glass / Roof
    SDL_SetRenderDrawColor(ren, 20, 20, 30, 100);
    float hood = (vert ? h : w) * 0.22f;
    SDL_FRect glass;
    
    // Rotate glass logic roughly
    if (v.angle > 315 || v.angle < 45) glass = { cx, cy - h/2 + 3, w/2 - 2, h - 6 }; // Right
    else if (v.angle > 45 && v.angle < 135) glass = { cx - w/2 + 3, cy, w - 6, h/2 - 2 }; // Down
    else if (v.angle > 135 && v.angle < 225) glass = { cx - w/2 + 2, cy - h/2 + 3, w/2 - 2, h - 6 }; // Left
    else glass = { cx - w/2 + 3, cy - h/2 + 2, w - 6, h/2 - 2 }; // Up
    SDL_RenderFillRect(ren, &glass);

    // Brake Lights
    if (v.braking) {
        SDL_SetRenderDrawColor(ren, 255, 0, 0, 255);
        SDL_FRect b1, b2;
        float bW = 5, bH = 5;
        // Determine rear based on angle
        if (v.angle > 315 || v.angle < 45) { // Right -> Rear is Left
            b1 = { cx - w/2, cy - h/2, bW, bH }; b2 = { cx - w/2, cy + h/2 - bH, bW, bH };
        } else if (v.angle > 45 && v.angle < 135) { // Down -> Rear is Top
            b1 = { cx - w/2, cy - h/2, bW, bH }; b2 = { cx + w/2 - bW, cy - h/2, bW, bH };
        } else if (v.angle > 135 && v.angle < 225) { // Left -> Rear is Right
            b1 = { cx + w/2 - bW, cy - h/2, bW, bH }; b2 = { cx + w/2 - bW, cy + h/2 - bH, bW, bH };
        } else { // Up -> Rear is Bottom
            b1 = { cx - w/2, cy + h/2 - bH, bW, bH }; b2 = { cx + w/2 - bW, cy + h/2 - bH, bW, bH };
        }
        SDL_RenderFillRect(ren, &b1); SDL_RenderFillRect(ren, &b2);
    }
}

void drawTrafficLight(SDL_Renderer* ren, float x, float y, LightColor s) {
    SDL_FRect housing = { x, y, 24, 60 };
    SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
    SDL_RenderFillRect(ren, &housing);
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
    SDL_RenderRect(ren, &housing);

    SDL_FRect rR = { x+4, y+4, 16, 16 };
    SDL_FRect rY = { x+4, y+22, 16, 16 };
    SDL_FRect rG = { x+4, y+40, 16, 16 };

    SDL_SetRenderDrawColor(ren, 60, 0, 0, 255); SDL_RenderFillRect(ren, &rR);
    SDL_SetRenderDrawColor(ren, 60, 60, 0, 255); SDL_RenderFillRect(ren, &rY);
    SDL_SetRenderDrawColor(ren, 0, 60, 0, 255); SDL_RenderFillRect(ren, &rG);

    if(s==L_RED) { SDL_SetRenderDrawColor(ren, 255, 0, 0, 255); SDL_RenderFillRect(ren, &rR); }
    if(s==L_YELLOW) { SDL_SetRenderDrawColor(ren, 255, 200, 0, 255); SDL_RenderFillRect(ren, &rY); }
    if(s==L_GREEN) { SDL_SetRenderDrawColor(ren, 0, 255, 0, 255); SDL_RenderFillRect(ren, &rG); }
}

void drawDashboard(SDL_Renderer* ren) {
    // Bottom Panel
    SDL_FRect panel = { 0, SZ - 40, SZ, 40 };
    SDL_SetRenderDrawColor(ren, 30, 30, 40, 230);
    SDL_RenderFillRect(ren, &panel);
    SDL_SetRenderDrawColor(ren, 100, 100, 200, 255);
    SDL_RenderLine(ren, 0, SZ-40, SZ, SZ-40);

    // Since we don't have TTF loaded in this snippet, we use simple graphics for stats
    // Draw "Flow Rate" Bar
    float flowPct = std::min(1.0f, stats.flowRate / 20.0f);
    SDL_FRect bar = { 100, SZ - 25, 200 * flowPct, 10 };
    SDL_SetRenderDrawColor(ren, 0, 255, 100, 255);
    SDL_RenderFillRect(ren, &bar);
    
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_FRect border = { 100, SZ - 25, 200, 10 };
    SDL_RenderRect(ren, &border);

    // Active Cars dots
    for(int i=0; i<std::min(stats.currentCars, 50); i++) {
        SDL_FRect d = { 350.0f + (i*8), (float)SZ-25, 4, 4 };
        SDL_RenderFillRect(ren, &d);
    }
}

// ==================================================================================
//                                  MAIN LOOP
// ==================================================================================

int main(int argc, char** argv) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow("Ultimate Traffic Simulator 2025", SZ, SZ, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, NULL);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    initScenery();
    bool running = true;
    
    // Performance Timer
    Uint64 perfLast = SDL_GetTicks();

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
        }

        // Timer
        Uint64 now = SDL_GetTicks();
        float dt = (now - perfLast) / 1000.0f; // Seconds
        perfLast = now;
        frameCount++;

        // Update Systems
        tm.update(16);
        
        // Spawn Logic
        if (frameCount % SPAWN_RATE_DEFAULT == 0) attemptSpawn();

        // Stats Update
        stats.currentCars = (int)traffic.size();
        if (frameCount % 60 == 0) {
            stats.flowRate = (int)((stats.totalCars / (now/1000.0f)) * 60); // Cars per minute
        }

        // Vehicle Logic
        for (auto& v : traffic) {
            if (!v.active) continue;

            // Decision Making
            bool blocked = checkSafeties(v);
            v.braking = blocked;

            // Physics Integration
            if (blocked) {
                v.speed = std::max(0.0f, v.speed - BRAKE_RATE);
            } else {
                v.speed = std::min(v.maxSpeed, v.speed + ACCEL_RATE);
            }

            // Movement State Machine
            if (!v.turning) {
                // Moving Straight on Lane
                float rad = v.angle * (M_PI / 180.0f);
                v.pos.x += std::cos(rad) * v.speed;
                v.pos.y += std::sin(rad) * v.speed;

                // Intersection Trigger
                // Check if inside the intersection box
                bool inIntersection = (v.pos.x > ENT && v.pos.x < EXT && v.pos.y > ENT && v.pos.y < EXT);
                
                if (inIntersection) {
                    v.turning = true;
                    v.turnStart = v.pos;
                    v.turnControl = { (float)CTR, (float)CTR }; // Default control is center

                    Direction tDir; 
                    int tSub = 1;

                    // --- SMART ROUTING LOGIC ---
                    if (v.sub() == 1) { // Center Lane
                        // "Straight" logic with Dynamic Lane Selection
                        tDir = (Direction)((v.origin + 2) % 4); 
                        
                        // Check which exit lane is freer (0 or 2)
                        // Note: Center lane (1) is forbidden for destination in this logic
                        int d0 = getLaneDensity(tDir, 0);
                        int d2 = getLaneDensity(tDir, 2);
                        
                        tSub = (d0 <= d2) ? 0 : 2; 

                        // S-Curve Control Point
                        Point endT = getLaneEnd(tDir, tSub);
                        // Bias control point toward target to make it an "S"
                        v.turnControl = lerp(v.turnStart, endT, 0.4f); 
                    } 
                    else if (v.sub() == 0) { // Left Turn
                        tDir = (Direction)((v.origin + 1) % 4);
                        tSub = 0; // Fast lane
                    } 
                    else { // Right Turn
                        tDir = (Direction)((v.origin + 3) % 4);
                        tSub = 2; // Slow lane
                    }
                    
                    v.targetLaneSub = tSub;
                    v.turnEnd = getLaneEnd(tDir, tSub);
                }
            } else {
                // Bezier Turning Logic
                v.turnProgress += (v.speed * 0.002f); // Speed controls turn rate
                
                if (v.turnProgress >= 1.0f) {
                    // Turn Complete - Despawn for loop simplicity
                    // In a bigger sim, they would return to "Straight" state
                    v.active = false; 
                } else {
                    Point oldPos = v.pos;
                    v.pos = getBezierPoint(v.turnStart, v.turnControl, v.turnEnd, v.turnProgress);
                    v.angle = getAngle(oldPos, v.pos);
                }
            }

            // Boundary Check
            if (v.pos.x < -150 || v.pos.x > SZ+150 || v.pos.y < -150 || v.pos.y > SZ+150) {
                v.active = false;
            }

            // Particle Gen
            if (v.speed > 0.5f && v.type != TYPE_BIKE && rand()%15==0) {
                Particle p;
                p.pos = v.pos;
                p.alpha = 200;
                p.size = 4;
                p.active = true;
                p.velX = ((rand()%10)-5)/10.0f;
                p.velY = ((rand()%10)-5)/10.0f;
                particles.push_back(p);
            }
        }

        // Particle Update
        for (auto& p : particles) {
            p.pos.x += p.velX;
            p.pos.y += p.velY;
            p.alpha -= 5;
            p.size += 0.15f;
            if (p.alpha <= 0) p.active = false;
        }

        // Cleanup Vectors
        traffic.erase(std::remove_if(traffic.begin(), traffic.end(), [](const Vehicle& v){ return !v.active; }), traffic.end());
        particles.erase(std::remove_if(particles.begin(), particles.end(), [](const Particle& p){ return !p.active; }), particles.end());

        // ==================================================================================
        //                                  DRAW
        // ==================================================================================
        SDL_SetRenderDrawColor(ren, 30, 30, 35, 255);
        SDL_RenderClear(ren);

        // 1. Background (Grass) - FIX: Using SDL_FRect
        SDL_SetRenderDrawColor(ren, 25, 45, 25, 255);
        SDL_FRect bg[4] = {
            {0, 0, (float)ENT, (float)ENT}, 
            {(float)EXT, 0, (float)ENT, (float)ENT}, 
            {0, (float)EXT, (float)ENT, (float)ENT}, 
            {(float)EXT, (float)EXT, (float)ENT, (float)ENT}
        };
        for(auto& b : bg) SDL_RenderFillRect(ren, &b);

        // Scenery (Trees)
        for (const auto& d : scenery) drawTree(ren, d);

        // 2. Roads
        SDL_SetRenderDrawColor(ren, 55, 55, 60, 255);
        SDL_FRect rV = {(float)ENT, 0, (float)RW, (float)SZ};
        SDL_FRect rH = {0, (float)ENT, (float)SZ, (float)RW};
        SDL_RenderFillRect(ren, &rV);
        SDL_RenderFillRect(ren, &rH);

        drawRoadMarkings(ren);

        // 3. Dynamic Entities
        // Particles
        for (const auto& p : particles) {
            SDL_SetRenderDrawColor(ren, 150, 150, 150, (Uint8)p.alpha);
            SDL_FRect pr = { p.pos.x, p.pos.y, p.size, p.size };
            SDL_RenderFillRect(ren, &pr);
        }

        // Cars
        for (const auto& v : traffic) drawVehicleShape(ren, v);

        // Traffic Lights
        // NW
        drawTrafficLight(ren, ENT - 40, ENT - 80, tm.getSignal(NORTH));
        // SE
        drawTrafficLight(ren, EXT + 16, EXT + 20, tm.getSignal(SOUTH));
        // NE
        drawTrafficLight(ren, EXT + 20, ENT - 80, tm.getSignal(EAST));
        // SW
        drawTrafficLight(ren, ENT - 40, EXT + 20, tm.getSignal(WEST));

        // UI Layer
        drawDashboard(ren);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}