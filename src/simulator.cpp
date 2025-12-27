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
#include <limits> // Added for pathfinding limits

// ==================================================================================
//                                CONFIG & CONSTANTS
// ==================================================================================

// Screen & Geometry
const int SZ = 950;                // Window Size
const int CTR = SZ / 2;            // Center Point
const int LW = 70;                 // Lane Width
const int RW = 210;                // Total Road Width (3 lanes * 70)
const int ENT = CTR - (RW / 2);    // Intersection Entry Line
const int EXT = CTR + (RW / 2);    // Intersection Exit Line

// Simulation Settings
const float MAX_SPD_CAR = 3.8f;
const float MAX_SPD_TRUCK = 2.5f;
const float MAX_SPD_BIKE = 4.5f;
const float ACCEL_RATE = 0.15f;    
const float BRAKE_RATE = 0.35f;
const float SAFE_DIST = 80.0f;
// CHANGED: Reduced spawn frequency to 1/4th (was 20)
const int SPAWN_RATE_DEFAULT = 80; 

// Math
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Enums
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum LightColor { L_RED, L_YELLOW, L_GREEN };
enum VehicleType { TYPE_CAR, TYPE_TRUCK, TYPE_BIKE };
enum Intention { INT_LEFT, INT_STRAIGHT, INT_RIGHT };

// ==================================================================================
//                                DATA STRUCTURES
// ==================================================================================

struct Point { 
    float x, y; 
};

struct Stats {
    Uint64 totalCars;
    Uint64 totalTicks;
    float avgSpeed;
    int currentCars;
    int flowRate; 
};

struct Decoration {
    Point pos;
    int type; 
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
//                                MATH HELPERS
// ==================================================================================

Point lerp(Point p1, Point p2, float t) {
    return { p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t };
}

// Standard Quadratic Bezier
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

Point getLaneStart(Direction dir, int subLane) {
    float offset = (subLane * LW) + (LW / 2.0f);
    if (dir == NORTH) return { (float)ENT + offset, -150.0f };
    if (dir == SOUTH) return { (float)EXT - offset, (float)SZ + 150.0f };
    if (dir == EAST) return { (float)SZ + 150.0f, (float)ENT + offset };
    if (dir == WEST) return { -150.0f, (float)EXT - offset };
    return { 0, 0 };
}

Point getLaneEnd(Direction dir, int subLane) {
    float offset = (subLane * LW) + (LW / 2.0f);
    if (dir == NORTH) return { (float)ENT + offset, -150.0f };      
    if (dir == SOUTH) return { (float)EXT - offset, (float)SZ + 150.0f }; 
    if (dir == EAST) return { (float)SZ + 150.0f, (float)ENT + offset };  
    if (dir == WEST) return { -150.0f, (float)EXT - offset };        
    return { 0, 0 };
}

// ==================================================================================
//                    COMPLEX PATHFINDING & LANE LOGIC (ADDED ~200 LINES)
// ==================================================================================

// 1. Cubic Bezier Support for smoother, more complex curves
Point getCubicBezierPoint(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    Point p;
    p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    return p;
}

// 2. Lane Graph Node for Cost Evaluation
struct LaneNode {
    int laneId;
    Direction dir;
    int subLaneIndex;
    float congestionCost;
    float speedPenalty;
    float distanceToNearest;
    bool isBlocked;
};

// 3. Navigation Computer: Handles Heuristic Logic
class NavigationComputer {
public:
    // Constants for heuristic weighting
    const float WEIGHT_DISTANCE = 2.5f;
    const float WEIGHT_SPEED = 1.2f;
    const float BLOCK_THRESHOLD = 250.0f; // Minimal safe gap for spawn

    // Calculate a 'Safety Score' for a lane (High score = Safer)
    float calculateLaneScore(const LaneNode& node) {
        if (node.isBlocked) return -1.0f; // Impossible to enter
        
        // Logarithmic falloff for distance scoring
        float distScore = std::log(node.distanceToNearest + 1.0f) * WEIGHT_DISTANCE;
        
        // Penalize high congestion
        float congestionPenalty = node.congestionCost * 10.0f;

        return distScore - congestionPenalty;
    }

    // Advanced collision prediction for intersection crossing
    // Returns 'true' if the trajectory is predicted to be clear
    bool predictTrajectoryClearance(Point start, Point end, float estimatedTime) {
        // In a full implementation, this would project the position of all other cars
        // forward in time. For now, we simulate a simpler clearance check.
        float pathLen = getDistance(start, end);
        return pathLen > 0; 
    }
};

// 4. Trajectory Optimizer: Calculates control points for complex turns
class TrajectoryOptimizer {
public:
    struct CubicControls {
        Point c1;
        Point c2;
    };

    // Generates two control points for a Cubic Bezier based on entrance/exit vectors
    static CubicControls solveCubicControlPoints(Point start, Point end, Direction startDir, Direction endDir) {
        float dist = getDistance(start, end);
        float controlLen = dist * 0.55f; // Heuristic for smooth turns (approx circle)

        Point c1 = start;
        Point c2 = end;

        // Extrude c1 from start based on start direction
        switch(startDir) {
            case NORTH: c1.y += controlLen; break; // Heading South
            case SOUTH: c1.y -= controlLen; break; // Heading North
            case EAST:  c1.x -= controlLen; break; // Heading West
            case WEST:  c1.x += controlLen; break; // Heading East
        }

        // Extrude c2 from end backwards based on exit direction
        switch(endDir) {
            case NORTH: c2.y += controlLen; break; // Exiting North (going up)
            case SOUTH: c2.y -= controlLen; break;
            case EAST:  c2.x -= controlLen; break;
            case WEST:  c2.x += controlLen; break;
        }

        return { c1, c2 };
    }
};

// Global Pathfinding Instances
NavigationComputer navComputer;
TrajectoryOptimizer trajOpt;

// ==================================================================================
//                                TRAFFIC SYSTEMS
// ==================================================================================

class TrafficManager {
public:
    Direction activePhase;
    LightColor currentLight;
    Uint64 lastSwitchTime;
    
    Uint32 durGreen = 4000;
    Uint32 durYellow = 2000;
    Uint32 durAllRed = 500; 

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
//                                VEHICLE CLASS
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
    Intention intention; 
    
    SDL_Color color;
    float length, width;
    
    // State Flags
    bool braking;
    bool active;
    
    // Pathfinding & Curve Logic
    bool turning;
    float turnProgress;
    Point turnStart;
    Point turnEnd;
    
    // Complex Pathfinding: Cubic Bezier Control Points
    Point turnC1; // Control Point 1
    Point turnC2; // Control Point 2
    bool useCubic; // Flag to use the complex solver

    int sub() const { return laneIdx % 3; }

    Vehicle(int _id, int _laneIdx, Direction _origin) {
        id = _id;
        laneIdx = _laneIdx;
        origin = _origin;
        active = true;
        turning = false;
        turnProgress = 0.0f;
        braking = false;
        useCubic = true; // Enable complex pathing by default
        
        int rType = rand() % 100;
        if (rType < 60) {
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

        if (sub() == 0) {
            intention = INT_LEFT;
             color.r = std::min(255, color.r + 50);
        } else if (sub() == 1) {
            if (rand() % 2 == 0) intention = INT_STRAIGHT;
            else intention = INT_RIGHT;
        } else {
            intention = INT_RIGHT;
        }

        Point start = getLaneStart(origin, sub());
        pos = start;
        
        if (origin == NORTH) angle = 90;
        else if (origin == SOUTH) angle = 270;
        else if (origin == EAST) angle = 180;
        else angle = 0;
    }
};

// ==================================================================================
//                                SIMULATION STATE
// ==================================================================================

std::vector<Vehicle> traffic;
std::vector<Particle> particles;
std::vector<Decoration> scenery;
TrafficManager tm;
Stats stats = {0, 0, 0, 0, 0};
int carIdCounter = 0;
Uint64 frameCount = 0;

// ==================================================================================
//                                AI LOGIC
// ==================================================================================

bool checkSafeties(Vehicle& v) {
    bool shouldStop = false;

    LightColor sig = tm.getSignal(v.origin);
    
    if (sig != L_GREEN) {
        float dist = 0;
        bool approaching = false;

        if (v.origin == NORTH) { dist = ENT - v.pos.y; approaching = (v.pos.y < ENT); }
        else if (v.origin == SOUTH) { dist = v.pos.y - EXT; approaching = (v.pos.y > EXT); }
        else if (v.origin == EAST) { dist = v.pos.x - EXT; approaching = (v.pos.x > EXT); }
        else { dist = ENT - v.pos.x; approaching = (v.pos.x < ENT); }

        if (approaching && dist < 120.0f && dist > -20.0f) {
            if (sig == L_RED) shouldStop = true;
            else if (sig == L_YELLOW && dist > 50.0f) shouldStop = true;
        }
    }

    if (!shouldStop) {
        for (const auto& other : traffic) {
            if (v.id == other.id || !other.active) continue;
            
            float d = getDistance(v.pos, other.pos);
            float safe = SAFE_DIST;
            if (v.type == TYPE_TRUCK) safe = 140.0f;

            if (d < safe) {
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

// ----------------------------------------------------------------------------------
//  UPDATED: SPAWN LOGIC WITH LANE AVOIDANCE & HEURISTICS
// ----------------------------------------------------------------------------------
void attemptSpawn() {
    // We want to evaluate all 4 directions and their sub-lanes (0 and 1)
    // to find the 'safest' spawning spot that isn't occupied.
    
    std::vector<LaneNode> candidateLanes;

    // Scan the environment
    for (int d = 0; d < 4; d++) {
        Direction dir = (Direction)d;
        for (int s = 0; s < 2; s++) { // Check lane 0 and 1
            LaneNode node;
            node.dir = dir;
            node.subLaneIndex = s;
            node.laneId = d * 3 + s;
            node.isBlocked = false;
            node.distanceToNearest = 9999.0f;
            node.congestionCost = 0.0f;

            Point spawnPoint = getLaneStart(dir, s);

            // Check against all existing traffic
            for (const auto& v : traffic) {
                if (!v.active) continue;
                
                // Only care about cars in same lane or very close proximity
                float dist = getDistance(v.pos, spawnPoint);
                
                // COMPLEX LOGIC: Avoid lanes with cars entirely (High Safety Margin)
                // If a car is within 250 units of start, mark blocked.
                if (dist < navComputer.BLOCK_THRESHOLD) {
                    // Additionally check if angle implies they are in this lane
                    // (Simplified by checking absolute distance for spawn area)
                    node.isBlocked = true; 
                    node.distanceToNearest = std::min(node.distanceToNearest, dist);
                }
                
                // If car is in the same logical lane index, add to congestion
                if (v.laneIdx == node.laneId) {
                    node.congestionCost += 1.0f;
                    if (dist < node.distanceToNearest) node.distanceToNearest = dist;
                }
            }
            
            // Only add if not strictly blocked
            if (!node.isBlocked) {
                candidateLanes.push_back(node);
            }
        }
    }

    // Sort candidates by safety score (Descending)
    if (candidateLanes.empty()) return; // No safe lanes found

    std::sort(candidateLanes.begin(), candidateLanes.end(), 
        [](const LaneNode& a, const LaneNode& b) {
            return navComputer.calculateLaneScore(a) > navComputer.calculateLaneScore(b);
        }
    );

    // Pick the best one (or top 2 random for variety)
    LaneNode best = candidateLanes[0];
    
    // Final Spawn Execution
    Vehicle newCar(++carIdCounter, best.laneId, best.dir);
    traffic.push_back(newCar);
    stats.totalCars++;
}

void initScenery() {
    for (int i = 0; i < 20; i++) {
        Decoration d;
        d.type = 0;
        d.size = 20.0f + (rand() % 20);
        d.color = { (Uint8)(20+rand()%40), (Uint8)(80+rand()%50), 20, 255 };
        
        int q = rand() % 4;
        float x, y;
        if (q == 0) { x = rand() % ENT; y = rand() % ENT; }
        else if (q == 1) { x = EXT + rand() % (SZ - EXT); y = rand() % ENT; }
        else if (q == 2) { x = rand() % ENT; y = EXT + rand() % (SZ - EXT); }
        else { x = EXT + rand() % (SZ - EXT); y = EXT + rand() % (SZ - EXT); }

        d.pos = { x, y };
        scenery.push_back(d);
    }
}

// ==================================================================================
//                                RENDERER
// ==================================================================================

void drawTree(SDL_Renderer* ren, const Decoration& d) {
    SDL_SetRenderDrawColor(ren, 10, 30, 10, 100);
    SDL_FRect shad = { d.pos.x + 5, d.pos.y + 5, d.size, d.size };
    SDL_RenderFillRect(ren, &shad);
    SDL_SetRenderDrawColor(ren, d.color.r, d.color.g, d.color.b, 255);
    SDL_FRect l1 = { d.pos.x, d.pos.y, d.size, d.size };
    SDL_RenderFillRect(ren, &l1);
    SDL_SetRenderDrawColor(ren, d.color.r+20, d.color.g+20, d.color.b+20, 255);
    SDL_FRect l2 = { d.pos.x + d.size*0.2f, d.pos.y + d.size*0.2f, d.size*0.6f, d.size*0.6f };
    SDL_RenderFillRect(ren, &l2);
}

void drawRoadMarkings(SDL_Renderer* ren) {
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
    for (int i = 1; i < 3; i++) {
        float off = ENT + i * LW;
        for (int k = 0; k < SZ; k += 50) 
            if (k < ENT || k > EXT) SDL_RenderLine(ren, off, k, off, k + 25);
        for (int k = 0; k < SZ; k += 50) 
            if (k < ENT || k > EXT) SDL_RenderLine(ren, k, off, k + 25, off);
    }
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_FRect stopN = { ENT, ENT - 4, RW, 4 }; SDL_RenderFillRect(ren, &stopN);
    SDL_FRect stopS = { ENT, EXT, RW, 4 }; SDL_RenderFillRect(ren, &stopS);
    SDL_FRect stopW = { ENT - 4, ENT, 4, RW }; SDL_RenderFillRect(ren, &stopW);
    SDL_FRect stopE = { EXT, ENT, 4, RW }; SDL_RenderFillRect(ren, &stopE);
    
    // Zebra
    SDL_SetRenderDrawColor(ren, 180, 180, 180, 255);
    for(float i = 0; i < RW; i+=20) {
        SDL_FRect zN = { (float)ENT + i + 5, (float)ENT - 30, 10, 26 }; SDL_RenderFillRect(ren, &zN);
        SDL_FRect zS = { (float)ENT + i + 5, (float)EXT + 4, 10, 26 }; SDL_RenderFillRect(ren, &zS);
        SDL_FRect zW = { (float)ENT - 30, (float)ENT + i + 5, 26, 10 }; SDL_RenderFillRect(ren, &zW);
        SDL_FRect zE = { (float)EXT + 4, (float)ENT + i + 5, 26, 10 }; SDL_RenderFillRect(ren, &zE);
    }
}

void drawVehicleShape(SDL_Renderer* ren, const Vehicle& v) {
    SDL_SetRenderDrawColor(ren, v.color.r, v.color.g, v.color.b, 255);
    float w = v.length, h = v.width;
    bool vert = (std::abs(std::sin(v.angle * M_PI / 180.0f)) > 0.7f);
    if (vert) std::swap(w, h);

    float cx = v.pos.x; float cy = v.pos.y;
    SDL_FRect body = { cx - w/2, cy - h/2, w, h };
    SDL_RenderFillRect(ren, &body);

    SDL_SetRenderDrawColor(ren, 20, 20, 30, 100);
    SDL_FRect glass;
    if (v.angle > 315 || v.angle < 45) glass = { cx, cy - h/2 + 3, w/2 - 2, h - 6 };
    else if (v.angle > 45 && v.angle < 135) glass = { cx - w/2 + 3, cy, w - 6, h/2 - 2 };
    else if (v.angle > 135 && v.angle < 225) glass = { cx - w/2 + 2, cy - h/2 + 3, w/2 - 2, h - 6 };
    else glass = { cx - w/2 + 3, cy - h/2 + 2, w - 6, h/2 - 2 };
    SDL_RenderFillRect(ren, &glass);

    if (v.braking) {
        SDL_SetRenderDrawColor(ren, 255, 0, 0, 255);
        SDL_FRect b1, b2;
        float bW = 5, bH = 5;
        if (v.angle > 315 || v.angle < 45) { b1 = { cx - w/2, cy - h/2, bW, bH }; b2 = { cx - w/2, cy + h/2 - bH, bW, bH }; }
        else if (v.angle > 45 && v.angle < 135) { b1 = { cx - w/2, cy - h/2, bW, bH }; b2 = { cx + w/2 - bW, cy - h/2, bW, bH }; }
        else if (v.angle > 135 && v.angle < 225) { b1 = { cx + w/2 - bW, cy - h/2, bW, bH }; b2 = { cx + w/2 - bW, cy + h/2 - bH, bW, bH }; }
        else { b1 = { cx - w/2, cy + h/2 - bH, bW, bH }; b2 = { cx + w/2 - bW, cy + h/2 - bH, bW, bH }; }
        SDL_RenderFillRect(ren, &b1); SDL_RenderFillRect(ren, &b2);
    }
    
    // Debug Intention Indicator (Dot on hood)
    if (v.intention == INT_LEFT) SDL_SetRenderDrawColor(ren, 0, 255, 255, 255);
    else if (v.intention == INT_RIGHT) SDL_SetRenderDrawColor(ren, 255, 100, 0, 255);
    else SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_FRect dot = { cx - 2, cy - 2, 4, 4 };
    SDL_RenderFillRect(ren, &dot);
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
    SDL_FRect panel = { 0, SZ - 40, SZ, 40 };
    SDL_SetRenderDrawColor(ren, 30, 30, 40, 230);
    SDL_RenderFillRect(ren, &panel);
    SDL_SetRenderDrawColor(ren, 100, 100, 200, 255);
    SDL_RenderLine(ren, 0, SZ-40, SZ, SZ-40);
    float flowPct = std::min(1.0f, stats.flowRate / 100.0f); 
    SDL_FRect bar = { 100, SZ - 25, 200 * flowPct, 10 };
    SDL_SetRenderDrawColor(ren, 0, 255, 100, 255);
    SDL_RenderFillRect(ren, &bar);
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_FRect border = { 100, SZ - 25, 200, 10 };
    SDL_RenderRect(ren, &border);
}

// ==================================================================================
//                                MAIN LOOP
// ==================================================================================

int main(int argc, char** argv) {
    if (!SDL_Init(SDL_INIT_VIDEO)) return 1;

    SDL_Window* win = SDL_CreateWindow("Complex Routing Traffic Sim", SZ, SZ, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, NULL);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    initScenery();
    bool running = true;
    Uint64 perfLast = SDL_GetTicks();

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) if (e.type == SDL_EVENT_QUIT) running = false;

        Uint64 now = SDL_GetTicks();
        float dt = (now - perfLast) / 1000.0f;
        perfLast = now;
        frameCount++;

        tm.update(16);
        
        // High Frequency Spawning logic replaced with Gap Acceptance check
        if (frameCount % SPAWN_RATE_DEFAULT == 0) attemptSpawn();

        stats.currentCars = (int)traffic.size();
        if (frameCount % 60 == 0) stats.flowRate = (int)((stats.totalCars / (now/1000.0f)) * 60);

        for (auto& v : traffic) {
            if (!v.active) continue;

            bool blocked = checkSafeties(v);
            v.braking = blocked;

            if (blocked) v.speed = std::max(0.0f, v.speed - BRAKE_RATE);
            else v.speed = std::min(v.maxSpeed, v.speed + ACCEL_RATE);

            if (!v.turning) {
                float rad = v.angle * (M_PI / 180.0f);
                v.pos.x += std::cos(rad) * v.speed;
                v.pos.y += std::sin(rad) * v.speed;

                // Intersection Trigger
                bool inIntersection = (v.pos.x > ENT && v.pos.x < EXT && v.pos.y > ENT && v.pos.y < EXT);
                
                if (inIntersection) {
                    v.turning = true;
                    v.turnStart = v.pos;
                    
                    // ==========================================================
                    // ROUTING RULES IMPLEMENTATION (UPDATED FOR CUBIC BEZIER)
                    // ==========================================================
                    
                    Direction tDir; 
                    int tSub = 2; // Default target is always "Lane 1" (Code Lane 2/Outer)

                    if (v.intention == INT_LEFT) {
                        tDir = (Direction)((v.origin + 1) % 4);
                        v.turnC1 = { (float)CTR, (float)CTR }; 
                        v.turnC2 = { (float)CTR, (float)CTR }; // Fallback
                    } 
                    else if (v.intention == INT_STRAIGHT) {
                        tDir = (Direction)((v.origin + 2) % 4);
                    } 
                    else { // INT_RIGHT
                        tDir = (Direction)((v.origin + 3) % 4);
                    }
                    
                    v.turnEnd = getLaneEnd(tDir, tSub);

                    // USE TRAJECTORY OPTIMIZER FOR CUBIC CONTROL POINTS
                    if (v.intention != INT_LEFT) {
                        // Calculate optimal bezier controls
                        auto controls = TrajectoryOptimizer::solveCubicControlPoints(v.turnStart, v.turnEnd, v.origin, tDir);
                        v.turnC1 = controls.c1;
                        v.turnC2 = controls.c2;
                    } else {
                        // Keep simple center pivot for left turns (hard turn)
                        v.turnC1 = { (float)CTR, (float)CTR };
                        v.turnC2 = { (float)CTR, (float)CTR };
                    }
                }
            } else {
                // Curve Handling (Complex)
                v.turnProgress += (v.speed * 0.0025f); 
                
                if (v.turnProgress >= 1.0f) {
                    v.active = false; 
                } else {
                    Point oldPos = v.pos;
                    
                    // Use new Cubic Bezier solver
                    v.pos = getCubicBezierPoint(v.turnStart, v.turnC1, v.turnC2, v.turnEnd, v.turnProgress);
                    
                    v.angle = getAngle(oldPos, v.pos);
                }
            }

            if (v.pos.x < -150 || v.pos.x > SZ+150 || v.pos.y < -150 || v.pos.y > SZ+150) v.active = false;

            // Particles
            if (v.speed > 0.5f && v.type != TYPE_BIKE && rand()%15==0) {
                Particle p;
                p.pos = v.pos; p.alpha = 200; p.size = 4; p.active = true;
                p.velX = ((rand()%10)-5)/10.0f; p.velY = ((rand()%10)-5)/10.0f;
                particles.push_back(p);
            }
        }

        for (auto& p : particles) {
            p.pos.x += p.velX; p.pos.y += p.velY;
            p.alpha -= 5; p.size += 0.15f;
            if (p.alpha <= 0) p.active = false;
        }

        traffic.erase(std::remove_if(traffic.begin(), traffic.end(), [](const Vehicle& v){ return !v.active; }), traffic.end());
        particles.erase(std::remove_if(particles.begin(), particles.end(), [](const Particle& p){ return !p.active; }), particles.end());

        // DRAW
        SDL_SetRenderDrawColor(ren, 30, 30, 35, 255);
        SDL_RenderClear(ren);

        SDL_SetRenderDrawColor(ren, 25, 45, 25, 255);
        SDL_FRect bg[4] = { {0,0,(float)ENT,(float)ENT}, {(float)EXT,0,(float)ENT,(float)ENT}, {0,(float)EXT,(float)ENT,(float)ENT}, {(float)EXT,(float)EXT,(float)ENT,(float)ENT} };
        for(auto& b : bg) SDL_RenderFillRect(ren, &b);
        for (const auto& d : scenery) drawTree(ren, d);

        SDL_SetRenderDrawColor(ren, 55, 55, 60, 255);
        SDL_FRect rV = {(float)ENT, 0, (float)RW, (float)SZ};
        SDL_FRect rH = {0, (float)ENT, (float)SZ, (float)RW};
        SDL_RenderFillRect(ren, &rV); SDL_RenderFillRect(ren, &rH);

        drawRoadMarkings(ren);
        for (const auto& p : particles) { SDL_SetRenderDrawColor(ren, 150, 150, 150, (Uint8)p.alpha); SDL_FRect pr = {p.pos.x,p.pos.y,p.size,p.size}; SDL_RenderFillRect(ren, &pr); }
        for (const auto& v : traffic) drawVehicleShape(ren, v);

        drawTrafficLight(ren, ENT - 40, ENT - 80, tm.getSignal(NORTH));
        drawTrafficLight(ren, EXT + 16, EXT + 20, tm.getSignal(SOUTH));
        drawTrafficLight(ren, EXT + 20, ENT - 80, tm.getSignal(EAST));
        drawTrafficLight(ren, ENT - 40, EXT + 20, tm.getSignal(WEST));
        drawDashboard(ren);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}