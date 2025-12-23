#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <time.h>

#pragma comment(lib, "ws2_32.lib")

int main() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
    
    srand((unsigned int)time(0));
    std::cout << "Traffic Generator Running... (Sending cars to 127.0.0.1:5000)" << std::endl;

    while (true) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
        
        sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_port = htons(5000);
        inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

        if (connect(s, (struct sockaddr*)&server, sizeof(server)) == 0) {
            int laneNum = (rand() % 4) + 1;
            std::string msg = std::to_string(laneNum);
            send(s, msg.c_str(), (int)msg.length(), 0);
            std::cout << "Dispatched vehicle to Lane: " << laneNum << std::endl;
        } else {
            std::cerr << "Could not connect to Simulator. Is it running?" << std::endl;
        }

        closesocket(s);
        
        // Randomize spawn rate to simulate real traffic pulses
        int delay = 500 + (rand() % 2000); 
        Sleep(delay); 
    }

    WSACleanup();
    return 0;
}