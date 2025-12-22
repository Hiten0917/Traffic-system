#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <time.h>

#pragma comment(lib, "ws2_32.lib")

int main() {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    srand((unsigned int)time(0));

    std::cout << "Traffic Generator Running... (Ctrl+C to quit)" << std::endl;

    while (true) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_port = htons(5000);
        server.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (connect(s, (struct sockaddr*)&server, sizeof(server)) == 0) {
            int laneNum = (rand() % 4) + 1;
            std::string msg = std::to_string(laneNum);
            send(s, msg.c_str(), (int)msg.length(), 0);
            std::cout << "Dispatched vehicle to Lane: " << laneNum << std::endl;
        }
        closesocket(s);
        Sleep(1200); // 1.2 second delay
    }

    WSACleanup();
    return 0;
}