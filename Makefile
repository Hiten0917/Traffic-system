CC = g++
CFLAGS = -I SDL-3/include -std=c++17
LDFLAGS = -L SDL-3/lib

# Critical order for MinGW: Source -> CFLAGS -> LDFLAGS -> LIBS
LIBS = -lSDL3_ttf -lSDL3 -lws2_32 -lgdi32 -lwinmm -lole32 -loleaut32 -limm32 -lshell32 -lversion -luuid

BUILD_DIR = build
SRC_DIR = src

all: create_dirs $(BUILD_DIR)/Simulator.exe $(BUILD_DIR)/TrafficGenerator.exe copy_dlls

create_dirs:
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

$(BUILD_DIR)/Simulator.exe: $(SRC_DIR)/Simulator.cpp
	$(CC) $(SRC_DIR)/Simulator.cpp -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS)

$(BUILD_DIR)/TrafficGenerator.exe: $(SRC_DIR)/TrafficGenerator.cpp
	$(CC) $(SRC_DIR)/TrafficGenerator.cpp -o $@ $(CFLAGS) -lws2_32

copy_dlls:
	copy SDL-3\bin\SDL3.dll $(BUILD_DIR)
	copy SDL-3\bin\SDL3_ttf.dll $(BUILD_DIR)

clean:
	@if exist $(BUILD_DIR) rd /s /q $(BUILD_DIR)