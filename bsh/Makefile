CXX      := g++
CXXFLAGS := -std=c++11 -O2 -Wall -Wno-deprecated-declarations \
            -include compat/linux_compat.h

# GLM is bundled at lib/include/glm/glm/ and included as <glm.hpp>
# compat/ provides a cl.hpp shim over the modern opencl.hpp (cl.hpp was removed in OpenCL 2.x)
# System headers are used for GL, FreeGLUT, OpenCL, and FreeType
INCLUDES := \
    -I. \
    -Icompat \
    -Ilib/include/glm/glm \
    -I/usr/include/freetype2

LIBS := -lGL -lGLEW -lglut -lOpenCL -lfreetype -lm

BINDIR  := bin
TARGET  := $(BINDIR)/boidSim
OBJDIR  := $(BINDIR)/obj

SRCS := \
    main_linux.cpp \
    stdafx.cpp \
    Simulation.cpp \
    gfx.cpp \
    CLHelper.cpp \
    LogFile.cpp \
    Shader.cpp \
    Shader_utils.cpp \
    OverlayText.cpp \
    WorldBox.cpp \
    WorldGround.cpp \
    Skybox.cpp \
    Column.cpp \
    Tunnel.cpp \
    BoidModelSimple.cpp \
    BoidModelGrid.cpp \
    BoidModelGrid_2D.cpp \
    BoidModelSH.cpp \
    BoidModelSH_2D.cpp \
    boidModelSHWay1.cpp \
    BoidModelSHWay2.cpp \
    BoidModelSHCombined.cpp \
    boidModelSHObstacle.cpp \
    BoidModelSHObstacleTunnel.cpp

OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)
	@cp -r textures $(BINDIR)/textures
	@cp FreeSans.otf FreeSans.ttf $(BINDIR)/

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

run: $(TARGET)
	cd $(BINDIR) && ./boidSim

clean:
	rm -rf $(BINDIR)
