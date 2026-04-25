CC ?= cc
TARGET := rocket_sim
SRC := main.c

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib 2>/dev/null)
RAYLIB_LIBS := $(shell pkg-config --libs raylib 2>/dev/null)

ifeq ($(strip $(RAYLIB_LIBS)),)
	RAYLIB_LIBS := -lraylib
endif

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	PLATFORM_LIBS := -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
else
	PLATFORM_LIBS := -lm -ldl -lpthread -lGL -lrt -lX11
endif

CFLAGS ?= -std=c99 -Wall -Wextra -O2

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(RAYLIB_CFLAGS) -o $@ $< $(RAYLIB_LIBS) $(PLATFORM_LIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
