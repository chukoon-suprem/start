CC = gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS = -lm

TARGET = motor_control_demo
SRCS = main.c clark_park_transform.c motor_control_wrapper.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
