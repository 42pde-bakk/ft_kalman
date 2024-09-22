NAME = ft_kalman

INC_DIR = ./srcs
SRC_DIR = ./srcs
OBJ_DIR = ./obj
SRC_EXT = cpp
OBJ_EXT = o
INC_EXT = hpp
#SRCS = $(wildcard $(SRC_DIR)/*.c)
SRCS = $(shell find $(SRC_DIR) -type f -name "*.$(SRC_EXT)")
HEADERS = $(shell find $(INC_DIR) -type f -name "*.$(INC_EXT)")
OBJECTSS = $(SRCS:.cpp=.o)
OBJS = $(patsubst $(SRC_DIR)/%,$(OBJ_DIR)/%,$(OBJECTSS))

# COLORS
PINK = \x1b[35;01m
BLUE = \x1b[34;01m
YELLOW = \x1b[33;01m
GREEN = \x1b[32;01m
RED = \x1b[31;01m
WHITE = \x1b[31;37m
RESET = \x1b[0m

CFLAGS = -Wall -Werror -Wextra -std=c++20
ifdef DEBUG
 CFLAGS += -g3 -fsanitize=address
endif
ifdef BONUS
 CFLAGS += -D BONUS=1
endif
SHELL := /bin/bash
export SHELL
export DEBUG
ifdef SPEED
 CFLAGS += -Ofast
endif

all: $(NAME)

$(NAME): $(OBJS) $(HEADERS)
	$(CXX) $(CFLAGS) $(INCLUDE) $(OBJS) -o $@
	@printf "$(PINK)Done building $(NAME) $(RESET)\n"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p "$(@D)"
	$(CXX) -c $(CFLAGS) $(INCLUDE) $< -o $@

clean:
	/bin/rm -f $(OBJS)
	@/bin/rm -f *.o *~ *.gch

fclean: clean
	/bin/rm -f $(NAME)

re:
	$(MAKE) fclean
	$(MAKE) all

bonus: BONUS=1
bonus:
	$(MAKE) re

debug:
	make DEBUG=1

PID_FILE := /tmp/imu-sensor-stream-linux.pid
test: all
	./imu-sensor-stream-linux & echo $$! > $(PID_FILE)
	@echo "Task started with PID: $$(cat $(PID_FILE))"
	./ft_kalman
	PID=$$(cat $(PID_FILE))
	@echo "Stopping task with PID: $$PID"
	kill $$PID && rm -f $(PID_FILE)

