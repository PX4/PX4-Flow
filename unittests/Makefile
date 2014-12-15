
CC=g++
CFLAGS=-I. -I../src -lm

all: tests

TEST_FILES=../src/sonar_mode_filter.c \
		tests.cpp

tests: $(TEST_FILES)
	$(CC) -o tests $(TEST_FILES) $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ tests
