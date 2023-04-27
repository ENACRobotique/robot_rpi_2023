BUILD_DIR=../../_build
SOURCE_DIR=.
CC=g2s


SOURCES = $(wildcard $(SOURCE_DIR)/*.dot)
OBJECTS = $(patsubst $(SOURCE_DIR)/%.dot,$(BUILD_DIR)/%.py,$(SOURCES))


build: $(OBJECTS)

$(OBJECTS): $(BUILD_DIR)/%.py : $(SOURCE_DIR)/%.dot
	$(CC) $< -o $@

clean:
	rm $(BUILD_DIR)/*.py
