common_sources = sensirion_config.h sensirion_common.h sensirion_common.c
uart_sources = sensirion_uart_hal.h sensirion_shdlc.h sensirion_shdlc.c
svm40_sources = svm40_uart.h svm40_uart.c

uart_implementation ?= sensirion_uart_hal.c

CFLAGS = -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC -I.

ifdef CI
    CFLAGS += -Werror
endif

.PHONY: all clean

all: svm40_uart_example_usage

svm40_uart_example_usage: clean
	$(CC) $(CFLAGS) -o $@ ${svm40_sources} ${uart_sources} \
		${uart_implementation} ${common_sources} svm40_uart_example_usage.c

clean:
	$(RM) svm40_uart_example_usage
