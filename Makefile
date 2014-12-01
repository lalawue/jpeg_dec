
CC=gcc -Wall
SRCS=jpeg_dec

all: $(foreach v, $(SRCS), out/$(v).out)

out/jpeg_dec.out: jpeg_dec.c
	@mkdir -p out
	$(CC) $< -o $@ -I$(INCDIR) -L$(LIBDIR) $(foreach v, $(LIBS), -l$(v))

clean:
	rm -rf out
