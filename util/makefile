PROGS	=	bpctl_util

CLEANFILES = $(PROGS) *.o
NO_MAN=
CFLAGS = -O2 -pipe



all: $(PROGS)

nmd-test: bpctl_util.o
	$(CC) $(CFLAGS) -o bpctl_util bpctl_util.o $(LDFLAGS)


install: all
	install $(PROGS) /bin

clean:
	-@rm -rf $(CLEANFILES)

