DESTDIR ?=

turbostat : turbostat.c
CFLAGS +=	-Wall
CFLAGS +=	-I../../../../arch/x86/include/

clean :
	rm -f turbostat

install :
	install turbostat ${DESTDIR}/usr/bin/turbostat
	install turbostat.8 ${DESTDIR}/usr/share/man/man8
