
KMOD=	bpmod
SRCS=	bp_mod.c
SRCS+= device_if.h bus_if.h pci_if.h opt_bdg.h

afterinstall:
	@-( if [ `mount -t devfs | wc -l` = 0 ]; then \
	      rm -f /dev/bpmod  && mknod /dev/bpmod c 253 0 ; \
	    fi; \
	)         

clean:
	rm -f device_if.h bus_if.h pci_if.h opt_bdg.h export_syms 
	rm -f *.o *.kld *.ko
	rm -f @ machine 

.include <bsd.kmod.mk>

