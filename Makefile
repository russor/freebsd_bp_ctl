# $FreeBSD: src/share/examples/kld/syscall/Makefile,v 1.2 2002/07/11 13:38:05 ru Exp $

SUBDIR=	driver util

load unload: _SUBDIR 


.include <bsd.subdir.mk>
