// SPDX-License-Identifier: GPL-2.0-only

#include <generated/compile.h>
#include <generated/utsrelease.h>
#include <linux/proc_ns.h>
#include <linux/refcount.h>
#include <linux/uts.h>
#include <linux/utsname.h>

struct uts_namespace init_uts_ns = {
	.ns.count = REFCOUNT_INIT(2),
	.name = {
		.sysname	= UTS_SYSNAME,
		.nodename	= UTS_NODENAME,
		.release	= UTS_RELEASE,
		.version	= UTS_VERSION,
		.machine	= UTS_MACHINE,
		.domainname	= UTS_DOMAINNAME,
	},
	.user_ns = &init_user_ns,
	.ns.inum = PROC_UTS_INIT_INO,
#ifdef CONFIG_UTS_NS
	.ns.ops = &utsns_operations,
#endif
};

/*
 * FIXED STRINGS! Don't touch!
 *
 * GOcontroll-aanpassing: vervang de standaard "(USER@HOST)" build-host string
 * door een copyright-vermelding. Houdt de parens-structuur van de banner intact
 * zodat parsers (lscpu, dmesg-color, etc) niets stuk gaan.
 */
const char linux_banner[] =
	"Linux version " UTS_RELEASE
	" (Copyright 2026 GOtech Group BV)"
	" (" LINUX_COMPILER ") " UTS_VERSION "\n";
