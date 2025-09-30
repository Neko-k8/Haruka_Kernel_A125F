/* scripts/extract-cert.c - OpenSSL3-safe extractor for build scripts
 *
 * Copyright © 2014-2015 Red Hat, Inc.
 * Copyright © 2015      Intel Corporation.
 * This derived version: improvements for OpenSSL 3+ compatibility,
 * better resource cleanup and error handling.
 *
 * Authors: David Howells, David Woodhouse
 * Modifications: ForgGPT (upgrade/cleanup for OpenSSL3+, runtime ENGINE load)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <err.h>
#include <dlfcn.h>
#include <unistd.h>

#include <openssl/bio.h>
#include <openssl/pem.h>
#include <openssl/err.h>
#include <openssl/engine.h>

#define PKEY_ID_PKCS7 2

/* Buffer sizes */
#define ERR_BUF_SZ 256
#define NAME_BUF_SZ 256

static __attribute__((noreturn))
void format(void)
{
	fprintf(stderr,
		"Usage: scripts/extract-cert <source> <dest>\n");
	exit(2);
}

/*
 * Display OpenSSL error stack. Works with OpenSSL 3+ and older versions.
 */
static void display_openssl_errors(int lineno)
{
	char buf[ERR_BUF_SZ];
	unsigned long e;

#if OPENSSL_VERSION_NUMBER >= 0x30000000L
	const char *file = NULL, *func = NULL, *data = NULL;
	int line = 0, flags = 0;

	if (ERR_peek_error() == 0)
		return;
	fprintf(stderr, "OpenSSL errors at %s:%d (script line %d):\n",
		__FILE__, __LINE__, lineno);

	while ((e = ERR_get_error_all(&file, &line, &func, &data, &flags))) {
		ERR_error_string_n(e, buf, sizeof(buf));
		fprintf(stderr, "- %s (lib=%d reason=%d) %s:%d %s\n",
			buf, ERR_GET_LIB(e), ERR_GET_REASON(e),
			(file && file[0]) ? file : "?", line,
			(func && func[0]) ? func : "");
	}
#else
	const char *file;
	int line;

	if (ERR_peek_error() == 0)
		return;
	fprintf(stderr, "OpenSSL errors at %s:%d (script line %d):\n",
		__FILE__, __LINE__, lineno);

	while ((e = ERR_get_error_line(&file, &line))) {
		ERR_error_string_n(e, buf, sizeof(buf));
		fprintf(stderr, "- %s %s:%d\n", buf, file ? file : "?", line);
	}
#endif
}

/* Drain error queue without printing (used after attempted ops we expect may add errors) */
static void drain_openssl_errors(void)
{
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
	const char *file = NULL, *func = NULL, *data = NULL;
	int line = 0, flags = 0;
	if (ERR_peek_error() == 0)
		return;
	while (ERR_get_error_all(&file, &line, &func, &data, &flags)) { }
#else
	const char *file;
	int line;
	if (ERR_peek_error() == 0)
		return;
	while (ERR_get_error_line(&file, &line)) { }
#endif
}

/* Macro wrapper: show SSL errors then die if cond holds */
#define ERR_CHECK(cond, fmt, ...)                 \
	do {                                          \
		bool __cond = (cond);                    \
		if (__cond) {                            \
			display_openssl_errors(__LINE__);    \
			err(1, fmt, ## __VA_ARGS__);        \
		}                                         \
	} while (0)

static const char *key_pass;
static BIO *wb = NULL;
static char *cert_dst = NULL;
int kbuild_verbose = 0;

/* Write X509 in DER to output BIO (wb). Creates/opens wb on demand. */
static void write_cert(X509 *x509)
{
	char buf[NAME_BUF_SZ];

	if (!wb) {
		wb = BIO_new_file(cert_dst, "wb");
		ERR_CHECK(!wb, "%s", cert_dst);
	}

	/* Convert subject to a one-line string for verbose log */
	X509_NAME_oneline(X509_get_subject_name(x509), buf, sizeof(buf));
	ERR_CHECK(!i2d_X509_bio(wb, x509), "%s", cert_dst);

	if (kbuild_verbose)
		fprintf(stderr, "Extracted cert: %s\n", buf);
}

/* ---- Runtime ENGINE function ptrs (used on OpenSSL 3 to avoid deprecation warnings) ---- */
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
typedef void (*engine_load_builtin_engines_fn)(void);
typedef ENGINE *(*engine_by_id_fn)(const char *);
typedef int (*engine_init_fn)(ENGINE *);
typedef int (*engine_ctrl_cmd_string_fn)(ENGINE *, const char *, const char *, int);
typedef int (*engine_ctrl_cmd_fn)(ENGINE *, const char *, long, void *, void *, int);
typedef void (*engine_finish_fn)(ENGINE *);
typedef void (*engine_free_fn)(ENGINE *);

static engine_load_builtin_engines_fn p_ENGINE_load_builtin_engines = NULL;
static engine_by_id_fn p_ENGINE_by_id = NULL;
static engine_init_fn p_ENGINE_init = NULL;
static engine_ctrl_cmd_string_fn p_ENGINE_ctrl_cmd_string = NULL;
static engine_ctrl_cmd_fn p_ENGINE_ctrl_cmd = NULL;
static engine_finish_fn p_ENGINE_finish = NULL;
static engine_free_fn p_ENGINE_free = NULL;

/* Load the necessary ENGINE symbols from libcrypto at runtime. Fatal if not present. */
static void load_engine_symbols_or_die(void)
{
	void *sym;

	/* Use RTLD_DEFAULT so it searches already-loaded libcrypto in the process */
	sym = dlsym(RTLD_DEFAULT, "ENGINE_load_builtin_engines");
	if (!sym) errx(1, "Missing ENGINE_load_builtin_engines symbol in libcrypto");
	p_ENGINE_load_builtin_engines = (engine_load_builtin_engines_fn)sym;

	sym = dlsym(RTLD_DEFAULT, "ENGINE_by_id");
	if (!sym) errx(1, "Missing ENGINE_by_id symbol in libcrypto");
	p_ENGINE_by_id = (engine_by_id_fn)sym;

	sym = dlsym(RTLD_DEFAULT, "ENGINE_init");
	if (!sym) errx(1, "Missing ENGINE_init symbol in libcrypto");
	p_ENGINE_init = (engine_init_fn)sym;

	sym = dlsym(RTLD_DEFAULT, "ENGINE_ctrl_cmd_string");
	if (!sym) errx(1, "Missing ENGINE_ctrl_cmd_string symbol in libcrypto");
	p_ENGINE_ctrl_cmd_string = (engine_ctrl_cmd_string_fn)sym;

	sym = dlsym(RTLD_DEFAULT, "ENGINE_ctrl_cmd");
	if (!sym) errx(1, "Missing ENGINE_ctrl_cmd symbol in libcrypto");
	p_ENGINE_ctrl_cmd = (engine_ctrl_cmd_fn)sym;

	/* Cleanup symbols */
	sym = dlsym(RTLD_DEFAULT, "ENGINE_finish");
	if (!sym) errx(1, "Missing ENGINE_finish symbol in libcrypto");
	p_ENGINE_finish = (engine_finish_fn)sym;

	sym = dlsym(RTLD_DEFAULT, "ENGINE_free");
	if (!sym) errx(1, "Missing ENGINE_free symbol in libcrypto");
	p_ENGINE_free = (engine_free_fn)sym;
}
#endif

int main(int argc, char **argv)
{
	char *cert_src;

	/* Initialize OpenSSL -- use recommended initializer if available */
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
	/* OpenSSL 1.1.0+ initializes library on demand; but calling this is harmless */
	OPENSSL_init_crypto(0, NULL);
#else
	OpenSSL_add_all_algorithms();
#endif
	/* Load error strings (older API is fine across versions) */
	ERR_load_crypto_strings();
	ERR_clear_error();

	kbuild_verbose = atoi(getenv("KBUILD_VERBOSE") ? : "0");
	key_pass = getenv("KBUILD_SIGN_PIN");

	if (argc != 3)
		format();

	cert_src = argv[1];
	cert_dst = argv[2];

	/* If source empty, create empty destination (legacy behaviour) */
	if (!cert_src[0]) {
		FILE *f = fopen(cert_dst, "wb");
		ERR_CHECK(!f, "%s", cert_dst);
		fclose(f);
		return 0;
	}

	/* PKCS#11 URL handler */
	if (!strncmp(cert_src, "pkcs11:", 7)) {
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
		/* Use runtime-loaded ENGINE symbols to avoid deprecation warnings at compile-time */
		load_engine_symbols_or_die();
#endif
		ENGINE *e = NULL;
		bool engine_inited = false;
		struct {
			const char *cert_id;
			X509 *cert;
		} parms;
		parms.cert_id = cert_src;
		parms.cert = NULL;

#if OPENSSL_VERSION_NUMBER >= 0x30000000L
		/* call via function pointers loaded from libcrypto */
		p_ENGINE_load_builtin_engines();
		drain_openssl_errors();
		e = p_ENGINE_by_id("pkcs11");
		ERR_CHECK(!e, "Load PKCS#11 ENGINE");
		/* init engine */
		if (p_ENGINE_init(e))
			drain_openssl_errors();
		else
			ERR_CHECK(1, "ENGINE_init");
		engine_inited = true;
		/* set PIN if provided */
		if (key_pass)
			ERR_CHECK(!p_ENGINE_ctrl_cmd_string(e, "PIN", key_pass, 0), "Set PKCS#11 PIN");
		/* LOAD_CERT_CTRL: engine-specific; pass parameters pointer */
		p_ENGINE_ctrl_cmd(e, "LOAD_CERT_CTRL", 0, &parms, NULL, 1);
#else
		/* legacy path (pre-3.0) */
		ENGINE_load_builtin_engines();
		drain_openssl_errors();
		e = ENGINE_by_id("pkcs11");
		ERR_CHECK(!e, "Load PKCS#11 ENGINE");
		if (ENGINE_init(e))
			drain_openssl_errors();
		else
			ERR_CHECK(1, "ENGINE_init");
		engine_inited = true;
		if (key_pass)
			ERR_CHECK(!ENGINE_ctrl_cmd_string(e, "PIN", key_pass, 0), "Set PKCS#11 PIN");
		ENGINE_ctrl_cmd(e, "LOAD_CERT_CTRL", 0, &parms, NULL, 1);
#endif
		ERR_CHECK(!parms.cert, "Get X.509 from PKCS#11");
		/* write cert and cleanup engine references */
		cert_dst = argv[2];
		write_cert(parms.cert);
		X509_free(parms.cert);
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
		if (engine_inited) {
			p_ENGINE_finish(e);
			p_ENGINE_free(e);
		}
#else
		if (engine_inited) {
			ENGINE_finish(e);
			ENGINE_free(e);
		}
#endif
		if (wb)
			BIO_free(wb);
		return 0;
	}

	/* Otherwise read PEM file containing one or more certs */
	{
		BIO *b = BIO_new_file(cert_src, "rb");
		X509 *x509 = NULL;

		ERR_CHECK(!b, "%s", cert_src);

		cert_dst = argv[2];

		for (;;) {
			/* Read next PEM certificate */
			x509 = PEM_read_bio_X509(b, NULL, NULL, NULL);
			if (wb && !x509) {
				/* If wb already open and we got EOF (PEM_R_NO_START_LINE), stop cleanly */
				unsigned long err = ERR_peek_last_error();
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
				if (ERR_GET_LIB(err) == ERR_LIB_PEM &&
				    ERR_GET_REASON(err) == PEM_R_NO_START_LINE) {
					ERR_clear_error();
					break;
				}
#else
				if (ERR_GET_LIB(err) == ERR_LIB_PEM &&
				    ERR_GET_REASON(err) == PEM_R_NO_START_LINE) {
					ERR_clear_error();
					break;
				}
#endif
			}
			ERR_CHECK(!x509, "%s", cert_src);

			/* Write and free */
			write_cert(x509);
			X509_free(x509);
			x509 = NULL;
		}

		BIO_free(b);
		if (wb)
			BIO_free(wb);
	}

	/* Success */
	return 0;
}

