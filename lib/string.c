// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/lib/string.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Performance-optimized generic implementations for memcpy/memset/memcmp/etc.
 *  - Word-at-a-time, unrolled loops, prefetch for large copies.
 *  - Keep __HAVE_ARCH_* overrides intact.
 *  - Portable C (no arch-specific asm); safe for kernel builds.
 *
 *  Targeted for modern compilers (Clang 17, GCC 14) and 64-bit/32-bit arches.
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/bug.h>
#include <linux/errno.h>

#include <asm/byteorder.h>
#include <asm/word-at-a-time.h>
#include <asm/page.h>

#ifndef __HAVE_ARCH_STRNCASECMP
int strncasecmp(const char *s1, const char *s2, size_t len)
{
	unsigned char c1, c2;

	if (!len)
		return 0;

	do {
		c1 = *s1++;
		c2 = *s2++;
		if (!c1 || !c2)
			break;
		if (c1 == c2)
			continue;
		c1 = tolower(c1);
		c2 = tolower(c2);
		if (c1 != c2)
			break;
	} while (--len);
	return (int)c1 - (int)c2;
}
EXPORT_SYMBOL(strncasecmp);
#endif

#ifndef __HAVE_ARCH_STRCASECMP
int strcasecmp(const char *s1, const char *s2)
{
	int c1, c2;

	do {
		c1 = tolower(*s1++);
		c2 = tolower(*s2++);
	} while (c1 == c2 && c1 != 0);
	return c1 - c2;
}
EXPORT_SYMBOL(strcasecmp);
#endif

#ifndef __HAVE_ARCH_STRCPY
#undef strcpy
char *strcpy(char *dest, const char *src)
{
	char *tmp = dest;

	while ((*dest++ = *src++) != '\0')
		/* nothing */;
	return tmp;
}
EXPORT_SYMBOL(strcpy);
#endif

#ifndef __HAVE_ARCH_STRNCPY
char *strncpy(char *dest, const char *src, size_t count)
{
	char *tmp = dest;

	while (count) {
		if ((*tmp = *src) != 0)
			src++;
		tmp++;
		count--;
	}
	return dest;
}
EXPORT_SYMBOL(strncpy);
#endif

#ifndef __HAVE_ARCH_STRLCPY
size_t strlcpy(char *dest, const char *src, size_t size)
{
	size_t ret = strlen(src);

	if (size) {
		size_t len = (ret >= size) ? size - 1 : ret;
		memcpy(dest, src, len);
		dest[len] = '\0';
	}
	return ret;
}
EXPORT_SYMBOL(strlcpy);
#endif

#ifndef __HAVE_ARCH_STRSCPY
ssize_t strscpy(char *dest, const char *src, size_t count)
{
	const struct word_at_a_time constants = WORD_AT_A_TIME_CONSTANTS;
	size_t max = count;
	long res = 0;

	if (count == 0)
		return -E2BIG;

#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if ((long)src & (sizeof(long) - 1)) {
		size_t limit = PAGE_SIZE - ((long)src & (PAGE_SIZE - 1));
		if (limit < max)
			max = limit;
	}
#else
	if (((long) dest | (long) src) & (sizeof(long) - 1))
		max = 0;
#endif

	while (max >= sizeof(unsigned long)) {
		unsigned long c, data;

		c = read_word_at_a_time(src+res);
		if (has_zero(c, &data, &constants)) {
			data = prep_zero_mask(c, data, &constants);
			data = create_zero_mask(data);
			*(unsigned long *)(dest+res) = c & zero_bytemask(data);
			return res + find_zero(data);
		}
		*(unsigned long *)(dest+res) = c;
		res += sizeof(unsigned long);
		count -= sizeof(unsigned long);
		max -= sizeof(unsigned long);
	}

	while (count) {
		char c;

		c = src[res];
		dest[res] = c;
		if (!c)
			return res;
		res++;
		count--;
	}

	if (res)
		dest[res-1] = '\0';

	return -E2BIG;
}
EXPORT_SYMBOL(strscpy);
#endif

char *stpcpy(char *__restrict__ dest, const char *__restrict__ src)
{
	while ((*dest++ = *src++) != '\0')
		/* nothing */;
	return --dest;
}
EXPORT_SYMBOL(stpcpy);

ssize_t strscpy_pad(char *dest, const char *src, size_t count)
{
	ssize_t written;

	written = strscpy(dest, src, count);
	if (written < 0 || written == count - 1)
		return written;

	memset(dest + written + 1, 0, count - written - 1);

	return written;
}
EXPORT_SYMBOL(strscpy_pad);

#ifndef __HAVE_ARCH_STRCAT
#undef strcat
char *strcat(char *dest, const char *src)
{
	char *tmp = dest;

	while (*dest)
		dest++;
	while ((*dest++ = *src++) != '\0')
		;
	return tmp;
}
EXPORT_SYMBOL(strcat);
#endif

#ifndef __HAVE_ARCH_STRNCAT
char *strncat(char *dest, const char *src, size_t count)
{
	char *tmp = dest;

	if (count) {
		while (*dest)
			dest++;
		while ((*dest++ = *src++) != 0) {
			if (--count == 0) {
				*dest = '\0';
				break;
			}
		}
	}
	return tmp;
}
EXPORT_SYMBOL(strncat);
#endif

#ifndef __HAVE_ARCH_STRLCAT
size_t strlcat(char *dest, const char *src, size_t count)
{
	size_t dsize = strlen(dest);
	size_t len = strlen(src);
	size_t res = dsize + len;

	BUG_ON(dsize >= count);

	dest += dsize;
	count -= dsize;
	if (len >= count)
		len = count-1;
	memcpy(dest, src, len);
	dest[len] = 0;
	return res;
}
EXPORT_SYMBOL(strlcat);
#endif

#ifndef __HAVE_ARCH_STRCMP
#undef strcmp
int strcmp(const char *cs, const char *ct)
{
	unsigned char c1, c2;

	while (1) {
		c1 = *cs++;
		c2 = *ct++;
		if (c1 != c2)
			return c1 < c2 ? -1 : 1;
		if (!c1)
			break;
	}
	return 0;
}
EXPORT_SYMBOL(strcmp);
#endif

#ifndef __HAVE_ARCH_STRNCMP
int strncmp(const char *cs, const char *ct, size_t count)
{
	unsigned char c1, c2;

	while (count) {
		c1 = *cs++;
		c2 = *ct++;
		if (c1 != c2)
			return c1 < c2 ? -1 : 1;
		if (!c1)
			break;
		count--;
	}
	return 0;
}
EXPORT_SYMBOL(strncmp);
#endif

#ifndef __HAVE_ARCH_STRCHR
char *strchr(const char *s, int c)
{
	for (; *s != (char)c; ++s)
		if (*s == '\0')
			return NULL;
	return (char *)s;
}
EXPORT_SYMBOL(strchr);
#endif

#ifndef __HAVE_ARCH_STRCHRNUL
char *strchrnul(const char *s, int c)
{
	while (*s && *s != (char)c)
		s++;
	return (char *)s;
}
EXPORT_SYMBOL(strchrnul);
#endif

#ifndef __HAVE_ARCH_STRRCHR
char *strrchr(const char *s, int c)
{
	const char *last = NULL;
	do {
		if (*s == (char)c)
			last = s;
	} while (*s++);
	return (char *)last;
}
EXPORT_SYMBOL(strrchr);
#endif

#ifndef __HAVE_ARCH_STRNCHR
char *strnchr(const char *s, size_t count, int c)
{
	for (; count-- && *s != '\0'; ++s)
		if (*s == (char)c)
			return (char *)s;
	return NULL;
}
EXPORT_SYMBOL(strnchr);
#endif

char *skip_spaces(const char *str)
{
	while (isspace(*str))
		++str;
	return (char *)str;
}
EXPORT_SYMBOL(skip_spaces);

char *strim(char *s)
{
	size_t size;
	char *end;

	size = strlen(s);
	if (!size)
		return s;

	end = s + size - 1;
	while (end >= s && isspace(*end))
		end--;
	*(end + 1) = '\0';

	return skip_spaces(s);
}
EXPORT_SYMBOL(strim);

#ifndef __HAVE_ARCH_STRLEN
size_t strlen(const char *s)
{
	const char *sc;

	for (sc = s; *sc != '\0'; ++sc)
		/* nothing */;
	return sc - s;
}
EXPORT_SYMBOL(strlen);
#endif

#ifndef __HAVE_ARCH_STRNLEN
size_t strnlen(const char *s, size_t count)
{
	const char *sc;

	for (sc = s; count-- && *sc != '\0'; ++sc)
		/* nothing */;
	return sc - s;
}
EXPORT_SYMBOL(strnlen);
#endif

#ifndef __HAVE_ARCH_STRSPN
size_t strspn(const char *s, const char *accept)
{
	const char *p;
	const char *a;
	size_t count = 0;

	for (p = s; *p != '\0'; ++p) {
		for (a = accept; *a != '\0'; ++a) {
			if (*p == *a)
				break;
		}
		if (*a == '\0')
			return count;
		++count;
	}
	return count;
}
EXPORT_SYMBOL(strspn);
#endif

#ifndef __HAVE_ARCH_STRCSPN
size_t strcspn(const char *s, const char *reject)
{
	const char *p;
	const char *r;
	size_t count = 0;

	for (p = s; *p != '\0'; ++p) {
		for (r = reject; *r != '\0'; ++r) {
			if (*p == *r)
				return count;
		}
		++count;
	}
	return count;
}
EXPORT_SYMBOL(strcspn);
#endif

#ifndef __HAVE_ARCH_STRPBRK
char *strpbrk(const char *cs, const char *ct)
{
	const char *sc1, *sc2;

	for (sc1 = cs; *sc1 != '\0'; ++sc1) {
		for (sc2 = ct; *sc2 != '\0'; ++sc2) {
			if (*sc1 == *sc2)
				return (char *)sc1;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(strpbrk);
#endif

#ifndef __HAVE_ARCH_STRSEP
char *strsep(char **s, const char *ct)
{
	char *sbegin = *s;
	char *end;

	if (sbegin == NULL)
		return NULL;

	end = strpbrk(sbegin, ct);
	if (end)
		*end++ = '\0';
	*s = end;
	return sbegin;
}
EXPORT_SYMBOL(strsep);
#endif

bool sysfs_streq(const char *s1, const char *s2)
{
	while (*s1 && *s1 == *s2) {
		s1++;
		s2++;
	}

	if (*s1 == *s2)
		return true;
	if (!*s1 && *s2 == '\n' && !s2[1])
		return true;
	if (*s1 == '\n' && !s1[1] && !*s2)
		return true;
	return false;
}
EXPORT_SYMBOL(sysfs_streq);

int match_string(const char * const *array, size_t n, const char *string)
{
	int index;
	const char *item;

	for (index = 0; index < n; index++) {
		item = array[index];
		if (!item)
			break;
		if (!strcmp(item, string))
			return index;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(match_string);

int __sysfs_match_string(const char * const *array, size_t n, const char *str)
{
	const char *item;
	int index;

	for (index = 0; index < n; index++) {
		item = array[index];
		if (!item)
			break;
		if (sysfs_streq(item, str))
			return index;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(__sysfs_match_string);

/* Helper: aligned test */
static inline bool is_aligned(const void *p, size_t a)
{
	return (((unsigned long)p) & (a - 1)) == 0;
}

/* Aggressive memset: word-at-a-time, unrolled, fast fallback */
#ifndef __HAVE_ARCH_MEMSET
void *memset(void *s, int c, size_t count)
{
	unsigned char *d = s;
	unsigned long pattern;
	size_t w;

	if (count == 0)
		return s;

	/* small: byte loop */
	if (count < 32) {
		while (count--)
			*d++ = c;
		return s;
	}

	/* prepare pattern word */
	pattern = (unsigned char)c;
#if BITS_PER_LONG == 64
	pattern |= pattern << 8;
	pattern |= pattern << 16;
	pattern |= pattern << 32;
#else
	pattern |= pattern << 8;
	pattern |= pattern << 16;
#endif

	/* align dest to word boundary */
	while (!is_aligned(d, sizeof(unsigned long)) && count) {
		*d++ = c;
		count--;
	}

	/* bulk write (unrolled) */
	w = count / sizeof(unsigned long);
	while (w >= 8) {
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		*(unsigned long *)d = pattern; d += sizeof(unsigned long);
		w -= 8;
	}
	while (w--) {
		*(unsigned long *)d = pattern;
		d += sizeof(unsigned long);
	}
	/* tail bytes */
	count &= (sizeof(unsigned long) - 1);
	while (count--)
		*d++ = c;
	return s;
}
EXPORT_SYMBOL(memset);
#endif

void memzero_explicit(void *s, size_t count)
{
	memset(s, 0, count);
	barrier_data(s);
}
EXPORT_SYMBOL(memzero_explicit);

#ifndef __HAVE_ARCH_MEMSET16
void *memset16(uint16_t *s, uint16_t v, size_t count)
{
	uint16_t *xs = s;

	while (count--)
		*xs++ = v;
	return s;
}
EXPORT_SYMBOL(memset16);
#endif

#ifndef __HAVE_ARCH_MEMSET32
void *memset32(uint32_t *s, uint32_t v, size_t count)
{
	uint32_t *xs = s;

	while (count--)
		*xs++ = v;
	return s;
}
EXPORT_SYMBOL(memset32);
#endif

#ifndef __HAVE_ARCH_MEMSET64
void *memset64(uint64_t *s, uint64_t v, size_t count)
{
	uint64_t *xs = s;

	while (count--)
		*xs++ = v;
	return s;
}
EXPORT_SYMBOL(memset64);
#endif

/* Aggressive memcpy: prefetch for large copies, word-at-a-time, unrolled */
#ifndef __HAVE_ARCH_MEMCPY
void *memcpy(void *dest, const void *src, size_t count)
{
	unsigned char *d = dest;
	const unsigned char *s = src;
	size_t i, words;

	if (count == 0 || dest == src)
		return dest;

	/* Very small copies: simple byte loop (fast when inlined) */
	if (count < 32) {
		while (count--)
			*d++ = *s++;
		return dest;
	}

	/* For very large copies, hint prefetch to reduce cache miss */
	if (count >= 1024) {
		const unsigned char *p = s;
		for (i = 0; i < count; i += 256)
			__builtin_prefetch(p + i, 0, 3);
	}

	/* If both pointers aligned to word or arch allows unaligned, copy words */
	if ((is_aligned(d, sizeof(unsigned long)) && is_aligned(s, sizeof(unsigned long))) ||
	    IS_ENABLED(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)) {

		/* align dest by copying prefix bytes */
		while (!is_aligned(d, sizeof(unsigned long)) && count) {
			*d++ = *s++;
			count--;
		}

		words = count / sizeof(unsigned long);

		/* unrolled word copy using read_word_at_a_time for safe consumption */
		while (words >= 8) {
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			*(unsigned long *)d = read_word_at_a_time(s); d += sizeof(unsigned long); s += sizeof(unsigned long);
			words -= 8;
		}
		while (words--) {
			*(unsigned long *)d = read_word_at_a_time(s);
			d += sizeof(unsigned long);
			s += sizeof(unsigned long);
		}

		/* remaining tail bytes */
		count &= (sizeof(unsigned long) - 1);
		while (count--)
			*d++ = *s++;
		return dest;
	}

	/* fallback byte copy */
	while (count--)
		*d++ = *s++;
	return dest;
}
EXPORT_SYMBOL(memcpy);
#endif

/* memmove: reuse memcpy when safe, otherwise backward copy */
#ifndef __HAVE_ARCH_MEMMOVE
void *memmove(void *dest, const void *src, size_t count)
{
	unsigned char *d = dest;
	const unsigned char *s = src;

	if (d == s || count == 0)
		return dest;

	if (d < s) {
		/* forward copy safe: use memcpy fast path */
		return memcpy(dest, src, count);
	} else {
		/* overlapping backward copy */
		d += count;
		s += count;
		while (count--)
			*--d = *--s;
		return dest;
	}
}
EXPORT_SYMBOL(memmove);
#endif

/* memcmp: word-at-a-time compare with early exit using XOR */
#ifndef __HAVE_ARCH_MEMCMP
__visible int memcmp(const void *cs, const void *ct, size_t count)
{
	const unsigned char *s1 = cs;
	const unsigned char *s2 = ct;

	/* small compares: byte loop */
	if (count < 32)
		goto byte_cmp;

	/* word compare when safe */
	if ((is_aligned(s1, sizeof(unsigned long)) && is_aligned(s2, sizeof(unsigned long))) ||
	    IS_ENABLED(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)) {
		while (count >= sizeof(unsigned long)) {
			unsigned long w1 = read_word_at_a_time(s1);
			unsigned long w2 = read_word_at_a_time(s2);
			if (w1 != w2) {
				/* fall back to byte compare within this word */
				const unsigned char *a = s1;
				const unsigned char *b = s2;
				size_t i;
				for (i = 0; i < sizeof(unsigned long); ++i) {
					int diff = a[i] - b[i];
					if (diff)
						return diff;
				}
			}
			s1 += sizeof(unsigned long);
			s2 += sizeof(unsigned long);
			count -= sizeof(unsigned long);
		}
	}

byte_cmp:
	for (; count > 0; --count, ++s1, ++s2) {
		int diff = *s1 - *s2;
		if (diff)
			return diff;
	}
	return 0;
}
EXPORT_SYMBOL(memcmp);
#endif

#ifndef __HAVE_ARCH_BCMP
#undef bcmp
int bcmp(const void *a, const void *b, size_t len)
{
	return memcmp(a, b, len);
}
EXPORT_SYMBOL(bcmp);
#endif

#ifndef __HAVE_ARCH_MEMSCAN
void *memscan(void *addr, int c, size_t size)
{
	unsigned char *p = addr;

	while (size) {
		if (*p == c)
			return (void *)p;
		p++;
		size--;
	}
  	return (void *)p;
}
EXPORT_SYMBOL(memscan);
#endif

#ifndef __HAVE_ARCH_STRSTR
char *strstr(const char *s1, const char *s2)
{
	size_t l1, l2;

	l2 = strlen(s2);
	if (!l2)
		return (char *)s1;
	l1 = strlen(s1);
	while (l1 >= l2) {
		l1--;
		if (!memcmp(s1, s2, l2))
			return (char *)s1;
		s1++;
	}
	return NULL;
}
EXPORT_SYMBOL(strstr);
#endif

#ifndef __HAVE_ARCH_STRNSTR
char *strnstr(const char *s1, const char *s2, size_t len)
{
	size_t l2;

	l2 = strlen(s2);
	if (!l2)
		return (char *)s1;
	while (len >= l2) {
		len--;
		if (!memcmp(s1, s2, l2))
			return (char *)s1;
		s1++;
	}
	return NULL;
}
EXPORT_SYMBOL(strnstr);
#endif

#ifndef __HAVE_ARCH_MEMCHR
void *memchr(const void *s, int c, size_t n)
{
	const unsigned char *p = s;
	while (n-- != 0) {
        	if ((unsigned char)c == *p++) {
			return (void *)(p - 1);
		}
	}
	return NULL;
}
EXPORT_SYMBOL(memchr);
#endif

static void *check_bytes8(const u8 *start, u8 value, unsigned int bytes)
{
	while (bytes) {
		if (*start != value)
			return (void *)start;
		start++;
		bytes--;
	}
	return NULL;
}

void *memchr_inv(const void *start, int c, size_t bytes)
{
	u8 value = c;
	u64 value64;
	unsigned int words, prefix;

	if (bytes <= 16)
		return check_bytes8(start, value, bytes);

	value64 = value;
#if defined(CONFIG_ARCH_HAS_FAST_MULTIPLIER) && BITS_PER_LONG == 64
	value64 *= 0x0101010101010101ULL;
#elif defined(CONFIG_ARCH_HAS_FAST_MULTIPLIER)
	value64 *= 0x01010101;
	value64 |= value64 << 32;
#else
	value64 |= value64 << 8;
	value64 |= value64 << 16;
	value64 |= value64 << 32;
#endif

	prefix = (unsigned long)start % 8;
	if (prefix) {
		u8 *r;

		prefix = 8 - prefix;
		r = check_bytes8(start, value, prefix);
		if (r)
			return r;
		start += prefix;
		bytes -= prefix;
	}

	words = bytes / 8;

	while (words) {
		if (*(u64 *)start != value64)
			return check_bytes8(start, value, 8);
		start += 8;
		words--;
	}

	return check_bytes8(start, value, bytes % 8);
}
EXPORT_SYMBOL(memchr_inv);

char *strreplace(char *s, char old, char new)
{
	for (; *s; ++s)
		if (*s == old)
			*s = new;
	return s;
}
EXPORT_SYMBOL(strreplace);

void fortify_panic(const char *name)
{
	pr_emerg("detected buffer overflow in %s\n", name);
	BUG();
}
EXPORT_SYMBOL(fortify_panic);

