#include "xconvert.h"
#include <stdarg.h>
#include <math.h>

static char *pstr;

static int log10n (double val)
{
  int ret = 0;

  while (val >= 10) {
    if (val >= 100000) {
      val /= 100000; ret += 5;
    } else {
      val /= 10; ret++;
    }
  }
  while (val < 1) {
    if (val < 0.00001) {
      val *= 100000; ret -= 5;
    } else {
      val *= 10; ret--;
    }
  }
  return ret;
}


static double pwr10n (int val)
{
  double ret = 1;

  while (val > 0) {
    if (val >= 5) {
      ret *= 100000; val -= 5;
    } else {
      ret *= 10; val--;
    }
  }
  while (val < 0) {
    if (val <= -5) {
      ret /= 100000; val += 5;
    } else {
      ret /= 10; val++;
    }
  }
  return ret;
}


static void f2a(
  char* buf,
  double input,
  int digit,
  char format
)
{
  int dec;
  int exp = 0, n = 0;
  char s = 0;
  double pwr;
  const char *err = 0;

  if (isnan(input)) {
    err = "NaV";
  } else {
    if (digit < 0) digit = 6;
    if (input < 0) {
      input = -input; s = 0x2d;
    } else {
      s = 0x2b;
    }
    if (isinf(input)) {
      err = "INF";
    } else {
      if (format == 0x66) {
        input += pwr10n(-digit) / 2;
        n = log10n(input);
        if (n < 0) n = 0;
        if (n + digit + 3 >= 32) err = "OL";
      } else {
        if (input != 0) {
          input += pwr10n(log10n(input) - digit) / 2;
          exp = log10n(input);
          if (exp > 99 || digit + 6 >= 32) {
            err = "OL";
          } else {
            if (exp < -99) exp = -99;
            input /= pwr10n(exp);
          }
        }
      }
    }
    if (!err) {
      if (s == 0x2d) *buf++ = s;
      do {
        pwr = pwr10n(n);
        dec = input / pwr; input -= dec * pwr;
        if (n == -1) *buf++ = 0x2e;
        *buf++ = 0x30 + dec;
      } while (--n >= -digit);
      if (format != 0x66) {
        *buf++ = format;
        if (exp < 0) {
          exp = -exp; *buf++ = 0x2d;
        } else {
          *buf++ = 0x2b;
        }
        *buf++ = 0x30 + exp / 10;
        *buf++ = 0x30 + exp % 10;
      }
    }
  }
  if (err) {
    if (s) *buf++ = s;
    do *buf++ = *err++; while (*err);
  }
  *buf = 0;
}

void xfwritec (
  void(*f)(int),
  int ch
)
{
  if (f) {
    f(ch);
  } else if (pstr) {
    *pstr++ = ch;
  }
}

static void xfconvert (
  void(*f)(int),
  const char* format,
  va_list arg
)
{
  unsigned int div, i, j, k, m;
  int val, digit;
  char s[32], ch, dec, *ptr, c;
  long num;
  unsigned long unum;

  for (;;) {
    ch = *format++;
    if (!ch) break;
    if (ch != 0x25) {
      xfwritec(f, ch); continue;
    }
    m = k = 0;
    c = 0x20; digit = -1;
    ch = *format++;
    if (ch == 0x30) {
      c = 0x30; ch = *format++;
    } else {
      if (ch == 0x2d) {
        m = 2; ch = *format++;
      }
    }
    if (ch == 0x2a) {
      val = va_arg(arg, int);
      if (val < 0) {
        val = 0 - val; m = 2;
      }
      k = val; ch = *format++;
    } else {
      while (ch >= 0x30 && ch <= 0x39) {
        k = k * 10 + ch - 0x30;
        ch = *format++;
      }
    }
    if (ch == 0x2e) {
      ch = *format++;
      if (ch == 0x2a) {
        digit = va_arg(arg, int);
        ch = *format++;
      } else {
        digit = 0;
        while (ch >= 0x30 && ch <= 0x39) {
          digit = digit * 10 + ch - 0x30;
          ch = *format++;
        }
      }
    }
    if (ch == 0x6c) {
      m |= 4; ch = *format++;
    }
    if (!ch) break;
    switch (ch) {
      case 0x62:
        div = 2; break;
      case 0x6f:
        div = 8; break;
      case 0x64:
      case 0x75:
        div = 10; break;
      case 0x78:
      case 0x58:
        div = 16; break;
      case 0x63:
        xfwritec(f, (char)va_arg(arg, int)); continue;
      case 0x73:
        ptr = va_arg(arg, char*);
        if (!ptr) *ptr = 0;
        j = strlen(ptr);
        if (digit >= 0 && j > (unsigned int)digit) j = digit;
        for ( ; !(m & 2) && j < k; j++) xfwritec(f, c);
        while (*ptr && digit--) xfwritec(f, *ptr++);
        while (j++ < k) xfwritec(f, 0x20);
        continue;
      case 0x66:
      case 0x65:
      case 0x45:
        f2a(ptr = s, va_arg(arg, double), digit, ch);
        if (*ptr == 0x2d) xfwritec(f, *ptr++);
        for (j = strlen(ptr); !(m & 2) && j < k; j++) xfwritec(f, c);
        while (*ptr) xfwritec(f, *ptr++);
        while (j++ < k) xfwritec(f, 0x20);
        continue;
      default:
        xfwritec(f, ch); continue;
    }

    if (m & 4) {
      num = (long)va_arg(arg, long);
    } else {
      num = (ch == 0x64) ? (long)va_arg(arg, int) : (long)va_arg(arg, unsigned int);
    }

    if (ch == 0x64 && num < 0) {
      num = 0 - num; m |= 1;
    }
    i = 0; unum = num;
    do {
      dec = (char)(unum % div); unum /= div;
      if (dec > 9) dec += (ch == 0x78) ? 0x27 : 0x07;
      s[i++] = dec + 0x30;
    } while (unum != 0 && i < sizeof s);
    if (m & 1) {
      xfwritec(f, 0x2d);
    }
    for (j = i; !(m & 2) && j < k; j++) xfwritec(f, c);
    do xfwritec(f, s[--i]); while (i != 0);
    while (j++ < k) xfwritec(f, 0x20);
  }
}

void xconvert (
  char* buf,
  const char* format,
  ...
)
{
  va_list arg;

  pstr = buf;
  va_start(arg, format);
  xfconvert(0, format, arg);
  va_end(arg);
  *pstr = 0;
  pstr = 0;
}
