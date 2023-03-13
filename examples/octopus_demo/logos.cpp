#include "logos.h"

void decodeLogo(uint8_t *bitmap, const uint32_t *logo)
{
  uint8_t i, j;

  for (i = 0; i < 128; i++)
    bitmap[i] = 0;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < 32; j++)
    {
      bitmap[(4 * j) + i] = (((logo[i * 8] & 1 << j) >> j) << 7) |
                            (((logo[(i * 8) + 1] & 1 << j) >> j) << 6) |
                            (((logo[(i * 8) + 2] & 1 << j) >> j) << 5) |
                            (((logo[(i * 8) + 3] & 1 << j) >> j) << 4) |
                            (((logo[(i * 8) + 4] & 1 << j) >> j) << 3) |
                            (((logo[(i * 8) + 5] & 1 << j) >> j) << 2) |
                            (((logo[(i * 8) + 6] & 1 << j) >> j) << 1) |
                            (((logo[(i * 8) + 7] & 1 << j) >> j));
    }
  }
}
