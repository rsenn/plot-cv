#include <png.hpp>

int
main() {
  png::image<png::index_pixel> image(256, 256);
  png::palette pal(64);

  for(size_t i = 0; i < pal.size(); ++i) { pal[i] = png::color((i & (0x3 << 4)) << 2, (i & (0x3 << 2)) << 4, (i & 0x3) << 6); }
  image.set_palette(pal);

  for(png::uint_32 y = 0; y < image.get_height(); ++y) {
    for(png::uint_32 x = 0; x < image.get_width(); ++x) {
      unsigned char p = ((y ^ (~x)) | ((x >> 3) & (y >> 2))) & 0x3f;

      image[y][x] = png::index_pixel(p);
    }
  }
  image.write("palette.png");
}
