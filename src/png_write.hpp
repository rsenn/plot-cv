#ifndef PNG_WRITE_HPP
#define PNG_WRITE_HPP

#include <opencv2/core.hpp>
#include <png++/png.hpp>
#include <array>

template<class ColorType>
void
write_mat(const std::string& filename, const cv::Mat& mat, const std::array<ColorType,256>& palette) {

  png::image<png::index_pixel> image(mat.cols, mat.rows);

  png::palette pal(palette.size());
  size_t i = 0;

  for(const auto& color : palette) {
    pal[i] = png::color(color[0], color[1], color[2]);
    i++;
  }

  image.set_palette(pal);

  for(png::uint_32 y = 0; y < mat.rows; ++y)
    for(png::uint_32 x = 0; x < mat.cols; ++x) image[y][x] = png::index_pixel(mat.at<uchar>(y, x));

  image.write(filename);
}

#endif /* PNG_WRITE_HPP */
