#pragma once

#include <vector>

#include "../communication/Protocol.h"
#include "../types/Types.h"

/**
 * @brief Generates optimized movement patterns for pick and place operations
 */
class PatternGenerator {
 public:
  /**
   * @brief Constructs a pattern generator with piece size and border
   * @param pieceSize Size of the square pieces in inches
   * @param borderSize Required empty border size in inches
   */
  PatternGenerator(double pieceSize = 3.0, double borderSize = 0.5)
      : pieceSize_(pieceSize), borderSize_(borderSize) {}

  /**
   * @brief Generates a sequence of movements for placing pieces in a grid
   * @param rows Number of rows in the grid
   * @param cols Number of columns in the grid
   * @param startX X coordinate of the grid origin
   * @param startY Y coordinate of the grid origin
   * @param gridWidth Width of the grid in inches
   * @param gridLength Length of the grid in inches
   * @return Vector of points representing placement positions
   */
  std::vector<Point> generatePattern(int rows, int cols, double startX = 0,
                                     double startY = 0, double gridWidth = 26.0,
                                     double gridLength = 35.0) {
    std::vector<Point> pattern;
    if (rows <= 0 || cols <= 0) return pattern;

    // Calculate usable area dimensions
    double usableWidth = gridWidth - 2 * borderSize_;
    double usableLength = gridLength - 2 * borderSize_;

    // Calculate spacing between pieces
    double xSpacing = usableWidth / (cols - 1);
    double ySpacing = usableLength / (rows - 1);

    // Add border offset to starting position
    double adjustedStartX = startX + borderSize_;
    double adjustedStartY = startY + borderSize_;

    // Verify spacing is sufficient (compare with piece size)
    if (xSpacing < pieceSize_ || ySpacing < pieceSize_) {
      Protocol::error("Pieces won't fit in grid with required spacing");
      return pattern;
    }

    // Generate pattern points with adjusted start position
    for (int row = rows - 1; row >= 0; --row) {
      bool leftToRight = ((rows - 1 - row) % 2) == 0;

      if (leftToRight) {
        for (int col = 0; col < cols; ++col) {
          double x = adjustedStartX + (col * xSpacing);
          double y = adjustedStartY + (row * ySpacing);
          pattern.emplace_back(x, y);
        }
      } else {
        for (int col = cols - 1; col >= 0; --col) {
          double x = adjustedStartX + (col * xSpacing);
          double y = adjustedStartY + (row * ySpacing);
          pattern.emplace_back(x, y);
        }
      }
    }

    return pattern;
  }

 private:
  const double pieceSize_;   // Size of each piece in inches
  const double borderSize_;  // Required border size in inches
};