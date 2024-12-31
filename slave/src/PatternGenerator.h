#pragma once

#include <vector>

/**
 * @brief Represents a 2D point in space
 */
struct Point {
  double x;
  double y;

  Point(double x = 0, double y = 0) : x(x), y(y) {}
};

/**
 * @brief Generates optimized movement patterns for pick and place operations
 */
class PatternGenerator {
 public:
  /**
   * @brief Constructs a pattern generator with tray dimensions and piece size
   * @param trayWidth Width of the tray in inches
   * @param trayLength Length of the tray in inches
   * @param pieceSize Size of the square pieces in inches
   * @param borderSize Required empty border size in inches
   */
  PatternGenerator(double trayWidth = 26.0, double trayLength = 35.0,
                   double pieceSize = 3.0, double borderSize = 0.5)
      : trayWidth_(trayWidth),
        trayLength_(trayLength),
        pieceSize_(pieceSize),
        borderSize_(borderSize) {}

  /**
   * @brief Generates a sequence of movements for placing pieces in a grid
   * @param rows Number of rows in the grid
   * @param cols Number of columns in the grid
   * @return Vector of points representing placement positions
   */
  std::vector<Point> generatePattern(int rows, int cols) {
    std::vector<Point> pattern;
    if (rows <= 0 || cols <= 0) return pattern;

    // Calculate usable area dimensions
    double usableWidth = trayWidth_ - 2 * borderSize_;
    double usableLength = trayLength_ - 2 * borderSize_;

    // Calculate spacing between pieces
    double xSpacing = (usableWidth - pieceSize_) / (cols - 1);
    double ySpacing = (usableLength - pieceSize_) / (rows - 1);

    // Verify spacing is sufficient
    if (xSpacing < pieceSize_ || ySpacing < pieceSize_) {
      Serial.println(
          F("Error: Pieces won't fit in tray with required spacing"));
      return pattern;
    }

    // Generate grid positions in a snake pattern
    for (int row = 0; row < rows; ++row) {
      bool rightToLeft = (row % 2) == 1;  // Alternate direction each row

      for (int col = 0; col < cols; ++col) {
        int actualCol = rightToLeft ? (cols - 1 - col) : col;

        // Calculate center position for this piece
        double x = borderSize_ + (actualCol * xSpacing);
        double y = borderSize_ + (row * ySpacing);

        pattern.emplace_back(x, y);
      }
    }

    return pattern;
  }

 private:
  const double trayWidth_;   // Width of the tray in inches
  const double trayLength_;  // Length of the tray in inches
  const double pieceSize_;   // Size of each piece in inches
  const double borderSize_;  // Required border size in inches
};