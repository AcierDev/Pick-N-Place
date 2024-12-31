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
        borderSize_(borderSize),
        pickupPoint_(borderSize, borderSize) {}

  /**
   * @brief Generates a sequence of movements for placing pieces in a grid
   * @param rows Number of rows in the grid
   * @param cols Number of columns in the grid
   * @return Vector of points representing the sequence of movements
   */
  std::vector<Point> generatePattern(int rows, int cols) {
    std::vector<Point> pattern;
    if (rows <= 0 || cols <= 0) return pattern;

    // Calculate available space and spacing between pieces
    double availableWidth = trayWidth_ - 2 * borderSize_ - pieceSize_;
    double availableLength = trayLength_ - 2 * borderSize_ - pieceSize_;

    // Calculate spacing between pieces
    double xSpacing = availableWidth / (cols > 1 ? cols - 1 : 1);
    double ySpacing = availableLength / (rows > 1 ? rows - 1 : 1);

    // Verify spacing is sufficient
    if (xSpacing < 0 || ySpacing < 0) {
      Serial.println(F("Error: Pieces won't fit in tray"));
      return pattern;
    }

    // Generate placement positions in a snake pattern
    std::vector<Point> placements;
    for (int row = 0; row < rows; ++row) {
      bool rightToLeft = (row % 2) == 1;
      for (int col = 0; col < cols; ++col) {
        int actualCol = rightToLeft ? (cols - 1 - col) : col;

        // Calculate center position for this piece
        double x = borderSize_ + (pieceSize_ / 2) + (actualCol * xSpacing);
        double y = borderSize_ + (pieceSize_ / 2) + (row * ySpacing);

        placements.emplace_back(x, y);
      }
    }

    // Generate movement sequence
    for (const Point& placement : placements) {
      // Move to pickup position
      pattern.push_back(pickupPoint_);

      // Add safe intermediate points if needed
      if (needsToAvoidPlacements(pickupPoint_, placement, placements)) {
        // Move up, over, then down
        double safeHeight = trayLength_ + pieceSize_;
        pattern.emplace_back(Point(pickupPoint_.x, safeHeight));
        pattern.emplace_back(Point(placement.x, safeHeight));
      }

      // Move to placement position
      pattern.push_back(placement);
    }

    // Return to home
    pattern.push_back(Point(0, 0));

    return pattern;
  }

 private:
  const double trayWidth_;
  const double trayLength_;
  const double pieceSize_;
  const double borderSize_;
  const Point pickupPoint_;

  /**
   * @brief Generates a safe path to the target avoiding previously placed
   * pieces
   * @param pattern Vector to append path points to
   * @param target Target position to reach
   * @param placements Vector of all placement positions
   */
  void generateSafePath(std::vector<Point>& pattern, const Point& target,
                        const std::vector<Point>& placements) {
    // Find highest safe Y coordinate for current movement
    double safeY = trayLength_ + pieceSize_;

    // Add waypoints to safely reach target
    Point current = pattern.back();

    // Move up to safe height if necessary
    if (needsToAvoidPlacements(current, target, placements)) {
      pattern.emplace_back(current.x, safeY);
      pattern.emplace_back(target.x, safeY);
    }
  }

  /**
   * @brief Checks if direct movement between points would intersect with
   * placements
   * @param start Starting point
   * @param end Ending point
   * @param placements Vector of placement positions
   * @return true if path needs to avoid placements
   */
  bool needsToAvoidPlacements(const Point& start, const Point& end,
                              const std::vector<Point>& placements) {
    // Check if we're moving over any previously placed pieces
    for (const Point& p : placements) {
      if (p.y < std::max(start.y, end.y) && p.x > std::min(start.x, end.x) &&
          p.x < std::max(start.x, end.x)) {
        return true;
      }
    }
    return false;
  }
};