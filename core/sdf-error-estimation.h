
#pragma once

#include <vector>

#include "Vector2.hpp"
#include "Shape.h"
#include "Projection.h"
#include "Scanline.h"
#include "BitmapRef.hpp"

namespace msdfgen {

/// Analytically constructs a scanline at y evaluating fill by linear interpolation of the SDF.
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation = MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation = MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation = MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);

/// Estimates the portion of the area that will be filled incorrectly when rendering using the SDF.
double estimateSDFError(const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);
double estimateSDFError(const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);
double estimateSDFError(const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);

/// Per-tile statistics of the SDF reconstruction error. See estimateSDFErrorTiled.
struct TileSDFError {
    /// Zero-based column of the tile in the tile grid, counting from the left in projected pixel coordinates.
    int tileX;
    /// Zero-based row of the tile in the tile grid, counting from the bottom in projected pixel coordinates.
    int tileY;
    /// Inclusive lower bound of the tile in projected pixel X coordinates.
    int xMin;
    /// Exclusive upper bound of the tile in projected pixel X coordinates.
    int xMax;
    /// Inclusive lower bound of the tile in projected pixel Y coordinates.
    int yMin;
    /// Exclusive upper bound of the tile in projected pixel Y coordinates.
    int yMax;
    /// Area of the tile measured in shape coordinates.
    double area;
    /// Area of the tile whose fill is reconstructed incorrectly, measured in shape coordinates.
    double errorArea;
    /// Fraction of the tile's area that is reconstructed incorrectly, i.e. errorArea / area.
    double error;
};

/// Splits the region evaluated by estimateSDFError into a grid of tiles and estimates the reconstruction error of each tile separately.
/// The tiles cover the same region as the whole-image estimate, from the center of the corner pixels to the center of the opposite corner pixels.
/// Tile boundaries are placed at pixel centers (half-integer pixel coordinates), so a curve crossing a boundary is integrated on exactly one side of it.
/// The tiles therefore partition the region without overlap, and both the sums of their areas and error areas match a whole-image evaluation over the same region.
/// The area-weighted average of the per-tile error values equals the value returned by estimateSDFError with the same arguments.
/// Tile sizes may not divide the bitmap dimensions evenly, in which case the tiles on the right and top edges are clipped.
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);

/// Equivalent to estimateSDFErrorTiled with equal horizontal and vertical tile size.
inline void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileSize, FillRule fillRule = FILL_NONZERO) {
    estimateSDFErrorTiled(tiles, sdf, shape, projection, scanlinesPerRow, tileSize, tileSize, fillRule);
}
inline void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileSize, FillRule fillRule = FILL_NONZERO) {
    estimateSDFErrorTiled(tiles, sdf, shape, projection, scanlinesPerRow, tileSize, tileSize, fillRule);
}
inline void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileSize, FillRule fillRule = FILL_NONZERO) {
    estimateSDFErrorTiled(tiles, sdf, shape, projection, scanlinesPerRow, tileSize, tileSize, fillRule);
}

// Old version of the function API's kept for backwards compatibility
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Projection &projection, double y, bool inverseYAxis);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Projection &projection, double y, bool inverseYAxis);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Projection &projection, double y, bool inverseYAxis);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y);
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y);
double estimateSDFError(const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);
double estimateSDFError(const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);
double estimateSDFError(const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule = FILL_NONZERO);

void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule = FILL_NONZERO);

}
