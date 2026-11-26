
#include "sdf-error-estimation.h"

#include <cmath>
#include "arithmetics.hpp"

namespace msdfgen {

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation) {
    if (!(sdf.width > 0 && sdf.height > 0))
        return line.setIntersections(std::vector<Scanline::Intersection>());
    double pixelY = clamp(projection.projectY(y)-.5, double(sdf.height-1));
    if (yAxisOrientation == Y_DOWNWARD)
        pixelY = sdf.height-1-pixelY;
    int b = (int) floor(pixelY);
    int t = b+1;
    double bt = pixelY-b;
    if (t >= sdf.height) {
        b = sdf.height-1;
        t = sdf.height-1;
        bt = 1;
    }
    bool inside = false;
    std::vector<Scanline::Intersection> intersections;
    float lv, rv = mix(*sdf(0, b), *sdf(0, t), bt);
    if ((inside = rv > .5f)) {
        Scanline::Intersection intersection = { -1e240, 1 };
        intersections.push_back(intersection);
    }
    for (int l = 0, r = 1; r < sdf.width; ++l, ++r) {
        lv = rv;
        rv = mix(*sdf(r, b), *sdf(r, t), bt);
        if (lv != rv) {
            double lr = double(.5f-lv)/double(rv-lv);
            if (lr >= 0 && lr <= 1) {
                Scanline::Intersection intersection = { projection.unprojectX(l+lr+.5), sign(rv-lv) };
                intersections.push_back(intersection);
            }
        }
    }
#ifdef MSDFGEN_USE_CPP11
    line.setIntersections((std::vector<Scanline::Intersection> &&) intersections);
#else
    line.setIntersections(intersections);
#endif
}

template <int N>
void scanlineMSDF(Scanline &line, const BitmapConstSection<float, N> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation) {
    if (!(sdf.width > 0 && sdf.height > 0))
        return line.setIntersections(std::vector<Scanline::Intersection>());
    double pixelY = clamp(projection.projectY(y)-.5, double(sdf.height-1));
    if (yAxisOrientation == Y_DOWNWARD)
        pixelY = sdf.height-1-pixelY;
    int b = (int) floor(pixelY);
    int t = b+1;
    double bt = pixelY-b;
    if (t >= sdf.height) {
        b = sdf.height-1;
        t = sdf.height-1;
        bt = 1;
    }
    bool inside = false;
    std::vector<Scanline::Intersection> intersections;
    float lv[3], rv[3];
    rv[0] = mix(sdf(0, b)[0], sdf(0, t)[0], bt);
    rv[1] = mix(sdf(0, b)[1], sdf(0, t)[1], bt);
    rv[2] = mix(sdf(0, b)[2], sdf(0, t)[2], bt);
    if ((inside = median(rv[0], rv[1], rv[2]) > .5f)) {
        Scanline::Intersection intersection = { -1e240, 1 };
        intersections.push_back(intersection);
    }
    for (int l = 0, r = 1; r < sdf.width; ++l, ++r) {
        lv[0] = rv[0], lv[1] = rv[1], lv[2] = rv[2];
        rv[0] = mix(sdf(r, b)[0], sdf(r, t)[0], bt);
        rv[1] = mix(sdf(r, b)[1], sdf(r, t)[1], bt);
        rv[2] = mix(sdf(r, b)[2], sdf(r, t)[2], bt);
        Scanline::Intersection newIntersections[4];
        int newIntersectionCount = 0;
        for (int i = 0; i < 3; ++i) {
            if (lv[i] != rv[i]) {
                double lr = double(.5f-lv[i])/double(rv[i]-lv[i]);
                if (lr >= 0 && lr <= 1) {
                    float v[3] = {
                        mix(lv[0], rv[0], lr),
                        mix(lv[1], rv[1], lr),
                        mix(lv[2], rv[2], lr)
                    };
                    if (median(v[0], v[1], v[2]) == v[i]) {
                        newIntersections[newIntersectionCount].x = projection.unprojectX(l+lr+.5);
                        newIntersections[newIntersectionCount].direction = sign(rv[i]-lv[i]);
                        ++newIntersectionCount;
                    }
                }
            }
        }
        // Sort new intersections
        if (newIntersectionCount >= 2) {
            if (newIntersections[0].x > newIntersections[1].x)
                newIntersections[3] = newIntersections[0], newIntersections[0] = newIntersections[1], newIntersections[1] = newIntersections[3];
            if (newIntersectionCount >= 3 && newIntersections[1].x > newIntersections[2].x) {
                newIntersections[3] = newIntersections[1], newIntersections[1] = newIntersections[2], newIntersections[2] = newIntersections[3];
                if (newIntersections[0].x > newIntersections[1].x)
                    newIntersections[3] = newIntersections[0], newIntersections[0] = newIntersections[1], newIntersections[1] = newIntersections[3];
            }
        }
        for (int i = 0; i < newIntersectionCount; ++i) {
            if ((newIntersections[i].direction > 0) == !inside) {
                intersections.push_back(newIntersections[i]);
                inside = !inside;
            }
        }
        // Consistency check
        float rvScalar = median(rv[0], rv[1], rv[2]);
        if ((rvScalar > .5f) != inside && rvScalar != .5f && !intersections.empty()) {
            intersections.pop_back();
            inside = !inside;
        }
    }
#ifdef MSDFGEN_USE_CPP11
    line.setIntersections((std::vector<Scanline::Intersection> &&) intersections);
#else
    line.setIntersections(intersections);
#endif
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation) {
    scanlineMSDF(line, sdf, projection, y, yAxisOrientation);
}
void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Projection &projection, double y, YAxisOrientation yAxisOrientation) {
    scanlineMSDF(line, sdf, projection, y, yAxisOrientation);
}

template <int N>
double estimateSDFErrorInner(const BitmapConstSection<float, N> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule) {
    if (sdf.width <= 1 || sdf.height <= 1 || scanlinesPerRow < 1)
        return 0;
    double subRowSize = 1./scanlinesPerRow;
    double xFrom = projection.unprojectX(.5);
    double xTo = projection.unprojectX(sdf.width-.5);
    double overlapFactor = 1/(xTo-xFrom);
    double error = 0;
    Scanline refScanline, sdfScanline;
    for (int row = 0; row < sdf.height-1; ++row) {
        for (int subRow = 0; subRow < scanlinesPerRow; ++subRow) {
            double bt = (subRow+.5)*subRowSize;
            double y = projection.unprojectY(row+bt+.5);
            shape.scanline(refScanline, y);
            scanlineSDF(sdfScanline, sdf, projection, y, shape.getYAxisOrientation());
            error += 1-overlapFactor*Scanline::overlap(refScanline, sdfScanline, xFrom, xTo, fillRule);
        }
    }
    return error/((sdf.height-1)*scanlinesPerRow);
}

double estimateSDFError(const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFErrorInner(sdf, shape, projection, scanlinesPerRow, fillRule);
}
double estimateSDFError(const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFErrorInner(sdf, shape, projection, scanlinesPerRow, fillRule);
}
double estimateSDFError(const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFErrorInner(sdf, shape, projection, scanlinesPerRow, fillRule);
}

template <int N>
void estimateSDFErrorTiledInner(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, N> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    tiles.clear();
    if (sdf.width <= 1 || sdf.height <= 1 || scanlinesPerRow < 1 || tileWidth < 1 || tileHeight < 1)
        return;
    // The sub-row strips are the Voronoi regions of the sampled scanlines within the evaluated region.
    double subRowSize = 1./scanlinesPerRow;
    int columns = (sdf.width-1+tileWidth-1)/tileWidth;
    int rows = (sdf.height-1+tileHeight-1)/tileHeight;
    std::vector<double> tileMismatchLengths(columns*rows, 0.);
    Scanline refScanline, sdfScanline;
    for (int row = 0; row < sdf.height-1; ++row) {
        for (int subRow = 0; subRow < scanlinesPerRow; ++subRow) {
            double bt = (subRow+.5)*subRowSize;
            double y = projection.unprojectY(row+bt+.5);
            shape.scanline(refScanline, y);
            scanlineSDF(sdfScanline, sdf, projection, y, shape.getYAxisOrientation());
            // Tile boundaries lie on pixel centers, so every sub-row strip belongs to exactly one tile row.
            int tileRow = (int) ((row+(subRow+.5)*subRowSize)/tileHeight);
            if (tileRow >= rows)
                tileRow = rows-1;
            // Split the scanline at tile boundaries. Each segment is assigned to a single tile,
            // so curves crossing the boundaries are integrated on exactly one side and never counted twice or missed.
            for (int tileCol = 0; tileCol < columns; ++tileCol) {
                double tileXFrom = projection.unprojectX(tileCol*tileWidth+.5);
                double tileXTo = projection.unprojectX(min((tileCol+1)*tileWidth, sdf.width-1)+.5);
                double overlap = Scanline::overlap(refScanline, sdfScanline, tileXFrom, tileXTo, fillRule);
                tileMismatchLengths[tileRow*columns+tileCol] += (tileXTo-tileXFrom)-overlap;
            }
        }
    }
    // Height of one sub-row strip in shape coordinates. The strips partition the height of one projected pixel row.
    double stripHeight = fabs(projection.unprojectY(1.)-projection.unprojectY(0.))*subRowSize;
    tiles.reserve(columns*rows);
    for (int tileRow = 0; tileRow < rows; ++tileRow) {
        int yMin = tileRow*tileHeight;
        int yMax = min((tileRow+1)*tileHeight, sdf.height-1);
        double tileYFrom = projection.unprojectY(yMin+.5);
        double tileYTo = projection.unprojectY(yMax+.5);
        for (int tileCol = 0; tileCol < columns; ++tileCol) {
            int xMin = tileCol*tileWidth;
            int xMax = min((tileCol+1)*tileWidth, sdf.width-1);
            double tileXFrom = projection.unprojectX(xMin+.5);
            double tileXTo = projection.unprojectX(xMax+.5);
            double errorArea = tileMismatchLengths[tileRow*columns+tileCol]*stripHeight;
            double tileArea = fabs(tileYTo-tileYFrom)*fabs(tileXTo-tileXFrom);
            TileSDFError tile;
            tile.tileX = tileCol;
            tile.tileY = tileRow;
            tile.xMin = xMin;
            tile.xMax = xMax;
            tile.yMin = yMin;
            tile.yMax = yMax;
            tile.area = tileArea;
            tile.errorArea = errorArea;
            tile.error = tileArea > 0 ? errorArea/tileArea : 0;
            tiles.push_back(tile);
        }
    }
}

void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiledInner(tiles, sdf, shape, projection, scanlinesPerRow, tileWidth, tileHeight, fillRule);
}
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiledInner(tiles, sdf, shape, projection, scanlinesPerRow, tileWidth, tileHeight, fillRule);
}
void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Projection &projection, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiledInner(tiles, sdf, shape, projection, scanlinesPerRow, tileWidth, tileHeight, fillRule);
}

// Legacy API

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Projection &projection, double y, bool inverseYAxis) {
    scanlineSDF(line, sdf, projection, y, inverseYAxis ? MSDFGEN_Y_AXIS_NONDEFAULT_ORIENTATION : MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Projection &projection, double y, bool inverseYAxis) {
    scanlineSDF(line, sdf, projection, y, inverseYAxis ? MSDFGEN_Y_AXIS_NONDEFAULT_ORIENTATION : MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Projection &projection, double y, bool inverseYAxis) {
    scanlineSDF(line, sdf, projection, y, inverseYAxis ? MSDFGEN_Y_AXIS_NONDEFAULT_ORIENTATION : MSDFGEN_Y_AXIS_DEFAULT_ORIENTATION);
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 1> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y) {
    scanlineSDF(line, sdf, Projection(scale, translate), y, inverseYAxis);
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 3> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y) {
    scanlineSDF(line, sdf, Projection(scale, translate), y, inverseYAxis);
}

void scanlineSDF(Scanline &line, const BitmapConstSection<float, 4> &sdf, const Vector2 &scale, const Vector2 &translate, bool inverseYAxis, double y) {
    scanlineSDF(line, sdf, Projection(scale, translate), y, inverseYAxis);
}

double estimateSDFError(const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFError(sdf, shape, Projection(scale, translate), scanlinesPerRow, fillRule);
}

double estimateSDFError(const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFError(sdf, shape, Projection(scale, translate), scanlinesPerRow, fillRule);
}

double estimateSDFError(const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, FillRule fillRule) {
    return estimateSDFError(sdf, shape, Projection(scale, translate), scanlinesPerRow, fillRule);
}

void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 1> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiled(tiles, sdf, shape, Projection(scale, translate), scanlinesPerRow, tileWidth, tileHeight, fillRule);
}

void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 3> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiled(tiles, sdf, shape, Projection(scale, translate), scanlinesPerRow, tileWidth, tileHeight, fillRule);
}

void estimateSDFErrorTiled(std::vector<TileSDFError> &tiles, const BitmapConstSection<float, 4> &sdf, const Shape &shape, const Vector2 &scale, const Vector2 &translate, int scanlinesPerRow, int tileWidth, int tileHeight, FillRule fillRule) {
    estimateSDFErrorTiled(tiles, sdf, shape, Projection(scale, translate), scanlinesPerRow, tileWidth, tileHeight, fillRule);
}

}
