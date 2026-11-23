
/*
 * ASYMMETRIC DISTANCE RANGE TESTS
 *
 * Verifies that when the signed distance range is asymmetric (different inner
 * and outer extents), the edge transition of the generated distance field is
 * not masked or displaced by saturation at either end of the range:
 *
 *  - DistanceMapping unit tests: asymmetric ranges map the zero-distance
 *    threshold to a value other than 0.5, deltas are unaffected by translate.
 *  - Regression tests: per-pixel sampled values of the same shape rendered
 *    with symmetric, asymmetric, very narrow inner, and very narrow outer
 *    ranges are compared against the analytic signed distance of the shape,
 *    at low and high resolution, under non-uniform (anisotropic) scaling.
 *  - Coverage (inside/outside classification at the zero-distance threshold)
 *    and transition width must stay consistent when DistanceMapping and the
 *    Projection change.
 *  - Saturation at the positive and negative endpoints (and after byte
 *    conversion) must not be mistaken for the geometric boundary.
 *
 * msdfgen sign convention: the signed distance is POSITIVE inside the shape
 * and negative outside. Range(lower, upper) maps the outermost representable
 * distance (lower, negative) to 0 and the innermost (upper, positive) to 1.
 *
 * All assertions are on concrete pixels / sampled values. No GPU or external
 * resources are required.
 */

#include <cstdio>
#include <cmath>
#include <algorithm>

#include <msdfgen.h>

namespace {

int failures = 0;
int checks = 0;
int printedFailures = 0;

void reportFailure(const char *file, int line, const char *expr) {
    ++failures;
    if (++printedFailures <= 25)
        std::printf("FAIL %s:%d: %s\n", file, line, expr);
}

void reportFailureNear(const char *file, int line, const char *expr, double a, double b, double tol) {
    ++failures;
    if (++printedFailures <= 25)
        std::printf("FAIL %s:%d: %s: |%.17g - %.17g| = %.3g > %g\n", file, line, expr, a, b, std::fabs(a-b), tol);
}

}

#define CHECK(cond) do { ++checks; if (!(cond)) reportFailure(__FILE__, __LINE__, #cond); } while (0)
#define CHECK_NEAR(a, b, tol) do { ++checks; double _a = (a), _b = (b), _t = (tol); if (!(std::fabs(_a-_b) <= _t)) reportFailureNear(__FILE__, __LINE__, #a " ~ " #b, _a, _b, _t); } while (0)

namespace {

using namespace msdfgen;

// The test shape: an axis-aligned rectangle (0, 0) - (8, 8).
// Its true signed distance (positive inside) is known analytically.
const double RECT_X0 = 0, RECT_Y0 = 0, RECT_X1 = 8, RECT_Y1 = 8;

double rectSignedDistance(Point2 p) {
    double dx = std::max(std::max(RECT_X0-p.x, p.x-RECT_X1), 0.0);
    double dy = std::max(std::max(RECT_Y0-p.y, p.y-RECT_Y1), 0.0);
    if (dx > 0 || dy > 0)
        return -std::sqrt(dx*dx+dy*dy);
    return std::min(std::min(p.x-RECT_X0, RECT_X1-p.x), std::min(p.y-RECT_Y0, RECT_Y1-p.y));
}

Shape makeRectangle() {
    Shape shape;
    Contour &contour = shape.addContour();
    contour.addEdge(EdgeHolder(Point2(RECT_X0, RECT_Y0), Point2(RECT_X1, RECT_Y0)));
    contour.addEdge(EdgeHolder(Point2(RECT_X1, RECT_Y0), Point2(RECT_X1, RECT_Y1)));
    contour.addEdge(EdgeHolder(Point2(RECT_X1, RECT_Y1), Point2(RECT_X0, RECT_Y1)));
    contour.addEdge(EdgeHolder(Point2(RECT_X0, RECT_Y1), Point2(RECT_X0, RECT_Y0)));
    shape.orientContours();
    return shape;
}

// Non-uniform (anisotropic) projection: X and Y scales differ by a factor of 2.
// Shape region [-2, 10] x [-2, 10] is covered at both resolutions, and the
// rectangle boundary falls exactly between pixel rows/columns, so no pixel
// center lies exactly on the edge.
const Vector2 TRANSLATE(2, 2);
const int LOW_W = 48, LOW_H = 96;
const Vector2 LOW_SCALE(4, 8); // 4 px/unit in X, 8 px/unit in Y
const int HIGH_W = 192, HIGH_H = 384;
const Vector2 HIGH_SCALE(16, 32); // 4x the low resolution

// The four tested distance ranges for the same shape.
const Range RANGE_SYMMETRIC(-4, 4);          // symmetric control
const Range RANGE_ASYMMETRIC(-1, 7);         // asymmetric: narrow outer (1), wide inner (7)
const Range RANGE_NARROW_OUTER(-0.125, 7.875); // very narrow outer range
const Range RANGE_NARROW_INNER(-7.875, 0.125); // very narrow inner range
const Range RANGES[] = { RANGE_SYMMETRIC, RANGE_ASYMMETRIC, RANGE_NARROW_OUTER, RANGE_NARROW_INNER };
const int N_RANGES = 4;

Bitmap<float, 1> renderSdf(const Shape &shape, int w, int h, const Vector2 &scale, Range range) {
    Bitmap<float, 1> bmp(w, h);
    generateSDF(bmp, shape, Projection(scale, TRANSLATE), range, GeneratorConfig(false));
    return bmp;
}

Point2 pixelCenter(const Vector2 &scale, int x, int y) {
    return Projection(scale, TRANSLATE).unproject(Point2(x+.5, y+.5));
}

// Width of the first run of pixels with values strictly inside (0, 1)
// along the horizontal scanline y, starting from x = 0.
int firstTransitionRunX(const Bitmap<float, 1> &bmp, int y) {
    int x = 0;
    while (x < bmp.width() && !(*bmp(x, y) > 0.f && *bmp(x, y) < 1.f))
        ++x;
    int start = x;
    while (x < bmp.width() && *bmp(x, y) > 0.f && *bmp(x, y) < 1.f)
        ++x;
    return x-start;
}

// Same along the vertical scanline x, starting from y = 0.
int firstTransitionRunY(const Bitmap<float, 1> &bmp, int x) {
    int y = 0;
    while (y < bmp.height() && !(*bmp(x, y) > 0.f && *bmp(x, y) < 1.f))
        ++y;
    int start = y;
    while (y < bmp.height() && *bmp(x, y) > 0.f && *bmp(x, y) < 1.f)
        ++y;
    return y-start;
}

// X index of the first pixel classified as inside (value above zero level) on scanline y.
int firstInsideX(const Bitmap<float, 1> &bmp, int y, double zeroLevel) {
    for (int x = 0; x < bmp.width(); ++x)
        if (*bmp(x, y) > zeroLevel)
            return x;
    return -1;
}

int firstInsideY(const Bitmap<float, 1> &bmp, int x, double zeroLevel) {
    for (int y = 0; y < bmp.height(); ++y)
        if (*bmp(x, y) > zeroLevel)
            return y;
    return -1;
}

void testDistanceMappingUnits() {
    // Asymmetric range: outer extent 1, inner extent 7.
    DistanceMapping m(RANGE_ASYMMETRIC);
    CHECK_NEAR(m(-1), 0, 1e-12);
    CHECK_NEAR(m(7), 1, 1e-12);
    // Zero-distance threshold is NOT at 0.5 for an asymmetric range.
    CHECK_NEAR(m(0), 0.125, 1e-12);
    CHECK(std::fabs(m(0)-.5) > .1);
    // Deltas are scaled but not translated.
    CHECK_NEAR(m(DistanceMapping::Delta(8)), 1, 1e-12);
    CHECK_NEAR(m(DistanceMapping::Delta(2)), 0.25, 1e-12);
    // Inverse round-trip.
    DistanceMapping inv = m.inverse();
    CHECK_NEAR(inv(m(3.7)), 3.7, 1e-12);
    CHECK_NEAR(inv(m(-0.9)), -0.9, 1e-12);
    // Static inverse: value -> distance.
    DistanceMapping staticInv = DistanceMapping::inverse(RANGE_ASYMMETRIC);
    CHECK_NEAR(staticInv(0), -1, 1e-12);
    CHECK_NEAR(staticInv(1), 7, 1e-12);
    CHECK_NEAR(staticInv(0.125), 0, 1e-12);
    // Symmetric control range keeps the zero threshold at 0.5.
    CHECK_NEAR(DistanceMapping(RANGE_SYMMETRIC)(0), 0.5, 1e-12);
    // Very narrow outer / inner ranges push the zero threshold to the extremes.
    CHECK_NEAR(DistanceMapping(RANGE_NARROW_OUTER)(0), 0.015625, 1e-12);
    CHECK_NEAR(DistanceMapping(RANGE_NARROW_INNER)(0), 0.984375, 1e-12);
}

// Regression: every sampled pixel value must equal the mapped analytic
// distance, for all ranges and both resolutions.
void testSampledValuesAgainstAnalytic(const Shape &shape) {
    const struct {
        int w, h;
        Vector2 scale;
    } resolutions[] = {
        { LOW_W, LOW_H, LOW_SCALE },
        { HIGH_W, HIGH_H, HIGH_SCALE }
    };
    for (int r = 0; r < N_RANGES; ++r) {
        DistanceMapping mapping(RANGES[r]);
        for (int res = 0; res < 2; ++res) {
            Bitmap<float, 1> bmp = renderSdf(shape, resolutions[res].w, resolutions[res].h, resolutions[res].scale, RANGES[r]);
            for (int y = 0; y < bmp.height(); ++y) {
                for (int x = 0; x < bmp.width(); ++x) {
                    double expected = mapping(rectSignedDistance(pixelCenter(resolutions[res].scale, x, y)));
                    CHECK_NEAR(*bmp(x, y), expected, 2e-6);
                }
            }
        }
    }
    // Hard-coded regression anchors (low resolution, asymmetric range (-1, 7)).
    Bitmap<float, 1> bmp = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGE_ASYMMETRIC);
    // Pixel (23, 47): shape position (3.875, 3.9375), distance 3.875, mapped to 0.609375.
    CHECK_NEAR(*bmp(23, 47), 0.609375, 1e-6);
    // Pixel (8, 16): shape position (0.125, 0.0625), distance 0.0625, mapped to 0.1328125.
    CHECK_NEAR(*bmp(8, 16), 0.1328125, 1e-6);
    // Pixel (7, 15): shape position (-0.125, -0.0625), distance -sqrt(0.01953125).
    CHECK_NEAR(*bmp(7, 15), (1-std::sqrt(0.01953125))/8, 1e-6);
}

// Coverage (inside/outside classification at the zero-distance threshold) must
// be identical for all ranges: saturation must not be misjudged as the
// geometric boundary.
void testCoverageInvariance(const Shape &shape) {
    Bitmap<float, 1> bmp[N_RANGES];
    for (int r = 0; r < N_RANGES; ++r)
        bmp[r] = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGES[r]);

    int insideCount[N_RANGES] = { 0, 0, 0, 0 };
    for (int y = 0; y < LOW_H; ++y) {
        for (int x = 0; x < LOW_W; ++x) {
            bool inside[N_RANGES];
            for (int r = 0; r < N_RANGES; ++r) {
                double zeroLevel = DistanceMapping(RANGES[r])(0);
                inside[r] = *bmp[r](x, y) > zeroLevel;
                insideCount[r] += inside[r];
            }
            // Same shape, same projection: classification must not depend on the range,
            // even when one end of the range is saturated over most of the bitmap.
            CHECK(inside[1] == inside[0]);
            CHECK(inside[2] == inside[0]);
            CHECK(inside[3] == inside[0]);
        }
    }
    // Rectangle spans pixels x in [8, 39], y in [16, 79] -> 32*64 = 2048 inside pixels.
    for (int r = 0; r < N_RANGES; ++r)
        CHECK(insideCount[r] == 2048);
    // Coverage agrees with the analytic area ratio 64/144 = 4/9.
    CHECK_NEAR(insideCount[0]/double(LOW_W*LOW_H), 4/9.0, 1e-12);
}

// Low vs. high resolution: classification and coverage must be consistent.
void testResolutionConsistency(const Shape &shape) {
    const Range range = RANGE_ASYMMETRIC;
    const double zeroLevel = DistanceMapping(range)(0);
    Bitmap<float, 1> low = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, range);
    Bitmap<float, 1> high = renderSdf(shape, HIGH_W, HIGH_H, HIGH_SCALE, range);

    int lowInside = 0, highInside = 0;
    for (int y = 0; y < LOW_H; ++y) {
        for (int x = 0; x < LOW_W; ++x) {
            bool lowIn = *low(x, y) > zeroLevel;
            lowInside += lowIn;
            // Each low-res pixel corresponds to a 4x4 block of high-res pixels;
            // because the boundary aligns with pixel boundaries at both
            // resolutions, the classification must match exactly.
            for (int dy = 0; dy < 4; ++dy)
                for (int dx = 0; dx < 4; ++dx)
                    CHECK((*high(4*x+dx, 4*y+dy) > zeroLevel) == lowIn);
        }
    }
    for (int y = 0; y < HIGH_H; ++y)
        for (int x = 0; x < HIGH_W; ++x)
            highInside += *high(x, y) > zeroLevel;
    CHECK(lowInside == 2048);
    CHECK(highInside == 32768);
    CHECK_NEAR(lowInside/double(LOW_W*LOW_H), highInside/double(HIGH_W*HIGH_H), 1e-12);
}

// Transition width must scale with the range width (DistanceMapping) and with
// the projection scale, and differ between X and Y under non-uniform scaling.
void testTransitionWidth(const Shape &shape) {
    // Symmetric range (-1, 1): band of values in (0, 1) is 2 shape units wide.
    Bitmap<float, 1> low = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, Range(-1, 1));
    int bandX = firstTransitionRunX(low, 47); // scanline through the rectangle's middle
    int bandY = firstTransitionRunY(low, 23);
    CHECK(std::abs(bandX-8) <= 1);  // 2 units * 4 px/unit
    CHECK(std::abs(bandY-16) <= 1); // 2 units * 8 px/unit
    // Non-uniform scaling: Y band is twice as wide in pixels as the X band.
    CHECK(std::abs(bandY-2*bandX) <= 2);

    // Doubling the range width doubles the transition width in pixels.
    Bitmap<float, 1> lowWide = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, Range(-2, 2));
    CHECK(std::abs(firstTransitionRunX(lowWide, 47)-2*bandX) <= 2);
    CHECK(std::abs(firstTransitionRunY(lowWide, 23)-2*bandY) <= 2);

    // 4x projection scale: transition width scales by 4 in pixels,
    // i.e. stays constant in shape units.
    Bitmap<float, 1> high = renderSdf(shape, HIGH_W, HIGH_H, HIGH_SCALE, Range(-1, 1));
    CHECK(std::abs(firstTransitionRunX(high, 191)-4*bandX) <= 4);
    CHECK(std::abs(firstTransitionRunY(high, 95)-4*bandY) <= 4);
}

// The zero-distance crossing (geometric boundary) must sit at the same pixels
// for all ranges and both resolutions.
void testZeroCrossingPosition(const Shape &shape) {
    for (int r = 0; r < N_RANGES; ++r) {
        double zeroLevel = DistanceMapping(RANGES[r])(0);
        Bitmap<float, 1> low = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGES[r]);
        Bitmap<float, 1> high = renderSdf(shape, HIGH_W, HIGH_H, HIGH_SCALE, RANGES[r]);
        // Boundary x = 0 projects to pixel boundary 7.5 (low) / 31.5 (high).
        CHECK(firstInsideX(low, 47, zeroLevel) == 8);
        CHECK(firstInsideX(high, 191, zeroLevel) == 32);
        // Boundary y = 0 projects to pixel boundary 15.5 (low) / 63.5 (high).
        CHECK(firstInsideY(low, 23, zeroLevel) == 16);
        CHECK(firstInsideY(high, 95, zeroLevel) == 64);
        // The values of the pixels straddling the boundary are symmetric
        // around the zero-distance threshold, wherever that threshold lies.
        double outV = *low(7, 47), inV = *low(8, 47);
        CHECK(outV < zeroLevel);
        CHECK(inV > zeroLevel);
        CHECK_NEAR((outV-zeroLevel)+(inV-zeroLevel), 0, 2e-6);
    }
}

// Saturation at the negative and positive endpoints must not be confused with
// the geometric boundary.
void testEndpointSaturation(const Shape &shape) {
    // Asymmetric range (-1, 7): the outer (negative) end saturates away from
    // the shape, the inner end never saturates (max depth in the bitmap is
    // 3.875 < 7).
    {
        Bitmap<float, 1> bmp = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGE_ASYMMETRIC);
        int saturatedLow = 0, expectedSaturatedLow = 0, saturatedHigh = 0, zeroBytes = 0;
        for (int y = 0; y < LOW_H; ++y) {
            for (int x = 0; x < LOW_W; ++x) {
                float v = *bmp(x, y);
                double d = rectSignedDistance(pixelCenter(LOW_SCALE, x, y));
                expectedSaturatedLow += d <= -1;
                saturatedLow += v <= 0.f;
                saturatedHigh += v >= 1.f;
                if (v <= 0.f)
                    CHECK(d < 0); // saturated pixels are strictly outside, not on the boundary
                if (pixelFloatToByte(v) == 0x00) {
                    ++zeroBytes;
                    CHECK(d <= -1+1e-9); // byte-level saturation only away from the shape
                }
            }
        }
        CHECK(saturatedLow == expectedSaturatedLow);
        CHECK(saturatedLow > 1000); // saturation is massive, yet coverage is unaffected (tested above)
        CHECK(zeroBytes == expectedSaturatedLow);
        CHECK(saturatedHigh == 0);
    }
    // Very narrow inner range (-7.875, 0.125): most of the inside saturates at
    // the positive endpoint, but the saturated region must stay strictly
    // inside the geometric boundary.
    {
        Bitmap<float, 1> bmp = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGE_NARROW_INNER);
        double zeroLevel = DistanceMapping(RANGE_NARROW_INNER)(0);
        int saturatedHigh = 0, fullBytes = 0;
        for (int y = 0; y < LOW_H; ++y) {
            for (int x = 0; x < LOW_W; ++x) {
                float v = *bmp(x, y);
                double d = rectSignedDistance(pixelCenter(LOW_SCALE, x, y));
                if (v >= 1.f) {
                    ++saturatedHigh;
                    CHECK(d > 0); // saturated pixels are strictly inside
                    CHECK(d >= 0.125-1e-9); // and at least the inner range away from the boundary
                }
                fullBytes += pixelFloatToByte(v) == 0xff;
            }
        }
        // Pixels with depth >= 0.125: x in [8, 39], y in [17, 78] -> 32*62 = 1984.
        CHECK(saturatedHigh == 1984);
        CHECK(fullBytes == 1984);
        // An unsaturated inside pixel still exists between the boundary and the
        // saturated region: pixel (8, 16) has distance 0.0625 < 0.125.
        float edgeV = *bmp(8, 16);
        CHECK(edgeV > float(zeroLevel));
        CHECK(edgeV < 1.f);
        CHECK_NEAR(edgeV, 0.9921875, 1e-6);
    }
    // Very narrow outer range (-0.125, 7.875): most of the outside saturates at
    // the negative endpoint, but the saturated region must stay strictly outside.
    {
        Bitmap<float, 1> bmp = renderSdf(shape, LOW_W, LOW_H, LOW_SCALE, RANGE_NARROW_OUTER);
        double zeroLevel = DistanceMapping(RANGE_NARROW_OUTER)(0);
        int saturatedLow = 0, expectedSaturatedLow = 0;
        for (int y = 0; y < LOW_H; ++y) {
            for (int x = 0; x < LOW_W; ++x) {
                float v = *bmp(x, y);
                double d = rectSignedDistance(pixelCenter(LOW_SCALE, x, y));
                expectedSaturatedLow += d <= -0.125;
                if (v <= 0.f) {
                    ++saturatedLow;
                    CHECK(d < 0); // saturated pixels are strictly outside
                    CHECK(d <= -0.125+1e-9);
                }
            }
        }
        CHECK(saturatedLow == expectedSaturatedLow);
        CHECK(saturatedLow > 1000);
        // An unsaturated outside pixel still exists between the boundary and the
        // saturated region: pixel (23, 15) has distance -0.0625.
        float edgeV = *bmp(23, 15);
        CHECK(edgeV > 0.f);
        CHECK(edgeV < float(zeroLevel));
        CHECK_NEAR(edgeV, 0.0078125, 1e-6);
    }
}

}

int main() {
    Shape shape = makeRectangle();

    testDistanceMappingUnits();
    testSampledValuesAgainstAnalytic(shape);
    testCoverageInvariance(shape);
    testResolutionConsistency(shape);
    testTransitionWidth(shape);
    testZeroCrossingPosition(shape);
    testEndpointSaturation(shape);

    if (failures) {
        std::printf("FAILED: %d of %d checks failed\n", failures, checks);
        return 1;
    }
    std::printf("PASSED: all %d checks\n", checks);
    return 0;
}
