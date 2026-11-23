// Unit and regression tests for multi-channel distance field error correction.
//
// The tests distinguish two kinds of defects that the correction pass must treat differently:
//   * inversion artifacts - the linearly interpolated median crosses the shape threshold even
//     though both texel medians are on the same side of it. These must be corrected even when the
//     texel is protected (by corners, edges, or protectAll).
//   * ordinary distance deviations - the interpolated median is merely out of the expected range.
//     These must be left alone when the texel is protected, and corrected otherwise.
//
// Every test compares DISABLED, EDGE_PRIORITY and INDISCRIMINATE (plus EDGE_ONLY where it matters)
// and asserts both the PROTECTED/ERROR stencil flags and the exact corrected pixel values.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "msdfgen.h"
#include "core/MSDFErrorCorrection.h"

namespace {

using namespace msdfgen;

// ---------------------------------------------------------------------------------------------
// Minimal assertion harness
// ---------------------------------------------------------------------------------------------

int g_checks = 0;
int g_failures = 0;
std::string g_scenario;

std::string fmt(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.6f", value);
    return buffer;
}

std::string fmtPixel(int x, int y) {
    return "(" + fmt(x) + "," + fmt(y) + ")";
}

void reportFailure(const char *file, int line, const std::string &message) {
    std::printf("FAIL [%s] %s:%d: %s\n", g_scenario.c_str(), file, line, message.c_str());
    ++g_failures;
}

void checkImpl(bool condition, const char *file, int line, const std::string &message) {
    ++g_checks;
    if (!condition)
        reportFailure(file, line, message);
}

void checkNearImpl(double actual, double expected, double tolerance, const char *file, int line, const std::string &label) {
    ++g_checks;
    if (!(std::fabs(actual-expected) <= tolerance))
        reportFailure(file, line, label + ": expected " + fmt(expected) + " +/- " + fmt(tolerance) + ", got " + fmt(actual));
}

#define CHECK_TRUE(condition) checkImpl((condition), __FILE__, __LINE__, std::string("check failed: ") + (#condition))
#define CHECK_MSG(condition, message) checkImpl((condition), __FILE__, __LINE__, (message))
#define CHECK_NEAR(actual, expected, tolerance, label) checkNearImpl((actual), (expected), (tolerance), __FILE__, __LINE__, (label))

// ---------------------------------------------------------------------------------------------
// Fixtures and helpers
// ---------------------------------------------------------------------------------------------

const double GOLDEN_TOLERANCE = 1e-5;
const double EXACT_TOLERANCE = 1e-6;

// A small texture of three-channel texels, used to hand-craft defective MSDF input.
struct Texture {
    int width, height;
    std::vector<float> values; // 3 channels per pixel, row-major

    Texture(int w, int h, float background) : width(w), height(h), values(std::size_t(3*w*h), background) { }

    float *pixel(int x, int y) { return &values[3*(std::size_t(y)*width+x)]; }
    const float *pixel(int x, int y) const { return &values[3*(std::size_t(y)*width+x)]; }

    void set(int x, int y, float r, float g, float b) {
        float *p = pixel(x, y);
        p[0] = r, p[1] = g, p[2] = b;
    }
};

Bitmap<float, 3> toBitmap(const Texture &texture) {
    Bitmap<float, 3> bitmap(texture.width, texture.height);
    for (int y = 0; y < texture.height; ++y)
        for (int x = 0; x < texture.width; ++x) {
            const float *src = texture.pixel(x, y);
            float *dst = bitmap(x, y);
            dst[0] = src[0], dst[1] = src[1], dst[2] = src[2];
        }
    return bitmap;
}

// Square with four differently colored edges, so that all four corners are real corners.
Shape makeSquare(double x0, double y0, double x1, double y1) {
    Shape shape;
    Contour &contour = shape.addContour();
    contour.addEdge(EdgeHolder(Point2(x0, y0), Point2(x1, y0), RED));
    contour.addEdge(EdgeHolder(Point2(x1, y0), Point2(x1, y1), GREEN));
    contour.addEdge(EdgeHolder(Point2(x1, y1), Point2(x0, y1), BLUE));
    contour.addEdge(EdgeHolder(Point2(x0, y1), Point2(x0, y0), MAGENTA));
    return shape;
}

// Asymmetric arrow used for the full generation regression test.
Shape makeArrow() {
    Shape shape;
    Contour &contour = shape.addContour();
    contour.addEdge(EdgeHolder(Point2(1, 3), Point2(4, 1)));
    contour.addEdge(EdgeHolder(Point2(4, 1), Point2(4, 2.5)));
    contour.addEdge(EdgeHolder(Point2(4, 2.5), Point2(6, 2.5)));
    contour.addEdge(EdgeHolder(Point2(6, 2.5), Point2(6, 4.5)));
    contour.addEdge(EdgeHolder(Point2(6, 4.5), Point2(4, 4.5)));
    contour.addEdge(EdgeHolder(Point2(4, 4.5), Point2(4, 6)));
    contour.addEdge(EdgeHolder(Point2(4, 6), Point2(1, 3)));
    shape.normalize();
    edgeColoringSimple(shape, 3.0);
    return shape;
}

// Identity projection with a 16 unit distance range, so that texture values are in [0, 1] and
// the expected per-texel deviation span is a known fraction of a pixel unit.
SDFTransformation textureTransformation() {
    return SDFTransformation(Projection(), DistanceMapping(Range(16.0)));
}

SDFTransformation arrowTransformation() {
    return SDFTransformation(Projection(Vector2(0.75), Vector2()), DistanceMapping(Range(4.0)));
}

float median3(float a, float b, float c) {
    return msdfgen::median(a, b, c);
}

Bitmap<float, 3> applyCorrection(const Bitmap<float, 3> &input, const Shape &shape, ErrorCorrectionConfig::Mode mode, ErrorCorrectionConfig::DistanceCheckMode distanceMode = ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE) {
    Bitmap<float, 3> result(input);
    MSDFGeneratorConfig config(true, ErrorCorrectionConfig(mode, distanceMode));
    BitmapSection<float, 3> section(result);
    msdfErrorCorrection(section, shape, textureTransformation(), config);
    return result;
}

enum ProtectionMode {
    PROTECT_NONE = 0,
    PROTECT_CORNERS = 1,
    PROTECT_EDGES = 2,
    PROTECT_ALL = 4
};

Bitmap<byte, 1> computeStencil(const Bitmap<float, 3> &input, const Shape *shape, int protection) {
    Bitmap<byte, 1> stencil(input.width(), input.height());
    MSDFErrorCorrection ec(stencil, textureTransformation());
    if (protection & PROTECT_CORNERS)
        ec.protectCorners(*shape);
    if (protection & PROTECT_EDGES)
        ec.protectEdges<3>(input);
    if (protection & PROTECT_ALL)
        ec.protectAll();
    ec.findErrors<3>(input);
    return stencil;
}

bool isProtected(const Bitmap<byte, 1> &stencil, int x, int y) {
    return (*stencil(x, y) & MSDFErrorCorrection::PROTECTED) != 0;
}

bool isError(const Bitmap<byte, 1> &stencil, int x, int y) {
    return (*stencil(x, y) & MSDFErrorCorrection::ERROR) != 0;
}

typedef std::pair<int, int> PixelCoord;

bool pixelOrder(const PixelCoord &a, const PixelCoord &b) {
    return a.second < b.second || (a.second == b.second && a.first < b.first);
}

std::vector<PixelCoord> changedPixels(const Bitmap<float, 3> &a, const Bitmap<float, 3> &b) {
    std::vector<PixelCoord> changed;
    for (int y = 0; y < a.height(); ++y)
        for (int x = 0; x < a.width(); ++x) {
            const float *pa = a(x, y), *pb = b(x, y);
            if (pa[0] != pb[0] || pa[1] != pb[1] || pa[2] != pb[2])
                changed.push_back(PixelCoord(x, y));
        }
    std::sort(changed.begin(), changed.end(), pixelOrder);
    return changed;
}

std::vector<PixelCoord> errorPixels(const Bitmap<byte, 1> &stencil) {
    std::vector<PixelCoord> errors;
    for (int y = 0; y < stencil.height(); ++y)
        for (int x = 0; x < stencil.width(); ++x)
            if (isError(stencil, x, y))
                errors.push_back(PixelCoord(x, y));
    std::sort(errors.begin(), errors.end(), pixelOrder);
    return errors;
}

std::vector<PixelCoord> expectedPixels(std::initializer_list<PixelCoord> list) {
    std::vector<PixelCoord> pixels(list.begin(), list.end());
    std::sort(pixels.begin(), pixels.end(), pixelOrder);
    return pixels;
}

std::string describePixels(const std::vector<PixelCoord> &pixels) {
    if (pixels.empty())
        return "<none>";
    std::string result;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        if (!result.empty())
            result += ' ';
        result += fmtPixel(pixels[i].first, pixels[i].second);
    }
    return result;
}

void expectErrorPixels(const Bitmap<byte, 1> &stencil, std::initializer_list<PixelCoord> expected, const std::string &label) {
    std::vector<PixelCoord> actual = errorPixels(stencil);
    CHECK_MSG(actual == expectedPixels(expected), label + ": flagged texels " + describePixels(actual) + ", expected " + describePixels(expectedPixels(expected)));
}

// Asserts that exactly the expected texels changed and that every corrected texel now holds its
// pre-correction median in all three channels.
void expectCorrectedPixels(const Bitmap<float, 3> &baseline, const Bitmap<float, 3> &result, std::initializer_list<PixelCoord> expectedChanges, const std::string &label) {
    std::vector<PixelCoord> actual = changedPixels(baseline, result);
    std::vector<PixelCoord> expected = expectedPixels(expectedChanges);
    CHECK_MSG(actual == expected, label + ": corrected texels " + describePixels(actual) + ", expected " + describePixels(expected));
    for (std::size_t i = 0; i < actual.size(); ++i) {
        int x = actual[i].first, y = actual[i].second;
        const float *before = baseline(x, y);
        const float *after = result(x, y);
        float expectedValue = median3(before[0], before[1], before[2]);
        CHECK_MSG(after[0] == after[1] && after[1] == after[2], label + ": corrected texel " + fmtPixel(x, y) + " not converted to a single channel");
        CHECK_NEAR(after[0], expectedValue, EXACT_TOLERANCE, label + ": corrected value at " + fmtPixel(x, y));
        CHECK_NEAR(after[1], expectedValue, EXACT_TOLERANCE, label + ": corrected value at " + fmtPixel(x, y));
        CHECK_NEAR(after[2], expectedValue, EXACT_TOLERANCE, label + ": corrected value at " + fmtPixel(x, y));
    }
}

// Asserts that the corrected result is identical to the uncorrected baseline.
void expectUnchanged(const Bitmap<float, 3> &baseline, const Bitmap<float, 3> &result, const std::string &label) {
    expectCorrectedPixels(baseline, result, {}, label);
}

// Asserts that the RGB channels of a four-channel bitmap equal the three-channel result.
void expectRgbEqual(const Bitmap<float, 3> &rgb, const Bitmap<float, 4> &rgba, const std::string &label) {
    for (int y = 0; y < rgb.height(); ++y)
        for (int x = 0; x < rgb.width(); ++x) {
            const float *a = rgb(x, y);
            const float *b = rgba(x, y);
            if (a[0] != b[0] || a[1] != b[1] || a[2] != b[2])
                CHECK_MSG(false, label + ": RGB differs at " + fmtPixel(x, y));
        }
}

// Asserts that the alpha channel is bit-for-bit unchanged.
void expectAlphaUnchanged(const Bitmap<float, 4> &before, const Bitmap<float, 4> &after, const std::string &label) {
    for (int y = 0; y < before.height(); ++y)
        for (int x = 0; x < before.width(); ++x)
            if (before(x, y)[3] != after(x, y)[3])
                CHECK_MSG(false, label + ": alpha channel rewritten at " + fmtPixel(x, y));
}

// ---------------------------------------------------------------------------------------------
// Hand-crafted fixtures
// ---------------------------------------------------------------------------------------------

// Inversion: both texel medians are below 0.5, but linear interpolation of the median rises above
// 0.5 in between (medians 0.20 and 0.48).
Texture inversionFixture() {
    Texture texture(6, 6, 0.2f);
    texture.set(2, 2, 0.20f, 0.53f, 0.08f); // median 0.20
    texture.set(3, 2, 0.90f, 0.48f, 0.08f); // median 0.48
    return texture;
}

// Ordinary deviation: medians 0.45 and 0.50, interpolated median leaves the expected range but
// does not cross the shape threshold.
Texture deviationFixture() {
    Texture texture(10, 6, 0.2f);
    texture.set(7, 3, 0.90f, 0.05f, 0.45f); // median 0.45
    texture.set(8, 3, 0.50f, 0.40f, 0.85f); // median 0.50
    return texture;
}

// Multi-channel detail right next to an edge. Correcting it indiscriminately would destroy the
// channel separation that produces the sharp edge.
Texture edgeDetailFixture() {
    Texture texture(6, 4, 0.2f);
    texture.set(2, 1, 0.00f, 0.45f, 0.45f);
    texture.set(3, 1, 0.51f, 0.00f, 0.51f);
    return texture;
}

// A deviation pair that sits inside the texels enveloping a shape corner (corner at 2.5, 2.5).
Texture cornerDeviationFixture() {
    Texture texture(8, 8, 0.2f);
    texture.set(2, 2, 0.90f, 0.05f, 0.45f); // median 0.45
    texture.set(3, 2, 0.50f, 0.40f, 0.85f); // median 0.50
    return texture;
}

// An inversion pair inside the texels enveloping a shape corner (both medians 0.53, dip below 0.5).
Texture cornerInversionFixture() {
    Texture texture(8, 8, 0.2f);
    texture.set(2, 2, 0.48f, 0.53f, 0.70f); // median 0.53
    texture.set(3, 2, 0.53f, 0.53f, 0.10f); // median 0.53
    return texture;
}

// ---------------------------------------------------------------------------------------------
// Unit tests: protection semantics
// ---------------------------------------------------------------------------------------------

// A channel-median inversion must be corrected even when every texel is protected.
void testInversionSurvivesFullProtection() {
    Texture texture = inversionFixture();
    Bitmap<float, 3> baseline = toBitmap(texture);
    Shape empty;

    Bitmap<byte, 1> stencil = computeStencil(baseline, &empty, PROTECT_ALL);
    for (int y = 0; y < baseline.height(); ++y)
        for (int x = 0; x < baseline.width(); ++x)
            CHECK_TRUE(isProtected(stencil, x, y));
    CHECK_TRUE(isError(stencil, 2, 2));
    expectErrorPixels(stencil, {{2, 2}}, "inversion/full protection");

    Bitmap<float, 3> disabled = applyCorrection(baseline, empty, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 3> edgePriority = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 3> edgeOnly = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 3> indiscriminate = applyCorrection(baseline, empty, ErrorCorrectionConfig::INDISCRIMINATE);

    expectUnchanged(baseline, disabled, "inversion/DISABLED");
    expectCorrectedPixels(baseline, edgePriority, {{2, 2}}, "inversion/EDGE_PRIORITY");
    expectCorrectedPixels(baseline, edgeOnly, {{2, 2}}, "inversion/EDGE_ONLY");
    expectCorrectedPixels(baseline, indiscriminate, {{2, 2}}, "inversion/INDISCRIMINATE");
    CHECK_NEAR(edgePriority(2, 2)[0], 0.20, EXACT_TOLERANCE, "inversion/EDGE_PRIORITY corrected value");
    CHECK_MSG(changedPixels(edgePriority, indiscriminate).empty(), "inversion: protected and indiscriminate correction must agree");
}

// An ordinary deviation must be suppressed when the texel is protected, but corrected otherwise.
void testDeviationSuppressedByProtection() {
    Texture texture = deviationFixture();
    Bitmap<float, 3> baseline = toBitmap(texture);
    Shape empty;

    Bitmap<byte, 1> unprotected = computeStencil(baseline, &empty, PROTECT_NONE);
    Bitmap<byte, 1> protectedStencil = computeStencil(baseline, &empty, PROTECT_ALL);
    expectErrorPixels(unprotected, {{7, 3}}, "deviation/unprotected");
    expectErrorPixels(protectedStencil, {}, "deviation/full protection");

    Bitmap<float, 3> disabled = applyCorrection(baseline, empty, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 3> edgePriority = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 3> edgeOnly = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 3> indiscriminate = applyCorrection(baseline, empty, ErrorCorrectionConfig::INDISCRIMINATE);

    expectUnchanged(baseline, disabled, "deviation/DISABLED");
    expectCorrectedPixels(baseline, edgePriority, {{7, 3}}, "deviation/EDGE_PRIORITY");
    expectUnchanged(baseline, edgeOnly, "deviation/EDGE_ONLY");
    expectCorrectedPixels(baseline, indiscriminate, {{7, 3}}, "deviation/INDISCRIMINATE");
    CHECK_NEAR(edgePriority(7, 3)[0], 0.45, EXACT_TOLERANCE, "deviation/EDGE_PRIORITY corrected value");
}

// Multi-channel detail near an edge must be kept intact by edge protection, while indiscriminate
// correction flattens it.
void testEdgeDetailPreservedByEdgeProtection() {
    Texture texture = edgeDetailFixture();
    Bitmap<float, 3> baseline = toBitmap(texture);
    Shape empty;

    Bitmap<byte, 1> edgeStencil = computeStencil(baseline, &empty, PROTECT_EDGES);
    Bitmap<byte, 1> allStencil = computeStencil(baseline, &empty, PROTECT_ALL);
    CHECK_TRUE(isProtected(edgeStencil, 2, 1));
    CHECK_MSG(!isError(edgeStencil, 2, 1), "edge detail: protected texel (2,1) must not be flagged");
    expectErrorPixels(allStencil, {}, "edge detail/full protection");

    Bitmap<float, 3> disabled = applyCorrection(baseline, empty, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 3> edgePriority = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 3> edgeOnly = applyCorrection(baseline, empty, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 3> indiscriminate = applyCorrection(baseline, empty, ErrorCorrectionConfig::INDISCRIMINATE);

    expectUnchanged(baseline, disabled, "edge detail/DISABLED");
    expectUnchanged(baseline, edgePriority, "edge detail/EDGE_PRIORITY");
    expectUnchanged(baseline, edgeOnly, "edge detail/EDGE_ONLY");
    expectCorrectedPixels(baseline, indiscriminate, {{2, 1}}, "edge detail/INDISCRIMINATE");

    // The protected detail must survive unchanged, the indiscriminate pass must flatten it.
    CHECK_MSG(edgePriority(2, 1)[0] == 0.0f && edgePriority(2, 1)[1] == 0.45f && edgePriority(2, 1)[2] == 0.45f, "edge detail/EDGE_PRIORITY altered the edge texel");
    CHECK_MSG(indiscriminate(2, 1)[0] == indiscriminate(2, 1)[1] && indiscriminate(2, 1)[1] == indiscriminate(2, 1)[2], "edge detail/INDISCRIMINATE did not flatten the texel");
    CHECK_MSG(indiscriminate(2, 1)[0] != edgePriority(2, 1)[0], "edge detail: edge protection and indiscriminate correction must differ");
}

// Corner protection suppresses an ordinary deviation at the corner.
void testCornerProtectionSuppressesDeviation() {
    Texture texture = cornerDeviationFixture();
    Bitmap<float, 3> baseline = toBitmap(texture);
    Shape shape = makeSquare(2.5, 2.5, 5.5, 5.5);

    Bitmap<byte, 1> unprotected = computeStencil(baseline, &shape, PROTECT_NONE);
    Bitmap<byte, 1> cornerStencil = computeStencil(baseline, &shape, PROTECT_CORNERS);
    expectErrorPixels(unprotected, {{2, 2}}, "corner deviation/unprotected");
    CHECK_TRUE(isProtected(cornerStencil, 2, 2));
    CHECK_MSG(!isError(cornerStencil, 2, 2), "corner deviation: protected corner texel (2,2) must not be flagged");

    Bitmap<float, 3> disabled = applyCorrection(baseline, shape, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 3> edgePriority = applyCorrection(baseline, shape, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 3> edgeOnly = applyCorrection(baseline, shape, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 3> indiscriminate = applyCorrection(baseline, shape, ErrorCorrectionConfig::INDISCRIMINATE);

    expectUnchanged(baseline, disabled, "corner deviation/DISABLED");
    expectUnchanged(baseline, edgePriority, "corner deviation/EDGE_PRIORITY");
    expectUnchanged(baseline, edgeOnly, "corner deviation/EDGE_ONLY");
    expectCorrectedPixels(baseline, indiscriminate, {{2, 2}}, "corner deviation/INDISCRIMINATE");
}

// The very same corner protection does NOT suppress an inversion artifact at the corner.
void testCornerProtectionDoesNotSuppressInversion() {
    Texture texture = cornerInversionFixture();
    Bitmap<float, 3> baseline = toBitmap(texture);
    Shape shape = makeSquare(2.5, 2.5, 5.5, 5.5);

    Bitmap<byte, 1> cornerStencil = computeStencil(baseline, &shape, PROTECT_CORNERS);
    CHECK_TRUE(isProtected(cornerStencil, 2, 2));
    CHECK_TRUE(isError(cornerStencil, 2, 2));
    CHECK_TRUE(isError(cornerStencil, 3, 2));
    expectErrorPixels(cornerStencil, {{2, 2}, {3, 2}}, "corner inversion/corner protection");

    Bitmap<float, 3> disabled = applyCorrection(baseline, shape, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 3> edgePriority = applyCorrection(baseline, shape, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 3> edgeOnly = applyCorrection(baseline, shape, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 3> indiscriminate = applyCorrection(baseline, shape, ErrorCorrectionConfig::INDISCRIMINATE);

    expectUnchanged(baseline, disabled, "corner inversion/DISABLED");
    expectCorrectedPixels(baseline, edgePriority, {{2, 2}, {3, 2}}, "corner inversion/EDGE_PRIORITY");
    expectCorrectedPixels(baseline, edgeOnly, {{2, 2}, {3, 2}}, "corner inversion/EDGE_ONLY");
    expectCorrectedPixels(baseline, indiscriminate, {{2, 2}, {3, 2}}, "corner inversion/INDISCRIMINATE");
    CHECK_NEAR(edgePriority(2, 2)[0], 0.53, EXACT_TOLERANCE, "corner inversion/EDGE_PRIORITY corrected value");
}

// ---------------------------------------------------------------------------------------------
// Unit test: the alpha channel of an MTSDF must never be rewritten
// ---------------------------------------------------------------------------------------------

Bitmap<float, 4> makeMtsdfInput(const Texture &texture) {
    Bitmap<float, 4> bitmap(texture.width, texture.height);
    for (int y = 0; y < texture.height; ++y)
        for (int x = 0; x < texture.width; ++x) {
            const float *src = texture.pixel(x, y);
            float *dst = bitmap(x, y);
            dst[0] = src[0], dst[1] = src[1], dst[2] = src[2];
            // A per-texel true-distance value, distinct from every RGB channel.
            dst[3] = 0.05f+0.031f*float(x)+0.017f*float(y);
        }
    return bitmap;
}

void runMtsdfCase(const char *name, const Texture &texture, const Shape &shape) {
    Bitmap<float, 4> baseline = makeMtsdfInput(texture);
    Bitmap<float, 3> rgbBaseline = toBitmap(texture);

    const ErrorCorrectionConfig::Mode modes[] = {
        ErrorCorrectionConfig::EDGE_PRIORITY,
        ErrorCorrectionConfig::EDGE_ONLY,
        ErrorCorrectionConfig::INDISCRIMINATE
    };
    const char *modeNames[] = { "EDGE_PRIORITY", "EDGE_ONLY", "INDISCRIMINATE" };

    for (int mi = 0; mi < 3; ++mi) {
        std::string label = std::string("MTSDF/") + name + "/" + modeNames[mi];

        Bitmap<float, 4> fourChannel(baseline);
        MSDFGeneratorConfig config(true, ErrorCorrectionConfig(modes[mi], ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE));
        BitmapSection<float, 4> section(fourChannel);
        msdfErrorCorrection(section, shape, textureTransformation(), config);

        expectAlphaUnchanged(baseline, fourChannel, label + "/alpha");

        Bitmap<float, 3> threeChannel = applyCorrection(rgbBaseline, shape, modes[mi]);
        expectRgbEqual(threeChannel, fourChannel, label + "/rgb");
    }
}

void testMtsdfAlphaChannelUntouched() {
    runMtsdfCase("inversion", inversionFixture(), Shape());
    runMtsdfCase("deviation", deviationFixture(), Shape());
    runMtsdfCase("edge detail", edgeDetailFixture(), Shape());
    runMtsdfCase("corner inversion", cornerInversionFixture(), makeSquare(2.5, 2.5, 5.5, 5.5));

    // Low-level apply<4> must confine itself to the three color channels as well.
    Texture texture = inversionFixture();
    Bitmap<float, 4> bitmap = makeMtsdfInput(texture);
    Bitmap<float, 4> before(bitmap);
    Bitmap<byte, 1> stencil(bitmap.width(), bitmap.height());
    MSDFErrorCorrection ec(stencil, textureTransformation());
    ec.protectAll();
    ec.findErrors<4>(bitmap);
    ec.apply<4>(bitmap);
    expectAlphaUnchanged(before, bitmap, "MTSDF/apply<4>/alpha");
    float expectedValue = median3(before(2, 2)[0], before(2, 2)[1], before(2, 2)[2]);
    CHECK_NEAR(bitmap(2, 2)[0], expectedValue, EXACT_TOLERANCE, "MTSDF/apply<4> corrected red");
    CHECK_NEAR(bitmap(2, 2)[1], expectedValue, EXACT_TOLERANCE, "MTSDF/apply<4> corrected green");
    CHECK_NEAR(bitmap(2, 2)[2], expectedValue, EXACT_TOLERANCE, "MTSDF/apply<4> corrected blue");
}

// ---------------------------------------------------------------------------------------------
// Regression tests: full generation path on a small texture
// ---------------------------------------------------------------------------------------------

const int ARROW_SIZE = 8;

// Golden output of generateMSDF on the arrow with error correction DISABLED. These are the exact
// expected values of the generated field before any correction.
const float kArrowBaseline[ARROW_SIZE][ARROW_SIZE][3] = {
    { {1.031588f, 0.146447f, 1.031588f}, {0.846688f, 0.000000f, 0.846688f}, {0.661788f, 0.333333f, 0.661788f}, {0.666667f, 0.666667f, 0.476887f}, {0.500000f, 0.958333f, 0.958333f}, {0.833333f, 0.958333f, 0.958333f}, {1.166667f, 0.958333f, 1.166667f}, {1.500000f, 0.958333f, 1.500000f} },
    { {0.754238f, 0.382149f, 0.754238f}, {0.569338f, 0.146447f, 0.569338f}, {0.384437f, 0.333333f, 0.384437f}, {0.666667f, 0.625000f, 0.625000f}, {0.500000f, 0.625000f, 0.625000f}, {0.833333f, 0.625000f, 0.833333f}, {1.166667f, 0.625000f, 1.166667f}, {1.500000f, 0.625000f, 1.500000f} },
    { {0.617851f, 0.617851f, 0.476887f}, {0.382149f, 0.382149f, 0.291987f}, {0.333333f, 0.291667f, 0.291667f}, {0.666667f, 0.291667f, 0.291667f}, {0.500000f, 0.291667f, 0.500000f}, {0.833333f, 0.291667f, 0.833333f}, {1.166667f, 0.291667f, 1.166667f}, {1.500000f, 0.291667f, 1.500000f} },
    { {0.853553f, 0.853553f, 0.199537f}, {0.617851f, 0.617851f, 0.014637f}, {0.382149f, 0.382149f, 0.333333f}, {0.541667f, 0.541667f, 0.666667f}, {0.541667f, 0.541667f, 0.500000f}, {0.833333f, 0.541667f, 0.833333f}, {1.166667f, 0.541667f, 1.166667f}, {1.500000f, 0.541667f, 1.500000f} },
    { {1.089256f, 1.089256f, -0.077813f}, {0.853553f, 0.853553f, 0.000000f}, {0.617851f, 0.617851f, 0.333333f}, {0.382149f, 0.666667f, 0.666667f}, {0.875000f, 0.875000f, 0.500000f}, {0.875000f, 0.875000f, 0.833333f}, {1.166667f, 0.875000f, 1.166667f}, {1.500000f, 0.875000f, 1.500000f} },
    { {1.324958f, 1.324958f, -0.333333f}, {1.089256f, 1.089256f, 0.000000f}, {0.853553f, 0.853553f, 0.333333f}, {0.617851f, 0.666667f, 0.666667f}, {0.382149f, 1.000000f, 1.000000f}, {1.208333f, 1.208333f, 0.833333f}, {1.208333f, 1.208333f, 1.166667f}, {1.500000f, 1.208333f, 1.500000f} },
    { {1.560660f, 1.560660f, -0.333333f}, {1.324958f, 1.324958f, 0.000000f}, {1.089256f, 1.089256f, 0.333333f}, {0.853553f, 0.853553f, 0.666667f}, {0.617851f, 1.000000f, 1.000000f}, {0.382149f, 1.333333f, 1.333333f}, {1.541667f, 1.541667f, 1.166667f}, {1.541667f, 1.541667f, 1.500000f} },
    { {1.796362f, 1.796362f, -0.333333f}, {1.560660f, 1.560660f, 0.000000f}, {1.324958f, 1.324958f, 0.333333f}, {1.089256f, 1.089256f, 0.666667f}, {0.853553f, 1.000000f, 1.000000f}, {0.617851f, 1.333333f, 1.333333f}, {1.875000f, 1.666667f, 1.166667f}, {1.875000f, 1.875000f, 1.500000f} }
};

Bitmap<float, 3> generateArrowMsdf(const Shape &shape, ErrorCorrectionConfig::Mode mode, ErrorCorrectionConfig::DistanceCheckMode distanceMode) {
    Bitmap<float, 3> bitmap(ARROW_SIZE, ARROW_SIZE);
    MSDFGeneratorConfig config(true, ErrorCorrectionConfig(mode, distanceMode));
    BitmapSection<float, 3> section(bitmap);
    generateMSDF(section, shape, arrowTransformation(), config);
    return bitmap;
}

Bitmap<float, 4> generateArrowMtsdf(const Shape &shape, ErrorCorrectionConfig::Mode mode) {
    Bitmap<float, 4> bitmap(ARROW_SIZE, ARROW_SIZE);
    MSDFGeneratorConfig config(true, ErrorCorrectionConfig(mode, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE));
    BitmapSection<float, 4> section(bitmap);
    generateMTSDF(section, shape, arrowTransformation(), config);
    return bitmap;
}

Bitmap<float, 1> generateArrowSdf(const Shape &shape) {
    Bitmap<float, 1> bitmap(ARROW_SIZE, ARROW_SIZE);
    BitmapSection<float, 1> section(bitmap);
    generateSDF(section, shape, arrowTransformation());
    return bitmap;
}

void testGeneratedArrowMsdfRegression() {
    Shape shape = makeArrow();

    Bitmap<float, 3> baseline = generateArrowMsdf(shape, ErrorCorrectionConfig::DISABLED, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    for (int y = 0; y < ARROW_SIZE; ++y)
        for (int x = 0; x < ARROW_SIZE; ++x)
            for (int c = 0; c < 3; ++c) {
                char label[64];
                std::snprintf(label, sizeof(label), "arrow baseline (%d,%d) channel %d", x, y, c);
                CHECK_NEAR(baseline(x, y)[c], kArrowBaseline[y][x][c], GOLDEN_TOLERANCE, label);
            }

    Bitmap<float, 3> edgePriority = generateArrowMsdf(shape, ErrorCorrectionConfig::EDGE_PRIORITY, ErrorCorrectionConfig::CHECK_DISTANCE_AT_EDGE);
    Bitmap<float, 3> edgeOnly = generateArrowMsdf(shape, ErrorCorrectionConfig::EDGE_ONLY, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    Bitmap<float, 3> indiscriminate = generateArrowMsdf(shape, ErrorCorrectionConfig::INDISCRIMINATE, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);

    // Edge priority keeps the edge-adjacent texel (3,2) intact; the other artifacts are corrected.
    expectCorrectedPixels(baseline, edgePriority, {{4, 0}, {4, 4}, {4, 5}, {5, 5}, {5, 6}, {6, 6}, {6, 7}}, "arrow/EDGE_PRIORITY");

    // Indiscriminate correction additionally flattens the edge detail at (3,2).
    expectCorrectedPixels(baseline, indiscriminate, {{3, 2}, {4, 0}, {4, 4}, {4, 5}, {5, 5}, {5, 6}, {6, 6}, {6, 7}}, "arrow/INDISCRIMINATE");

    // EDGE_ONLY protects all texels and only fixes inversions; this fixture has none, so nothing changes.
    expectUnchanged(baseline, edgeOnly, "arrow/EDGE_ONLY");

    // (3,2) is the regression-level demonstration of the distinction: preserved by edge priority,
    // flattened by indiscriminate correction.
    const float *edgeDetail = baseline(3, 2);
    CHECK_MSG(edgePriority(3, 2)[0] == edgeDetail[0] && edgePriority(3, 2)[1] == edgeDetail[1] && edgePriority(3, 2)[2] == edgeDetail[2], "arrow/EDGE_PRIORITY modified the edge detail texel (3,2)");
    float edgeDetailMedian = median3(edgeDetail[0], edgeDetail[1], edgeDetail[2]);
    CHECK_NEAR(edgeDetailMedian, 0.291667, GOLDEN_TOLERANCE, "arrow baseline median at (3,2)");
    CHECK_NEAR(indiscriminate(3, 2)[0], edgeDetailMedian, EXACT_TOLERANCE, "arrow/INDISCRIMINATE corrected value at (3,2)");
    CHECK_MSG(edgePriority(3, 2)[0] != indiscriminate(3, 2)[0], "arrow: edge priority and indiscriminate correction must differ at (3,2)");
}

void testGeneratedArrowMtsdfAlpha() {
    Shape shape = makeArrow();

    Bitmap<float, 1> sdf = generateArrowSdf(shape);
    Bitmap<float, 4> disabled = generateArrowMtsdf(shape, ErrorCorrectionConfig::DISABLED);
    Bitmap<float, 4> edgePriority = generateArrowMtsdf(shape, ErrorCorrectionConfig::EDGE_PRIORITY);
    Bitmap<float, 4> edgeOnly = generateArrowMtsdf(shape, ErrorCorrectionConfig::EDGE_ONLY);
    Bitmap<float, 4> indiscriminate = generateArrowMtsdf(shape, ErrorCorrectionConfig::INDISCRIMINATE);

    Bitmap<float, 4> *corrected[] = { &edgePriority, &edgeOnly, &indiscriminate };
    const char *correctedNames[] = { "EDGE_PRIORITY", "EDGE_ONLY", "INDISCRIMINATE" };

    for (int y = 0; y < ARROW_SIZE; ++y)
        for (int x = 0; x < ARROW_SIZE; ++x) {
            std::string label = "arrow MTSDF alpha " + fmtPixel(x, y);
            CHECK_NEAR(disabled(x, y)[3], sdf(x, y)[0], EXACT_TOLERANCE, label + " vs generateSDF");
            for (int mi = 0; mi < 3; ++mi) {
                CHECK_MSG(disabled(x, y)[3] == (*corrected[mi])(x, y)[3], label + " rewritten by " + correctedNames[mi]);
                CHECK_NEAR((*corrected[mi])(x, y)[3], sdf(x, y)[0], EXACT_TOLERANCE, label + " vs generateSDF under " + correctedNames[mi]);
            }
        }

    // The RGB channels of the MTSDF must match the MSDF for the same mode.
    Bitmap<float, 3> msdfDisabled = generateArrowMsdf(shape, ErrorCorrectionConfig::DISABLED, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    Bitmap<float, 3> msdfEdgePriority = generateArrowMsdf(shape, ErrorCorrectionConfig::EDGE_PRIORITY, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    Bitmap<float, 3> msdfEdgeOnly = generateArrowMsdf(shape, ErrorCorrectionConfig::EDGE_ONLY, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    Bitmap<float, 3> msdfIndiscriminate = generateArrowMsdf(shape, ErrorCorrectionConfig::INDISCRIMINATE, ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
    Bitmap<float, 3> *msdfByMode[] = { &msdfDisabled, &msdfEdgePriority, &msdfEdgeOnly, &msdfIndiscriminate };
    Bitmap<float, 4> *mtsdfByMode[] = { &disabled, &edgePriority, &edgeOnly, &indiscriminate };
    const char *modeNames[] = { "DISABLED", "EDGE_PRIORITY", "EDGE_ONLY", "INDISCRIMINATE" };

    for (int mi = 0; mi < 4; ++mi)
        expectRgbEqual(*msdfByMode[mi], *mtsdfByMode[mi], std::string("arrow/") + modeNames[mi]);

    // The alpha channel must also be untouched at texels whose RGB was actually corrected.
    CHECK_MSG(edgePriority(4, 5)[0] != msdfDisabled(4, 5)[0], "arrow: expected RGB correction at (4,5) under EDGE_PRIORITY");
    CHECK_NEAR(edgePriority(4, 5)[3], sdf(4, 5)[0], EXACT_TOLERANCE, "arrow MTSDF alpha at a corrected texel");
}

// The choice of distance check must not change the result for this fixture.
void testDistanceCheckModeInvariance() {
    Shape shape = makeArrow();
    const ErrorCorrectionConfig::Mode modes[] = {
        ErrorCorrectionConfig::EDGE_PRIORITY,
        ErrorCorrectionConfig::EDGE_ONLY,
        ErrorCorrectionConfig::INDISCRIMINATE
    };
    const char *modeNames[] = { "EDGE_PRIORITY", "EDGE_ONLY", "INDISCRIMINATE" };

    for (int mi = 0; mi < 3; ++mi) {
        Bitmap<float, 3> unchecked = generateArrowMsdf(shape, modes[mi], ErrorCorrectionConfig::DO_NOT_CHECK_DISTANCE);
        Bitmap<float, 3> checked = generateArrowMsdf(shape, modes[mi], ErrorCorrectionConfig::CHECK_DISTANCE_AT_EDGE);
        std::vector<PixelCoord> differences = changedPixels(unchecked, checked);
        CHECK_MSG(differences.empty(), std::string("arrow/") + modeNames[mi] + ": distance check changed texels " + describePixels(differences));
    }
}

// ---------------------------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------------------------

struct TestCase {
    const char *name;
    void (*function)();
};

std::vector<TestCase> &registry() {
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar {
    Registrar(const char *name, void (*function)()) {
        TestCase testCase = { name, function };
        registry().push_back(testCase);
    }
};

#define TEST_CASE(name) \
    void name(); \
    Registrar registrar_##name(#name, name); \
    void name()

TEST_CASE(inversion_survives_full_protection) { testInversionSurvivesFullProtection(); }
TEST_CASE(deviation_suppressed_by_protection) { testDeviationSuppressedByProtection(); }
TEST_CASE(edge_detail_preserved_by_edge_protection) { testEdgeDetailPreservedByEdgeProtection(); }
TEST_CASE(corner_protection_suppresses_deviation) { testCornerProtectionSuppressesDeviation(); }
TEST_CASE(corner_protection_does_not_suppress_inversion) { testCornerProtectionDoesNotSuppressInversion(); }
TEST_CASE(mtsdf_alpha_channel_untouched) { testMtsdfAlphaChannelUntouched(); }
TEST_CASE(generated_arrow_msdf_regression) { testGeneratedArrowMsdfRegression(); }
TEST_CASE(generated_arrow_mtsdf_alpha) { testGeneratedArrowMtsdfAlpha(); }
TEST_CASE(distance_check_mode_invariance) { testDistanceCheckModeInvariance(); }

}

int main() {
    std::vector<TestCase> &cases = registry();
    for (std::size_t i = 0; i < cases.size(); ++i) {
        g_scenario = cases[i].name;
        int failuresBefore = g_failures;
        cases[i].function();
        std::printf("%s %s\n", g_failures == failuresBefore ? "PASS" : "FAIL", cases[i].name);
    }
    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
