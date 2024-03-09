#pragma once

#include <boost/qvm/all.hpp>
#include <cstdint>
#include <random>

namespace qvm = boost::qvm;

/**
 * @brief Convenience struct for a 2D floating point vector
 * All functionality is inherited from qvm::vec
 */
struct Vec2DFloat : public qvm::vec<float, 2>
{
    // Explicitly use the member variable 'a' in the base qvm::vec class
    using qvm::vec<float, 2>::a;

    /**
     * @brief Instantiate an empty Vec2DFloat
     *
     */
    Vec2DFloat() {}

    /**
     * @brief Instantiate a Vec2DFloat with initial component values
     *
     * @param i
     * @param j
     */
    Vec2DFloat(float i, float j) : qvm::vec<float, 2>{i, j} {}

    /**
     * @brief Convenience array accessor for a
     *
     * @param idx
     * @return float&
     */
    float & operator[](std::size_t idx) { return a[idx]; }

    /**
     * @brief Convenience array accessor for a
     *
     * @param idx
     * @return const float&
     */
    const float & operator[](std::size_t idx) const { return a[idx]; }
};

// NOLINTBEGIN
// This boost::qvm:: namespace segment is just boilerplate to register the Vec2DFloat type with qvm's vector operations.
// See https://www.boost.org/doc/libs/1_74_0/libs/qvm/doc/html/index.html#vec_traits for more info.
// Even though its boilerplate, it triggers linter errors so disable linting for this section.
namespace boost
{
namespace qvm
{
template <>
struct vec_traits<Vec2DFloat>
{
    static int const dim = 2;

    using scalar_type = float;

    template <int I>
    static inline scalar_type & write_element(Vec2DFloat & v)
    {
        return v.a[I];
    }

    template <int I>
    static inline scalar_type read_element(Vec2DFloat const & v)
    {
        return v.a[I];
    }
};
}  // namespace qvm
}  // namespace boost
// NOLINTEND
namespace defaults
{
constexpr float MAX_HEADING_CHANGE   = 2.0;    // Max degree change per tick
constexpr float MAX_SPEED_CHANGE     = 1.0;    // Min degree change per tick
constexpr float MIN_AIS_SHIP_DIST    = 0.001;  // Min 111m (at equator) distance of ais ships from Polaris
constexpr float MAX_AIS_SHIP_DIST    = 0.1;    // Max 11.1km (at equator) distance of ais ships from Polaris
constexpr int   MIN_AIS_SHIP_WIDTH_M = 2;      // A boat this small likely won't have AIS
constexpr int   MAX_AIS_SHIP_WIDTH_M = 49;     // Typical container ship width
// Minimum and maximum ratios pulled from: http://marine.marsh-design.com/content/length-beam-ratio
constexpr int    MIN_AIS_SHIP_L_W_RATIO = 2;                                // Ship length should be at least 2x width
constexpr int    MAX_AIS_SHIP_L_W_RATIO = 16;                               // Ship length should be at most 16x width
constexpr int    UPDATE_RATE_MS         = 500;                              // Update frequency
constexpr int    SEED                   = 123456;                           // Randomization seed
constexpr int    NUM_SIM_SHIPS          = 20;                               // Number of ais ships to simulate
const Vec2DFloat POLARIS_START_POS{49.28397458822112, -123.6525841364974};  // some point in the Strait of Georgia;
}  // namespace defaults

/**
 * @brief Struct that mirrors the definition of custom_interfaces::msg::HelperAISShip
 *
 */
struct AisShip
{
    Vec2DFloat lat_lon_;
    float      speed_;
    float      heading_;
    uint32_t   id_;
    uint32_t   width_;
    uint32_t   length_;
    int8_t     rot_;
};

/**
 * @brief Extra per ship simulation parameters
 */
struct SimShipConfig
{
    float    max_heading_change_;  // Max degree change per tick
    float    max_speed_change_;    // Min degree change per tick
    float    max_ship_dist_;       // Maximum distance from Polaris
    float    min_ship_dist_;       // Minimum distance from Polaris
    uint32_t min_ship_width_m_;    // Minimum ship width in meters
    uint32_t max_ship_width_m_;    // Maximum ship width in meters
    uint32_t min_ship_l_w_ratio_;  // Minimum ship length:width ratio
    uint32_t max_ship_l_w_ratio_;  // Maximum ship length:width ratio
};

class MockAisShip : public AisShip
{
public:
    /**
     * @brief Construct a new Mock Ais Ship object
     *
     * @param seed            Random seed
     * @param id              ID of the new sim ship
     * @param polaris_lat_lon Position of Poalris
     * @param config          Sim ship constraints
     */
    MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon, SimShipConfig config);

    /**
     * @brief Update the AIS Ship instance
     *
     * @param polaris_lat_lon Current position of Polaris
     */
    void tick(const Vec2DFloat & polaris_lat_lon);

private:
    std::mt19937  mt_rng_;  // Random number generator
    SimShipConfig config_;  // Sim ship constraints
};

/**
 * Simulate Ais Ships
 */
class MockAis
{
public:
    /**
     * @brief Construct a new Mock AIS simulation
     *
     * @param seed            Random seeds
     * @param num_ships       Number of sim ships to spawn
     * @param polaris_lat_lon Position of Polaris
     */
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon);

    /**
     * @brief Construct a new Mock AIS simulation
     *
     * @param seed            Random seeds
     * @param num_ships       Number of sim ships to spawn
     * @param polaris_lat_lon Position of Polaris
     * @param config          Extra sim ship constraints
     */
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon, SimShipConfig config);

    /**
     * @brief Get the current AIS ships
     *
     * @return A vector of AisShip objects
     */
    std::vector<AisShip> ships() const;

    /**
     * @brief Update the current position of Polaris
     *
     * @param lat_lon Polaris' position
     */
    void updatePolarisPos(const Vec2DFloat & lat_lon);

    /**
     * @brief Update every simulated AIS ship
     *
     */
    void tick();

private:
    Vec2DFloat               polaris_lat_lon_;  // Polaris' current position
    std::vector<MockAisShip> ships_;            // Vector of all simulated Ais ships
};
