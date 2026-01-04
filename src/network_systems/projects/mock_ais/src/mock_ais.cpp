#include "mock_ais.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <random>

#include "cmn_hdrs/shared_constants.h"

namespace
{
/**
 * @brief Convert degress to radians
 *
 * @param degrees
 * @return radians
 */
float degToRad(const float & degrees)
{
    return static_cast<float>(degrees * M_PI / 180.0);  // NOLINT(readability-magic-numbers)
}

/**
 * @brief Bound a heading to our current limits
 *
 * @param heading
 * @return bounded heading
 */
float boundHeading(const float & heading)
{
    if (heading < HEADING_LBND) {
        return heading + HEADING_UBND;
    }
    if (heading >= HEADING_UBND) {
        return heading - HEADING_UBND;
    }
    return heading;
}

/**
 * @brief Convert a heading to a 2D direction unit vector
 *
 * @param heading
 * @return unit direction vector
 */
Vec2DFloat headingToVec2D(const float & heading)
{
    float angle = degToRad(heading);
    // Since 0 is north (y-axis), use sin for x and cos for y
    return {std::sin(angle), std::cos(angle)};
}
}  // namespace

MockAisShip::MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon, SimShipConfig config)
: mt_rng_(seed), config_(config)
{
    static const std::array<float, 2>       pos_or_neg = {-1.0, 1.0};
    std::uniform_real_distribution<float>   lat_dist(config_.min_ship_dist_, config_.max_ship_dist_);
    std::uniform_real_distribution<float>   lon_dist(config_.min_ship_dist_, config_.max_ship_dist_);
    std::uniform_real_distribution<float>   speed_dist(SOG_SPEED_LBND, SOG_SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_int_distribution<uint32_t> pos_or_neg_dist(0, 1);
    std::uniform_int_distribution<uint32_t> beam_dist(config_.min_ship_width_m_, config_.max_ship_width_m_);
    std::uniform_int_distribution<uint32_t> l_w_ratio_dist(config_.min_ship_l_w_ratio_, config_.max_ship_l_w_ratio_);
    std::uniform_int_distribution<int8_t>   rot_dist(ROT_LBND, ROT_UBND);

    id_      = id;
    lat_lon_ = {
      polaris_lat_lon[0] + pos_or_neg[pos_or_neg_dist(mt_rng_)] * lat_dist(mt_rng_),
      polaris_lat_lon[1] + pos_or_neg[pos_or_neg_dist(mt_rng_)] * lon_dist(mt_rng_)};
    speed_   = speed_dist(mt_rng_);
    heading_ = heading_dist(mt_rng_);
    width_   = beam_dist(mt_rng_);
    length_  = width_ * l_w_ratio_dist(mt_rng_);
    rot_     = rot_dist(mt_rng_);
}

void MockAisShip::tick(const Vec2DFloat & polaris_lat_lon)
{
    std::uniform_real_distribution<float> heading_dist(
      heading_ - config_.max_heading_change_, heading_ + config_.max_heading_change_);
    std::uniform_real_distribution<float> speed_dist(
      speed_ - config_.max_speed_change_, speed_ + config_.max_speed_change_);
    std::uniform_int_distribution<int8_t> rot_dist(ROT_LBND, ROT_UBND);

    float speed = speed_dist(mt_rng_);
    if (speed > SOG_SPEED_UBND) {
        speed = SOG_SPEED_UBND;
    } else if (speed < SOG_SPEED_LBND) {
        speed = SOG_SPEED_LBND;
    }

    heading_                               = boundHeading(heading_dist(mt_rng_));
    speed_                                 = speed;
    Vec2DFloat dir_vec                     = headingToVec2D(heading_);
    Vec2DFloat displacement                = dir_vec * speed_;
    Vec2DFloat new_pos                     = lat_lon_ + displacement;
    Vec2DFloat polaris_to_new_pos_vec      = new_pos - polaris_lat_lon;
    float      polaris_to_new_pos_distance = qvm::mag(polaris_to_new_pos_vec);

    if (polaris_to_new_pos_distance < config_.min_ship_dist_) {
        qvm::normalize(polaris_to_new_pos_vec);
        lat_lon_ = polaris_lat_lon + config_.min_ship_dist_ * polaris_to_new_pos_vec;
    } else if (polaris_to_new_pos_distance > config_.max_ship_dist_) {
        qvm::normalize(polaris_to_new_pos_vec);
        lat_lon_ = polaris_lat_lon + config_.max_ship_dist_ * polaris_to_new_pos_vec;
    } else {
        lat_lon_ = new_pos;
    }

    // ROT does not necessarily depend on actual change in heading, so we can use a random number
    rot_ = rot_dist(mt_rng_);
}

MockAis::MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon)
: MockAis(
    seed, num_ships, polaris_lat_lon,
    {.max_heading_change_ = defaults::MAX_HEADING_CHANGE,
     .max_speed_change_   = defaults::MAX_SPEED_CHANGE,
     .max_ship_dist_      = defaults::MAX_AIS_SHIP_DIST,
     .min_ship_dist_      = defaults::MIN_AIS_SHIP_DIST,
     .min_ship_width_m_   = defaults::MIN_AIS_SHIP_WIDTH_M,
     .max_ship_width_m_   = defaults::MAX_AIS_SHIP_WIDTH_M,
     .min_ship_l_w_ratio_ = defaults::MIN_AIS_SHIP_L_W_RATIO,
     .max_ship_l_w_ratio_ = defaults::MAX_AIS_SHIP_L_W_RATIO})
{
}

MockAis::MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon, SimShipConfig config)
: polaris_lat_lon_(polaris_lat_lon)
{
    for (uint32_t i = 0; i < num_ships; i++) {
        ships_.push_back(MockAisShip(seed + i, i, polaris_lat_lon, config));
    }
}

std::vector<AisShip> MockAis::ships() const
{
    std::vector<AisShip> ships;
    for (const MockAisShip & ship : ships_) {
        ships.push_back(ship);
    }
    return ships;
}

void MockAis::updatePolarisPos(const Vec2DFloat & lat_lon) { polaris_lat_lon_ = lat_lon; }

void MockAis::tick()
{
    for (MockAisShip & ship : ships_) {
        ship.tick(polaris_lat_lon_);
    }
}
