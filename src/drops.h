#ifndef ___DROPBOT__DROPS__H___
#define ___DROPBOT__DROPS__H___

#include <stdint.h>
#include <algorithm>
#include <array>
#include <set>
#include <vector>

namespace dropbot {
namespace drops {


struct ChannelNeighbours {
  uint8_t up;
  uint8_t down;
  uint8_t left;
  uint8_t right;
};


constexpr float C_THRESHOLD = 3e-12;


/**
 * @brief Assign each channel with a capacitance greater than the specified
 * threshold to a *"drop group"*.
 *
 * Each *drop group* contains a list of channels that correspond to a
 * contiguous set of electrodes where:
 *
 *   1. Each electrode has some detected liquid on it.
 *   2. There exists a path between any two electrodes in the set (given the
 *      list of channel neighbours).
 *
 * Given the above criteria, we assume that each *drop group* corresponds to a
 * single drop of liquid, spanning one or more electrodes.
 *
 * @tparam Neighbours
 * @tparam Capacitances
 * @tparam Channels
 * @param neighbours  Container of neighbour structs (with `up`, `down`,
 *   `left`, and `right` attributes), one for for each channel with the index
 *   position in the container corresponding to the respective channel number.
 * @param capacitances  List of capacitance measurements, each corresponding to
 *   respective channel number in `channels`.
 * @param channels  List of channel numbers, each corresponding to respective
 *   capacitance measurement in `capacitances`.
 * @param c_threshold  Threshold (in Farads) to consider a channel "occupied".
 *
 * @return  A list of *drop groups*, where each *drop group* is represented as
 * a *list of corresponding channels*.
 */
template <typename Neighbours, typename Capacitances, typename Channels>
std::vector<std::vector<uint8_t> > get_drops(Neighbours &neighbours,
                                             Capacitances &capacitances,
                                             Channels &channels,
                                             float c_threshold=C_THRESHOLD) {
  const auto max_channel = *std::max_element(channels.begin(), channels.end());
  std::vector<uint8_t> drop_member(max_channel + 1, 0xFF);

  // Assign each channel to a **drop group**.
  for (auto it_channel = channels.begin(); it_channel != channels.end();
       it_channel++) {
    if (capacitances[*it_channel] <= C_THRESHOLD) {
      continue;
    }
    const auto &neighbours_i = neighbours[*it_channel];
    for (auto& x_i : std::array<uint8_t, 5>({static_cast<uint8_t>(*it_channel),
                                             neighbours_i.up,
                                             neighbours_i.down,
                                             neighbours_i.left,
                                             neighbours_i.right})) {
      if (x_i == 0xFF) {
        // No neighbour to channel `i` in this direction.
        continue;
      }
      if (drop_member[x_i] < 0xFF) {
        // Channel is already assigned to a **drop group**.
        // Remap to current **drop group**.
        const auto original_drop_i = drop_member[x_i];
        for (auto j = 0; j < drop_member.size(); j++) {
          if (original_drop_i == drop_member[j]) {
            drop_member[j] = *it_channel;
          }
        }
      } else if (capacitances[x_i] > C_THRESHOLD) {
        // Neighbour is partially covered.  Map to current **drop group**.
        drop_member[x_i] = *it_channel;
      }
    }
  }

  std::vector<std::vector<uint8_t> > drops;
  drops.reserve(drop_member.size() / 4);

  // Pack each **drop group** as list of member channels.
  auto sorted = drop_member;
  auto part_id = std::partition(sorted.begin(), sorted.end(),
                                +[](uint8_t v){ return v != 0xFF; });
  std::sort(sorted.begin(), part_id);
  {
    auto end = std::unique(sorted.begin(), part_id);

    for (auto it = sorted.begin(); it != end; it++) {
      const auto drop_id_i = *it;
      std::vector<uint8_t> drop_i;
      drop_i.reserve(drop_member.size() / 4);

      auto j = 0;
      for (auto it_j = drop_member.begin(); it_j != drop_member.end(); it_j++, j++) {
        const auto drop_id_j = *it_j;
        if (drop_id_i == drop_id_j) {
          drop_i.push_back(j);
        }
      }
      drop_i.shrink_to_fit();
      drops.push_back(drop_i);
    }
  }
  drops.shrink_to_fit();
  return drops;
}


/**
 * @brief Pack drops lists into single array.
 *
 * Each drop is encoded with a length `n + 1`, where `n` is the number of
 * channels in the drop.  The first value in the encoded drop is `n`, followed
 * by the drop channel numbers.
 *
 * For example:
 *
 *     [[0, 3, 7], [13, 2]]
 *
 * is encoded as:
 *
 *     [3, 0, 3, 7, 2, 13, 2]
 *
 * @tparam Drops
 * @tparam Array
 * @param drops  List of drops (as returned by `get_drops()`).
 * @param packed_drops  Output container for packed drops.
 */
template <typename Drops, typename Array>
void pack_drops(const Drops &drops, Array &packed_drops) {
  /*
   */
  for (auto it_drop = drops.begin(); it_drop != drops.end(); it_drop++) {
    const auto &drop = *it_drop;
    packed_drops.data[packed_drops.length++] = drop.size();
    std::copy(drop.begin(), drop.end(), &packed_drops.data[packed_drops.length]);
    packed_drops.length += drop.size();
  }
}


/**
 * @tparam Drops
 * @tparam Neighbours
 * @param drops  List of drops (as returned by `get_drops()`).
 * @param neighbours  Container of neighbour structs (with `up`, `down`,
 *   `left`, and `right` attributes), one for for each channel with the index
 *   position in the container corresponding to the respective channel number.
 *
 * @return  Set of channels belonging to the specified set of drops, as well as
 *   any neighbouring channels.
 *
 *   For example:
 *
 *         Drops         Drops +
 *                      Neighbours
 *       ---------      ----xx----
 *       ---XX----      ---xXXx---
 *       ----X----      ----xXx---
 *       ---------      -----x----
 */
template <typename Drops, typename Neighbours>
std::set<uint8_t> drop_channels(Drops const &drops, Neighbours const &neighbours) {
  std::set<uint8_t> drop_channels;
  for (auto const &drop_i : drops) {
    for (auto const channel_i : drop_i) {
      const auto &neighbours_i = neighbours[channel_i];
      for (auto neighbour_i :
           std::array<uint8_t, 5>({channel_i, neighbours_i.up,
                                   neighbours_i.down, neighbours_i.left,
                                   neighbours_i.right})) {
          if (neighbour_i != 0xff) {
              drop_channels.insert(neighbour_i);
          }
      }
    }
  }
  return drop_channels;
}


}  // namespace drops {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__DROPS__H___
