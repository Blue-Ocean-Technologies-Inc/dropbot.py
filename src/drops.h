#ifndef ___DROPBOT__DROPS__H___
#define ___DROPBOT__DROPS__H___

#include <stdint.h>
#include <algorithm>
#include <vector>

namespace dropbot {
namespace drops {


constexpr float C_THRESHOLD = 3e-12;


template <typename Neighbours, typename Capacitances, typename Channels>
std::vector<std::vector<uint8_t> > get_drops(Neighbours &neighbours,
                                             Capacitances &capacitances,
                                             Channels &channels,
                                             float c_threshold=C_THRESHOLD) {
  /* Assign each channel with a capacitance greater than the specified
   * threshold to a *"drop group"*.
   *
   * Each *drop group* contains a list of channels that correspond to a
   * contiguous set of electrodes where:
   *
   *   1. Each electrode has some detected liquid on it.
   *   2. There exists a path between any two electrodes in the set (given the
   *      list of channel neighbours).
   *
   * Given the above criteria, we assume that each *drop group* corresponds to
   * a single drop of liquid, spanning one or more electrodes.
   *
   * Returns
   * -------
   * std::vector<std::vector<uint8_t> >
   *    A list of *drop groups*, where each *drop group* is represented as a
   *    *list of corresponding channels*.
   */
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


}  // namespace drops {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__DROPS__H___
