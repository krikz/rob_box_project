#ifndef PATHCALC_SNAPSHOT_HPP_
#define PATHCALC_SNAPSHOT_HPP_

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

namespace pathcalc_snapshot
{

// Version 1 stores every scalar and array element in little-endian byte order.
constexpr uint32_t kVersion = 1;

struct Metadata
{
  std::string output_path;
  int32_t nx = 0;
  int32_t ny = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double start_world_x = 0.0;
  double start_world_y = 0.0;
  double goal_world_x = 0.0;
  double goal_world_y = 0.0;
  double best_pose_world_x = 0.0;
  double best_pose_world_y = 0.0;
  int32_t map_start_x = 0;
  int32_t map_start_y = 0;
  int32_t map_goal_x = 0;
  int32_t map_goal_y = 0;
  bool use_astar = false;
  bool allow_unknown = false;
  int64_t timestamp_ns = 0;
};

struct Snapshot
{
  Metadata metadata;
  int32_t navfn_start_x = 0;
  int32_t navfn_start_y = 0;
  int32_t navfn_goal_x = 0;
  int32_t navfn_goal_y = 0;
  int32_t stc = 0;
  float dx = 0.0F;
  float dy = 0.0F;
  int32_t npath = 0;
  const uint8_t * costarr = nullptr;
  const float * potarr = nullptr;
  const float * pathx = nullptr;
  const float * pathy = nullptr;
};

namespace detail
{

inline bool isLittleEndian()
{
  const uint16_t marker = 1;
  return *reinterpret_cast<const uint8_t *>(&marker) == 1;
}

template<typename T>
void writePod(std::ofstream & output, const T & value)
{
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(value));
  if (!isLittleEndian()) {
    std::reverse(bytes, bytes + sizeof(value));
  }
  output.write(reinterpret_cast<const char *>(bytes), sizeof(value));
}

template<typename T>
void writeArray(std::ofstream & output, const T * values, size_t size)
{
  if (isLittleEndian()) {
    output.write(reinterpret_cast<const char *>(values), size * sizeof(T));
    return;
  }
  for (size_t i = 0; i < size; ++i) {
    writePod(output, values[i]);
  }
}

inline void validate(const Snapshot & snapshot)
{
  const Metadata & metadata = snapshot.metadata;
  if (metadata.output_path.empty() || metadata.nx <= 0 || metadata.ny <= 0 || snapshot.npath < 0) {
    throw std::invalid_argument("invalid PathCalc snapshot dimensions or output path");
  }
  if (!snapshot.costarr || !snapshot.potarr || !snapshot.pathx || !snapshot.pathy) {
    throw std::invalid_argument("null PathCalc snapshot array");
  }
}

inline void writeFile(const std::string & path, const Snapshot & snapshot)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot create PathCalc snapshot temporary file");
  }
  const Metadata & metadata = snapshot.metadata;
  output.write("NAVFSNAP", 8);
  writePod(output, kVersion);
  writePod(output, metadata.nx);
  writePod(output, metadata.ny);
  writePod(output, metadata.resolution);
  writePod(output, metadata.origin_x);
  writePod(output, metadata.origin_y);
  writePod(output, metadata.start_world_x);
  writePod(output, metadata.start_world_y);
  writePod(output, metadata.goal_world_x);
  writePod(output, metadata.goal_world_y);
  writePod(output, metadata.best_pose_world_x);
  writePod(output, metadata.best_pose_world_y);
  writePod(output, metadata.map_start_x);
  writePod(output, metadata.map_start_y);
  writePod(output, metadata.map_goal_x);
  writePod(output, metadata.map_goal_y);
  writePod(output, static_cast<uint8_t>(metadata.use_astar));
  writePod(output, static_cast<uint8_t>(metadata.allow_unknown));
  writePod(output, metadata.timestamp_ns);
  writePod(output, snapshot.navfn_start_x);
  writePod(output, snapshot.navfn_start_y);
  writePod(output, snapshot.navfn_goal_x);
  writePod(output, snapshot.navfn_goal_y);
  writePod(output, snapshot.stc);
  writePod(output, snapshot.dx);
  writePod(output, snapshot.dy);
  writePod(output, snapshot.npath);
  const size_t cells = static_cast<size_t>(metadata.nx) * static_cast<size_t>(metadata.ny);
  writeArray(output, snapshot.costarr, cells);
  writeArray(output, snapshot.potarr, cells);
  writeArray(output, snapshot.pathx, static_cast<size_t>(snapshot.npath));
  writeArray(output, snapshot.pathy, static_cast<size_t>(snapshot.npath));
  output.flush();
  if (!output) {
    throw std::runtime_error("cannot write PathCalc snapshot temporary file");
  }
}

}  // namespace detail

inline void writeAtomic(const Snapshot & snapshot)
{
  detail::validate(snapshot);
  const std::string & final_path = snapshot.metadata.output_path;
  if (std::ifstream(final_path, std::ios::binary).good()) {
    return;
  }
  const std::string temporary_path = final_path + ".tmp";
  try {
    detail::writeFile(temporary_path, snapshot);
    if (std::rename(temporary_path.c_str(), final_path.c_str()) != 0) {
      throw std::runtime_error(std::string("cannot rename PathCalc snapshot: ") + std::strerror(errno));
    }
  } catch (...) {
    std::remove(temporary_path.c_str());
    throw;
  }
}

}  // namespace pathcalc_snapshot

#endif  // PATHCALC_SNAPSHOT_HPP_
