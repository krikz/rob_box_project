#include "pathcalc_snapshot.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace
{

using pathcalc_snapshot::Metadata;
using pathcalc_snapshot::Snapshot;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::string readFile(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

template<typename T>
T readPod(std::ifstream & input)
{
  T value;
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  require(static_cast<bool>(input), "unexpected end of snapshot");
  return value;
}

template<typename T>
std::vector<T> readArray(std::ifstream & input, size_t size)
{
  std::vector<T> values(size);
  input.read(reinterpret_cast<char *>(values.data()), values.size() * sizeof(T));
  require(static_cast<bool>(input), "truncated snapshot array");
  return values;
}

Snapshot makeSnapshot(const std::string & output_path)
{
  Snapshot snapshot;
  snapshot.metadata.output_path = output_path;
  snapshot.metadata.nx = 3;
  snapshot.metadata.ny = 2;
  snapshot.metadata.resolution = 0.05;
  snapshot.metadata.origin_x = -28.55;
  snapshot.metadata.origin_y = -21.85;
  snapshot.metadata.start_world_x = -3.37;
  snapshot.metadata.start_world_y = 3.58;
  snapshot.metadata.goal_world_x = -6.0;
  snapshot.metadata.goal_world_y = 4.0;
  snapshot.metadata.best_pose_world_x = -5.95;
  snapshot.metadata.best_pose_world_y = 4.05;
  snapshot.metadata.map_start_x = 499;
  snapshot.metadata.map_start_y = 503;
  snapshot.metadata.map_goal_x = 446;
  snapshot.metadata.map_goal_y = 511;
  snapshot.metadata.use_astar = true;
  snapshot.metadata.allow_unknown = false;
  snapshot.metadata.timestamp_ns = 1794757714671303835LL;
  snapshot.navfn_start_x = 446;
  snapshot.navfn_start_y = 511;
  snapshot.navfn_goal_x = 499;
  snapshot.navfn_goal_y = 503;
  snapshot.stc = 550477;
  snapshot.dx = 0.815F;
  snapshot.dy = -0.402F;
  snapshot.npath = 4;
  return snapshot;
}

void checkRoundTrip(const std::string & path, int64_t timestamp_ns)
{
  Snapshot expected = makeSnapshot(path);
  expected.metadata.timestamp_ns = timestamp_ns;
  const std::vector<uint8_t> cost = {0, 1, 50, 252, 253, 255};
  const std::vector<float> potential = {1.0F, 2.5F, 1.0e10F, 4.0F, 5.0F, 6.0F};
  const std::vector<float> path_x = {446.0F, 446.5F, 447.0F, 499.0F};
  const std::vector<float> path_y = {511.0F, 511.5F, 512.0F, 503.0F};
  expected.costarr = cost.data();
  expected.potarr = potential.data();
  expected.pathx = path_x.data();
  expected.pathy = path_y.data();
  pathcalc_snapshot::writeAtomic(expected);
  const std::string raw = readFile(path);
  require(raw.compare(8, 4, std::string("\x01\x00\x00\x00", 4)) == 0, "version is not little-endian");

  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "snapshot was not created");
  char magic[8];
  input.read(magic, sizeof(magic));
  require(std::memcmp(magic, "NAVFSNAP", sizeof(magic)) == 0, "bad snapshot magic");
  require(readPod<uint32_t>(input) == 1U, "bad snapshot version");

  const Metadata & metadata = expected.metadata;
  require(readPod<int32_t>(input) == metadata.nx, "nx round-trip");
  require(readPod<int32_t>(input) == metadata.ny, "ny round-trip");
  require(readPod<double>(input) == metadata.resolution, "resolution round-trip");
  require(readPod<double>(input) == metadata.origin_x, "origin_x round-trip");
  require(readPod<double>(input) == metadata.origin_y, "origin_y round-trip");
  require(readPod<double>(input) == metadata.start_world_x, "start_world_x round-trip");
  require(readPod<double>(input) == metadata.start_world_y, "start_world_y round-trip");
  require(readPod<double>(input) == metadata.goal_world_x, "goal_world_x round-trip");
  require(readPod<double>(input) == metadata.goal_world_y, "goal_world_y round-trip");
  require(readPod<double>(input) == metadata.best_pose_world_x, "best_pose_world_x round-trip");
  require(readPod<double>(input) == metadata.best_pose_world_y, "best_pose_world_y round-trip");
  require(readPod<int32_t>(input) == metadata.map_start_x, "map_start_x round-trip");
  require(readPod<int32_t>(input) == metadata.map_start_y, "map_start_y round-trip");
  require(readPod<int32_t>(input) == metadata.map_goal_x, "map_goal_x round-trip");
  require(readPod<int32_t>(input) == metadata.map_goal_y, "map_goal_y round-trip");
  require(readPod<uint8_t>(input) == metadata.use_astar, "use_astar round-trip");
  require(readPod<uint8_t>(input) == metadata.allow_unknown, "allow_unknown round-trip");
  require(readPod<int64_t>(input) == metadata.timestamp_ns, "timestamp_ns round-trip");
  require(readPod<int32_t>(input) == expected.navfn_start_x, "navfn_start_x round-trip");
  require(readPod<int32_t>(input) == expected.navfn_start_y, "navfn_start_y round-trip");
  require(readPod<int32_t>(input) == expected.navfn_goal_x, "navfn_goal_x round-trip");
  require(readPod<int32_t>(input) == expected.navfn_goal_y, "navfn_goal_y round-trip");
  require(readPod<int32_t>(input) == expected.stc, "stc round-trip");
  require(readPod<float>(input) == expected.dx, "dx round-trip");
  require(readPod<float>(input) == expected.dy, "dy round-trip");
  require(readPod<int32_t>(input) == expected.npath, "npath round-trip");
  require(readArray<uint8_t>(input, cost.size()) == cost, "costarr round-trip");
  require(readArray<float>(input, potential.size()) == potential, "potarr round-trip");
  require(readArray<float>(input, path_x.size()) == path_x, "pathx round-trip");
  require(readArray<float>(input, path_y.size()) == path_y, "pathy round-trip");
  require(input.peek() == std::ifstream::traits_type::eof(), "trailing snapshot bytes");
  require(std::ifstream(path + ".tmp").good() == false, "temporary file survived rename");
}

}  // namespace

int main(int argc, char ** argv)
{
  require(argc == 2, "usage: pathcalc_snapshot_test WORK_DIR");
  const std::string work = argv[1];
  const std::string final_path = work + "/snapshot.bin";

  checkRoundTrip(final_path, 1794757714671303835LL);
  std::cout << "PASS: versioned round-trip of every scalar and array; atomic rename; temp absent\n";

  const std::string before = readFile(final_path);
  Snapshot second = makeSnapshot(final_path);
  second.metadata.timestamp_ns = 1794757714671303836LL;
  const std::vector<uint8_t> cost = {0, 1, 50, 252, 253, 255};
  const std::vector<float> potential = {1.0F, 2.5F, 1.0e10F, 4.0F, 5.0F, 6.0F};
  const std::vector<float> path_x = {446.0F, 446.5F, 447.0F, 499.0F};
  const std::vector<float> path_y = {511.0F, 511.5F, 512.0F, 503.0F};
  second.costarr = cost.data();
  second.potarr = potential.data();
  second.pathx = path_x.data();
  second.pathy = path_y.data();
  pathcalc_snapshot::writeAtomic(second);
  require(readFile(final_path) == before, "second exhaustion overwrote first snapshot");
  require(std::ifstream(final_path + ".tmp").good() == false, "second write left temporary file");
  std::cout << "PASS: first-write-wins preserves the original snapshot\n";

  Snapshot invalid = makeSnapshot(final_path);
  bool rejected = false;
  try {
    pathcalc_snapshot::writeAtomic(invalid);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "invalid array size was not rejected");
  require(readFile(final_path) == before, "failed write changed final snapshot");
  require(std::ifstream(final_path + ".tmp").good() == false, "failed write left temporary file");
  std::cout << "PASS: null array preserves previous final and leaves no temp\n";
  return 0;
}
