#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

template<typename T>
T readPod(std::ifstream & input)
{
  T value;
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!input) {
    throw std::runtime_error("truncated snapshot");
  }
  return value;
}

template<typename T>
std::vector<T> readArray(std::ifstream & input, size_t count)
{
  std::vector<T> values(count);
  input.read(reinterpret_cast<char *>(values.data()), count * sizeof(T));
  if (!input) {
    throw std::runtime_error("truncated snapshot array");
  }
  return values;
}

struct Snapshot
{
  int32_t nx;
  int32_t ny;
  int32_t npath;
  int start[2];
  int goal[2];
  std::vector<uint8_t> costarr;
  std::vector<float> potarr;
};

Snapshot load(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  char magic[8];
  input.read(magic, sizeof(magic));
  if (std::string(magic, sizeof(magic)) != "NAVFSNAP" || readPod<uint32_t>(input) != 1) {
    throw std::runtime_error("unsupported snapshot");
  }
  Snapshot snapshot;
  snapshot.nx = readPod<int32_t>(input);
  snapshot.ny = readPod<int32_t>(input);
  for (int i = 0; i < 9; ++i) {
    readPod<double>(input);
  }
  for (int i = 0; i < 4; ++i) {
    readPod<int32_t>(input);
  }
  readPod<uint8_t>(input);
  readPod<uint8_t>(input);
  readPod<int64_t>(input);
  snapshot.start[0] = readPod<int32_t>(input);
  snapshot.start[1] = readPod<int32_t>(input);
  snapshot.goal[0] = readPod<int32_t>(input);
  snapshot.goal[1] = readPod<int32_t>(input);
  readPod<int32_t>(input);
  readPod<float>(input);
  readPod<float>(input);
  snapshot.npath = readPod<int32_t>(input);
  const size_t cells = static_cast<size_t>(snapshot.nx) * snapshot.ny;
  snapshot.costarr = readArray<uint8_t>(input, cells);
  snapshot.potarr = readArray<float>(input, cells);
  input.seekg(8LL * snapshot.npath, std::ios::cur);
  if (!input || input.peek() != std::ifstream::traits_type::eof()) {
    throw std::runtime_error("snapshot has invalid path arrays");
  }
  return snapshot;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: pathcalc_snapshot_replay_test SNAPSHOT\n";
    return 2;
  }
  Snapshot snapshot = load(argv[1]);
  if (snapshot.nx != 1000 || snapshot.ny != 1000 || snapshot.npath != 4000) {
    std::cerr << "REGRESSION_ERROR: unexpected fixture dimensions\n";
    return 2;
  }
  nav2_navfn_planner::NavFn navfn(snapshot.nx, snapshot.ny);
  const size_t cells = snapshot.costarr.size();
  std::copy(snapshot.costarr.begin(), snapshot.costarr.end(), navfn.costarr);
  std::copy(snapshot.potarr.begin(), snapshot.potarr.end(), navfn.potarr);
  std::fill(navfn.gradx, navfn.gradx + cells, 0.0F);
  std::fill(navfn.grady, navfn.grady + cells, 0.0F);
  navfn.setStart(snapshot.start);
  navfn.setGoal(snapshot.goal);
  const int result = navfn.calcPath(4000);
  const int npath = navfn.getPathLen();
  std::cout << "SNAPSHOT_REPLAY result=" << result << " npath=" << npath;
  if (npath > 0) {
    std::cout << " final=(" << navfn.getPathX()[npath - 1] << ","
              << navfn.getPathY()[npath - 1] << ")";
  }
  std::cout << "\n";
  if (result <= 0) {
    std::cerr << "REGRESSION_FAIL: exact #4682 snapshot path extraction returned 0\n";
    return 1;
  }
  if (navfn.getPathX()[npath - 1] != snapshot.goal[0] ||
    navfn.getPathY()[npath - 1] != snapshot.goal[1])
  {
    std::cerr << "REGRESSION_FAIL: extracted path did not end at target\n";
    return 1;
  }
  std::cout << "REGRESSION_PASS: exact #4682 snapshot path extraction reached target\n";
  return 0;
}
