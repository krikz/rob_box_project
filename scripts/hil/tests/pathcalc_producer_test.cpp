#include "nav2_navfn_planner/navfn.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool exists(const std::string & path)
{
  return std::ifstream(path, std::ios::binary).good();
}

std::string readFile(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::vector<unsigned char> freeMap(int nx, int ny)
{
  return std::vector<unsigned char>(static_cast<size_t>(nx) * static_cast<size_t>(ny), 0);
}

}  // namespace

int main(int argc, char ** argv)
{
  require(argc == 2, "usage: pathcalc_producer_test WORK_DIR");
  const std::string output = std::string(argv[1]) + "/producer.bin";
  const int nx = 30;
  const int ny = 30;
  const std::vector<unsigned char> costmap = freeMap(nx, ny);
  int start[2] = {2, 2};
  int goal[2] = {27, 27};

  nav2_navfn_planner::NavFn success(nx, ny);
  success.setCostmap(costmap.data(), true, true);
  success.setStart(start);
  success.setGoal(goal);
  require(success.calcNavFnAstar(), "A* fixture did not reach requested goal");
  success.snapshot_metadata.output_path = output;
  require(success.calcPath(4000) > 0, "successful calcPath fixture failed");
  require(!exists(output), "successful calcPath created a snapshot");
  require(!exists(output + ".tmp"), "successful calcPath created a temporary snapshot");
  std::cout << "PASS: successful calcPath creates no snapshot\n";

  nav2_navfn_planner::NavFn failure(nx, ny);
  failure.setCostmap(costmap.data(), true, true);
  failure.setStart(start);
  failure.setGoal(goal);
  require(failure.calcNavFnAstar(), "A* failure fixture did not reach requested goal");
  failure.snapshot_metadata.output_path = output;
  require(failure.calcPath(1) == 0, "one-step calcPath unexpectedly succeeded");
  require(exists(output), "path exhaustion did not create a snapshot");
  require(!exists(output + ".tmp"), "path exhaustion left a temporary snapshot");
  std::ifstream input(output, std::ios::binary);
  char magic[8];
  input.read(magic, sizeof(magic));
  require(std::string(magic, sizeof(magic)) == "NAVFSNAP", "producer snapshot has bad magic");
  std::cout << "PASS: terminal calcPath exhaustion atomically creates versioned snapshot\n";

  const std::string first = readFile(output);
  failure.snapshot_metadata.timestamp_ns = 99;
  require(failure.calcPath(1) == 0, "second one-step calcPath unexpectedly succeeded");
  require(readFile(output) == first, "second exhaustion overwrote first producer snapshot");
  require(!exists(output + ".tmp"), "second exhaustion left a temporary snapshot");
  std::cout << "PASS: repeated exhaustion preserves first snapshot\n";
  return 0;
}
