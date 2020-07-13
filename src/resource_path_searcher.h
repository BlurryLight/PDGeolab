#pragma once

#ifndef FILESYSTEM_TS
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <string>
#include <unordered_map>
#include <vector>
namespace PD {

class ResourcePathSearcher {
private:
#ifndef FILESYSTEM_TS
  using Path = std::filesystem::path;
#else
  using Path = std::experimental::filesystem::path;
#endif
  std::vector<Path> search_paths_;
  mutable std::unordered_map<std::string, std::string> cached_pathes_;

public:
  static Path root_path;
  ResourcePathSearcher();
  ResourcePathSearcher(const ResourcePathSearcher &) = delete;
  ResourcePathSearcher &operator=(const ResourcePathSearcher &) = delete;
  ResourcePathSearcher(ResourcePathSearcher &&) = delete;
  void add_path(const std::string &path);
  // pass a vector such as {"src","cores","xxx.cc"}
  std::string find_path(std::vector<std::string> filenames) const;
  std::string find_path(const std::string &filename) const;
};

} // namespace PD
