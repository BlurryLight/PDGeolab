#include "resource_path_searcher.h"
#include "cmake_vars.h"
#include <iostream>
using namespace PD;
fs::path ResourcePathSearcher::root_path = Path(
    ROOT_DIR); // ROOT_DIR is defined by CMake, which is the project root dir
ResourcePathSearcher::ResourcePathSearcher() {
  search_paths_.push_back(root_path);
}

void ResourcePathSearcher::add_path(const std::string &path) {
  Path p(path);
  if (!p.is_absolute())
    p = fs::absolute(p);
  search_paths_.push_back(p);
}

std::string ResourcePathSearcher::find_path(const std::string &filename) const {
  if (cached_pathes_.find(filename) != cached_pathes_.end()) {
    std::cout << "Resource: " << filename << " found in cache!" << std::endl;
    return cached_pathes_.at(filename);
  }
  for (const Path &p : search_paths_) {
    auto path = p / filename;
    if (fs::exists(path)) {
      std::cout << "Resource: " << path << " found!" << std::endl;
      // It may cause problems in Windows because Windows native path is encoded
      // in UTF16LE To Handle this problem will need complex mechanism like
      // writing wstring overloads for all related functions.I won't bother to
      // do that.
      auto res =
          path.is_absolute() ? path.u8string() : fs::absolute(path).u8string();
      this->cached_pathes_.emplace(filename, res);
      return res;
    }
  }
  throw std::runtime_error("ResourcePathSearch cannot find " +
                           std::string(filename));
}

std::string
ResourcePathSearcher::find_path(std ::vector<std::string> filenames) const {
  Path ps;
  for (const auto &i : filenames) {
    ps /= i;
  }
  if (cached_pathes_.find(ps) != cached_pathes_.end()) {
    std::cout << "Resource: " << ps << " found in cache!" << std::endl;
    return cached_pathes_.at(ps);
  }
  for (const Path &p : search_paths_) {
    auto path = p / ps;
    if (fs::exists(path)) {
      std::cout << "Resource: " << path << " found!" << std::endl;
      auto res =
          path.is_absolute() ? path.u8string() : fs::absolute(path).u8string();
      this->cached_pathes_.emplace(ps.string(), res);
      return res;
    }
  }
  throw std::runtime_error("ResourcePathSearch cannot find " +
                           *filenames.rbegin());
}
