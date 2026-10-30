#pragma once

#include <filesystem>
#include <string>

namespace dt46 {
std::string sha256(const std::filesystem::path& path);
}
