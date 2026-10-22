#pragma once
#include "interfaces/transport.hpp"
#include <atomic>
#include <filesystem>
#include <thread>

namespace dt46 {
class DebugControl {
public:
    explicit DebugControl(const std::filesystem::path& session_file);
    ~DebugControl();
    std::atomic<bool> requested=false,close_requested=false,save_requested=false;
private:
    std::uint16_t port=0;std::string key;wire::Socket listener;
    std::filesystem::path session;std::jthread worker;
};
}
