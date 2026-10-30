#include "platform/file_hash.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(const std::string& content, const std::string& expected) {
    const auto path = std::filesystem::temp_directory_path() / "dt46_file_hash_check.bin";
    {
        std::ofstream output(path, std::ios::binary);
        output << content;
        if (!output) throw std::runtime_error("Cannot write test file");
    }
    const auto actual = dt46::sha256(path);
    std::filesystem::remove(path);
    if (actual != expected) throw std::runtime_error("SHA256 mismatch: " + actual + " != " + expected);
}
}

int main() {
    try {
        check("", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        check("abc", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        check(std::string(1000, 'a'), "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    std::cout << "file hash check passed\n";
    return 0;
}
