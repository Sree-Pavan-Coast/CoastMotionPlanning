#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "coastmotionplanning/visualizer/planner_visualizer_server.hpp"
#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace {

constexpr const char* kDefaultHost = "127.0.0.1";

bool openBrowser(const std::string& url) {
#if defined(__APPLE__)
    const std::string command = "open \"" + url + "\"";
#elif defined(__linux__)
    const std::string command = "xdg-open \"" + url + "\"";
#elif defined(_WIN32)
    const std::string command = "start \"\" \"" + url + "\"";
#else
    return false;
#endif
    return std::system(command.c_str()) == 0;
}

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0
              << " [--port <port>] [--open-browser] [--configs-root <path>]\n";
}

} // namespace

int main(int argc, char** argv) {
    int port = 8080;
    bool open_browser = false;
    std::filesystem::path configs_root = std::filesystem::path(COAST_REPO_ROOT) / "configs";

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--open-browser") {
            open_browser = true;
            continue;
        }
        if (arg == "--port") {
            if (index + 1 >= argc) {
                std::cerr << "--port requires a value.\n";
                return 1;
            }
            port = std::stoi(argv[++index]);
            continue;
        }
        if (arg == "--configs-root") {
            if (index + 1 >= argc) {
                std::cerr << "--configs-root requires a value.\n";
                return 1;
            }
            configs_root = argv[++index];
            continue;
        }

        std::cerr << "Unknown argument: " << arg << "\n";
        printUsage(argv[0]);
        return 1;
    }

    try {
        auto service = std::make_shared<coastmotionplanning::visualizer::PlannerVisualizerService>(
            coastmotionplanning::visualizer::PlannerVisualizerServiceConfig{
                configs_root,
                "Pro_XD"
            });

        coastmotionplanning::visualizer::PlannerVisualizerServer server(
            coastmotionplanning::visualizer::PlannerVisualizerServerConfig{
                kDefaultHost,
                port,
                COAST_PLANNER_VISUALIZER_ASSET_DIR
            },
            service);

        server.start();
        const auto url = server.baseUrl();
        std::cout << "Planner visualizer running at " << url << std::endl;

        if (open_browser && !openBrowser(url)) {
            std::cerr << "Failed to open browser automatically. Navigate to " << url << std::endl;
        }

        server.wait();
    } catch (const std::exception& e) {
        std::cerr << "Planner visualizer failed to start: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
