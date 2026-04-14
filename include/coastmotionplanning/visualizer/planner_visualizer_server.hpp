#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <thread>

namespace httplib {
class Server;
}

namespace coastmotionplanning {
namespace visualizer {

class PlannerVisualizerService;

struct PlannerVisualizerServerConfig {
    std::string host{"127.0.0.1"};
    int port{8080};
    std::filesystem::path asset_dir;
};

class PlannerVisualizerServer {
public:
    PlannerVisualizerServer(PlannerVisualizerServerConfig config,
                            std::shared_ptr<PlannerVisualizerService> service);
    ~PlannerVisualizerServer();

    int start();
    void stop();
    void wait();

    std::string baseUrl() const;
    int port() const { return bound_port_; }

private:
    void registerRoutes();

    PlannerVisualizerServerConfig config_;
    std::shared_ptr<PlannerVisualizerService> service_;
    std::unique_ptr<httplib::Server> server_;
    std::thread server_thread_;
    int bound_port_{0};
};

} // namespace visualizer
} // namespace coastmotionplanning
