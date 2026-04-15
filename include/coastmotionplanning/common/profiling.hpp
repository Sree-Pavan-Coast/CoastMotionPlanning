#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace coastmotionplanning {
namespace common {

struct ProfilingScopeSummary {
    std::string scope_name;
    uint64_t count{0};
    double total_ms{0.0};
    double avg_ms{0.0};
    double max_ms{0.0};
};

class ProfilingCollector {
public:
    void record(const std::string& scope_name, double elapsed_ms) {
        auto& summary = summaries_[scope_name];
        summary.scope_name = scope_name;
        ++summary.count;
        summary.total_ms += elapsed_ms;
        summary.max_ms = std::max(summary.max_ms, elapsed_ms);
    }

    std::vector<ProfilingScopeSummary> snapshotSortedByTotalMs() const {
        std::vector<ProfilingScopeSummary> results;
        results.reserve(summaries_.size());
        for (const auto& entry : summaries_) {
            ProfilingScopeSummary summary = entry.second;
            if (summary.count > 0) {
                summary.avg_ms = summary.total_ms / static_cast<double>(summary.count);
            }
            results.push_back(std::move(summary));
        }

        std::sort(
            results.begin(),
            results.end(),
            [](const ProfilingScopeSummary& lhs, const ProfilingScopeSummary& rhs) {
                if (lhs.total_ms != rhs.total_ms) {
                    return lhs.total_ms > rhs.total_ms;
                }
                return lhs.scope_name < rhs.scope_name;
            });
        return results;
    }

    void mergeFrom(const ProfilingCollector& other) {
        for (const auto& entry : other.summaries_) {
            const auto& other_summary = entry.second;
            auto& summary = summaries_[entry.first];
            summary.scope_name = other_summary.scope_name;
            summary.count += other_summary.count;
            summary.total_ms += other_summary.total_ms;
            summary.max_ms = std::max(summary.max_ms, other_summary.max_ms);
        }
    }

private:
    std::unordered_map<std::string, ProfilingScopeSummary> summaries_;
};

class ScopedProfilingTimer {
public:
    ScopedProfilingTimer(ProfilingCollector* collector, std::string scope_name)
        : collector_(collector),
          scope_name_(std::move(scope_name)),
          start_time_(Clock::now()) {}

    ~ScopedProfilingTimer() {
        if (collector_ == nullptr) {
            return;
        }
        const auto end_time = Clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time_).count();
        collector_->record(scope_name_, elapsed_ms);
    }

private:
    using Clock = std::chrono::steady_clock;

    ProfilingCollector* collector_{nullptr};
    std::string scope_name_;
    Clock::time_point start_time_;
};

} // namespace common
} // namespace coastmotionplanning
