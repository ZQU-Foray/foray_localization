// Copyright 2026 ZQU-Foray

#include "foray/localization_map/aligner_factory.hpp"

#include "foray/localization_map/icp_aligner.hpp"

#if defined(FORAY_WITH_PCL)
#include "foray/localization_map/pcl_gicp_aligner.hpp"
#endif

namespace foray::localization_map {

std::unique_ptr<Aligner> make_aligner(const AlignOptions& options) {
#if defined(FORAY_WITH_PCL)
    return std::make_unique<PclGicpAligner>(options);
#else
    return std::make_unique<IcpAligner>(options);
#endif
}

std::string default_aligner_name() {
#if defined(FORAY_WITH_PCL)
    return "pcl_gicp";
#else
    return "icp（未启用 PCL）";
#endif
}

} // namespace foray::localization_map
