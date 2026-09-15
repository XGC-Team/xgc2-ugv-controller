#pragma once

#include <ros/ros.h>

#include <map>
#include <stdexcept>
#include <string>

#include "unicycle_ugv_controller/common/heading_recovery.h"

namespace unicycle_ugv_controller {

inline HeadingRecoveryConfig loadHeadingRecovery(const ros::NodeHandle& nh) {
    HeadingRecoveryConfig cfg;
    XmlRpc::XmlRpcValue value;
    if (nh.hasParam("flatness/heading_recovery")) {
        if (!nh.getParam("flatness/heading_recovery", value) ||
            value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::invalid_argument("flatness/heading_recovery must be a mapping");
        }
        const std::map<std::string, double*> fields{{"gain", &cfg.gain},
                                                    {"axis_bias", &cfg.axis_bias},
                                                    {"rate_damping", &cfg.rate_damping}};
        for (auto it = value.begin(); it != value.end(); ++it) {
            const auto field = fields.find(it->first);
            if (field == fields.end()) {
                throw std::invalid_argument("unknown flatness/heading_recovery/" + it->first);
            }
            if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                *field->second = static_cast<int>(it->second);
            } else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                *field->second = static_cast<double>(it->second);
            } else {
                throw std::invalid_argument("heading recovery parameter must be numeric: " +
                                            it->first);
            }
        }
    }
    cfg.validate();
    return cfg;
}

}  // namespace unicycle_ugv_controller
