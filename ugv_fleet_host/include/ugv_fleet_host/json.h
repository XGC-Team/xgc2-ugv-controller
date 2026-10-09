#pragma once
#include <json/json.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
namespace ugv_fleet_host::wire {
inline Json::Value parse(const std::string& body) {
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["allowComments"] = false;
    builder["allowTrailingCommas"] = false;
    builder["strictRoot"] = false;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value value;
    std::string error;
    if (!reader->parse(body.data(), body.data() + body.size(), &value, &error))
        throw std::invalid_argument("invalid JSON document");
    return value;
}
inline std::string serialize(const Json::Value& value) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}
inline const Json::Value& member(const Json::Value& value, std::string_view key) {
    const std::string owned_key(key);
    if (!value.isObject() || !value.isMember(owned_key))
        throw std::invalid_argument("missing JSON field");
    return value[owned_key];
}
inline const Json::Value& array(const Json::Value& value) {
    if (!value.isArray())
        throw std::invalid_argument("JSON array required");
    return value;
}
inline const Json::Value& object(const Json::Value& value) {
    if (!value.isObject())
        throw std::invalid_argument("JSON object required");
    return value;
}
inline std::string string(const Json::Value& value) {
    if (!value.isString())
        throw std::invalid_argument("JSON string required");
    return value.asString();
}
inline bool boolean(const Json::Value& value) {
    if (!value.isBool())
        throw std::invalid_argument("JSON boolean required");
    return value.asBool();
}
}  // namespace ugv_fleet_host::wire
