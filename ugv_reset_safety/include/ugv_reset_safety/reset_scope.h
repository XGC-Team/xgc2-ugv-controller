#pragma once

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ugv_reset_safety {
// One declaration from frozen Session.robots, not a roster file or a second body/spec resolver.
// Only the scope fields kind and namespace are read. Vehicle Sessions remain shape/limits truth.
inline std::vector<std::string> expectedResetNamespaces(const std::string& json) {
    if (json.empty()) return {}; // observation only; the Coordinator refuses admissions
    const auto first_char = json.find_first_not_of(" \t\r\n");
    const auto last_char = json.find_last_not_of(" \t\r\n");
    if (first_char == std::string::npos || json[first_char] != '[' || json[last_char] != ']')
        throw std::invalid_argument("expectedScope must be an array");
    boost::property_tree::ptree rows;
    std::istringstream input(json);
    boost::property_tree::read_json(input, rows);
    if (!rows.data().empty())
        throw std::invalid_argument("expectedScope must be a frozen robot array");
    std::set<std::string> names;
    for (const auto& row : rows) {
        if (!row.first.empty()) throw std::invalid_argument("expectedScope must be an array");
        const std::string kind = row.second.get<std::string>("kind");
        if (kind != "scout_mini" && kind != "mecanum_ugv") continue;
        std::string name = row.second.get<std::string>("namespace");
        if (!name.empty() && name.front() == '/') name.erase(0,1);
        if (name.empty() || name.back() == '/' || name.find("//") != std::string::npos)
            throw std::invalid_argument("invalid expected Reset namespace");
        bool first = true;
        for (unsigned char c : name) {
            if (c == '/') { first=true; continue; }
            const bool letter = (c>='a'&&c<='z') || (c>='A'&&c<='Z') || c=='_';
            if (!letter && !(c>='0'&&c<='9'&&!first))
                throw std::invalid_argument("invalid expected Reset namespace");
            first=false;
        }
        if (!names.insert(name).second) throw std::invalid_argument("duplicate expected Reset namespace");
    }
    return {names.begin(),names.end()};
}
} // namespace ugv_reset_safety
