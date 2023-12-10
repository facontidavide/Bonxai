#pragma once

#include <iterator>
#include <sstream>

#include <bonxai/bonxai.hpp>
#include <bonxai/serialization.hpp>

#include <bonxai_msgs/msg/bonxai.hpp>

namespace bonxai_msgs
{
    /**
     * @brief Convert bonxai grid to ros2 msg. The header will NOT be populated.
     */
    template <class CellT>
    void toRosMsg(const Bonxai::VoxelGrid<CellT> &grid, bonxai_msgs::msg::Bonxai &msg)
    {
        std::ostringstream ofile(std::ios::binary);
        Bonxai::Serialize(ofile, grid);
        msg.raw_data.clear();
        const auto& data_as_string = ofile.str();
        msg.raw_data.reserve(data_as_string.size());
        msg.raw_data.insert(
                msg.raw_data.end(),
                data_as_string.begin(),
                data_as_string.end()
            );
        // TODO (jjd9): where to get frame_id and timestamp?
    }

    /**
     * @brief Convert ros2 msg to bonxai grid. The header will NOT be transferred.
     */
    template <class CellT>
    Bonxai::VoxelGrid<CellT> fromRosMsg(const bonxai_msgs::msg::Bonxai &msg, std::string &type_name = {})
    {
        std::istringstream ifile(std::string(reinterpret_cast<const char *>(msg.raw_data.data()), msg.raw_data.size()), std::ios::binary);
        char header[256];
        ifile.getline(header, 256);

        const Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
        type_name = info.type_name;
        return Bonxai::Deserialize<CellT>(ifile, info);
    }

}