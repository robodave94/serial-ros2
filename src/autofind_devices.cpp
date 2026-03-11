/**
 * @file autofind_devices.cpp
 * @brief ROS2 node for auto-finding serial devices by PID:VID and serial numbers
 */

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <map>
#include <string>
#include <vector>

class AutofindDevicesNode : public rclcpp::Node
{
public:
    AutofindDevicesNode() : Node("autofind_devices")
    {
        // Declare parameters
        this->declare_parameter<std::string>("pidvid", "");
        this->declare_parameter<std::string>("serial_number_1", "");
        this->declare_parameter<std::string>("serial_number_2", "");

        // Get parameters
        std::string pidvid = this->get_parameter("pidvid").as_string();
        std::string serial_number_1 = this->get_parameter("serial_number_1").as_string();
        std::string serial_number_2 = this->get_parameter("serial_number_2").as_string();

        // Validate parameters
        if (pidvid.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'pidvid' is required but not provided");
            return;
        }

        if (serial_number_1.empty() && serial_number_2.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Both serial numbers are empty. Will only scan for devices with PID:VID.");
        }

        RCLCPP_INFO(this->get_logger(), "Searching for devices with PID:VID: %s", pidvid.c_str());
        RCLCPP_INFO(this->get_logger(), "Serial Number 1: %s", serial_number_1.empty() ? "(not provided)" : serial_number_1.c_str());
        RCLCPP_INFO(this->get_logger(), "Serial Number 2: %s", serial_number_2.empty() ? "(not provided)" : serial_number_2.c_str());

        // Perform device search
        find_and_identify_devices(pidvid, serial_number_1, serial_number_2);
    }

private:
    void find_and_identify_devices(const std::string &pidvid,
                                   const std::string &serial_number_1,
                                   const std::string &serial_number_2)
    {
        std::vector<std::string> device_paths;

        // Try to find devices matching the PID:VID
        try
        {
            device_paths = serial::findSerialMultipleSerialDevicePathsByPIDVID(pidvid);
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "No devices found with PID:VID %s: %s", pidvid.c_str(), e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error accessing serial ports: %s", e.what());
            return;
        }

        size_t num_devices = device_paths.size();
        RCLCPP_INFO(this->get_logger(), "Found %zu device(s) with PID:VID %s", num_devices, pidvid.c_str());

        // Track which serial numbers were found and their paths
        std::string path_serial_1;
        std::string path_serial_2;
        bool found_serial_1 = false;
        bool found_serial_2 = false;

        // Check each device for the serial numbers
        for (const auto &path : device_paths)
        {
            bool matches_serial_1 = false;
            bool matches_serial_2 = false;

            if (!serial_number_1.empty())
            {
                try
                {
                    matches_serial_1 = serial::findStringCharacteristicInDevicePath(path, serial_number_1);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Error checking serial number 1 on %s: %s", path.c_str(), e.what());
                }
            }

            if (!serial_number_2.empty())
            {
                try
                {
                    matches_serial_2 = serial::findStringCharacteristicInDevicePath(path, serial_number_2);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Error checking serial number 2 on %s: %s", path.c_str(), e.what());
                }
            }

            if (matches_serial_1)
            {
                if (found_serial_1)
                {
                    RCLCPP_WARN(this->get_logger(), "Multiple devices match serial number 1 (%s): %s and %s",
                                serial_number_1.c_str(), path_serial_1.c_str(), path.c_str());
                }
                else
                {
                    path_serial_1 = path;
                    found_serial_1 = true;
                }
            }

            if (matches_serial_2)
            {
                if (found_serial_2)
                {
                    RCLCPP_WARN(this->get_logger(), "Multiple devices match serial number 2 (%s): %s and %s",
                                serial_number_2.c_str(), path_serial_2.c_str(), path.c_str());
                }
                else
                {
                    path_serial_2 = path;
                    found_serial_2 = true;
                }
            }
        }

        // Handle results based on number of devices found
        if (num_devices == 1)
        {
            RCLCPP_WARN(this->get_logger(), "Only one device found with PID:VID %s", pidvid.c_str());
            handle_single_device(device_paths[0], serial_number_1, serial_number_2, found_serial_1, found_serial_2);
        }
        else if (num_devices == 2)
        {
            handle_two_devices(device_paths, serial_number_1, serial_number_2,
                               path_serial_1, path_serial_2, found_serial_1, found_serial_2);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "More than two devices (%zu) found with PID:VID %s", num_devices, pidvid.c_str());
            handle_multiple_devices(device_paths, serial_number_1, serial_number_2,
                                    path_serial_1, path_serial_2, found_serial_1, found_serial_2);
        }
    }

    void handle_single_device(const std::string &path,
                              const std::string &serial_number_1,
                              const std::string &serial_number_2,
                              bool found_serial_1,
                              bool found_serial_2)
    {
        RCLCPP_INFO(this->get_logger(), "Single device path: %s", path.c_str());

        if (found_serial_1 && found_serial_2)
        {
            RCLCPP_INFO(this->get_logger(), "Device matches BOTH serial numbers:");
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 1: %s", path.c_str(), serial_number_1.c_str());
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 2: %s", path.c_str(), serial_number_2.c_str());
        }
        else if (found_serial_1)
        {
            RCLCPP_INFO(this->get_logger(), "Device matches Serial Number 1:");
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number: %s", path.c_str(), serial_number_1.c_str());
            if (!serial_number_2.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Serial Number 2 (%s) was NOT found on this device", serial_number_2.c_str());
            }
        }
        else if (found_serial_2)
        {
            RCLCPP_INFO(this->get_logger(), "Device matches Serial Number 2:");
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number: %s", path.c_str(), serial_number_2.c_str());
            if (!serial_number_1.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Serial Number 1 (%s) was NOT found on this device", serial_number_1.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Device does not match either serial number");
            if (!serial_number_1.empty())
            {
                RCLCPP_WARN(this->get_logger(), "  Serial Number 1 (%s) NOT found", serial_number_1.c_str());
            }
            if (!serial_number_2.empty())
            {
                RCLCPP_WARN(this->get_logger(), "  Serial Number 2 (%s) NOT found", serial_number_2.c_str());
            }
        }
    }

    void handle_two_devices(const std::vector<std::string> &paths,
                            const std::string &serial_number_1,
                            const std::string &serial_number_2,
                            const std::string &path_serial_1,
                            const std::string &path_serial_2,
                            bool found_serial_1,
                            bool found_serial_2)
    {
        RCLCPP_INFO(this->get_logger(), "Two devices found - attempting to align serial numbers to paths");

        // Print all found paths first
        for (const auto &path : paths)
        {
            RCLCPP_INFO(this->get_logger(), "  Device path: %s", path.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "--- Serial Number Alignment Results ---");

        if (found_serial_1 && found_serial_2)
        {
            if (path_serial_1 == path_serial_2)
            {
                RCLCPP_WARN(this->get_logger(), "Both serial numbers found on the SAME device: %s", path_serial_1.c_str());
                RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 1: %s", path_serial_1.c_str(), serial_number_1.c_str());
                RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 2: %s", path_serial_2.c_str(), serial_number_2.c_str());

                // Find the other device
                for (const auto &path : paths)
                {
                    if (path != path_serial_1)
                    {
                        RCLCPP_WARN(this->get_logger(), "  Path: %s -> No matching serial number found", path.c_str());
                    }
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Successfully aligned both serial numbers to different devices:");
                RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 1: %s", path_serial_1.c_str(), serial_number_1.c_str());
                RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 2: %s", path_serial_2.c_str(), serial_number_2.c_str());
            }
        }
        else if (found_serial_1)
        {
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 1: %s", path_serial_1.c_str(), serial_number_1.c_str());
            if (!serial_number_2.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Serial Number 2 (%s) was NOT found on any device", serial_number_2.c_str());
            }
            // Identify the unmatched device
            for (const auto &path : paths)
            {
                if (path != path_serial_1)
                {
                    RCLCPP_WARN(this->get_logger(), "  Path: %s -> No matching serial number", path.c_str());
                }
            }
        }
        else if (found_serial_2)
        {
            RCLCPP_INFO(this->get_logger(), "  Path: %s -> Serial Number 2: %s", path_serial_2.c_str(), serial_number_2.c_str());
            if (!serial_number_1.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Serial Number 1 (%s) was NOT found on any device", serial_number_1.c_str());
            }
            // Identify the unmatched device
            for (const auto &path : paths)
            {
                if (path != path_serial_2)
                {
                    RCLCPP_WARN(this->get_logger(), "  Path: %s -> No matching serial number", path.c_str());
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Neither serial number was found on any device!");
            if (!serial_number_1.empty())
            {
                RCLCPP_WARN(this->get_logger(), "  Serial Number 1 (%s) NOT found", serial_number_1.c_str());
            }
            if (!serial_number_2.empty())
            {
                RCLCPP_WARN(this->get_logger(), "  Serial Number 2 (%s) NOT found", serial_number_2.c_str());
            }
            for (const auto &path : paths)
            {
                RCLCPP_WARN(this->get_logger(), "  Path: %s -> Unknown device", path.c_str());
            }
        }
    }

    void handle_multiple_devices(const std::vector<std::string> &paths,
                                 const std::string &serial_number_1,
                                 const std::string &serial_number_2,
                                 const std::string &path_serial_1,
                                 const std::string &path_serial_2,
                                 bool found_serial_1,
                                 bool found_serial_2)
    {
        RCLCPP_WARN(this->get_logger(), "Found %zu devices - more than expected (2)", paths.size());

        // Print all found paths
        RCLCPP_INFO(this->get_logger(), "All device paths found:");
        for (const auto &path : paths)
        {
            RCLCPP_INFO(this->get_logger(), "  - %s", path.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "--- Serial Number Identification Results ---");

        if (found_serial_1)
        {
            RCLCPP_INFO(this->get_logger(), "Serial Number 1 (%s) found on path: %s",
                        serial_number_1.c_str(), path_serial_1.c_str());
        }
        else if (!serial_number_1.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Serial Number 1 (%s) was NOT found on any device", serial_number_1.c_str());
        }

        if (found_serial_2)
        {
            RCLCPP_INFO(this->get_logger(), "Serial Number 2 (%s) found on path: %s",
                        serial_number_2.c_str(), path_serial_2.c_str());
        }
        else if (!serial_number_2.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Serial Number 2 (%s) was NOT found on any device", serial_number_2.c_str());
        }

        // List unidentified devices
        RCLCPP_INFO(this->get_logger(), "--- Unidentified Devices ---");
        int unidentified_count = 0;
        for (const auto &path : paths)
        {
            if (path != path_serial_1 && path != path_serial_2)
            {
                RCLCPP_WARN(this->get_logger(), "  Path: %s -> No matching serial number", path.c_str());
                unidentified_count++;
            }
        }

        if (unidentified_count == 0)
        {
            RCLCPP_INFO(this->get_logger(), "  All devices have been identified");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "  %d device(s) could not be identified", unidentified_count);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<AutofindDevicesNode>();
        // Node does its work in constructor, but we spin briefly to allow logging to complete
        rclcpp::spin_some(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("autofind_devices"), "Unhandled exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
