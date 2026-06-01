#include "rover_odometry/pinpoint_i2c.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace rover_odometry
{
    namespace
    {

        int openI2cDevice(const std::string & path, int addr)
        {
            const int fd = ::open(path.c_str(), O_RDWR);
            if (fd < 0) {
                throw std::system_error(errno, std::generic_category(), "failed to open " + path);
            }

            if (::ioctl(fd, I2C_SLAVE, addr) < 0) {
                const int saved_errno = errno;
                ::close(fd);
                throw std::system_error(
                saved_errno, std::generic_category(), "failed to select I2C slave for " + path);
            }

            return fd;
        }

    }

    PinpointI2C::PinpointI2C(int bus, int addr, Endian endian)
    : bus_(bus), addr_(addr), endian_(endian)
    {
    }

    PinpointI2C::~PinpointI2C() = default;

    std::vector<std::uint8_t> PinpointI2C::readBytes(std::uint8_t reg, std::size_t length)
    {
        // Open per transaction so failures from unplugged/restarted hardware do
        // not leave a stale file descriptor inside the node.
        const int fd = openI2cDevice(devicePath(), addr_);

        std::vector<std::uint8_t> buffer(length);

        const auto reg_value = static_cast<std::uint8_t>(reg);
        const ssize_t write_result = ::write(fd, &reg_value, 1);
        if (write_result != 1) {
            const int saved_errno = errno;
            ::close(fd);
            throw std::system_error(
            saved_errno, std::generic_category(), "failed to write register address");
        }

        const ssize_t read_result = ::read(fd, buffer.data(), buffer.size());
        if (read_result != static_cast<ssize_t>(buffer.size())) {
            const int saved_errno = errno;
            ::close(fd);
            throw std::system_error(
            saved_errno, std::generic_category(), "failed to read register data");
        }

        ::close(fd);
        return buffer;
    }

    void PinpointI2C::writeBytes(std::uint8_t reg, const std::vector<std::uint8_t> & data)
    {
        const int fd = openI2cDevice(devicePath(), addr_);

        std::vector<std::uint8_t> buffer;
        buffer.reserve(1 + data.size());
        buffer.push_back(reg);
        buffer.insert(buffer.end(), data.begin(), data.end());

        const ssize_t write_result = ::write(fd, buffer.data(), buffer.size());
        if (write_result != static_cast<ssize_t>(buffer.size())) {
            const int saved_errno = errno;
            ::close(fd);
            throw std::system_error(
            saved_errno, std::generic_category(), "failed to write register data");
        }

        ::close(fd);
    }

    std::uint32_t PinpointI2C::decodeU32(const std::vector<std::uint8_t> & bytes) const
    {
        if (bytes.size() != sizeof(std::uint32_t)) {
            throw std::runtime_error("decodeU32 expected exactly 4 bytes");
        }

        std::array<std::uint8_t, sizeof(std::uint32_t)> raw{};
        std::copy(bytes.begin(), bytes.end(), raw.begin());

        if (endian_ == Endian::Big) {
            std::reverse(raw.begin(), raw.end());
        }

        std::uint32_t value = 0;
        std::memcpy(&value, raw.data(), sizeof(value));
        return value;
    }

    float PinpointI2C::decodeF32(const std::vector<std::uint8_t> & bytes) const
    {
        if (bytes.size() != sizeof(float)) {
            throw std::runtime_error("decodeF32 expected exactly 4 bytes");
        }

        std::array<std::uint8_t, sizeof(float)> raw{};
        std::copy(bytes.begin(), bytes.end(), raw.begin());

        if (endian_ == Endian::Big) {
            std::reverse(raw.begin(), raw.end());
        }

        float value = 0.0f;
        std::memcpy(&value, raw.data(), sizeof(value));
        return value;
    }

    std::vector<std::uint8_t> PinpointI2C::encodeU32(std::uint32_t value) const
    {
        std::array<std::uint8_t, sizeof(std::uint32_t)> raw{};
        std::memcpy(raw.data(), &value, sizeof(value));

        if (endian_ == Endian::Big) {
            std::reverse(raw.begin(), raw.end());
        }

        return std::vector<std::uint8_t>(raw.begin(), raw.end());
    }

    std::vector<std::uint8_t> PinpointI2C::encodeF32(float value) const
    {
        std::array<std::uint8_t, sizeof(float)> raw{};
        std::memcpy(raw.data(), &value, sizeof(value));

        if (endian_ == Endian::Big) {
            std::reverse(raw.begin(), raw.end());
        }

        return std::vector<std::uint8_t>(raw.begin(), raw.end());
    }

    std::uint32_t PinpointI2C::readU32(std::uint8_t reg)
    {
        return decodeU32(readBytes(reg, sizeof(std::uint32_t)));
    }

    float PinpointI2C::readF32(std::uint8_t reg)
    {
        return decodeF32(readBytes(reg, sizeof(float)));
    }

    void PinpointI2C::writeU32(std::uint8_t reg, std::uint32_t value)
    {
        writeBytes(reg, encodeU32(value));
    }

    void PinpointI2C::writeF32(std::uint8_t reg, float value)
    {
        writeBytes(reg, encodeF32(value));
    }

    void PinpointI2C::sendControl(std::uint32_t control_bits)
    {
        // Pinpoint command bits are written to register 4.
        writeU32(4, control_bits);
    }

    void PinpointI2C::resetImu()
    {
        sendControl(CTRL_RESET_IMU);
    }

    void PinpointI2C::resetPosAndImu()
    {
        sendControl(CTRL_RESET_IMU_AND_POS);
    }

    void PinpointI2C::setEncoderDirections(bool x_reversed, bool y_reversed)
    {
        std::uint32_t bits = 0;
        bits |= x_reversed ? CTRL_SET_X_REVERSED : CTRL_SET_X_FORWARD;
        bits |= y_reversed ? CTRL_SET_Y_REVERSED : CTRL_SET_Y_FORWARD;
        sendControl(bits);
    }

    void PinpointI2C::setTicksPerMm(float ticks_per_mm)
    {
        writeF32(14, ticks_per_mm);
    }

    void PinpointI2C::setPodOffsetsMm(float x_pod_offset_mm, float y_pod_offset_mm)
    {
        writeF32(15, x_pod_offset_mm);
        writeF32(16, y_pod_offset_mm);
    }

    void PinpointI2C::setYawScalar(float yaw_scalar)
    {
        writeF32(17, yaw_scalar);
    }

    void PinpointI2C::setPositionMmRad(float x_mm, float y_mm, float yaw_rad)
    {
        // Pose registers are stored in millimeters for x/y and radians for yaw.
        writeF32(8, x_mm);
        writeF32(9, y_mm);
        writeF32(10, yaw_rad);
    }

    std::string PinpointI2C::devicePath() const
    {
        return "/dev/i2c-" + std::to_string(bus_);
    }

    PinpointI2C::Endian parseEndian(const std::string & endian_string)
    {
        if (endian_string == "little") {
            return PinpointI2C::Endian::Little;
        }

        if (endian_string == "big") {
            return PinpointI2C::Endian::Big;
        }

        throw std::invalid_argument("endian must be 'little' or 'big'");
    }
}
